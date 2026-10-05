"""Observed-state-only BLOCKED recovery. Physical footprint is never reduced."""
import math
from .studio_core import collision_reason,path_clearance,zone_limits
from .avoidance import detour,rejoin_detour,clear_segment
class BlockedRecovery:
 def __init__(self):
  self.enabled=False;self.phase='IDLE';self.priority=0;self.back_limit=.18;self.side_margin=.01;self.back_speed=.08;self.next_try=0.;self.events=[];self.rear=None;self.heading=0.;self.remaining=0.;self.plan=None;self.reason='';self.attempts=0;self.anchor=None
 @property
 def active(self):return self.phase!='IDLE'
 def reset(self):self.phase='IDLE';self.plan=None
 def new_goal(self):self.reset();self.attempts=0;self.anchor=None;self.next_try=0.
 def allowed(self,sim):
  return self.enabled and sim.obstacle_policy in ('avoid','adaptive','auto','reroute') and not sim._obstacle_latched and not sim._manual_stop_reason and bool(sim.route) and not(sim.obstacle_policy=='auto' and sim._auto_block and sim._auto_block[-1] in ('wait','stop'))
 def radius(self,sim):return max(sim.collision_radius,sim.map.robot_model['radius'],sim.applied_limits.get('radius',0))
 def turn_radius(self,sim):return max(self.radius(sim),math.hypot(sim.map.robot_model['length'],sim.map.robot_model['width'])/2)
 def log(self,sim,event):
  self.events.append(dict(time_s=round(sim._avoid_time,2),priority=self.priority,event=event));self.events=self.events[-200:];sim.avoidance_status=event
 def begin(self,sim,reason):
  if self.active:return True
  if not self.allowed(sim) or sim._avoid_time<self.next_try:return False
  s=sim.state;r=self.radius(sim);origin=(s.x,s.y)
  if sim._reroute_forced:
   try:sim.map.route(sim.map.nearest(s.x,s.y),s.target)
   except ValueError:return False
  if self.anchor is not None and math.dist(origin,self.anchor)<.5 and self.attempts>=2:return False
  if self.anchor is None or math.dist(origin,self.anchor)>=.5:self.attempts=0

  if collision_reason(sim.map,*origin,r):return False
  self.heading=s.theta;unit=(-math.cos(s.theta),-math.sin(s.theta));previous=sim.map.nodes.get(s.last_node)
  rear=(origin[0]+unit[0]*.55,origin[1]+unit[1]*.55)
  if previous:
   p=(previous['x'],previous['y']);projection=(p[0]-s.x)*unit[0]+(p[1]-s.y)*unit[1]
   if projection>.15:rear=p
  rear_clear=clear_segment(sim.map,origin,rear,r+.01)
  self.rear=rear;self.reason=reason;self.priority=1 if rear_clear else 2
  if rear_clear and not collision_reason(sim.map,*origin,self.turn_radius(sim)):
   self.phase='TURN';self.log(sim,'회복 1순위 · 뒤쪽 통로 확인 · 제자리 180° 회전')
  else:
   distance=0.;limit=self.back_limit if not rear_clear else max(self.back_limit,self.turn_radius(sim)-r+.08)
   # Entire swept reverse segment must be free; stop before the rear obstacle.
   for i in range(1,int(limit/.01)+1):
    p=(origin[0]+unit[0]*i*.01,origin[1]+unit[1]*i*.01)
    if not clear_segment(sim.map,origin,p,r+.01):break
    distance=i*.01
   distance=max(0,distance-.02)
   if distance<.015:
    self.next_try=sim._avoid_time+2.;self.log(sim,'회복 대기 · 후진/회전 공간 부족');return False
   self.remaining=distance;self.phase='BACK';self.log(sim,f'회복 {self.priority}순위 · 충돌 확인 후 제한 후진 {distance:.2f}m')
  self.attempts+=1;self.anchor=origin
  sim._velocity=0.;s.blocked=False;sim._collision_blocked=False
  return True
 def _finish(self,sim,status):
  self.reset();self.next_try=sim._avoid_time+2.;sim._velocity=0.;sim.state.speed=0.;self.log(sim,status)
 def step(self,sim,dt):
  if not self.active:return False
  s=sim.state;s.speed=0.;sim._velocity=0.
  if not sim.route or s.mode!='RUNNING':self.reset();return False
  if s.stopped or not s.motor or sim._obstacle_latched:return True
  r=self.radius(sim);origin=(s.x,s.y);limits=zone_limits(sim.map,s.x,s.y,sim.applied_limits)
  if self.phase=='BACK':
   amount=min(self.remaining,self.back_speed*dt,limits['maxspeed']*dt);unit=(-math.cos(self.heading),-math.sin(self.heading));end=(s.x+unit[0]*amount,s.y+unit[1]*amount)
   if not clear_segment(sim.map,origin,end,r+.01):
    self._finish(sim,'회복 후진 중 신규 장애물 · 정지 후 재탐색');s.blocked=True;sim._collision_blocked=True;return True
   s.x,s.y=end;self.remaining-=amount;s.speed=-amount/max(dt,1e-9);s.blocked=False;sim._collision_blocked=False
   if self.remaining<=.001:
    if self.priority==1 and not collision_reason(sim.map,s.x,s.y,self.turn_radius(sim)):self.phase='TURN';self.log(sim,'뒤 여유 확보 · 제자리 180° 회전')
    else:self.phase='PLAN';self.log(sim,'회복 2순위 · 후진 완료 · 측면 추가 감지 여유 축소 후 전방 우회 탐색')
   return True
  if self.phase=='TURN':
   if collision_reason(sim.map,*origin,self.turn_radius(sim)):
    self._finish(sim,'회전 중 장애물 접근 · 회전 정지');s.blocked=True;sim._collision_blocked=True;return True
   target=self.heading+math.pi;delta=math.atan2(math.sin(target-s.theta),math.cos(target-s.theta));step=max(-limits['maxrot']*dt,min(limits['maxrot']*dt,delta));s.theta=math.atan2(math.sin(s.theta+step),math.cos(s.theta+step))
   if abs(delta)<=max(.015,abs(step)):self.phase='PLAN';self.log(sim,'회복 1순위 · 회전 완료 · 뒤쪽 경로 재생성')
   s.blocked=False;sim._collision_blocked=False;return True
  node=sim.map.nodes[sim.route[0]];goal=(node['x'],node['y']);points=None
  if self.priority==1 and self.rear and math.dist(origin,self.rear)>.04 and clear_segment(sim.map,origin,self.rear,r+.01):
   tail=detour(sim.map,self.rear,goal,r,safety_margin=self.side_margin,max_cells=40000,risk_aware=True)
   if tail:points=[origin]+tail
  if points is None and self.priority==2:
   reference=getattr(sim,'_reference_waypoints',[])
   if reference and math.dist(reference[-1],goal)<1e-5:points=rejoin_detour(sim.map,origin,reference,r,safety_margin=self.side_margin,risk_aware=True)
   if not points:points=detour(sim.map,origin,goal,r,safety_margin=self.side_margin,max_cells=40000,grid_step=.08,risk_aware=True)
  destination=sim.route[0]
  if not points and sim.state.target and destination!=sim.state.target:
   final=sim.map.nodes[sim.state.target]
   start=self.rear if self.priority==1 and self.rear and clear_segment(sim.map,origin,self.rear,r+.01) else origin
   tail=detour(sim.map,start,(final['x'],final['y']),r,safety_margin=self.side_margin,max_cells=40000,grid_step=.08,risk_aware=True)
   if tail:points=([origin]+tail if start!=origin else tail);destination=sim.state.target
  if not points:
   self._finish(sim,'회복 통로 없음 · 안전 정지 후 다른 경로 재탐색');s.blocked=True;sim._collision_blocked=True;return True
  # Recheck every segment against current observed positions after the turn/back.
  if path_clearance(sim.map,origin,points[1:],r,sum(math.dist(a,b) for a,b in zip(points,points[1:])))[1]:
   self._finish(sim,'회복 경로에 신규 장애물 · 안전 정지');s.blocked=True;sim._collision_blocked=True;return True
  priority=self.priority
  if destination!=sim.route[0]:sim.route=[destination];sim._segment_start=None
  sim._waypoints=points[1:];sim._local_avoidance_active=True;sim._segment_reverse=False;sim._arrival_align=False
  sim._recovery_narrow=True;sim._reroute_block_at=None;sim._reroute_failures=0;sim._avoid_next=sim._avoid_time+2.;sim._route_scan_next=sim._avoid_time+2.
  s.blocked=False;sim._collision_blocked=False;sim.block_reason='';self._finish(sim,f'회복 {priority}순위 경로 적용 · '+('뒤쪽으로 이동 후 목적지 재접근' if priority==1 else '전방 장애물 우회 → 기존 경로 복귀'))
  return True
