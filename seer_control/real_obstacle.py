"""Real mission recovery orchestration; motion remains owned by SEER navigation."""
from .decision_log import decide, audited, snapshot
import math,time
class RealObstacleRecovery:
 def __init__(self,console,goal):
  self.c=console;self.goal=goal;self.phase='IDLE';self.since=None;self.attempt=0;self.sent=0.;self.ack=0.;self.skip=False;self.samples=[];self.kind='판단 중';self.status='실기 장애물 관측 대기'
 def event(self,text):
  decide(self,'REAL.장애물 전환',getattr(self,'observed_state',{}),dict(goal=self.goal,phase=self.phase,kind=self.kind,policy=self.c.sim.obstacle_policy,attempt=self.attempt,attempt_limit=self.c.sim.reroute_attempt_limit,wait_s=self.c.sim.reroute_wait_s,auto_wait_s=self.c.sim.auto_wait_s),'장애물 대응 단계 전환',text,force=True)
  self.status=text;self.c.real_obstacle_status=text;self.c.log('REAL_OBSTACLE',text)
  runner=self.c.studio_runner
  if runner.report:runner.report.add(runner.cycle,runner.index,'장애물 대응',self.goal,text,runner.elapsed)
 def observe(self,state,now):
  x,y=state.get('block_x'),state.get('block_y')
  if type(x) in (int,float) and type(y) in (int,float) and math.isfinite(x+y):
   self.samples.append((now,x,y));self.samples=[p for p in self.samples if now-p[0]<=3]
   if len(self.samples)>1 and self.samples[-1][0]-self.samples[0][0]>=1:
    # Only observed block-point changes; identity/person type is not inferred.
    distance=math.dist(self.samples[0][1:],self.samples[-1][1:])
    self.kind='움직임 감지' if distance>.2 else '정체 관측'
    decide(self,'REAL.장애물 관측',state,dict(block_point=[x,y],samples=len(self.samples),observation_s=self.samples[-1][0]-self.samples[0][0],displacement_m=distance,moving_displacement_m=.2,min_observation_s=1),'관측 결과: '+self.kind,'관측 결과를 대기 시간 선택에 사용',identity=self.kind)
 @audited('REAL.장애물 복구')
 def poll(self,state,now=None):
  now=time.monotonic() if now is None else now;c=self.c
  self.observed_state=dict(state)
  if not c.studio_config.get('real_obstacle_recovery',False):
   decide(self,'REAL.장애물 복구 설정',state,dict(enabled=False),'GUI 실기 장애물 복구 비활성','SEER 제어기 주행 상태를 관측 · GUI 자동 복구 미실행')
   return None
  policy=c.sim.obstacle_policy
  if state.get('emergency') or state.get('stopped') or state.get('motor') is False:raise ValueError('실기 회복 중 비상정지/모터 상태 확인')
  if self.phase=='HOLD':return False
  if self.phase=='CANCEL':
   record=getattr(c,'real_command_acks',{}).get('cancel')
   if record and record[0]>=self.sent:
    response=record[1]
    if int(response.get('ret_code',0))!=0:raise ValueError('취소 거부: '+str(response))
    if not self.ack:self.ack=record[0]
   if now-self.sent>15:raise ValueError('취소/정지 확인 시간 초과 · 자동 재주행 중단')
   terminal=state.get('task') in ('CANCELED','COMPLETED','IDLE','NONE','FAILED') or state.get('task_status') in (0,4,5,6)
   confirmed=bool(self.ack) and c.last_state>self.ack and terminal and abs(float(state.get('speed',0)))<=.02
   decide(self,'REAL.취소/정지 확인',state,dict(cancel_sent_at=self.sent,cancel_ack_at=self.ack,state_at=c.last_state,terminal=terminal,speed=state.get('speed'),max_speed=.02,timeout_s=15),'취소 ACK 이후 최신 정지 상태 확인' if confirmed else '취소/정지 확인 조건 미충족','후속 정책 실행' if confirmed else '재주행 금지 · 최신 상태 대기',identity=(bool(self.ack),c.last_state>self.ack,terminal,confirmed))
   if not confirmed:return False
   if policy=='stop':self.phase='HOLD';self.event('정지 정책 · 취소/정지 확인 완료 · 미션 취소 후 새 작업 필요');return False
   if self.skip:
    self.phase='SKIPPED';reason='실기 장애물 재탐색 한도 초과 · 목적지 미도착'
    self.event(reason);c.sim.skipped_goals[self.goal]=dict(goal=self.goal,reason=reason)
    return dict(skip=True,goal=self.goal,reason=reason)
   if time.monotonic()-c.last_laser_rx>2 or not c.real_laser_points:raise ValueError('최신 실기 LiDAR 없이 자동 재탐색하지 않습니다.')
   c.studio_command_error=None;c.send_command('navigate_free',c._station_nav_payload(c.map.nodes[self.goal]));self.phase='NAV_ACK';self.sent=now
   self.event(f'자유 경로 재탐색 요청 {self.attempt}회 · {self.goal}');return False
  if self.phase=='NAV_ACK':
   record=getattr(c,'real_command_acks',{}).get('navigate_free')
   if not record or record[0]<self.sent:
    if now-self.sent>15:raise ValueError('자유 경로 명령 응답 시간 초과')
    decide(self,'REAL.재주행 응답',state,dict(command='navigate_free',sent_at=self.sent,elapsed_s=now-self.sent,timeout_s=15),'최신 재주행 ACK 미수신','명령 재전송 없이 응답 대기')
    return False
   if int(record[1].get('ret_code',0))!=0:raise ValueError('제어기 자유 주행 거부: '+str(record[1]))
   self.phase='IDLE';self.since=None;self.event('자유 경로 ACK 수신 · 명령 수락 확인 · 실제 주행은 상태로 별도 확인');return False
  blocked=state.get('blocked') is True or state.get('task')=='FAILED' or state.get('task_status')==5
  if not blocked:
   if self.since is not None:self.event('장애물 해제 · 기존 주행 계속')
   self.since=None;self.samples=[];return None
  self.observe(state,now)
  if self.since is None:self.since=now;self.event('장애물 감지 · 대기 시작')
  c.real_obstacle_status=f'{self.kind} · 대기 {now-self.since:.1f}초 · 재탐색 {self.attempt}회'
  if policy=='stop':
   self.sent=now;self.phase='CANCEL';c.send_command('cancel',{});self.event('정지 정책 · 취소 요청');return False
  if policy=='wait':
   decide(self,'REAL.장애물 대응',state,dict(policy=policy,kind=self.kind,elapsed_s=now-self.since),'해제까지 대기 정책','GUI 재주행 요청 없이 제어기 장애물 해제 대기')
   return False
  wait=max(0,float(c.sim.reroute_wait_s))
  if policy in ('auto','adaptive') and self.kind!='정체 관측':wait=max(wait,float(c.sim.auto_wait_s))
  if now-self.since<wait:
   decide(self,'REAL.장애물 대응',state,dict(policy=policy,kind=self.kind,elapsed_s=now-self.since,wait_s=wait,attempt=self.attempt),'대기 시간 미도달','정지/장애물 상태 관측 유지',identity=(policy,self.kind))
   return False
  if time.monotonic()-c.last_laser_rx>2 or not c.real_laser_points:
   c.real_obstacle_status='LiDAR 수신 지연 · 재탐색 대기'
   decide(self,'REAL.재탐색 센서 검증',state,dict(lidar_age_s=time.monotonic()-c.last_laser_rx,max_age_s=2,point_count=len(c.real_laser_points)),'최신 LiDAR 조건 미충족','재주행 요청 보류 · 센서 대기')
   return False
  self.skip=self.attempt>=c.sim.reroute_attempt_limit;self.attempt+=not self.skip
  self.sent=now;self.ack=0.;self.phase='CANCEL';c.send_command('cancel',{})
  self.event('목적지 패스 전 취소 확인' if self.skip else '대기 종료 · 취소/정지 확인 후 자유 경로 재탐색');return False
