"""World-position temporal classification; SIM observations, no type-flag shortcut."""
import math
from collections import deque
from types import SimpleNamespace
from .studio_core import path_clearance

CLASSES={'dynamic':'동적','static':'정적','unknown':'판단 중'}
SCENARIOS={'wait_recover':'대기 → 주변 우회 → 다른 경로 / 패스','wait':'해제될 때까지 대기','stop':'정지 / 수동 재개','avoid':'주변 우회 후 기존 경로 복귀','reroute':'다른 연결 경로 / 불가 목적지 패스','wait_avoid':'설정 시간 대기 → 주변 우회','wait_reroute':'설정 시간 대기 → 다른 경로 / 패스'}
DEFAULTS={'dynamic':'wait','static':'wait_recover','unknown':'wait'}

class ObstacleTracker:
 def __init__(self):
  self.tracks={};self.time=0.;self.detections={};self.next_id=1;self.known=[]
  self.sensor_range=8.;self.lost_s=1.;self.retain_s=30.
 def register_map(self,obstacles):
  self.known=[(float(o['x']),float(o['y']),float(o['radius'])) for o in obstacles if o.get('map_fixed',not o.get('dynamic'))]
 def is_known(self,obs):
  if obs.get('map_fixed') is True:return True
  if obs.get('map_fixed') is False:return False
  return any(math.dist((obs['x'],obs['y']),(x,y))<=.1 and abs(obs['radius']-r)<=.1 for x,y,r in self.known)
 def observe(self,model,robot,dt):
  from .avoidance import clear_segment
  now=self.time+dt;assigned=set()
  for obs in model.obstacles:
   if self.is_known(obs):
    obs.pop('_track_id',None);continue
   position=(float(obs['x']),float(obs['y']))
   if math.dist(position,(robot.x,robot.y))>self.sensor_range:continue
   scene=SimpleNamespace(walls=model.walls,virtual_walls=model.virtual_walls,area_records=[],obstacles=[o for o in model.obstacles if o is not obs])
   if not clear_segment(scene,(robot.x,robot.y),position,0.):continue
   candidates=[]
   for key,record in self.detections.items():
    if key in assigned:continue
    same=record['source'] is obs or bool(obs.get('id') and obs.get('id')==record['source'].get('id'))
    elapsed=now-record['last_seen']
    distance=math.dist(position,(record['x'],record['y']))
    if same or elapsed<=3 and distance<=min(1.5,.3+record['speed']*elapsed) and abs(record['radius']-obs['radius'])<=.15:
     candidates.append((0 if same else 1,distance,key))
   if candidates:key=min(candidates)[2];record=self.detections[key]
   else:
    key=f'TRK{self.next_id:03d}';self.next_id+=1
    record=dict(id=key,source=obs,x=position[0],y=position[1],radius=obs['radius'],kind='unknown',speed=0.,first_seen=now,last_seen=now,trail=deque(maxlen=120),state='추적 중')
    self.detections[key]=record
   if now-record['last_seen']>=self.lost_s:record['trail'].clear()
   obs['_track_id']=key;assigned.add(key);record.update(source=obs,x=position[0],y=position[1],radius=obs['radius'],last_seen=now,state='추적 중')
   if not record['trail'] or math.dist(position,record['trail'][-1])>=.04:record['trail'].append(position)
  for key,record in list(self.detections.items()):
   if now-record['last_seen']>self.retain_s:del self.detections[key]
   elif now-record['last_seen']>=self.lost_s:record['state']='미관측'
 def records(self):return list(self.detections.values())

 def update(self,obstacles,dt,moving_speed=.08,static_s=2.):
  self.time+=dt;present=set()
  for obs in obstacles:
   key=obs.get('_track_id',id(obs));present.add(key);p=(float(obs['x']),float(obs['y']))
   t=self.tracks.setdefault(key,dict(samples=deque(),kind='unknown',speed=0.,still_since=self.time,origin=p))
   samples=t['samples'];samples.append((self.time,p))
   while len(samples)>1 and self.time-samples[0][0]>.8:samples.popleft()
   elapsed=self.time-samples[0][0];distance=math.dist(p,samples[0][1])
   speed=distance/elapsed if elapsed>.0001 else 0.;t['speed']=speed
   if elapsed>=.3 and speed>=moving_speed and distance>=.025:
    t['kind']='dynamic';t['still_since']=self.time;t['origin']=p
   elif math.dist(p,t['origin'])>=.025:
    t['still_since']=self.time;t['origin']=p
   elif self.time-t['still_since']>=static_s:t['kind']='static'
   if self.is_known(obs):t['kind']='static';t['speed']=0.;speed=0.
   obs['_classification']=t['kind'];obs['_observed_speed']=round(speed,3)
   origin=samples[0][1]
   obs['_observed_velocity']=((p[0]-origin[0])/elapsed,(p[1]-origin[1])/elapsed) if elapsed>=.3 and not self.is_known(obs) else (0.,0.)
   record=self.detections.get(obs.get('_track_id'))
   if record and record['source'] is obs and record['last_seen']>=self.time-1e-8:record.update(kind=t['kind'],speed=speed)

  self.tracks={k:v for k,v in self.tracks.items() if k in present}
 def blocker(self,model,start,points,radius,horizon,reason):
  if reason.startswith(('벽/','금지구역','기존 연결')):return ('map',reason),'static',0.,None
  found=[]
  for obs in model.obstacles:
   scene=SimpleNamespace(walls=[],virtual_walls=[],area_records=[],obstacles=[obs])
   distance,hit=path_clearance(scene,start,points,radius,horizon)
   if hit:found.append((distance,id(obs),obs))
  if not found:return ('unknown',reason),'unknown',0.,None
  _,key,obs=min(found,key=lambda item:item[:2]);track=self.tracks.get(obs.get('_track_id',key),{})
  return obs.get('_track_id',key),track.get('kind','unknown'),track.get('speed',0.),obs
