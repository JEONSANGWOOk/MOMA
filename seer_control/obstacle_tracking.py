"""World-position temporal classification; SIM observations, no type-flag shortcut."""
import math
from collections import deque
from types import SimpleNamespace
from .studio_core import path_clearance

CLASSES={'dynamic':'동적','static':'정적','unknown':'판단 중'}
SCENARIOS={'wait':'해제될 때까지 대기','stop':'정지 / 수동 재개','avoid':'주변 우회 후 기존 경로 복귀','reroute':'다른 연결 경로 / 불가 목적지 패스','wait_avoid':'설정 시간 대기 → 주변 우회','wait_reroute':'설정 시간 대기 → 다른 경로 / 패스'}
DEFAULTS={'dynamic':'wait','static':'wait_reroute','unknown':'wait'}

class ObstacleTracker:
 def __init__(self):self.tracks={};self.time=0.
 def update(self,obstacles,dt,moving_speed=.08,static_s=2.):
  self.time+=dt;present=set()
  for obs in obstacles:
   key=id(obs);present.add(key);p=(float(obs['x']),float(obs['y']))
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
   obs['_classification']=t['kind'];obs['_observed_speed']=round(speed,3)
  self.tracks={k:v for k,v in self.tracks.items() if k in present}
 def blocker(self,model,start,points,radius,horizon,reason):
  if reason.startswith(('벽/','금지구역','기존 연결')):return ('map',reason),'static',0.,None
  found=[]
  for obs in model.obstacles:
   scene=SimpleNamespace(walls=[],virtual_walls=[],area_records=[],obstacles=[obs])
   distance,hit=path_clearance(scene,start,points,radius,horizon)
   if hit:found.append((distance,id(obs),obs))
  if not found:return ('unknown',reason),'unknown',0.,None
  _,key,obs=min(found,key=lambda item:item[:2]);track=self.tracks.get(key,{})
  return key,track.get('kind','unknown'),track.get('speed',0.),obs
