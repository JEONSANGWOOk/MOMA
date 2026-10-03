import time,unittest
from types import SimpleNamespace
from unittest.mock import Mock
from seer_control.model import MapModel,Simulator
from seer_control.studio_ui import ConsoleAdapter
from seer_control.studio_core import MissionRunner

class LoopRecoveryTests(unittest.TestCase):
 def console(self,m):
  s=Simulator(m);s.obstacle_policy='auto';s.auto_wait_s=.2;s.auto_static_s=.5;s.reroute_wait_s=.2
  c=SimpleNamespace(sim=s,map=m,real=False,connected=True,sim_powered=True,control_enabled=False,studio_arm_safe=True,current_state=s.status,log=Mock(),studio_bridge=Mock(),studio_config={'arm':{}},studio_results={})
  c.studio_runner=MissionRunner(ConsoleAdapter(c));return c
 def test_long_actual_navigation_does_not_cancel_loop(self):
  m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=70,y=0)],edges=[['A','B']],walls=[]));c=self.console(m);r=c.studio_runner;r.start([dict(type='Path Nav',goal='B'),dict(type='Path Nav',goal='A')],repeat=2);now=time.monotonic();long_seen=False
  for i in range(14000):
   c.sim.tick(.1);r.tick(now+i*.1,.1)
   long_seen|=r.elapsed>r.actions[r.index if r.index<len(r.actions) else 0]['timeout_s'] and r.status=='RUNNING'
   if r.status in ('COMPLETED','FAILED'):break
  self.assertTrue(long_seen);self.assertEqual(r.status,'COMPLETED',r.error);self.assertEqual(r.cycle,2);self.assertFalse(r.skipped)
 def test_inactive_destination_skips_and_infinite_loop_continues(self):
  m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=3,y=0),dict(id='C',x=0,y=2)],edges=[['A','B'],['A','C']],walls=[],obstacles=[dict(x=3,y=0,radius=.3,map_fixed=False)]));c=self.console(m);c.sim.auto_static_s=30;r=c.studio_runner;r.start([dict(type='Path Nav',goal='B'),dict(type='Set DO',channel=0,value=True),dict(type='Path Nav',goal='C')],repeat=0);now=time.monotonic();r.tick(now,.1);r.actions[0]['timeout_s']=2
  for i in range(1,500):
   c.sim.tick(.1);r.tick(now+i*.1,.1)
   if r.cycle>=1:break
  self.assertEqual(r.status,'RUNNING',r.error);self.assertGreaterEqual(r.cycle,1);self.assertEqual(r.skipped[0]['goal'],'B');self.assertIn('시간 제한',r.skipped[0]['reason']);self.assertFalse(c.sim.do[0]);self.assertEqual(c.sim.state.last_node,'C')
 def test_arrival_dwell_survives_long_navigation_timer(self):
  m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=1,y=0)],edges=[['A','B']],walls=[]));c=self.console(m);r=c.studio_runner;r.start([dict(type='Path Nav',goal='B',delay_ms=30000)]);now=time.monotonic();r.tick(now,.1);r.actions[0]['timeout_s']=10
  for i in range(1,500):
   c.sim.tick(.1);r.tick(now+i*.1,.1)
   if r.status in ('COMPLETED','FAILED'):break
  self.assertEqual(r.status,'COMPLETED',r.error);self.assertGreater(r.elapsed,30)
 def test_stop_condition_does_not_auto_skip(self):
  m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=3,y=0)],edges=[['A','B']],walls=[]));c=self.console(m);r=c.studio_runner;r.start([dict(type='Path Nav',goal='B')]);now=time.monotonic();r.tick(now,.1);r.actions[0]['timeout_s']=1;c.sim.state.stopped=True;r.tick(now+2,.1)
  self.assertEqual(r.status,'FAILED');self.assertFalse(r.skipped)

 def test_repeated_graph_replans_do_not_cancel_loop(self):
  m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id=k,x=x,y=y) for k,x,y in [('A',0,0),('B',8,0),('C',0,2),('D',8,2),('F',0,4),('G',8,4)]],edges=[['A','B'],['A','C'],['C','D'],['D','B'],['C','F'],['F','G'],['G','D']],walls=[],obstacles=[dict(id='OTHER',dynamic=True,kind='amr',map_fixed=False,x=4,y=0,radius=.61,motion_path=[[4,0],[4,2]],speed_mps=.5,paused=True)]));c=self.console(m);c.sim.obstacle_policy='reroute';r=c.studio_runner;r.start([dict(type='Path Nav',goal='B'),dict(type='Path Nav',goal='A')],repeat=2);now=time.monotonic();r.tick(now,.1);r.actions[0]['timeout_s']=10;added=False;upper=False
  for i in range(1,3000):
   c.sim.tick(.1)
   if not added and '다른 연결 경로' in c.sim.avoidance_status:m.obstacles.append(dict(x=4,y=2,radius=.3,map_fixed=False));added=True
   upper|=c.sim.state.y>3.5;r.tick(now+i*.1,.1)
   if r.status in ('COMPLETED','FAILED'):break
  self.assertTrue(added and upper);self.assertEqual(r.status,'COMPLETED',r.error);self.assertEqual(r.cycle,2);self.assertFalse(r.skipped)
