import math,unittest
from seer_control.model import MapModel,Simulator
from seer_control.studio_core import collision_reason

class DeadlockTests(unittest.TestCase):
 def test_head_on_amrs_recover_without_overlap(self):
  m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=8,y=0)],edges=[['A','B']],walls=[],obstacles=[dict(id='OTHER',dynamic=True,map_fixed=False,kind='amr',x=8,y=0,radius=.61,motion_nodes=['B','A'],motion_path=[[8,0],[0,0]],speed_mps=.5,dwell_s=0)]))
  o=m.obstacles[0]
  s=Simulator(m);s.collision_radius=.61;s.obstacle_policy='auto';s.auto_wait_s=.5;s.auto_static_s=.5;s.navigate('B');both_stopped=False;detoured=False
  for _ in range(2000):
   s.tick(.1);both_stopped|=s.state.blocked and o.get('_motion')=='通行待機'
   both_stopped|=s.state.blocked and o.get('_motion')=='통행 대기'
   detoured|=abs(s.state.y)>.5
   self.assertGreater(math.dist((s.state.x,s.state.y),(o['x'],o['y'])),1.22-1e-6)
   if s.state.last_node=='B':break
  self.assertTrue(both_stopped);self.assertTrue(detoured);self.assertEqual(s.state.last_node,'B')
 def test_explicit_wait_policy_stays_waiting(self):
  m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=3,y=0)],edges=[['A','B']],walls=[],obstacles=[dict(id='P',dynamic=True,kind='person',x=1.5,y=0,radius=.3,motion_path=[[1.5,0],[1.5,2]],speed_mps=.5,paused=True)]))
  s=Simulator(m);s.obstacle_policy='wait';s.navigate('B')
  for _ in range(100):s.tick(.1)
  self.assertTrue(s.state.blocked);self.assertAlmostEqual(s.state.y,0)

 def test_local_failure_falls_back_to_connected_route(self):
  from unittest.mock import Mock
  m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id=k,x=x,y=y) for k,x,y in [('A',0,0),('B',3,0),('C',0,2),('D',3,2)]],edges=[['A','B'],['A','C'],['C','D'],['D','B']],walls=[],obstacles=[dict(x=1.5,y=0,radius=.3,map_fixed=False)]))
  s=Simulator(m);s.obstacle_policy='auto';s.auto_static_s=.5;s.auto_wait_s=.5;s.prefer_graph_routes=False;s._try_local_detour=Mock(return_value=False);s.navigate('B');upper=False
  for _ in range(1000):s.tick(.1);upper|=s.state.y>1.5
  self.assertTrue(s._try_local_detour.called);self.assertTrue(upper);self.assertEqual(s.state.last_node,'B')
 def test_all_recovery_routes_fail_marks_goal(self):
  m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=3,y=0)],edges=[['A','B']],walls=[],obstacles=[dict(x=3,y=0,radius=.3,map_fixed=False)]))
  s=Simulator(m);s.obstacle_policy='auto';s.auto_static_s=.5;s.auto_wait_s=.5;s.reroute_wait_s=.5;s.reroute_attempt_limit=2;s.navigate('B')
  for _ in range(200):s.tick(.1)
  self.assertIn('B',s.skipped_goals);self.assertNotEqual(s.state.last_node,'B')
