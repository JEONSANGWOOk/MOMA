import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
from seer_control.model import MapModel,Simulator
from seer_control.alternate_routes import alternate_route
from seer_control.studio_core import MissionRunner,collision_reason
from seer_control.studio_ui import ConsoleAdapter

class SearchAndMarkTests(unittest.TestCase):
 def test_return_anchor_found_after_previous_context_lost(self):
  m=MapModel.load(Path(__file__).resolve().parents[1]/'maps/demo.json');m.obstacles=[dict(dynamic=True,kind='amr',x=4.23,y=2,radius=.61)]
  plan=alternate_route(m,(5.48,2),'LM8',None,'LM1',[(5.48,2),(2,2)],.6045,True)
  self.assertEqual(plan['nodes'],['LM2','LM7','LM8'])
 def test_backtracking_and_very_long_connected_route(self):
  m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id=k,x=x,y=y) for k,x,y in [('A',0,0),('B',10,0),('C',0,50),('D',10,50)]],edges=[['A','B'],['A','C'],['C','D'],['D','B']],walls=[],obstacles=[dict(x=5,y=0,radius=.5)]))
  plan=alternate_route(m,(3,0),'B',None,'B',[(3,0),(10,0)],.3)
  self.assertEqual(plan['nodes'],['A','C','D','B']);self.assertEqual(plan['prefix'][-1],(0,0))
 def test_physical_corridor_checked_after_buffer_failure(self):
  m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=4,y=0)],edges=[['A','B']],walls=[[-1,.65,5,.65],[-1,-.65,5,-.65]]))
  plan=alternate_route(m,(0,0),'B','A','B',[(0,0),(4,0)],.61)
  self.assertTrue(plan['reduced_margin']);self.assertEqual(plan['nodes'][-1],'B');self.assertEqual(plan['prefix'][-1],(4,0))
 def test_off_lane_detour_reconnects_without_teleport(self):
  m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=4,y=0)],edges=[['A','B']],walls=[]))
  plan=alternate_route(m,(1,1),'B',None,'B',[(1,1),(2,1)],.3)
  self.assertIsNotNone(plan);self.assertEqual(plan['prefix'][0],(1,1));self.assertEqual(plan['nodes'][-1],'B');self.assertEqual(plan['prefix'][-1],(4,0))
 def test_future_dynamic_blocker_replans_before_next_node(self):
  m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id=k,x=x,y=y) for k,x,y in [('A',0,0),('B',4,0),('C',8,0),('E',4,4),('F',8,4)]],edges=[['A','B'],['B','C'],['B','E'],['E','F'],['F','C']],walls=[],obstacles=[dict(id='P',dynamic=True,kind='person',x=6,y=0,radius=.3,motion_path=[[6,0],[6,2]],speed_mps=.5,paused=True)]))
  s=Simulator(m);s.obstacle_policy='reroute';s.navigate('C');s.tick(.1)
  self.assertEqual(s.route,['A','B','E','F','C']);self.assertAlmostEqual(s.state.x,0)
 def scene(self):
  return MapModel(dict(format='amr-console-map-v1',nodes=[dict(id=k,x=x,y=0) for k,x in [('A',0),('B',2),('C',4)]],edges=[['A','B'],['B','C']],walls=[]))
 def test_new_mission_clears_old_marks(self):
  m=self.scene();s=Simulator(m);s.skipped_goals={'B':{'goal':'B'}};s.skip_result={'goal':'B'}
  c=SimpleNamespace(sim=s,map=m,real=False,connected=True,sim_powered=True,studio_arm_safe=True,current_state=s.status,log=Mock(),studio_config={'arm':{}},studio_bridge=Mock())
  r=c.studio_runner=MissionRunner(ConsoleAdapter(c));r.start([dict(type='Path Nav',goal='C')],repeat=0)
  self.assertFalse(s.skipped_goals);self.assertIsNone(s.skip_result)
 def test_successful_pass_through_clears_only_visited_mark(self):
  m=self.scene();s=Simulator(m);s.skipped_goals={'B':{'goal':'B'},'C':{'goal':'C'}};s.navigate('C')
  for _ in range(300):
   s.tick(.1)
   if s.state.last_node=='B':break
  self.assertNotIn('B',s.skipped_goals);self.assertIn('C',s.skipped_goals)
  for _ in range(300):s.tick(.1)
  self.assertFalse(s.skipped_goals);self.assertEqual(s.state.last_node,'C')

 def test_skip_mark_survives_cycle_then_clears_on_next_visit(self):
  import time
  m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=3,y=0),dict(id='C',x=0,y=2)],edges=[['A','B'],['A','C']],walls=[],obstacles=[dict(x=3,y=0,radius=.4,map_fixed=False)]));s=Simulator(m);s.obstacle_policy='auto';s.auto_static_s=.5;s.auto_wait_s=.1;s.reroute_wait_s=.1;s.reroute_attempt_limit=1
  c=SimpleNamespace(sim=s,map=m,real=False,connected=True,sim_powered=True,studio_arm_safe=True,current_state=s.status,log=Mock(),studio_config={'arm':{}},studio_bridge=Mock());r=c.studio_runner=MissionRunner(ConsoleAdapter(c));r.start([dict(type='Path Nav',goal='B'),dict(type='Path Nav',goal='C'),dict(type='Path Nav',goal='A')],repeat=2);now=time.monotonic();removed=False;cleared=False
  for i in range(1200):
   s.tick(.1);r.tick(now+i*.1,.1)
   if r.cycle==1 and not removed:
    self.assertIn('B',s.skipped_goals);m.obstacles=[];removed=True
   if removed and s.state.last_node=='B':cleared='B' not in s.skipped_goals
   if r.status in ('COMPLETED','FAILED'):break
  self.assertTrue(removed and cleared);self.assertEqual(r.status,'COMPLETED',r.error);self.assertEqual(len(r.skipped),1)
