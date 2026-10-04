import unittest
from seer_control.model import MapModel,Simulator
from seer_control.obstacle_tracking import ObstacleTracker
from seer_control.navigation_quality import projected_conflict,route_cost,forecast_risk
from seer_control.alternate_routes import alternate_route
from seer_control.avoidance import detour,clear_segment
from seer_control.smap import make_path_record

def scene():
 return MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=4,y=0)],edges=[['A','B']],walls=[],obstacles=[]))
class QualityTests(unittest.TestCase):
 def test_observed_velocity_not_actor_type_drives_forecast(self):
  m=scene();o=dict(id='P',x=1,y=1,radius=.2,map_fixed=False);m.obstacles=[o];t=ObstacleTracker()
  for i in range(9):o['y']=1-i*.1;t.update(m.obstacles,.1)
  vx,vy=o['_observed_velocity'];self.assertAlmostEqual(vx,0);self.assertAlmostEqual(vy,-1)
  for i in range(12):t.update(m.obstacles,.1)
  self.assertEqual(o['_observed_velocity'],(0.,0.))
 def test_crossing_actor_is_predicted_before_entering_lane(self):
  m=scene();o=dict(id='P',x=1,y=1,radius=.2,_observed_velocity=(0.,-.5));m.obstacles=[o]
  hit=projected_conflict(m,(0,0),[(4,0)],.23,.5)
  self.assertIsNotNone(hit);self.assertIs(hit['obstacle'],o);self.assertLess(hit['time'],2.5)
  o['_observed_velocity']=(0.,.5);self.assertIsNone(projected_conflict(m,(0,0),[(4,0)],.23,.5))
 def test_risk_and_turn_costs_distinguish_routes(self):
  m=scene();m.obstacles=[dict(x=2,y=1,radius=.2,_observed_velocity=(0.,-.5))]
  self.assertGreater(forecast_risk(m,(2,0),.23),forecast_risk(m,(2,3),.23))
  m.obstacles=[]
  self.assertGreater(route_cost(m,[(0,0),(1,0),(1,1)],.23),route_cost(m,[(0,0),(2,0)],.23))
 def test_optimized_graph_chooses_faster_longer_route(self):
  m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id=k,x=x,y=y) for k,x,y in [('A',0,0),('B',4,0),('C',0,1),('D',4,1)]],edges=[['A','B'],['A','C'],['C','D'],['D','B']],walls=[]))
  m.path_records=[]
  for a,b,speed in [('A','B',.05),('A','C',.45),('C','D',.45),('D','B',.45)]:
   m.path_records.append(dict(a=a,b=b,raw=make_path_record(m.nodes[a],m.nodes[b]),properties={'maxspeed':speed}))
  plan=alternate_route(m,(0,0),'B',None,None,[],.23,optimize=True)
  self.assertIn('D',plan['nodes']);self.assertEqual(plan['nodes'][-1],'B')
  plain=alternate_route(m,(0,0),'B',None,None,[],.23);self.assertEqual(plain['nodes'][-1],'B');self.assertNotIn('D',plain['nodes'])
 def test_risk_aware_grid_preserves_physical_collision_checks(self):
  m=scene();m.obstacles=[dict(x=2,y=0,radius=.4)]
  points=detour(m,(0,0),(4,0),.23,risk_aware=True)
  self.assertIsNotNone(points)
  self.assertTrue(all(clear_segment(m,a,b,.23) for a,b in zip(points,points[1:])))
 def test_watchdog_recovers_stationary_heading_branch(self):
  m=scene();m.obstacles=[dict(x=2,y=0,radius=.4,map_fixed=False)]
  s=Simulator(m);s.obstacle_policy='reroute';s.navigate('B')
  # Emulate an accumulated standstill without shortening mission timeouts.
  s._progress_pose=(s.state.x,s.state.y,s.state.theta);s._progress_at=-20
  s.tick(.1)
  self.assertIn('정체 탈출',s.avoidance_status);self.assertFalse(s.skipped_goals)
 def test_watchdog_respects_explicit_wait(self):
  s=Simulator(scene());s.obstacle_policy='auto';s.navigate('B');s._auto_block=('P','dynamic','wait')
  s._progress_pose=(s.state.x,s.state.y,s.state.theta);s._progress_at=-20;s.tick(.1)
  self.assertNotIn('정체 탈출',s.avoidance_status)

 def test_crossing_person_triggers_prediction_and_mission_finishes(self):
  from seer_control.studio_core import collision_reason
  m=scene();m.obstacles=[dict(id='P',dynamic=True,kind='person',map_fixed=False,x=1,y=1,radius=.2,motion_path=[[1,1],[1,-1]],speed_mps=.5,dwell_s=.5)]
  s=Simulator(m);s.obstacle_policy='auto';s.auto_scenarios={'dynamic':'wait_recover','static':'wait_recover','unknown':'avoid'};s.auto_wait_s=.5;s.auto_static_s=.5;s.navigate('B');predicted=False
  for i in range(1500):
   s.tick(.1);predicted|='예상 충돌' in s.detected_obstacle
   self.assertFalse(collision_reason(m,s.state.x,s.state.y,.23))
   if not s.route:break
  self.assertTrue(predicted);self.assertEqual(s.state.last_node,'B');self.assertFalse(s.skipped_goals)

 def test_hidden_actor_routine_never_changes_current_navigation_decision(self):
  m=scene()
  forbidden={'motion_path','node_path','speed_mps','_compiled','_goal','_direction','_heading','dwell_s','paused'}
  class ObservedOnly(dict):
   def get(self,key,default=None):
    if key in forbidden:raise AssertionError('Planner read hidden actor routine: '+key)
    return super().get(key,default)
   def __getitem__(self,key):
    if key in forbidden:raise AssertionError('Planner read hidden actor routine: '+key)
    return super().__getitem__(key)
  o=ObservedOnly(id='P',dynamic=True,x=2,y=1,radius=.2,_observed_velocity=(0.,-.5))
  m.obstacles=[o]
  def decision():
   hit=projected_conflict(m,(0,0),[(4,0)],.23,.5)
   return ((hit['time'],hit['distance']) if hit else None,route_cost(m,[(0,0),(4,0)],.23),detour(m,(0,0),(4,0),.23,risk_aware=True))
  baseline=decision()
  o.update(motion_path=[[2,1],[2,-1]],node_path=['A','B'],speed_mps=9,_compiled=([(-99,-99)],{}),_goal=88,_direction=-1,_heading=99,dwell_s=999,paused=True)
  self.assertEqual(decision(),baseline)
  o.update(motion_path=[[2,1],[100,100]],node_path=['B','A'],_goal=0,_direction=1,paused=False)
  self.assertEqual(decision(),baseline)
 def test_unobserved_motion_is_not_guessed_from_scripted_speed_or_heading(self):
  m=scene();m.obstacles=[dict(id='P',dynamic=True,x=1,y=1,radius=.2,motion_path=[[1,1],[1,-1]],speed_mps=10,_heading=-1.57,_goal=1)]
  self.assertIsNone(projected_conflict(m,(0,0),[(4,0)],.23,.5))
  self.assertEqual(forecast_risk(m,(1,0),.23),0.)
