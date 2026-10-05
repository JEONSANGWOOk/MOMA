import unittest
from seer_control.model import MapModel,Simulator
from seer_control.avoidance import rejoin_detour,clear_segment

def scene():
 return MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=10,y=0)],edges=[['A','B']],walls=[],obstacles=[dict(x=2,y=0,radius=.4,map_fixed=False)]))
class RejoinTests(unittest.TestCase):
 def test_first_safe_rejoin_then_straight_tail(self):
  m=scene();points=rejoin_detour(m,(0,0),[(0,0),(10,0)],.23,risk_aware=True)
  self.assertIsNotNone(points)
  join=next(i for i,p in enumerate(points) if p[0]>2.7 and abs(p[1])<1e-8)
  self.assertLess(points[join][0],3.1)
  self.assertTrue(all(abs(y)<1e-8 for x,y in points[join:]))
  self.assertTrue(all(clear_segment(m,a,b,.23) for a,b in zip(points,points[1:])))
 def test_off_lane_after_blocker_merges_at_nearest_projection(self):
  m=scene();points=rejoin_detour(m,(4,1),[(0,0),(10,0)],.23,risk_aware=True)
  self.assertEqual(points[1],(4.,0.));self.assertEqual(points[-1],(10,0))
  self.assertTrue(all(abs(y)<1e-8 for x,y in points[1:]))
 def test_recovery_keeps_original_line_for_subsequent_replans(self):
  s=Simulator(scene());s.obstacle_policy='reroute';s.navigate('B')
  s._reference_waypoints=[(0,0),(10,0)];original=list(s._reference_waypoints)
  self.assertTrue(s.try_recovery_detour());self.assertEqual(s._reference_waypoints,original)
  s.state.x=4;s.state.y=1
  self.assertTrue(s.try_recovery_detour());self.assertEqual(s._waypoints[0],(4.,0.))
  self.assertEqual(s._reference_waypoints,original)
 def test_stale_reference_cannot_rejoin_wrong_node(self):
  s=Simulator(scene());s.obstacle_policy='avoid';s.navigate('B');s._reference_waypoints=[(0,0),(4,0)]
  self.assertTrue(s._try_local_detour(s.map.nodes['B'],.23))
  self.assertEqual(s._waypoints[-1],(10,0))

 def test_moving_away_obstacle_allows_immediate_short_connector(self):
  from seer_control.avoidance import nearest_clear_rejoin
  m=scene();self.assertIsNone(nearest_clear_rejoin(m,(2,1),[(0,0),(10,0)],.23))
  m.obstacles[0]['y']=-2
  points=nearest_clear_rejoin(m,(2,1),[(0,0),(10,0)],.23)
  self.assertEqual(points,[(2,1),(2.,0.),(10,0)])
 def test_runtime_merge_replaces_long_remaining_detour(self):
  s=Simulator(scene());s.obstacle_policy='avoid';s.navigate('B')
  s.state.x=4;s.state.y=1;s._reference_waypoints=[(0,0),(10,0)]
  s._waypoints=[(6,1),(7,0),(10,0)];s._local_avoidance_active=True
  s.tick(.1)
  self.assertEqual(s._waypoints[0],(4.,0.));self.assertEqual(s._waypoints[-1],(10,0))
  self.assertIn('최단 연결',s.avoidance_status)

 def test_line_priority_preserves_nominal_reference_with_graph_scan(self):
  s=Simulator(scene());s.obstacle_policy='auto';s.prefer_graph_routes=True;s.prefer_line_rejoin=True
  s.auto_scenarios={k:'avoid' for k in s.auto_scenarios};s.navigate('B');s.tick(.1)
  self.assertTrue(s._local_avoidance_active)
  self.assertEqual(s._reference_waypoints,[(0.,0.),(10.,0.)])
  join=next(i for i,p in enumerate(s._waypoints) if p[0]>2.7 and abs(p[1])<1e-8)
  self.assertLess(s._waypoints[join][0],3.1)
  self.assertTrue(all(abs(y)<1e-8 for x,y in s._waypoints[join:]))
 def test_line_priority_reroute_rejoins_then_runs_straight(self):
  s=Simulator(scene());s.obstacle_policy='reroute';s.prefer_graph_routes=False;s.prefer_line_rejoin=True;s.reroute_wait_s=0;s.navigate('B')
  seen=False
  for _ in range(1800):
   s.tick(.05)
   seen=seen or s._local_avoidance_active
   if s.state.x>3.1:self.assertAlmostEqual(s.state.y,0,places=5)
   if s.state.last_node=='B':break
  self.assertTrue(seen);self.assertEqual(s.state.last_node,'B')
 def test_line_priority_does_not_override_explicit_wait(self):
  s=Simulator(scene());s.obstacle_policy='auto';s.prefer_graph_routes=True;s.prefer_line_rejoin=True
  s.auto_scenarios={k:'wait' for k in s.auto_scenarios};s.navigate('B')
  for _ in range(200):s.tick(.1)
  self.assertTrue(s.state.blocked);self.assertFalse(s._local_avoidance_active)
