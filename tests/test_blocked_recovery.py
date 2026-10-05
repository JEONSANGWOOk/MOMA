import unittest,math
from seer_control.model import MapModel,Simulator
from seer_control.studio_core import collision_reason

def scene(rear=False):
 obstacles=[dict(x=1.6,y=0,radius=.2)]
 if rear:obstacles.append(dict(x=.35,y=0,radius=.22))
 m=MapModel(dict(format='amr-console-map-v1',name='blocked',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=3,y=0)],edges=[['A','B']],walls=[[-1,-1,4,-1],[4,-1,4,1],[4,1,-1,1],[-1,1,-1,-1]],obstacles=obstacles))
 s=Simulator(m);s.state.x=1;s.navigate('B');s.route=['B'];s._segment_start='A';s._reference_waypoints=[(1,0),(3,0)];s.prefer_graph_routes=False;s.obstacle_policy='avoid';s.blocked_recovery.enabled=True;s.state.blocked=True;s._collision_blocked=True
 return s
class BlockedTests(unittest.TestCase):
 def run_recovery(self,rear):
  s=scene(rear);r=s.blocked_recovery;self.assertTrue(r.begin(s,'배치 장애물'));self.assertEqual(r.phase,'BACK')
  minimum=1;events=[]
  for i in range(1600):
   s.tick(.05);minimum=min(minimum,s.state.x);events.append(s.avoidance_status)
   self.assertFalse(collision_reason(s.map,s.state.x,s.state.y,s.map.robot_model['radius']))
   if s.state.last_node=='B':break
  self.assertEqual(s.state.last_node,'B',(r.phase,s.avoidance_status,s.state.x,s.state.y));self.assertLess(minimum,1-.02)
  self.assertEqual(s.map.robot_model['radius'],.23)
  return s,events
 def test_open_rear_uses_short_reverse_without_previous_node(self):
  s,events=self.run_recovery(False);self.assertFalse(any('180' in e for e in events));self.assertTrue(any('제한 후진' in e for e in events))
 def test_priority_two_backs_only_within_rear_clearance(self):
  s,events=self.run_recovery(True);self.assertTrue(any('제한 후진' in e for e in events));self.assertTrue(any('전방 장애물 우회' in e for e in events))
 def test_new_rear_obstacle_stops_backing(self):
  s=scene(True);r=s.blocked_recovery;r.begin(s,'배치 장애물');self.assertEqual(r.phase,'BACK');start=s.state.x
  s.map.obstacles.append(dict(x=.75,y=0,radius=.05));r.step(s,.1)
  self.assertAlmostEqual(s.state.x,start);self.assertTrue(s.state.blocked);self.assertFalse(r.active)
 def test_stop_and_wait_are_not_overridden(self):
  for policy in ('stop','wait'):
   s=scene();s.obstacle_policy=policy;self.assertFalse(s.blocked_recovery.begin(s,'배치 장애물'))
 def test_turn_does_not_sweep_into_close_obstacle(self):
  s=scene();s.map.obstacles[0]['x']=1.5;s.blocked_recovery.begin(s,'배치 장애물');self.assertEqual(s.blocked_recovery.phase,'BACK')
 def test_preview_shows_actual_reverse_segment(self):
  s=scene(True);s.blocked_recovery.begin(s,'배치 장애물');points=s.navigation_points();self.assertEqual(len(points),2);self.assertLess(points[-1][0],points[0][0])
 def test_unavailable_backward_space_is_not_forced(self):
  s=scene(True);s.map.obstacles[1]['x']=.52;self.assertFalse(s.blocked_recovery.begin(s,'배치 장애물'));self.assertFalse(s.blocked_recovery.active)

 def test_tick_uses_current_position_bypass_before_reverse(self):
  for rear in (False,True):
   s=scene(rear);s.state.blocked=True;s._collision_blocked=True;s.block_reason='배치 장애물'
   for _ in range(1600):
    s.tick(.05)
    self.assertFalse(collision_reason(s.map,s.state.x,s.state.y,s.map.robot_model['radius']))
    if s.state.last_node=='B':break
   self.assertEqual(s.state.last_node,'B');self.assertFalse(s.blocked_recovery.events)
   self.assertFalse(s._recovery_narrow)
 def test_recovery_does_not_rotate_to_previous_node(self):
  s=scene();r=s.blocked_recovery;r.begin(s,'배치 장애물');theta=s.state.theta
  self.assertEqual(r.phase,'BACK');r.step(s,.1);self.assertEqual(s.state.theta,theta)
  self.assertGreaterEqual(s.state.x,1-.18)

 def test_short_reverse_plan_rejoins_soon_after_obstacle(self):
  s=scene();s.map.walls=[[-1,-1,10,-1],[10,-1,10,1],[10,1,-1,1],[-1,1,-1,-1]];s.map.nodes['B']['x']=8;s._reference_waypoints=[(1.,0.),(8.,0.)]
  s.blocked_recovery.begin(s,'배치 장애물')
  for _ in range(100):
   s.blocked_recovery.step(s,.1)
   if not s.blocked_recovery.active:break
  self.assertFalse(s.blocked_recovery.active)
  join=next(i for i,p in enumerate(s._waypoints) if p[0]>1.85 and abs(p[1])<1e-8)
  self.assertLess(s._waypoints[join][0],2.4)
  self.assertTrue(all(abs(y)<1e-8 for x,y in s._waypoints[join:]))
  self.assertEqual(s._reference_waypoints,[(1.,0.),(8.,0.)])

 def test_obstacle_detection_alone_does_not_start_reverse(self):
  s=scene();s.state.blocked=False;s._collision_blocked=False
  self.assertFalse(s.blocked_recovery.begin(s,'배치 장애물'))
 def test_blocked_with_clear_local_bypass_does_not_retreat(self):
  s=scene();start=s.state.x;s.tick(.05)
  self.assertFalse(s.blocked_recovery.active);self.assertTrue(s._local_avoidance_active)
  self.assertGreaterEqual(s.state.x,start);self.assertEqual(s._reference_waypoints,[(1,0),(3,0)])
 def test_auto_wait_timer_is_not_skipped_by_recovery(self):
  s=scene();s.obstacle_policy='auto';s._auto_block=('obstacle','static','wait_avoid');s._auto_block_at=s._avoid_time
  self.assertFalse(s.blocked_recovery.allowed(s))

 def test_only_failed_current_position_plan_triggers_bounded_reverse(self):
  from unittest.mock import patch
  s=scene();start=s.state.x;theta=s.state.theta
  with patch.object(s,'_try_local_detour',return_value=False):s.tick(.05)
  self.assertEqual(s.blocked_recovery.phase,'BACK');self.assertLess(s.state.x,start)
  self.assertGreaterEqual(s.state.x,start-.18);self.assertEqual(s.state.theta,theta)
