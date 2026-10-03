import unittest
from types import SimpleNamespace
from seer_control.obstacle_tracking import ObstacleTracker
from seer_control.model import MapModel,Simulator

class TrackingTests(unittest.TestCase):
 def test_motion_inferred_without_dynamic_flag(self):
  t=ObstacleTracker();o={'x':0.,'y':0.,'dynamic':False}
  for _ in range(12):o['x']+=.02;t.update([o],.1)
  self.assertEqual(o['_classification'],'dynamic')
 def test_stopped_actor_becomes_static_then_dynamic(self):
  t=ObstacleTracker();o={'x':0.,'y':0.,'dynamic':True}
  for _ in range(25):t.update([o],.1)
  self.assertEqual(o['_classification'],'static')
  for _ in range(15):o['y']+=.03;t.update([o],.1)
  self.assertEqual(o['_classification'],'dynamic')
 def test_new_track_unknown_and_small_jitter_static(self):
  t=ObstacleTracker();o={'x':0.,'y':0.};t.update([o],.1)
  self.assertEqual(o['_classification'],'unknown')
  for i in range(30):o['x']=.003*(-1)**i;t.update([o],.1)
  self.assertEqual(o['_classification'],'static')
 def test_deleted_tracks_removed(self):
  t=ObstacleTracker();t.update([{'x':0,'y':0}],.1);t.update([],.1);self.assertFalse(t.tracks)
 def sim(self,actor=False):
  obs=dict(x=1.5,y=0,radius=.3)
  if actor:obs.update(dynamic=True,kind='person',id='P',motion_path=[[1.5,0],[1.5,2]],paused=True,speed_mps=.5,dwell_s=0)
  m=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id=k,x=x,y=y) for k,x,y in [('A',0,0),('B',3,0),('C',0,2),('D',3,2)]],edges=[['A','B'],['A','C'],['C','D'],['D','B']],walls=[],obstacles=[obs]))
  s=Simulator(m);s.obstacle_policy='auto';s.auto_static_s=.5;s.auto_wait_s=.5;s.reroute_wait_s=.5;s.navigate('B');return s,obs
 def test_auto_static_reroutes_on_graph(self):
  s,o=self.sim();upper=False
  for _ in range(800):s.tick(.1);upper|=s.state.y>1.5
  self.assertTrue(upper);self.assertEqual(s.state.last_node,'B')
 def test_paused_dynamic_actor_classified_static_and_rerouted(self):
  s,o=self.sim(True)
  for _ in range(800):s.tick(.1)
  self.assertEqual(o['_classification'],'static');self.assertEqual(s.state.last_node,'B')
 def test_per_obstacle_stop_override(self):
  s,o=self.sim();o['auto_scenarios']={'static':'stop'}
  for _ in range(50):s.tick(.1)
  self.assertTrue(s._obstacle_latched);self.assertNotEqual(s.state.last_node,'B')
 def test_unknown_wait_does_not_move_through_obstacle(self):
  s,o=self.sim();s.auto_static_s=30
  for _ in range(100):s.tick(.1)
  self.assertEqual(o['_classification'],'unknown');self.assertTrue(s.state.blocked);self.assertFalse(s.skipped_goals)
 def test_static_wait_override_does_not_reroute(self):
  s,o=self.sim();s.auto_scenarios['static']='wait'
  for _ in range(100):s.tick(.1)
  self.assertTrue(s.state.blocked);self.assertAlmostEqual(s.state.y,0)

 def test_dynamic_classification_survives_brief_stop(self):
  t=ObstacleTracker();o={'x':0.,'y':0.}
  for _ in range(15):o['x']+=.03;t.update([o],.1)
  for _ in range(5):t.update([o],.1)
  self.assertEqual(o['_classification'],'dynamic')
 def test_unreachable_goal_skip_then_next_node(self):
  s,o=self.sim();o['x']=3;s.reroute_attempt_limit=2
  for _ in range(300):
   s.tick(.1)
   if s.skipped_goals:break
  self.assertIn('B',s.skipped_goals);self.assertNotEqual(s.state.last_node,'B')
  s.navigate('C')
  for _ in range(600):s.tick(.1)
  self.assertEqual(s.state.last_node,'C')
 def test_wait_then_local_avoidance_selected(self):
  s,o=self.sim();s.auto_scenarios['static']='wait_avoid';offlane=False
  for _ in range(700):s.tick(.1);offlane|=abs(s.state.y)>.3
  self.assertTrue(offlane);self.assertEqual(s.state.last_node,'B')
