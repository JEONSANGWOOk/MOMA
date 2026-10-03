import unittest
from types import SimpleNamespace
from seer_control.obstacle_tracking import ObstacleTracker
from seer_control.dynamic_obstacles import actor_data

class NewTrackingTests(unittest.TestCase):
 def scene(self,obstacles=None,walls=None):return SimpleNamespace(obstacles=obstacles or [],walls=walls or [],virtual_walls=[])
 def step(self,t,m,n=1):
  for _ in range(n):t.observe(m,SimpleNamespace(x=0,y=0),.1);t.update(m.obstacles,.1)
 def obs(self,x=2,y=0):return dict(x=x,y=y,radius=.2,map_fixed=False)
 def test_existing_fixed_map_excluded(self):
  o=dict(x=2,y=0,radius=.2);t=ObstacleTracker();t.register_map([o]);m=self.scene([o]);self.step(t,m,30)
  self.assertFalse(t.records());self.assertEqual(o['_classification'],'static')
 def test_new_stationary_marked(self):
  o=self.obs();t=ObstacleTracker();m=self.scene([o]);self.step(t,m,30)
  self.assertEqual(o['_track_id'],'TRK001');self.assertEqual(t.records()[0]['kind'],'static')
 def test_id_survives_dynamic_to_static(self):
  o=self.obs();t=ObstacleTracker();m=self.scene([o]);key=None
  for _ in range(15):o['y']+=.03;self.step(t,m);key=key or o['_track_id'];self.assertEqual(o['_track_id'],key)
  self.assertEqual(t.records()[0]['kind'],'dynamic');self.step(t,m,35)
  self.assertEqual(t.records()[0]['kind'],'static');self.assertEqual(t.records()[0]['id'],key);self.assertGreater(len(t.records()[0]['trail']),3)
 def test_lost_and_expired(self):
  t=ObstacleTracker();m=self.scene([self.obs()]);self.step(t,m);m.obstacles=[];self.step(t,m,12)
  self.assertEqual(t.records()[0]['state'],'미관측');self.step(t,m,310);self.assertFalse(t.records())
 def test_reobserved_source_keeps_id(self):
  o=self.obs();t=ObstacleTracker();m=self.scene([o]);self.step(t,m);key=o['_track_id'];m.obstacles=[];self.step(t,m,15)
  m.obstacles=[o];self.step(t,m);self.assertEqual(o['_track_id'],key);self.assertEqual(t.records()[0]['state'],'추적 중')
 def test_new_dict_nearby_associated(self):
  t=ObstacleTracker();m=self.scene([self.obs()]);self.step(t,m);key=m.obstacles[0]['_track_id']
  m.obstacles=[self.obs(2.03)];self.step(t,m);self.assertEqual(m.obstacles[0]['_track_id'],key)
 def test_two_objects_have_unique_ids(self):
  t=ObstacleTracker();m=self.scene([self.obs(2,-1),self.obs(2,1)]);self.step(t,m)
  self.assertEqual(len(t.records()),2);self.assertNotEqual(m.obstacles[0]['_track_id'],m.obstacles[1]['_track_id'])
 def test_range_and_wall_occlusion(self):
  t=ObstacleTracker();m=self.scene([self.obs(9,0),self.obs(2,0)],[[1,-2,1,2]]);self.step(t,m)
  self.assertFalse(t.records());m.walls=[];self.step(t,m);self.assertEqual(len(t.records()),1)
 def test_runtime_ids_not_saved(self):
  o=self.obs();t=ObstacleTracker();self.step(t,self.scene([o]));saved=actor_data(o)
  self.assertNotIn('_track_id',saved);self.assertFalse(saved['map_fixed'])
