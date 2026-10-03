import unittest
from types import SimpleNamespace
from seer_control.model import MapModel
from seer_control.dynamic_obstacles import compile_actor_path,update_actors,validate_actor,actor_data
from seer_control.smap import make_path_record

class WaypointTests(unittest.TestCase):
 def model(self):
  return MapModel(dict(format='amr-console-map-v1',nodes=[dict(id=k,x=x,y=y) for k,x,y in [('A',0,0),('B',0,2),('C',2,2),('D',4,2)]],edges=[['A','B'],['B','C'],['C','D']],obstacles=[],walls=[]))
 def actor(self,kind='amr'):
  return dict(dynamic=True,kind=kind,id='ACTOR',x=0,y=0,radius=.2,speed_mps=1,dwell_s=0,motion_path=[[0,0],[2,2],[4,2]],motion_nodes=['A','C','D'])
 def test_amr_uses_intermediate_nodes(self):
  m=self.model();o=self.actor();points,stops=compile_actor_path(m,o)
  self.assertEqual(points[:4],[(0,0),(0,2),(2,2),(4,2)]);self.assertNotIn(1,stops)
  m.obstacles=[o];robot=SimpleNamespace(x=20,y=20)
  for _ in range(10):update_actors(m,.1,robot,.2)
  self.assertAlmostEqual(o['x'],0);self.assertGreater(o['y'],.9)
 def test_people_three_points_and_loop(self):
  m=self.model();o=self.actor('person');o.pop('motion_nodes');o['motion_mode']='loop';validate_actor(o)
  points,_=compile_actor_path(m,o);self.assertEqual(points,[(0,0),(2,2),(4,2),(0,0)])
  m.obstacles=[o]
  for _ in range(40):update_actors(m,.1,SimpleNamespace(x=20,y=20),.2)
  self.assertAlmostEqual(o['y'],2);self.assertGreater(o['x'],2)
 def test_disconnected_nodes_rejected(self):
  m=self.model();m.edges=[]
  with self.assertRaises(ValueError):compile_actor_path(m,self.actor())
 def test_directed_return_must_exist(self):
  m=self.model();m.path_records=[dict(a='A',b='B',raw=make_path_record(m.nodes['A'],m.nodes['B']),properties={}),dict(a='B',b='C',raw=make_path_record(m.nodes['B'],m.nodes['C']),properties={}),dict(a='C',b='D',raw=make_path_record(m.nodes['C'],m.nodes['D']),properties={})]
  with self.assertRaises(ValueError):compile_actor_path(m,self.actor())
 def test_persist_omits_runtime(self):
  o=self.actor();o['_compiled']=compile_actor_path(self.model(),o);o['x']=1
  saved=actor_data(o);self.assertNotIn('_compiled',saved);self.assertEqual(saved['x'],0);self.assertEqual(saved['motion_nodes'],['A','C','D'])

 def test_amr_uses_curve_geometry(self):
  m=self.model();raw=make_path_record(m.nodes['A'],m.nodes['B']);raw['controlPos1']={'x':1,'y':.5};raw['controlPos2']={'x':1,'y':1.5}
  m.path_records=[dict(a='A',b='B',raw=raw,properties={}),dict(a='B',b='A',raw=make_path_record(m.nodes['B'],m.nodes['A']),properties={})]
  o=self.actor();o['motion_nodes']=['A','B'];points,_=compile_actor_path(m,o)
  self.assertGreater(max(p[0] for p in points),.5);self.assertGreater(len(points),5)
