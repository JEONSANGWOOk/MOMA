import unittest
from seer_control.operator_missions import defaults,compile_mission,validate_template
from seer_control.model import MapModel
class OperatorMissionTests(unittest.TestCase):
 def setUp(self):
  self.map=MapModel({'format':'amr-console-map-v1','nodes':[dict(id='LM1',x=0,y=0)],'edges':[]});self.map.nodes={'LM1':dict(id='LM1',x=0,y=0),'LM2':dict(id='LM2',x=1,y=0),'LM3':dict(id='LM3',x=2,y=0)};self.map.edges=[('LM1','LM2'),('LM2','LM3')];self.arm={'operations':{'pick':{'method':'SetToolDO','id':0,'status':1}},'programs':{}};self.params={'목적지':'LM3','픽업':'LM1','복귀':'LM1','repeat':2};self.pose={'x':0,'y':0}
 def test_move_compiles_existing_path_nav(self):
  p=compile_mission(defaults()[0],self.params,self.map,self.arm,self.pose);self.assertEqual(p['actions'][0]['goal'],'LM3');self.assertEqual(p['route']['nodes'],['LM1','LM2','LM3'])
 def test_supply_default_cannot_execute_unbound_poses(self):
  with self.assertRaises(ValueError):compile_mission(defaults()[2],self.params,self.map,self.arm,self.pose)
 def test_arm_recipe_requires_station_binding(self):
  r=dict(name='공급',enabled=True,destinations=[],blocks=[dict(type='팔 작업',value='pick')])
  with self.assertRaises(ValueError):validate_template(r,self.map.nodes,self.arm)
 def test_taught_action_compiles_without_guessing(self):
  r=dict(name='공급',enabled=True,destinations=['LM3'],blocks=[dict(type='이동',value='목적지'),dict(type='팔 작업',value='pick')]);p=compile_mission(r,self.params,self.map,self.arm,self.pose);self.assertEqual(p['actions'][1]['operation'],'pick')
 def test_disallowed_destination(self):
  r=dict(name='공급',enabled=True,destinations=['LM2'],blocks=[dict(type='팔 작업',value='pick')])
  with self.assertRaises(ValueError):compile_mission(r,self.params,self.map,self.arm,self.pose)
 def test_missing_route_rejected(self):
  self.map.edges=[]
  with self.assertRaises(ValueError):compile_mission(defaults()[0],self.params,self.map,self.arm,self.pose)
 def test_repeat_bounded_integer(self):
  for n in (0,101,True,1.2):
   with self.assertRaises(ValueError):compile_mission(defaults()[0],dict(self.params,repeat=n),self.map,self.arm,self.pose)
 def test_wait_finite_and_bounded(self):
  for v in ('nan',-1,601):
   r=dict(name='대기',enabled=True,destinations=[],blocks=[dict(type='대기',value=v)])
   with self.assertRaises(ValueError):compile_mission(r,self.params,self.map,self.arm,self.pose)
 def test_loop_returns_pickup(self):
  p=compile_mission(defaults()[1],self.params,self.map,self.arm,self.pose);self.assertEqual([a['goal'] for a in p['actions']],['LM1','LM3','LM1'])
 def test_program_expands_and_retains_template_snapshot(self):
  self.arm['programs']['pick_program']=[dict(operation='pick')];r=dict(name='공급',enabled=True,destinations=['LM3'],blocks=[dict(type='팔 작업',value='pick_program')]);p=compile_mission(r,self.params,self.map,self.arm,self.pose);r['blocks'][0]['value']='changed';self.assertEqual(p['actions'][0]['operation'],'pick');self.assertEqual(p['template']['blocks'][0]['value'],'pick_program')

 def test_current_node_roundtrip(self):
  recipe=next(r for r in defaults() if r['name']=='현재 노드 왕복 테스트')
  p=compile_mission(recipe,self.params,self.map,self.arm,self.pose)
  self.assertEqual([a['goal'] for a in p['actions']],['LM3','LM1'])
  self.assertEqual(p['route']['nodes'],['LM1','LM2','LM3','LM2','LM1'])
 def test_current_node_requires_node_and_other_destination(self):
  recipe=next(r for r in defaults() if r['name']=='현재 노드 왕복 테스트')
  for pose,params in [({'x':.5,'y':0},self.params),(self.pose,dict(self.params,목적지='LM1'))]:
   with self.assertRaises(ValueError):compile_mission(recipe,params,self.map,self.arm,pose)
