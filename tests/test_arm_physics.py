import unittest,math,copy
from types import SimpleNamespace
from seer_control.arm_physics import ArmPhysics,obb,separation,inverse
from seer_control.geometry3d import box,transform,point,multiply
class FakeAsset:
 links={'tool':[(box((.02,.02,.02)),transform(),'#ffffff')]};joints=[]
 def link_transforms(self,positions):return {'tool':transform((positions['x'],0,positions['z']),(math.pi,0,0))}
class Kin:
 asset=FakeAsset();tip='tool'
 def positions(self,q):return dict(x=q[0],z=q[1])
 def fk(self,q):return self.asset.link_transforms(self.positions(q))['tool']
class PhysicsTests(unittest.TestCase):
 def setUp(self):
  self.p=ArmPhysics(Kin());self.io={k:{} for k in ('DO','DI','ToolDO','ToolDI')};self.q=[0,.8]
 def add(self,kind='finger',mass=.2,sealable=True):
  self.p.configure(dict(kind=kind));size=[.04,.04,.06] if kind=='finger' else [.12,.12,.03];offset=.095 if kind=='finger' else .14
  o=self.p.add_object('work','box' if kind=='finger' else 'panel',size,mass,[0,0,.8-offset,0,0,0],sealable=sealable)
  bottom=o['matrix'][2][3]-size[2]/2
  self.p.add_object('table','obstacle',[.16,.16,.04],10,[0,0,bottom-.02,0,0,0],True)
  return o
 def advance(self,n=100):
  for _ in range(n):self.p.advance(self.q,self.io,.01)
 def test_finger_io_closes_confirms_and_releases(self):
  obj=self.add();self.io['ToolDO'][0]=1;self.advance()
  self.assertIs(self.p.held,obj);self.assertEqual(self.io['ToolDI'][0],1);self.assertAlmostEqual(self.p.opening,.04,places=3)
  for _ in range(100):self.q[1]+=.0001;self.p.advance(self.q,self.io,.01)
  self.assertGreater(obj['matrix'][2][3],.71)
  self.io['ToolDO'][0]=0;self.advance();self.assertIsNone(self.p.held);self.assertEqual(self.io['ToolDI'][0],0)
 def test_force_friction_and_mass_can_prevent_grasp(self):
  self.add(mass=4);self.p.configure(dict(force_n=1));self.io['ToolDO'][0]=1;self.advance()
  self.assertIsNone(self.p.held);self.assertTrue(any('초과' in e['event'] for e in self.p.events))
 def test_vacuum_pressure_force_and_tool_payload(self):
  self.add('vacuum');self.io['ToolDO'][0]=1;self.advance(150)
  self.assertIsNotNone(self.p.held);self.assertGreater(self.p.pressure,40000)
  self.assertAlmostEqual(self.p.force,self.p.pressure*math.pi*(.04/2)**2)
  self.p.configure(dict(payload_limit_kg=.5));self.advance(1);self.assertIsNone(self.p.held)
 def test_non_sealable_surface_does_not_generate_vacuum(self):
  self.add('vacuum',sealable=False);self.io['ToolDO'][0]=1;self.advance()
  self.assertIsNone(self.p.held);self.assertEqual(self.p.pressure,0)
 def test_unconfigured_gripper_preserves_manual_di(self):
  self.io['ToolDI'][0]=1;self.advance(1);self.assertEqual(self.io['ToolDI'][0],1)
 def test_collision_sweeps_between_endpoints_without_moving_robot(self):
  self.p.add_object('wall','obstacle',[.03,.1,.1],10,[.1,0,.8,0,0,0],True)
  with self.assertRaisesRegex(ValueError,'충돌 예상'):self.p.check_motion([0,.8],[.2,.8])
  self.assertEqual(self.q,[0,.8]);self.assertTrue(self.p.blocked)
  self.p.advance(self.q,self.io,.01);self.assertTrue(self.p.risk)
 def test_near_collision_and_future_warning(self):
  self.p.add_object('wall','obstacle',[.02,.1,.1],10,[.04,0,.8,0,0,0],True)
  risks,near=self.p.contacts(self.q);self.assertFalse(risks);self.assertTrue(near)
  self.p.forecast([.04,.8]);self.assertTrue(self.p.future_risk)
 def test_scene_restore_and_invalid_settings(self):
  self.add();other=ArmPhysics(Kin());other.restore(self.p.snapshot());self.assertEqual(len(other.objects),2)
  with self.assertRaises(ValueError):other.configure(dict(vacuum_kpa=math.nan))
  with self.assertRaises(ValueError):other.configure(dict(seal_efficiency=2))
 def test_gravity_settles_workpiece_on_floor(self):
  o=self.p.add_object('fall','box',[.1,.1,.1],1,[0,0,.2,0,0,0]);self.advance()
  self.assertAlmostEqual(o['matrix'][2][3],.05);self.assertEqual(o['state'],'안착')
 def test_acceleration_breaks_grip(self):
  self.add();self.io['ToolDO'][0]=1;self.advance();self.assertIsNotNone(self.p.held)
  self.q[0]=.2;self.p.advance(self.q,self.io,.01);self.assertIsNone(self.p.held)
 def test_rotated_obb_and_inverse(self):
  t=transform((1,2,3),(.2,.3,.4));self.assertLess(math.dist(point(multiply(inverse(t),t),(0,0,0)),(0,0,0)),1e-10)
  self.assertGreater(separation(obb(transform(),(.2,.2,.2)),obb(transform((1,0,0),(0,0,.5)),(.2,.2,.2))),0)

 def test_ground_collision_blocks_arm_below_floor(self):
  with self.assertRaisesRegex(ValueError,'바닥'):self.p.check_motion([0,.1],[0,-.02])
 def test_custom_io_mapping(self):
  self.add();self.p.configure(dict(do_bank='DO',do_channel=3,di_bank='DI',di_channel=4));self.io['DO'][3]=1;self.io['ToolDI'][0]=1;self.advance()
  self.assertEqual(self.io['DI'][4],1);self.assertEqual(self.io['ToolDI'][0],1)
 def test_vacuum_off_releases_after_pressure_decay(self):
  self.add('vacuum');self.io['ToolDO'][0]=1;self.advance(150);self.assertIsNotNone(self.p.held)
  pressure=self.p.pressure;self.io['ToolDO'][0]=0;self.advance(1)
  self.assertIsNotNone(self.p.held);self.assertLess(self.p.pressure,pressure)
  self.advance(100);self.assertIsNone(self.p.held);self.assertEqual(self.io['ToolDI'][0],0)
