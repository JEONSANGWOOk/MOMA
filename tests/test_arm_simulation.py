import copy,math,unittest
from pathlib import Path
from seer_control.geometry3d import RobotDescription
from seer_control.arm_simulation import ArmSimulator,ArmKinematics,rotation_error
from seer_control.fairino_api import profile
ROOT=Path(__file__).resolve().parents[1]
class ArmTests(unittest.TestCase):
 def setUp(self):
  self.asset=RobotDescription.load(ROOT/'examples/fairino_fr5.urdf');self.sim=ArmSimulator(self.asset)
  self.start=[math.radians(v) for v in [0,-90,90,-90,-90,0]];self.sim.set_joints(self.start)
  self.cfg=profile();self.cfg['operations']={'pose':dict(method='MoveJ',target=[30,-100,100,-90,-70,20],vel=30)}
 def test_independent_sim_does_not_require_real_permissions_or_amr(self):
  self.sim.start(self.cfg,operation='pose')
  for _ in range(100):self.sim.tick(.05)
  self.assertEqual(self.sim.state,'COMPLETED')
  self.assertEqual([round(math.degrees(v)) for v in self.sim.q],self.cfg['operations']['pose']['target'])
  self.assertGreater(len(self.sim.trail),2)
 def test_ik_recovers_nearby_pose_with_orientation(self):
  kin=self.sim.kin;expected=[a+b for a,b in zip(self.start,[.08,.03,-.04,.02,.03,-.01])]
  pose=kin.pose(expected);q=kin.ik(pose,self.start)
  self.assertLess(math.dist(kin.pose(q)[:3],pose[:3]),.8)
  self.assertLess(math.sqrt(sum(v*v for v in rotation_error(kin.fk(expected),kin.fk(q)))),.004)
 def test_unreachable_ik_fails_without_changing_joints(self):
  with self.assertRaises(ValueError):self.sim.kin.ik([100000,100000,100000,0,0,0],self.start,iterations=25)
  self.assertEqual(self.sim.q,self.start)
 def test_movel_tracks_straight_tcp_and_arrives(self):
  expected=[a+b for a,b in zip(self.start,[0,.03,-.02,0,0,0])];target=self.sim.kin.pose(expected);source=self.sim.kin.pose(self.start)
  self.cfg['operations']['line']=dict(method='MoveL',target=target,vel=20)
  self.sim.start(self.cfg,operation='line')
  for _ in range(100):self.sim.tick(.025)
  self.assertEqual(self.sim.state,'COMPLETED',self.sim.error);self.assertLess(math.dist(self.sim.kin.pose(self.sim.q)[:3],target[:3]),1.)
  from seer_control.studio_core import point_segment_distance
  # Distance to the full 3D line, independent of render path smoothing.
  a=[v/1000 for v in source[:3]];b=[v/1000 for v in target[:3]];d=[y-x for x,y in zip(a,b)];norm=sum(v*v for v in d)
  for p in self.sim.trail:
   t=max(0,min(1,sum((p[i]-a[i])*d[i] for i in range(3))/norm));self.assertLess(math.dist(p,[a[i]+t*d[i] for i in range(3)]),.0015)
 def test_invalid_movel_does_not_begin_partial_move(self):
  self.cfg['operations']['bad']=dict(method='MoveL',target=[100000,0,0,0,0,0],vel=20)
  self.sim.start(self.cfg,operation='bad');self.sim.tick(.1)
  self.assertEqual(self.sim.state,'FAILED');self.assertEqual(self.sim.q,self.start)
 def test_input_wait_and_output_do_not_auto_confirm_grip(self):
  self.cfg['operations'].update(close=dict(method='SetToolDO',id=0,status=1),confirm=dict(method='WaitToolDI',id=1,status=1))
  self.cfg['programs']={'pick':[dict(operation='close'),dict(operation='confirm',timeout_s=2),dict(operation='pose')]}
  self.sim.start(self.cfg,program='pick');self.sim.tick(.1);self.sim.tick(.1)
  self.assertEqual(self.sim.index,1);self.assertEqual(self.sim.io['ToolDO'][0],1);self.assertEqual(self.sim.q,self.start)
  self.sim.io['ToolDI'][1]=1;self.sim.tick(.1);self.assertEqual(self.sim.index,2)
 def test_missing_references_are_rejected_before_start(self):
  self.cfg['programs']={'bad':[dict(operation='missing')]}
  with self.assertRaises(ValueError):self.sim.start(self.cfg,program='bad')
  self.assertEqual(self.sim.state,'IDLE')
 def test_timeout_does_not_run_following_move(self):
  self.cfg['operations']['confirm']=dict(method='WaitDI',id=0,status=1)
  self.cfg['programs']={'pick':[dict(operation='confirm',timeout_s=.2),dict(operation='pose')]}
  self.sim.start(self.cfg,program='pick');self.sim.tick(.3)
  self.assertEqual(self.sim.state,'FAILED');self.assertEqual(self.sim.q,self.start)
 def test_pause_stop_and_joint_limits(self):
  self.sim.start(self.cfg,operation='pose');self.sim.tick(.1);q=list(self.sim.q);self.sim.state='PAUSED';self.sim.tick(10);self.assertEqual(self.sim.q,q)
  self.sim.stop();self.sim.tick(10);self.assertEqual(self.sim.q,q)
  with self.assertRaises(ValueError):self.sim.set_joints([100]*6)
 def test_nonzero_tool_frame_is_not_silently_simulated(self):
  self.cfg['operations']['pose']['tool']=1;self.sim.start(self.cfg,operation='pose');self.sim.tick(.1)
  self.assertEqual(self.sim.state,'FAILED');self.assertEqual(self.sim.q,self.start)
if __name__=='__main__':unittest.main()
