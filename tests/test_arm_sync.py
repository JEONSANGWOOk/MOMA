import math,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
from seer_control.geometry3d import RobotDescription
from seer_control.arm_simulation import ArmSimulator
from seer_control.world3d import WorldView3D
from seer_control.studio_ui import ConsoleAdapter
from seer_control.fairino_api import profile
ROOT=Path(__file__).resolve().parents[1]
class SyncTests(unittest.TestCase):
 def setUp(self):
  self.asset=RobotDescription.load(ROOT/'examples/fairino_fr5.urdf');self.sim=ArmSimulator(self.asset)
  self.sim.q=[math.radians(v) for v in [10,-90,90,-90,-90,0]]
 def view(self,real=False):
  return NS(app=NS(real=real,arm_dev_sim=self.sim),arm_asset=self.asset,positions={'arm':{}},arm_follow=NS(get=lambda:True))
 def console(self):
  cfg=profile();cfg['driver']='fairino';cfg['operations']={'on':dict(method='SetToolDO',id=1,status=1)}
  return NS(real=False,connected=True,sim_powered=True,control_enabled=True,studio_config={'arm':cfg},arm_dev_config=cfg,arm_dev_sim=self.sim,sim=NS(arm={},route=['moving']),current_state=lambda:dict(x=0,y=0,theta=0,speed=.4),_aw_sync_main=Mock(),studio_runner=NS(index=0),log=Mock())
 def test_sim_main_uses_workspace_not_synthetic_or_old_feedback(self):
  v=self.view();values=WorldView3D._arm_positions(v,dict(status='RUNNING',progress=.5),{'joint1':2})
  self.assertEqual(values,self.sim.kin.positions(self.sim.q));self.assertEqual(v.arm_source,'워크스페이스 SIM 동기화')
 def test_real_main_keeps_measured_pose_separate(self):
  v=self.view(True);measured={j['name']:.2 for j in self.asset.movable()}
  self.assertEqual(WorldView3D._arm_positions(v,{},measured),measured)
  self.assertNotEqual(measured,self.sim.kin.positions(self.sim.q))
 def test_mission_io_uses_shared_arm_while_sim_amr_moves(self):
  c=self.console();a=dict(type='Arm Action',operation='on',duration_s=1,timeout_s=10);adapter=ConsoleAdapter(c);adapter.begin(a)
  self.assertIs(adapter.context['shared_arm'],self.sim);self.assertTrue(adapter.poll(a,.02,.02));self.assertEqual(self.sim.io['ToolDO'][1],1)
  self.assertEqual(c.arm_sim_owner,'workspace');c._aw_sync_main.assert_called()
 def test_real_moving_amr_still_blocks_arm_action(self):
  c=self.console();c.real=True
  with self.assertRaisesRegex(ValueError,'정지 확인'):ConsoleAdapter(c).begin(dict(type='Arm Action',operation='on',duration_s=1))
 def test_mission_waits_for_workspace_program_instead_of_overwriting(self):
  c=self.console();self.sim.start(c.arm_dev_config,operation='on');a=dict(type='Arm Action',operation='on',duration_s=1,timeout_s=10);adapter=ConsoleAdapter(c);adapter.begin(a)
  self.assertTrue(adapter.context['arm_wait']);self.assertFalse(adapter.poll(a,.02,.02));self.sim.tick(.02)
  self.assertTrue(adapter.poll(a,.02,.04))
