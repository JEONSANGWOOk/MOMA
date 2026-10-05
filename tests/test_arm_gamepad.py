import math,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
from seer_control.arm_gamepad import sim_jog,sdk_pulse,ButtonEdges,step_group,step_speed
from seer_control.arm_simulation import ArmSimulator
from seer_control.geometry3d import RobotDescription
from seer_control.fairino_api import SDKEngine
from seer_control.gamepad import PadGate,Sample,pov_buttons
from seer_control.gamepad_ui import GamepadMixin
from test_fairino import FakeRobot,config
ROOT=Path(__file__).resolve().parents[1]
def arm():
 s=ArmSimulator(RobotDescription.load(ROOT/'examples/fairino_fr5.urdf'));s.set_joints([math.radians(x) for x in (0,-90,90,-90,-90,0)]);return s
class Value:
 def __init__(self,v):self.v=v
 def get(self):return self.v
 def set(self,v):self.v=v
class Harness(GamepadMixin):
 def __init__(self):
  self.pad_enabled=Value(True);self.manual=Value(True);self.pad_arm_real=Value(False);self.pad_arm_group=Value('J1 / J2');self.pad_target=Value('로봇팔')
  self.real=False;self.sim_powered=True;self.reloc_mode=False;self.task_running=False;self.studio_runner=SimpleNamespace(active=False)
  self.operation_page='map';self.arm_workspace_page='arm';self.tabs=SimpleNamespace(select=lambda:'arm');self.focus_get=lambda:self;self.winfo_toplevel=lambda:self
  self.sim=SimpleNamespace(route=[],state=SimpleNamespace(stopped=False,motor=True));self.arm_dev_sim=arm();self.pad_gate=PadGate()
  self.pad_config=dict(forward='Y',turn='X',deadman=4,stop=2,zone=.15,invert_forward=True,invert_turn=True)
  self.pad_arm_time=None;self.pad_arm_active=False;self.held=None;self.pad_device=None;self.pad_status=Mock();self.arm_dev_canvas=Mock();self._aw_sync_main=Mock();self.action=Mock();self.release_drive=Mock()
 def sample(self,now,v=0,w=0,held=False):return Sample(dict(X=w,Y=-v),16 if held else 0,now,('sony',))
class ArmPadTests(unittest.TestCase):
 def test_shared_arm_joints_and_bounded_dt(self):
  s=arm();before=list(s.q);sim_jog(s,'J1 / J2',1,1,10)
  self.assertAlmostEqual(s.q[0]-before[0],math.radians(1.2));self.assertAlmostEqual(s.q[1]-before[1],math.radians(1.2));self.assertEqual(s.q[2:],before[2:])
 def test_collision_stops_without_mutating_pose(self):
  s=arm();before=list(s.q);s.physics.check_motion=Mock(side_effect=ValueError('충돌'))
  with self.assertRaisesRegex(ValueError,'충돌'):sim_jog(s,'J1 / J2',1,0,.05)
  self.assertEqual(s.q,before)
 def test_program_cannot_be_overridden(self):
  s=arm();s.state='RUNNING'
  with self.assertRaises(ValueError):sim_jog(s,'J1 / J2',1,1,.05)
 def test_tcp_uses_ik_then_collision_check(self):
  s=arm();before=list(s.q);s.kin.ik=Mock(return_value=before);pose=s.kin.pose(before);sim_jog(s,'TCP Z / RZ',1,1,.1)
  actual=s.kin.ik.call_args.args[0];self.assertAlmostEqual(actual[2]-pose[2],3);self.assertAlmostEqual(actual[5]-pose[5],1)
 def test_single_axis_real_pulse_is_small_and_expiring(self):
  p=sdk_pulse('TCP X / Y',.5,1,10);self.assertEqual((p['ref'],p['axis'],p['direction']),(2,1,1));self.assertEqual(p['distance'],.3);self.assertEqual(p['expires'],10.15)
 def test_ui_release_focus_and_mode_change_stop(self):
  h=Harness();h._pad_process(h.sample(10),10);h._pad_process(h.sample(10.05,1,1,True),10.05);q=list(h.arm_dev_sim.q);h._aw_sync_main.assert_called()
  h._pad_process(h.sample(10.1,1,1,False),10.1);self.assertEqual(h.arm_dev_sim.q,q);self.assertFalse(h.pad_arm_active)
  h.focus_get=lambda:None;h._pad_process(h.sample(10.15,1,1,True),10.15);self.assertEqual(h.arm_dev_sim.q,q)
  h._pad_toggle();self.assertFalse(h.pad_gate.armed)
 def test_sdk_jog_signature_and_final_stop(self):
  r=FakeRobot();r.StartJOG=Mock(return_value=0);r.ImmStopJOG=Mock(return_value=0);e=SDKEngine(config(),r)
  with patch('seer_control.fairino_api.time.monotonic',return_value=10),patch('seer_control.fairino_api.time.sleep'):
   e.call('jog',sdk_pulse('J3 / J4',1,0,10))
  r.StartJOG.assert_called_once_with(0,4,1,.2,vel=5,acc=20.);r.ImmStopJOG.assert_called_once()
 def test_stale_or_unsafe_sdk_pulse_never_moves(self):
  for stale,emergency in ((True,0),(False,1)):
   r=FakeRobot();r.emergency=emergency;r.StartJOG=Mock();r.ImmStopJOG=Mock();e=SDKEngine(config(),r)
   with patch('seer_control.fairino_api.time.monotonic',return_value=11 if stale else 10):
    with self.assertRaises(ValueError):e.call('jog',sdk_pulse('J1 / J2',1,0,10))
   r.StartJOG.assert_not_called()
 def test_sdk_rejection_still_attempts_stop(self):
  r=FakeRobot();r.StartJOG=Mock(return_value=14);r.ImmStopJOG=Mock(return_value=0);e=SDKEngine(config(),r)
  with patch('seer_control.fairino_api.time.monotonic',return_value=10):
   with self.assertRaises(RuntimeError):e.call('jog',sdk_pulse('J1 / J2',1,0,10))
  r.ImmStopJOG.assert_called_once()

 def test_real_ui_requires_explicit_enable_fresh_stationary_state(self):
  for failure in ('disabled','stale','moving','unknown_speed','error',None):
   h=Harness();h.real=True;h.pad_arm_real.set(failure!='disabled');h.connected=True;h.control_enabled=True
   h.fr5_client=Mock(connected=True);h.fr5_feedback=dict(status='ERROR' if failure=='error' else 'IDLE',motion_done=1,emergency=False,safety_stop=[0,0],errors=[0,0]);h.fr5_rx=7 if failure=='stale' else 10
   h.live=dict(speed=.1 if failure=='moving' else 0);h.current_state=lambda:dict(speed=None if failure=='unknown_speed' else h.live['speed']);h.last_state=10;h.studio_config={'arm':config()};h.fr5_pending=False;h._fr5_queue=Mock();h._arm_call=Mock()
   before=list(h.arm_dev_sim.q);h._pad_process(h.sample(10),10);h._pad_process(h.sample(10.05,1,0,True),10.05)
   self.assertEqual(h.arm_dev_sim.q,before);h._arm_call.assert_not_called()
   if failure:h._fr5_queue.assert_not_called()
   else:h._fr5_queue.assert_called_once()

 def test_mode_wrap_and_speed_clamp(self):
  self.assertEqual(step_group('J1 / J2',-1),'TCP RX / RY')
  self.assertEqual(step_speed(100,1),100);self.assertEqual(step_speed(10,-1),10)
 def test_edges_no_repeat_stale_or_hotplug(self):
  edges=ButtonEdges();sample=Sample(dict(X=0,Y=0),32,10,('sony',))
  self.assertEqual(edges.update(sample,10,True),0)
  sample.buttons=0;edges.update(sample,10,True);sample.buttons=32
  self.assertEqual(edges.update(sample,10,True),32);self.assertEqual(edges.update(sample,10,True),0)
  self.assertEqual(edges.update(sample,11,True),0);self.assertEqual(edges.update(sample,10,True),0)
 def test_mode_button_rearms_before_motion(self):
  h=Harness();h.pad_arm_speed=Value(50);h._pad_process(h.sample(10),10)
  s=h.sample(10.05,1,1,True);s.buttons|=32;before=list(h.arm_dev_sim.q);h._pad_process(s,10.05)
  self.assertEqual(h.pad_arm_group.get(),'J3 / J4');self.assertEqual(h.arm_dev_sim.q,before)
  h._pad_process(s,10.1);self.assertEqual(h.pad_arm_group.get(),'J3 / J4')
 def test_speed_scales_real_distance_and_velocity(self):
  p=sdk_pulse('J1 / J2',1,0,10,.1);self.assertAlmostEqual(p['distance'],.02);self.assertAlmostEqual(p['vel'],.5)
 def test_sim_gripper_requires_neutral_deadman_and_edge(self):
  h=Harness();h.pad_arm_speed=Value(50);h.pad_tool_message='';h.arm_dev_sim.physics.configure(dict(kind='finger'));h._aps_io=Mock()
  h._pad_process(h.sample(10),10);s=h.sample(10.05,held=True);s.buttons|=8;h._pad_process(s,10.05)
  h._aps_io.assert_called_once_with(1);h._pad_process(s,10.1);h._aps_io.assert_called_once()
  h._aps_io.reset_mock();s.buttons=0;h._pad_process(s,10.15);s.buttons=8;h._pad_process(s,10.2);h._aps_io.assert_not_called()
 def test_stale_real_gripper_never_sets_output(self):
  r=FakeRobot();r.SetToolDO=Mock(return_value=0);cfg=config();cfg['operations']['grip']={'method':'SetToolDO','id':0,'status':1};e=SDKEngine(cfg,r)
  with patch('seer_control.fairino_api.time.monotonic',return_value=11):
   with self.assertRaises(ValueError):e.call('pad_io',dict(operation='grip',expires=10.15))
  r.SetToolDO.assert_not_called()

 def test_pov_arrows_and_diagonals(self):
  self.assertEqual(pov_buttons(0),1<<32);self.assertEqual(pov_buttons(18000),1<<34)
  self.assertEqual(pov_buttons(4500),(1<<32)|(1<<33));self.assertEqual(pov_buttons(65535),0)
 def test_dpad_speed_edge(self):
  h=Harness();h.pad_arm_speed=Value(50);h._pad_process(h.sample(10),10)
  s=h.sample(10.05);s.buttons=pov_buttons(0);h._pad_process(s,10.05);self.assertEqual(h.pad_arm_speed.get(),75)
  h._pad_process(s,10.1);self.assertEqual(h.pad_arm_speed.get(),75)
  h._pad_process(h.sample(10.15),10.15);s.timestamp=10.2;s.buttons=pov_buttons(18000);h._pad_process(s,10.2);self.assertEqual(h.pad_arm_speed.get(),50)
