import copy,time,unittest
from unittest.mock import Mock
from test_fairino import FakeRobot,config
import test_fairino as fixtures
from seer_control.fairino_api import SDKEngine,validate
from seer_control.fairino_programs import template,program_actions,expand_actions,library,validate_programs

class ProgramTests(unittest.TestCase):
 def setUp(self):
  self.cfg=config();self.robot=FakeRobot()
  self.robot.SetDO=Mock(return_value=0);self.robot.SetToolDO=Mock(return_value=0)
  self.robot.GetDI=Mock(return_value=(0,0));self.robot.GetToolDI=Mock(return_value=(0,1))
  self.cfg['operations'].update(open=dict(method='SetToolDO',id=1,status=1),confirm=dict(method='WaitDI',id=3,status=1))
  self.cfg['programs']={'pick':[dict(operation='work'),dict(operation='open'),dict(operation='confirm',timeout_s=2),dict(wait_s=.5),dict(operation='safe_pose')]}
  self.engine=SDKEngine(self.cfg,self.robot)
 def test_templates_are_unresolved_names_not_motion_coordinates(self):
  steps=template('도어 열기');self.assertEqual(steps[0]['operation'],'safe_pose')
  self.assertFalse(any('target' in s for s in steps))
  self.cfg['programs']['door']=steps
  validate(self.cfg)
  with self.assertRaisesRegex(ValueError,'미등록'):program_actions(self.cfg,'door')
 def test_compilation_keeps_step_deadlines_and_wait(self):
  actions=program_actions(self.cfg,'pick')
  self.assertEqual(len(actions),5);self.assertEqual(actions[2]['timeout_s'],2)
  self.assertEqual(actions[3]['type'],'Wait');self.assertEqual(actions[3]['duration_s'],.5)
 def test_branch_destinations_remapped_after_program_expansion(self):
  actions=expand_actions(self.cfg,[dict(type='Arm Action',operation='pick'),dict(type='Branch DI',target=3),dict(type='Path Nav',goal='LM1')])
  self.assertEqual(len(actions),7);self.assertEqual(actions[5]['target'],7)
 def test_draft_validation_rejects_unsafe_structure(self):
  for steps in ([],[dict(operation='pick')],[dict(wait_s=float('nan'))],[dict(operation='work',wait_s=1)],[dict(operation='work',timeout_s=-1)]):
   cfg=copy.deepcopy(self.cfg);cfg['programs']['pick']=steps
   with self.assertRaises(ValueError):validate(cfg)
 def test_io_channel_and_logic_validation(self):
  for method,channel,status in [('SetToolDO',2,1),('WaitDI',16,1),('SetDO',True,1),('SetDO',0,2),('SetDO',0,True)]:
   cfg=copy.deepcopy(self.cfg);cfg['operations']['bad']=dict(method=method,id=channel,status=status)
   with self.assertRaises(ValueError):validate(cfg)
 def test_output_nonblocking_once_does_not_claim_grip_confirmation(self):
  self.engine.execute('open');self.robot.SetToolDO.assert_called_once_with(1,1,smooth=0,block=1)
  self.assertEqual(self.engine.status()['status'],'COMPLETED');self.robot.GetDI.assert_not_called()
 def test_sensor_wait_needs_measured_input(self):
  self.engine.execute('confirm');self.assertEqual(self.engine.status()['status'],'RUNNING')
  self.robot.GetDI.assert_called_with(3,block=1)
  self.robot.GetDI.return_value=(0,1);self.assertEqual(self.engine.status()['status'],'COMPLETED')
 def test_sensor_error_and_emergency_fail_without_output(self):
  self.engine.execute('confirm');self.robot.GetDI.return_value=(12,0)
  with self.assertRaises(RuntimeError):self.engine.status()
  self.robot.emergency=1
  self.assertEqual(self.engine.status()['status'],'ERROR')
  with self.assertRaises(ValueError):self.engine.execute('open')
  self.robot.SetToolDO.assert_not_called()
 def test_stop_and_pause_never_complete_wait(self):
  self.engine.execute('confirm');self.engine.call('pause')
  self.robot.GetDI.return_value=(0,1);self.assertEqual(self.engine.status()['status'],'PAUSED')
  self.engine.call('resume');self.assertEqual(self.engine.status()['status'],'COMPLETED')
  self.engine.call('stop');self.assertEqual(self.engine.status()['status'],'CANCELED')
 def test_library_excludes_connection_and_execution_permission(self):
  out=library(self.cfg)
  self.assertNotIn('ip',out);self.assertNotIn('verified',out);self.assertNotIn('sdk_path',out)
  out['operations']['work']['target'][0]=999
  self.assertEqual(self.cfg['operations']['work']['target'][0],10)
 def test_wait_then_motion_resets_io_state(self):
  self.engine.execute('open');self.engine.execute('work')
  self.assertIsNone(self.engine.io_task);self.assertEqual(self.engine.status()['status'],'RUNNING')

class MissionProgramTests(unittest.TestCase):
 def setup_console(self):
  fixture=fixtures.FairinoTests();fixture.setUp()
  fixture.cfg['operations']['confirm']=dict(method='WaitDI',id=1,status=1)
  fixture.cfg['programs']={'check':[dict(operation='confirm',timeout_s=1),dict(operation='safe_pose')]}
  fixture.robot.GetDI=Mock(return_value=(0,0))
  return fixture,fixture.mission_console()
 def test_input_timeout_stops_and_does_not_execute_next_pose(self):
  f,c=self.setup_console();runner=c.studio_runner;now=time.monotonic()
  runner.start(program_actions(f.cfg,'check'));runner.tick(now,.1);runner.tick(now+2,.1)
  self.assertEqual(runner.status,'FAILED');f.robot.MoveJ.assert_not_called();c._fr5_priority_stop.assert_called_once()
  self.assertEqual(runner.report.data()['records'][-1]['kind'],'실패')
 def test_sensor_confirmation_then_motion_completes_each_step(self):
  f,c=self.setup_console();runner=c.studio_runner;now=time.monotonic()
  runner.start(program_actions(f.cfg,'check'));runner.tick(now,.1)
  self.assertEqual(runner.index,0)
  f.robot.GetDI.return_value=(0,1);runner.tick(now+.6,.1)
  self.assertEqual(runner.index,1);runner.tick(now+.7,.1)
  f.engine.command_time-=1;runner.tick(now+1.4,.1)
  self.assertEqual(runner.status,'COMPLETED');self.assertTrue(c.studio_arm_safe)
 def test_amr_state_loss_during_arm_task_requests_stop(self):
  f,c=self.setup_console();runner=c.studio_runner;now=time.monotonic()
  runner.start(program_actions(f.cfg,'check'));runner.tick(now,.1);c.last_state-=5;runner.tick(now+.1,.1)
  self.assertEqual(runner.status,'FAILED');c._fr5_priority_stop.assert_called_once()
 def test_standalone_fr5_runs_without_seer_connection_or_commands(self):
  from types import SimpleNamespace
  f,c=self.setup_console();c.connected=False;c.control_enabled=False;c.last_state=0;c.fr5_client=SimpleNamespace(connected=True)
  c.studio_runner.start([dict(type='Arm Action',operation='safe_pose',_fr5_standalone=True)])
  now=time.monotonic();c.studio_runner.tick(now,.1);f.engine.command_time-=1;c.studio_runner.tick(now+1,.1)
  self.assertEqual(c.studio_runner.status,'COMPLETED');c.send_command.assert_not_called()
 def test_standalone_cannot_bypass_a_connected_amr(self):
  from types import SimpleNamespace
  f,c=self.setup_console();c.fr5_client=SimpleNamespace(connected=True)
  c.studio_runner.start([dict(type='Arm Action',operation='safe_pose',_fr5_standalone=True)])
  c.studio_runner.tick(time.monotonic(),.1)
  self.assertEqual(c.studio_runner.status,'FAILED');f.robot.MoveJ.assert_not_called()
 def test_pause_blocks_starting_another_sdk_operation(self):
  f,c=self.setup_console();f.engine.execute('confirm');f.engine.call('pause')
  with self.assertRaises(ValueError):f.engine.execute('safe_pose')
  f.robot.MoveJ.assert_not_called()

if __name__=='__main__':unittest.main()
