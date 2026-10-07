import unittest,time
from types import SimpleNamespace as NS
from unittest.mock import Mock
from seer_control.real_obstacle import RealObstacleRecovery
class RecoveryTests(unittest.TestCase):
 def setUp(self):
  self.now=time.monotonic()
  self.c=NS(studio_config={'real_obstacle_recovery':True},sim=NS(obstacle_policy='reroute',reroute_wait_s=1,auto_wait_s=2,reroute_attempt_limit=1,skipped_goals={}),studio_runner=NS(report=None),log=Mock(),send_command=Mock(),last_state=self.now,last_laser_rx=self.now,real_laser_points=[(1,1)],real_command_acks={},map=NS(nodes={'LM3':{}}),_station_nav_payload=lambda node:{'id':'LM3'})
  self.r=RealObstacleRecovery(self.c,'LM3');self.s={'blocked':True,'speed':0,'task':'WAITING','task_status':2}
 def cancel(self):
  self.r.poll(self.s,self.now);self.r.poll(self.s,self.now+2)
  self.assertEqual(self.r.phase,'CANCEL')
 def test_cancel_ack_and_fresh_terminal_state_required(self):
  self.cancel();self.r.poll(self.s,self.now+3);self.assertEqual(self.c.send_command.call_count,1)
  self.c.real_command_acks['cancel']=(self.now+2.1,{'ret_code':0});self.c.last_state=self.now+2.2
  self.r.poll(dict(self.s,task='CANCELED',task_status=6),self.now+3)
  self.assertEqual(self.c.send_command.call_args.args[0],'navigate_free')
 def test_cancel_rejection_never_moves(self):
  self.cancel();self.c.real_command_acks['cancel']=(self.now+2.1,{'ret_code':1})
  with self.assertRaises(ValueError):self.r.poll(self.s,self.now+3)
  self.assertEqual(self.c.send_command.call_count,1)
 def test_skip_requires_cancel_complete(self):
  self.r.attempt=1;self.cancel();self.c.real_command_acks['cancel']=(self.now+2.1,{'ret_code':0});self.c.last_state=self.now+2.2
  result=self.r.poll(dict(self.s,task='CANCELED',task_status=6),self.now+3)
  self.assertTrue(result['skip']);self.assertIn('LM3',self.c.sim.skipped_goals)
 def test_wait_does_not_send(self):
  self.c.sim.obstacle_policy='wait';self.r.poll(self.s,self.now);self.r.poll(self.s,self.now+20);self.c.send_command.assert_not_called()
 def test_disabled_does_not_send(self):
  self.c.studio_config['real_obstacle_recovery']=False;self.assertIsNone(self.r.poll(self.s,self.now));self.c.send_command.assert_not_called()
 def test_stale_lidar_does_not_retry(self):
  self.c.last_laser_rx=self.now-10;self.r.poll(self.s,self.now);self.r.poll(self.s,self.now+3);self.c.send_command.assert_not_called()
