import copy,math,time,unittest
from pathlib import Path
from unittest.mock import Mock
from types import SimpleNamespace
from tests.test_key_panel_real import calibrated,frame
from seer_control.key_panel_twin import KeyTwin,KeyPanelLink,metres
from seer_control.geometry3d import RobotDescription,transform,multiply
from seer_control.aruco_board import initialize_panel_arm,pose_from_matrix
from seer_control.key_panel_real import pose_matrix


class FakeClient:
    def __init__(self):
        self.connected=True;self.config={};self.calls=[]
        self.feedback=dict(tcp_mm_deg=[5,0,625,180,0,0],joints_rad=[0]*6,tool=1,user=0,
            status='IDLE',motion_done=1,telemetry_verified=True,force_torque=[0]*6)
    def call(self,c,kind,operation=None):
        self.calls.append((kind,copy.deepcopy(operation)))
        return copy.deepcopy(self.feedback) if kind=='panel_status' else dict(accepted=True)
    def stop(self):self.connected=False;self.calls.append(('stop',None));return dict(status='STOPPED')
    def close(self):self.connected=False


class CoordinationTests(unittest.TestCase):
    def setUp(self):
        self.client=FakeClient();self.twin=SimpleNamespace(fit=Mock(),predict=Mock(),comparison=Mock(return_value={}),socket=None,c=None)
        self.link=KeyPanelLink(self.twin,client=self.client);self.link.cal=calibrated()
        self.link.feedback=copy.deepcopy(self.client.feedback);self.link.rx=time.monotonic()
        self.record=frame(time.time())

    def drain(self):
        deadline=time.monotonic()+1
        while self.link.events.empty() and time.monotonic()<deadline:time.sleep(.005)
        self.link.tick(self.record)

    def test_sim_mode_reads_only_and_cannot_start_physical_plan(self):
        with self.assertRaises(ValueError):self.link.start(self.record)
        self.link.tick(self.record);self.drain()
        self.assertFalse(any(k=='panel_step' for k,_ in self.client.calls));self.twin.predict.assert_not_called()

    def test_both_sends_exact_predicted_target(self):
        self.link.set_mode('SIM+REAL');self.link.start(self.record);self.link.advance(self.record)
        self.drain()
        commands=[v for k,v in self.client.calls if k=='panel_step'];self.assertEqual(len(commands),1)
        self.twin.predict.assert_called_once_with(commands[0]['target'])
        self.assertEqual(commands[0]['camera_timestamp'],self.record['timestamp'])
        self.assertEqual(self.link.feedback['tcp_mm_deg'],[5,0,625,180,0,0])

    def test_real_mode_does_not_run_virtual_preflight(self):
        self.link.set_mode('REAL');self.link.start(self.record);self.link.advance(self.record);self.drain()
        self.twin.predict.assert_not_called();self.assertTrue(any(k=='panel_step' for k,_ in self.client.calls))

    def test_sim_failure_blocks_real_command_and_stops_both(self):
        self.link.set_mode('SIM+REAL');self.link.start(self.record);self.twin.predict.side_effect=ValueError('IK failed')
        self.link.events.put((self.link.epoch,'poll',True,copy.deepcopy(self.client.feedback)));self.link.tick(self.record);self.drain()
        self.assertIsNone(self.link.plan);self.assertTrue(any(k=='stop' for k,_ in self.client.calls))
        self.assertFalse(any(k=='panel_step' for k,_ in self.client.calls))

    def test_stale_camera_stops_and_old_epoch_cannot_resume(self):
        self.link.set_mode('SIM+REAL');self.link.start(self.record);old=self.link.epoch
        self.record['timestamp']-=1;self.link.tick(self.record);self.drain()
        self.link.events.put((old,'poll',True,dict(self.client.feedback,tcp_mm_deg=[999]*6)))
        self.link.tick(self.record)
        self.assertIsNone(self.link.plan);self.assertNotEqual(self.link.feedback['tcp_mm_deg'],[999]*6)

    def test_mode_change_requires_stopping_active_plan(self):
        self.link.set_mode('SIM+REAL');self.link.start(self.record)
        with self.assertRaises(ValueError):self.link.set_mode('SIM')
        self.assertEqual(self.link.mode,'SIM+REAL')


class TwinModelTests(unittest.TestCase):
    def setUp(self):
        root=Path(__file__).resolve().parents[1]
        self.twin=KeyTwin(RobotDescription.load(root/'examples/fairino_fr5.urdf'));initialize_panel_arm(self.twin.sim)
        self.q=list(self.twin.sim.q);self.tool=transform((0,0,.165),(.1,.05,0))
        tcp=multiply(self.twin.sim.kin.fk(self.q),self.tool)
        self.feedback=dict(joints_rad=self.q,tcp_mm_deg=pose_from_matrix(tcp,[r[3]*1000 for r in tcp[:3]]))
        self.twin.fit(self.feedback)

    def test_full_tool_transform_prediction_keeps_real_joints_separate(self):
        before=copy.deepcopy(self.feedback);target=list(before['tcp_mm_deg']);target[1]+=.4
        self.twin.predict(target);c=self.twin.comparison(before)
        self.assertEqual(self.feedback,before);self.assertGreater(c['tcp_difference_mm'],.35)
        self.assertLess(c['tcp_difference_mm'],.45)
        self.assertNotEqual(self.twin.sim.q,self.q)

    def test_fitting_once_does_not_hide_later_actual_error(self):
        moved=copy.deepcopy(self.feedback);moved['tcp_mm_deg'][0]+=3
        self.assertAlmostEqual(self.twin.comparison(moved)['tcp_difference_mm'],3,places=5)


if __name__=='__main__':unittest.main()
