import copy,math,time,unittest
from unittest.mock import Mock
from tests.test_fairino import FakeRobot,config
from seer_control.fairino_api import SDKEngine
from seer_control.geometry3d import identity,transform
from seer_control.aruco_board import FRONT,pose_from_matrix
from seer_control.key_panel_real import template,calibration,RealKeyPlan,socket_frame,pose_matrix


def calibrated():
    c=template();c.update(verified=True,key_tcp_confirmed=True,board_revision='measured',tool=1,
        T_base_camera_mm=[list(r) for r in identity()],T_board_socket_mm=[list(r) for r in identity()],
        T_socket_key_zero_mm=[list(r) for r in transform(rpy=(math.pi,0,0))],workspace_min_mm=[-100,-100,500],workspace_max_mm=[100,100,700],
        standby_mm=25,insert_mm=5,turn_deg=30,version_confirmed=True,controller_version='3.9.8',
        contact_verified=True,force_sensor_verified=True,controller_guard_verified=True,force_limit_n=5,torque_limit_nm=.5)
    return c


def frame(now=100.):
    return dict(timestamp=now,board=dict(valid=True,revision='measured',geometry_source='measured',markers_used=2,
        camera_xyz_m=[0.,0.,.6],rotation_vector_rad=[0.,0.,0.],reprojection_px=.1))


class RealTests(unittest.TestCase):
    def setUp(self):
        self.c=calibrated();self.record=frame();self.feedback=dict(tcp_mm_deg=[0,0,625,180,0,0],motion_done=1,status='IDLE',force_torque=[0]*6)
        self.robot=FakeRobot();self.robot.tcp=list(self.feedback['tcp_mm_deg'])
        self.robot.GetActualTCPNum=Mock(return_value=(0,1));self.robot.GetActualWObjNum=Mock(return_value=(0,0));self.robot.FT_GetForceTorqueRCS=Mock(return_value=(0,[0]*6))
        class LivePacket:
            value=0
            @property
            def frame_cnt(self):self.value=(self.value+1)%256;return self.value
        self.robot.robot_state_pkg=LivePacket()
        self.engine=SDKEngine(config(),self.robot)
        self.engine.panel_status()

    def test_no_default_calibration_and_reflections_rejected(self):
        for c in (None,template()):
            with self.assertRaises(ValueError):calibration(c)
        self.c['T_base_camera_mm'][0][0]=-1
        with self.assertRaises(ValueError):calibration(self.c)

    def test_calibrated_camera_translation_is_mm_and_not_virtual(self):
        self.c['T_base_camera_mm'][0][3]=37
        s=socket_frame(self.c,self.record,100)
        self.assertAlmostEqual(s[0][3],37);self.assertAlmostEqual(s[2][3],600)

    def test_measured_hybrid_center_changes_target_and_rejects_estimates(self):
        self.c.update(hybrid_target_verified=True,hybrid_target_revision='hole')
        self.record['handle_target']=dict(valid=True,confidence=.95,profile_revision='hole',board_revision='measured',dimension_source='measured',board_xyz_mm=[.6,-.3,0.])
        s=socket_frame(self.c,self.record,100);self.assertAlmostEqual(s[0][3],.6);self.assertAlmostEqual(s[1][3],-.3)
        for mode in ('missing','estimated','revision','height','range'):
            r=copy.deepcopy(self.record)
            if mode=='missing':r.pop('handle_target')
            if mode=='estimated':r['handle_target']['dimension_source']='estimated_plane'
            if mode=='revision':r['handle_target']['profile_revision']='other'
            if mode=='height':r['handle_target']['board_xyz_mm'][2]=1
            if mode=='range':r['handle_target']['board_xyz_mm'][0]=11
            with self.subTest(mode=mode):
                with self.assertRaises(ValueError):socket_frame(self.c,r,100)

    def test_synthetic_estimated_stale_and_wrong_revision_rejected(self):
        for change in ('demo','estimate','stale','revision'):
            r=copy.deepcopy(self.record)
            if change=='demo':r['vision_source']='built_in_demo'
            if change=='estimate':r['board']['geometry_source']='depth_estimate'
            if change=='stale':r['timestamp']=99.
            if change=='revision':r['board']['revision']='other'
            with self.assertRaises(ValueError):socket_frame(self.c,r,100)

    def test_contact_force_and_raw_jump_block(self):
        p=RealKeyPlan(self.c,True);p.step(self.record,self.feedback,100)
        r=copy.deepcopy(self.record);r['timestamp']=100.2;r['raw_board']=copy.deepcopy(r['board']);r['raw_board']['camera_xyz_m'][0]=.01
        with self.assertRaisesRegex(ValueError,'급변'):p.step(r,self.feedback,100.2)
        p=RealKeyPlan(self.c,True);p.stage='INSERT';self.feedback['force_torque']=[6,0,0,0,0,0]
        with self.assertRaisesRegex(ValueError,'힘/토크'):p.step(self.record,self.feedback,100)

    def run_plan(self,contact):
        p=RealKeyPlan(self.c,contact);commands=0;seen=set()
        for i in range(1100):
            t=100+i*.2;self.record['timestamp']=t;seen.add(p.stage)
            before=self.feedback['tcp_mm_deg'];command=p.step(self.record,self.feedback,t)
            if command:
                target=command['target'];self.assertLessEqual(math.dist(before[:3],target[:3]),.50001)
                self.feedback['tcp_mm_deg']=target;commands+=1
            if p.stage=='HOLD':break
        self.assertEqual(p.stage,'HOLD');self.assertGreater(commands,0)
        self.assertAlmostEqual(self.feedback['tcp_mm_deg'][2],625 if contact else 605,delta=.12)
        if contact:
            self.assertTrue({'INSERT','TURN','RETRACT'}<=seen);self.assertAlmostEqual(p.evidence['actual_turn_deg'],30,delta=.31)
            self.assertTrue(p.completed)
        else:self.assertFalse({'INSERT','TURN'}&seen)

    def test_air_approach_stops_before_slot(self):self.run_plan(False)
    def test_contact_sequence_from_measured_feedback(self):self.run_plan(True)

    def test_old_ninety_degree_calibration_is_rejected(self):
        self.c['turn_deg']=90
        with self.assertRaises(ValueError):calibration(self.c,True)

    def test_worker_retreats_outward_and_preserves_force_guard(self):
        spec=self.spec();spec['stage']='RETRACT';self.robot.tcp=[0,0,595,180,0,0];spec['target']=[0,0,595.1,180,0,0]
        self.engine.panel_step(spec);self.robot.MoveL.assert_called_once()
        self.setUp();spec=self.spec();spec['stage']='RETRACT';self.robot.tcp=[0,0,595,180,0,0];spec['target']=[0,0,594.9,180,0,0]
        with self.assertRaisesRegex(ValueError,'후퇴'):self.engine.panel_step(spec)
        self.robot.MoveL.assert_not_called()
        spec['target'][2]=595.1;self.robot.FT_GetForceTorqueRCS.return_value=(0,[6,0,0,0,0,0])
        with self.assertRaisesRegex(ValueError,'힘/토크'):self.engine.panel_step(spec)
        self.robot.MoveL.assert_not_called()

    def spec(self):return dict(target=[0,0,624.5,180,0,0],expires=time.monotonic()+.25,stage='APPROACH',calibration=self.c,socket_base_mm=[list(r) for r in transform((0,0,600))])

    def test_sdk_read_only_status_never_moves(self):
        self.engine.panel_status();self.robot.MoveL.assert_not_called()

    def test_stalled_controller_packet_is_not_fresh_feedback(self):
        from types import SimpleNamespace
        self.robot.robot_state_pkg=SimpleNamespace(frame_cnt=1)
        self.engine._panel_frame=(1,time.monotonic()-1)
        with self.assertRaisesRegex(RuntimeError,'정체'):self.engine.panel_status()
        self.robot.MoveL.assert_not_called()

    def test_worker_cannot_bypass_air_gap_or_insertion_guard(self):
        spec=self.spec();self.robot.tcp=[0,0,600,180,0,0];spec['target'][2]=599.5
        with self.assertRaisesRegex(ValueError,'5 mm'):self.engine.panel_step(spec)
        self.robot.MoveL.assert_not_called()
        spec['stage']='TURN';self.robot.tcp=[0,0,602,180,0,0];spec['target'][2]=601.5
        with self.assertRaisesRegex(ValueError,'회전'):self.engine.panel_step(spec)
        self.robot.MoveL.assert_not_called()

    def test_sdk_accepts_only_small_measured_tcp_steps(self):
        self.engine.panel_step(self.spec())
        self.robot.MoveL.assert_called_once_with([0.,0.,624.5,180.,0.,0.],tool=1,user=0,vel=1.,acc=10.,ovl=10.,blendR=0.)
        self.assertEqual(self.engine.panel_status()['status'],'RUNNING')

    def test_sdk_rechecks_limits_tool_force_lease_and_done(self):
        for mode in ('large','tool','force','expired','moving','emergency','camera_stale'):
            self.setUp();spec=self.spec()
            if mode=='large':spec['target'][2]=623.
            if mode=='tool':self.robot.GetActualTCPNum.return_value=(0,2)
            if mode=='force':self.robot.FT_GetForceTorqueRCS.return_value=(0,[6,0,0,0,0,0]);spec['stage']='INSERT'
            if mode=='expired':spec['expires']=time.monotonic()-1
            if mode=='moving':self.robot.done=0
            if mode=='emergency':self.robot.emergency=1
            if mode=='camera_stale':spec['camera_timestamp']=time.time()-1
            with self.subTest(mode=mode):
                with self.assertRaises(ValueError):self.engine.panel_step(spec)
                self.robot.MoveL.assert_not_called()


if __name__=='__main__':unittest.main()
