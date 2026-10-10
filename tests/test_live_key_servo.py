import copy,math,unittest
from unittest.mock import Mock
from seer_control.live_servo_start import LiveServoStart
from tests import test_key_panel as fixtures


class LiveKeyTests(unittest.TestCase):
    def test_live_start_waits_for_real_fresh_registered_camera(self):
        gate=LiveServoStart();gate.request('SIM');start=Mock()
        record=dict(timestamp=100,board=dict(valid=True,markers_used=2),vision_source='built_in_demo')
        self.assertFalse(gate.tick(record,True,start,100));start.assert_not_called()
        record.pop('vision_source')
        self.assertFalse(gate.tick(record,False,start,100))
        self.assertFalse(gate.tick(record,True,start,101))
        self.assertTrue(gate.tick(record,True,start,100));start.assert_called_once()
        self.assertFalse(gate.tick(record,True,start,100))

    def test_stop_cancels_pending_start_and_real_mode_is_not_armed(self):
        gate=LiveServoStart()
        with self.assertRaises(ValueError):gate.request('REAL')
        gate.request('SIM');gate.cancel();self.assertFalse(gate.pending)

    def test_each_new_marker_pose_changes_active_target_and_rotation(self):
        case=fixtures.KeyTests();case.setUp();servo=case.servo
        servo.start_key();servo.move_tip=Mock()
        case.frame();first_args=copy.deepcopy(servo.move_tip.call_args.args)
        case.record['board']['camera_xyz_m'][0]+=.003
        case.record['board']['rotation_vector_rad'][0]-=.01
        case.frame();second=servo.move_tip.call_args
        self.assertTrue(servo.enabled,servo.status);self.assertEqual(servo.updates,2)
        self.assertNotEqual(first_args[1],second.args[1]);self.assertNotEqual(first_args[0],second.args[0])
        self.assertGreater(sum(abs(v) for v in second.kwargs['angular_velocity']),0)
        self.assertEqual(servo.errors['sample_timestamp'],case.record['timestamp'])

    def test_insert_pauses_advance_but_recomputes_target_after_marker_shift(self):
        case=fixtures.KeyTests();case.setUp();servo=case.servo
        servo.start_key();servo.stage='INSERT';servo.distance=case.scene.mouth_mm+5;servo.move_tip=Mock()
        before=servo.distance
        case.record['board']['camera_xyz_m'][0]+=.005;case.frame()
        self.assertTrue(servo.enabled);self.assertEqual(servo.distance,before)
        self.assertTrue(servo.errors['advance_paused_for_alignment']);servo.move_tip.assert_called_once()

    def test_free_space_large_marker_move_reconfirms_and_resumes_without_restart(self):
        case=fixtures.KeyTests();case.setUp();servo=case.servo
        servo.start_key();servo.move_tip=Mock();case.frame();servo.move_tip.reset_mock()
        case.record['board']['camera_xyz_m'][0]+=.04
        for _ in range(2):
            case.frame();self.assertTrue(servo.enabled,servo.status);servo.move_tip.assert_not_called()
            self.assertIn('재확인',servo.status)
            confirmed=servo.reacquire_count
            for _ in range(3):servo.tick(case.record,now=case.record['timestamp'])
            self.assertEqual(servo.reacquire_count,confirmed)
        case.frame();self.assertTrue(servo.enabled);servo.move_tip.assert_called_once()
        self.assertEqual(servo.errors['sample_timestamp'],case.record['timestamp'])

    def test_contact_keeps_large_raw_jump_stop(self):
        case=fixtures.KeyTests();case.setUp();servo=case.servo
        servo.start_key();servo.move_tip=Mock();case.frame();servo.move_tip.reset_mock()
        feedback=case.scene.feedback();feedback['depth_mm']=1
        case.scene.feedback=Mock(return_value=feedback)
        case.record['board']['camera_xyz_m'][0]+=.04;case.frame()
        self.assertFalse(servo.enabled);self.assertIn('급변',servo.status);servo.move_tip.assert_not_called()

    def test_delayed_camera_sample_does_not_create_large_catchup_step(self):
        case=fixtures.KeyTests();case.setUp();servo=case.servo
        servo.start_key();servo.move_tip=Mock();case.frame()
        case.record['timestamp']+=.2;case.frame()
        self.assertLessEqual(servo.move_tip.call_args.args[2],1/30)

    def test_large_translation_resumes_with_actual_sim_ik_and_collision_checks(self):
        case=fixtures.KeyTests();case.setUp();servo=case.servo
        servo.start_key();servo.stage='HOLD'
        for _ in range(240):
            case.frame()
            if not servo.enabled or servo.errors.get('position_error_mm',999)<.1:break
        self.assertTrue(servo.enabled,servo.status)
        before=case.tool.tip_mm();case.record['board']['camera_xyz_m'][0]+=.04
        for _ in range(240):
            case.frame()
            if not servo.enabled or (servo.reacquire_count==0 and servo.errors['sample_timestamp']==case.record['timestamp'] and servo.errors['position_error_mm']<.3):break
        self.assertTrue(servo.enabled,servo.status)
        self.assertGreater(math.dist(before,case.tool.tip_mm()),35)
        self.assertLess(servo.errors['position_error_mm'],.3)


if __name__=='__main__':unittest.main()
