import copy
import math
import unittest
from tests import test_aruco_board as fixtures
from seer_control.visual_servo import VisualServo


class ServoTests(unittest.TestCase):
    def setUp(self):
        base=fixtures.BoardTests();base.setUp()
        self.m=base.mission;self.sim=base.sim;self.record=base.record
        self.servo=VisualServo(self.m);self.z=self.sim.kin.pose(self.sim.q)[2]

    def frame(self):
        self.record['timestamp']+=.2
        self.m.inspect(self.record,now=self.record['timestamp'])
        self.m.update_scene()
        self.servo.tick(self.record,now=self.record['timestamp'])

    def test_upright_panel_forward_backward_cycle(self):
        center,r=self.m.virtual_board()
        self.assertGreater(r[0][2],.999)
        self.assertGreater(r[2][1],.999)
        self.servo.start([0,0],200,100,True)
        positions=[]
        for _ in range(400):
            self.frame();positions.append(self.sim.kin.pose(self.sim.q)[:3])
            if not self.servo.enabled:break
        self.assertEqual(self.servo.stage,'COMPLETED',self.servo.status)
        self.assertGreater(positions[0][0]-min(p[0] for p in positions),95)
        self.assertLess(max(abs(p[2]-self.z) for p in positions),2)
        self.assertLess(abs(positions[-1][0]-positions[0][0]),2)

    def test_hold_tracks_changed_camera_target_and_rotation(self):
        self.servo.start([0,0],200,100)
        for _ in range(10):self.frame()
        before=self.sim.kin.pose(self.sim.q)
        self.record['board']['camera_xyz_m'][0]+=.01
        self.record['board']['rotation_vector_rad'][0]-=.03
        for _ in range(80):self.frame()
        self.assertTrue(self.servo.enabled,self.servo.status)
        self.assertGreater(math.dist(before[:3],self.sim.kin.pose(self.sim.q)[:3]),5)
        self.assertLess(self.servo.errors['position_error_mm'],1.3)
        self.assertLess(self.servo.errors['angle_error_deg'],1.)

    def test_repeated_frames_do_not_move_and_loss_stops(self):
        self.servo.start([10,0],200,100)
        self.frame();q=list(self.sim.q)
        for _ in range(20):self.servo.tick(self.record,now=self.record['timestamp'])
        self.assertEqual(q,self.sim.q)
        self.m.inspect(None,now=self.record['timestamp'])
        self.servo.tick(None,now=self.record['timestamp'])
        self.assertFalse(self.servo.enabled);self.assertEqual(q,self.sim.q)
        self.frame();self.assertFalse(self.servo.enabled)

    def test_raw_jump_and_stale_stop_without_motion(self):
        for fault in ('jump','stale'):
            self.setUp();self.servo.start([0,0],200,100);self.frame();q=list(self.sim.q)
            if fault=='jump':
                self.record['raw_board']=copy.deepcopy(self.record['board'])
                self.record['raw_board']['camera_xyz_m'][0]+=.04
                self.frame()
            else:self.servo.tick(self.record,now=self.record['timestamp']+1)
            self.assertFalse(self.servo.enabled);self.assertEqual(q,self.sim.q)

    def test_invalid_raw_pose_and_revision_stop(self):
        for fault in ('nan','revision'):
            self.setUp();self.servo.start([0,0],200,100);self.frame();q=list(self.sim.q)
            self.record['raw_board']=copy.deepcopy(self.record['board'])
            if fault=='nan':self.record['raw_board']['camera_xyz_m'][0]=float('nan')
            else:self.record['raw_board']['revision']='other'
            self.frame();self.assertFalse(self.servo.enabled);self.assertEqual(q,self.sim.q)

    def test_invalid_input_and_collision_reject_before_movement(self):
        with self.assertRaises(ValueError):self.servo.start([float('nan'),0],200,100)
        self.servo.start([20,0],200,100);q=list(self.sim.q)
        def blocked(a,b):raise ValueError('test obstacle')
        self.sim.physics.check_motion=blocked
        self.frame();self.assertFalse(self.servo.enabled);self.assertEqual(q,self.sim.q)


if __name__=='__main__':unittest.main()
