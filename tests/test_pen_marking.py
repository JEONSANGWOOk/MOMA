import copy
import math
import unittest
from tests import test_aruco_board as fixtures
from seer_control.pen_tool import PenTool
from seer_control.visual_servo import VisualServo


class MarkingTests(unittest.TestCase):
    def setUp(self):
        f=fixtures.BoardTests();f.setUp()
        self.m=f.mission;self.sim=f.sim;self.record=f.record
        self.pen=PenTool(self.m);self.m.set_reference(100);self.m.update_scene()
        self.servo=VisualServo(self.m)

    def frame(self,dt=1/30):
        self.record['timestamp']+=dt
        self.m.inspect(self.record,now=self.record['timestamp']);self.m.update_scene()
        self.servo.tick(self.record,now=self.record['timestamp']);self.pen.sync()

    def test_tip_tcp_and_gripper_geometry(self):
        flange=self.sim.kin.pose(self.sim.q)[:3];tip=self.pen.tip_mm()
        self.assertAlmostEqual(math.dist(flange,tip),self.pen.offset_mm)
        self.assertEqual(self.sim.physics.held,self.pen.pen)
        faces,lines=self.sim.physics.geometry(self.sim.q,False)
        faces,lines=self.pen.geometry(faces,lines)
        self.assertTrue(any(f[2]=='pen_tip' for f in faces))
        self.assertEqual(self.sim.physics.config['kind'],'finger')

    def test_two_actual_center_contacts_and_return_to_live_hold(self):
        self.servo.start_marking(self.pen,100)
        for _ in range(1200):
            self.frame()
            if self.servo.completed_marking or not self.servo.enabled:break
        self.assertEqual(self.servo.completed_marking,1,self.servo.status)
        self.assertEqual([m['marker_index'] for m in self.pen.marks],[0,1])
        for mark in self.pen.marks:
            self.assertLessEqual(mark['planar_error_mm'],.25)
            self.assertLessEqual(abs(mark['normal_error_mm']),.05)
            self.assertEqual(mark['planar_error_mm'],math.dist(mark['hit_local_mm'][:2],mark['target_local_mm'][:2]))
        self.assertTrue(self.servo.enabled);self.assertEqual(self.servo.stage,'HOLD')

    def test_marking_and_retract_track_moving_and_rotating_board(self):
        self.servo.start_marking(self.pen,100,(None,))
        seen=set()
        for _ in range(1500):
            if self.servo.stage in ('APPROACH','RETRACT'):
                seen.add(self.servo.stage)
                self.record['board']['camera_xyz_m'][0]+=.000015
                self.record['board']['rotation_vector_rad'][0]-=.00002
            self.frame()
            if self.servo.completed_marking or not self.servo.enabled:break
        self.assertEqual(self.servo.completed_marking,1,self.servo.status)
        self.assertEqual(seen,{'APPROACH','RETRACT'})
        self.assertLess(self.pen.marks[0]['planar_error_mm'],.6)
        self.assertTrue(self.servo.enabled)

    def test_contact_not_faked_and_missing_frame_never_marks(self):
        self.assertIsNone(self.pen.record_mark(0,[-65,0],100))
        self.servo.start_marking(self.pen,100)
        self.m.inspect(None,now=101);self.servo.tick(None,now=101)
        self.assertFalse(self.servo.enabled);self.assertEqual(self.pen.marks,[])

    def test_faster_correction_is_still_bounded(self):
        self.servo.start([20,0],100,20)
        previous=self.pen.tip_mm();self.frame()
        moved=math.dist(previous,self.pen.tip_mm())
        self.assertTrue(self.servo.enabled,self.servo.status)
        self.assertGreater(moved,3.)
        self.assertLess(moved,4.2)

    def test_invalid_speed_does_not_mutate_parameters(self):
        for speed,approach in [(0,60),(120,float('nan')),(500,60)]:
            with self.assertRaises(ValueError):self.servo.configure_speed(speed,approach)
        self.assertEqual(self.servo.speed_mm_s,120)


if __name__=='__main__':unittest.main()
