import math
import unittest
from tests import test_aruco_board as fixtures
from seer_control.key_panel import KeyTool,KeyPanelPhysics,KeyPanelScene,KeyServo
from seer_control.aruco_board import pose_from_matrix


class KeyTests(unittest.TestCase):
    def setUp(self):
        f=fixtures.BoardTests();f.setUp();self.m=f.mission;self.sim=f.sim;self.record=f.record
        self.sim.physics=KeyPanelPhysics(self.sim.kin)
        self.tool=KeyTool(self.m);self.m.set_reference(132)
        self.scene=KeyPanelScene(self.m,self.tool);self.scene.sync();self.servo=KeyServo(self.m,self.scene)

    def frame(self,moving=False):
        self.record['timestamp']+=1/30
        if moving:
            self.record['board']['camera_xyz_m'][0]+=.000005
            self.record['board']['rotation_vector_rad'][0]-=.000003
        self.m.inspect(self.record,now=self.record['timestamp'])
        self.scene.sync();self.servo.tick(self.record,now=self.record['timestamp'])

    def cycle(self,moving=False):
        self.servo.start_key();seen=set()
        for _ in range(1400):
            seen.add(self.servo.stage);self.frame(moving)
            if self.servo.completed or not self.servo.enabled:break
        self.assertTrue(self.scene.unlocked,self.servo.status)
        self.assertTrue(self.servo.completed,self.servo.status)
        f=self.scene.feedback();self.assertLess(abs(f['depth_mm']+100),.3)
        self.assertLess(abs(f['turn_deg']-30),.4);self.assertLess(f['lateral_mm'],.25)
        self.assertTrue({'ALIGN','APPROACH','INSERT','TURN','RETRACT'}<=seen)

    def test_insert_rotate_and_withdraw(self):
        self.cycle()
        self.assertEqual(self.servo.stage,'HOLD',self.servo.status)
        self.assertLess(self.scene.feedback()['depth_mm'],-99)
        self.assertTrue(self.scene.unlocked)

    def test_moving_panel_live_tracking(self):self.cycle(True)

    def test_wrong_key_cannot_turn(self):
        self.scene.correct_key=False;self.servo.start_key()
        for _ in range(1000):
            self.frame()
            if not self.servo.enabled:break
        self.assertFalse(self.scene.unlocked);self.assertFalse(self.servo.enabled)
        self.assertLess(abs(self.scene.rotor_deg),.5)

    def test_loss_and_shallow_insertion(self):
        self.assertFalse(self.scene.can_turn(self.scene.feedback()))
        self.servo.start_key();q=list(self.sim.q)
        self.m.inspect(None,now=101);self.servo.tick(None,now=101)
        self.assertFalse(self.servo.enabled);self.assertEqual(q,self.sim.q)
        self.assertFalse(self.scene.unlocked)

    def test_collision_queries_do_not_commit_lock(self):
        before=[o['matrix'] for o in self.scene.walls]
        self.sim.physics.contacts(self.sim.q)
        self.assertEqual(before,[o['matrix'] for o in self.scene.walls])
        self.assertFalse(self.scene.unlocked)
        self.assertEqual(len(self.tool.key['collision_parts']),2)

    def test_misaligned_insert_is_physically_blocked(self):
        s=self.scene;r=s.nominal();p=s.world([s.offset[0]+5,s.offset[1],10])
        q=self.sim.kin.ik(pose_from_matrix(r,[p[i]-r[i][2]*self.tool.offset_mm for i in range(3)]),self.sim.q)
        risks,_=self.sim.physics.contacts(q)
        self.assertTrue(any(a==self.tool.key['name'] and '슬롯' in b for a,b in risks),risks)
        before=list(self.sim.q)
        with self.assertRaises(ValueError):self.sim.set_joints(q)
        self.assertEqual(before,self.sim.q)


if __name__=='__main__':unittest.main()
