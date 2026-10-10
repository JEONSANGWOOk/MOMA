import copy
import math
import tempfile
import unittest
from pathlib import Path
from seer_control.aruco_arm_follow import CameraArmFollower,camera_origin,latest_record,rotation
from seer_control.arm_simulation import ArmSimulator
from seer_control.geometry3d import RobotDescription

ROOT=Path(__file__).resolve().parents[1]


class CameraFollowTests(unittest.TestCase):
    def setUp(self):
        self.sim=ArmSimulator(RobotDescription.load(ROOT/'examples/fairino_fr5.urdf'))
        self.sim.q=[math.radians(v) for v in [0,-90,90,-90,-90,0]]
        self.follow=CameraArmFollower(self.sim)
        self.follow.select(('DICT_4X4_1000',0))
        self.record=dict(timestamp=100.,markers=[dict(id=0,dictionary='DICT_4X4_1000',pose_valid=True,
            camera_xyz_m=[0.,0.,.6],rotation_vector_rad=[math.pi,0.,0.],reprojection_px=.2)])
        for i in range(3):
            self.record['timestamp']=100+i*.2
            self.follow.tick(self.record,now=self.record['timestamp'])
        self.follow.reference();self.follow.start()

    def test_camera_translation_updates_existing_arm_with_speed_bound(self):
        initial=self.sim.kin.pose(self.sim.q)
        self.record['markers'][0]['camera_xyz_m'][0]=-.04
        applied=0
        for i in range(20):
            before=self.sim.kin.pose(self.sim.q)
            self.record['timestamp']=101+i*.2
            applied+=self.follow.tick(self.record,dt=.1,now=self.record['timestamp'])
            self.assertLessEqual(math.dist(before[:3],self.sim.kin.pose(self.sim.q)[:3]),5.)
        self.assertGreater(applied,0,self.follow.status)
        self.assertGreater(math.dist(initial[:3],self.sim.kin.pose(self.sim.q)[:3]),10.)
        self.assertTrue(self.follow.enabled,self.follow.status)

    def test_camera_rotation_at_same_origin_does_not_create_translation(self):
        origin,_=camera_origin(self.record['markers'][0]);rvec=[3.,.12,.1];r=rotation(rvec)
        marker=self.record['markers'][0]
        marker['rotation_vector_rad']=rvec
        marker['camera_xyz_m']=[-sum(r[i][j]*origin[j] for j in range(3)) for i in range(3)]
        self.follow.tick(self.record,now=self.record['timestamp'])
        self.assertLess(math.sqrt(sum(v*v for v in self.follow.delta_mm)),1e-6)

    def test_lost_or_stale_marker_holds_and_requires_restart(self):
        initial=list(self.sim.q)
        self.follow.tick(dict(timestamp=101.,markers=[]),now=101.)
        self.assertEqual(initial,self.sim.q);self.assertFalse(self.follow.enabled)
        self.follow.tick(self.record,now=self.record['timestamp'])
        self.assertFalse(self.follow.enabled)
        for i in range(3):
            self.record['timestamp']=102+i*.2
            self.follow.tick(self.record,now=self.record['timestamp'])
        self.follow.reference();self.follow.start()
        self.follow.tick(self.record,now=200.)
        self.assertEqual(initial,self.sim.q);self.assertFalse(self.follow.enabled)

    def test_duplicate_id_error_nan_and_large_jump_hold(self):
        for kind in ('duplicate','error','nan','jump'):
            with self.subTest(kind=kind):
                self.setUp();record=copy.deepcopy(self.record);initial=list(self.sim.q)
                if kind=='duplicate':record['markers']*=2
                elif kind=='error':record['markers'][0]['reprojection_px']=3
                elif kind=='nan':record['markers'][0]['camera_xyz_m'][0]=float('nan')
                else:record['markers'][0]['camera_xyz_m'][0]=1.
                self.follow.tick(record,now=record['timestamp'])
                self.assertEqual(initial,self.sim.q);self.assertFalse(self.follow.enabled)

    def test_stability_counts_new_frames_and_selection_resets_reference(self):
        self.follow.select(('DICT_5X5_1000',0));self.record['markers'][0]['dictionary']='DICT_5X5_1000'
        for _ in range(10):self.follow.inspect(self.record,now=self.record['timestamp'])
        self.assertEqual(self.follow.stable,1)
        with self.assertRaises(ValueError):self.follow.reference()
        self.assertIsNone(self.follow.baseline)

    def test_tail_reader_ignores_incomplete_record(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'frames.jsonl'
            path.write_text('{"timestamp":1,"markers":[]}\n{"timestamp":2',encoding='utf-8')
            self.assertEqual(latest_record(path)['timestamp'],1)
            self.assertIsNone(latest_record(Path(folder)/'missing'))


if __name__=='__main__':unittest.main()
