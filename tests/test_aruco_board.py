import copy
import math
import unittest
from pathlib import Path
from seer_control.aruco_board import BoardArmMission,make_board,object_points
from seer_control.arm_simulation import ArmSimulator
from seer_control.geometry3d import RobotDescription

ROOT=Path(__file__).resolve().parents[1]


class BoardTests(unittest.TestCase):
    def setUp(self):
        self.sim=ArmSimulator(RobotDescription.load(ROOT/'examples/fairino_fr5.urdf'))
        self.sim.q=[math.radians(v) for v in [0,-90,90,-90,-90,0]]
        self.home=list(self.sim.q);self.mission=BoardArmMission(self.sim)
        self.config=make_board([dict(dictionary='DICT_4X4_1000',id=0),dict(dictionary='DICT_5X5_1000',id=0)],[100,100],130)
        self.mission.config=self.config
        self.record=dict(timestamp=100.,board=dict(valid=True,revision=self.config['revision'],markers_used=2,
            geometry_source='measured',camera_xyz_m=[0,0,.6],rotation_vector_rad=[math.pi,0,0],reprojection_px=.1))
        for i in range(3):self.record['timestamp']=100+i*.2;self.mission.inspect(self.record,now=self.record['timestamp'])
        self.mission.set_reference();self.mission.update_scene()

    def test_complete_cycle_preflight_and_execution_returns_home(self):
        snapshot=self.mission.snapshot([0,0],200,100)
        config=BoardArmMission.prepare(snapshot)
        self.assertEqual(self.sim.q,self.home)  # planning never moves live simulator
        self.mission.launch(snapshot,config)
        for _ in range(1600):
            self.mission.inspect(self.record,now=self.record['timestamp']);self.mission.tick(.05)
            if not self.mission.running:break
        self.assertEqual(self.sim.state,'COMPLETED',self.sim.error)
        self.assertLess(max(abs(a-b) for a,b in zip(self.sim.q,self.home)),1e-6)
        self.assertEqual([e['operation'] for e in self.sim.events],['align','approach','retract','home'])

    def test_board_motion_updates_target_in_registered_axes(self):
        original=self.mission.snapshot([0,0],200,100)['targets']['align']
        self.record['board']['camera_xyz_m'][0]=.02
        self.mission.inspect(self.record,now=self.record['timestamp'])
        moved=self.mission.snapshot([0,0],200,100)['targets']['align']
        self.assertAlmostEqual(math.dist(original[:3],moved[:3]),20.,places=6)

    def test_plane_rotation_changes_tool_orientation(self):
        original=self.mission.snapshot([0,0],200,100)['targets']['align']
        self.record['board']['rotation_vector_rad']=[math.pi-.1,0,0]
        self.mission.inspect(self.record,now=self.record['timestamp'])
        moved=self.mission.snapshot([0,0],200,100)['targets']['align']
        self.assertGreater(math.dist(original[3:],moved[3:]),5.)

    def test_loss_and_runtime_board_motion_stop_without_resuming(self):
        snapshot=self.mission.snapshot([0,0],200,100);config=BoardArmMission.prepare(snapshot)
        self.mission.launch(snapshot,config);self.mission.tick(.1)
        q=list(self.sim.q)
        self.mission.inspect(dict(timestamp=101,board=dict(valid=False,reason='missing')),now=101)
        self.assertFalse(self.mission.running);self.assertEqual(q,self.sim.q)
        self.mission.inspect(self.record,now=self.record['timestamp']);self.assertFalse(self.mission.running)
        self.mission.stable=3;self.mission.launch(snapshot,config)
        self.record['board']['camera_xyz_m'][0]=.02
        self.mission.inspect(self.record,now=self.record['timestamp'])
        self.assertFalse(self.mission.running);self.assertEqual(q,self.sim.q)

    def test_stale_config_error_and_duplicate_layout_rejected(self):
        self.mission.inspect(self.record,now=200);self.assertIsNone(self.mission.latest)
        self.record['board']['revision']='old';self.mission.inspect(self.record,now=self.record['timestamp'])
        self.assertIsNone(self.mission.latest)
        with self.assertRaises(ValueError):make_board([self.config['markers'][0]]*2,[100,100],130)
        with self.assertRaises(ValueError):make_board(self.config['markers'],[100,100],80)

    def test_invalid_offsets_distances_and_no_partial_unreachable_plan(self):
        for values in [([200,0],200,100),([0,0],100,200),([0,0],200,0)]:
            with self.assertRaises(ValueError):self.mission.snapshot(*values)
        snapshot=self.mission.snapshot([0,0],200,100)
        snapshot['targets']['approach'][0]=100000
        with self.assertRaises(ValueError):BoardArmMission.prepare(snapshot)
        self.assertEqual(self.sim.q,self.home)

    def test_board_corner_coordinates_include_registered_marker_rotation(self):
        points=object_points(self.config,0)
        self.assertAlmostEqual(sum(p[0] for p in points)/4,-.065)
        changed=copy.deepcopy(self.config);changed['markers'][0]['rotation_deg']=90
        rotated=object_points(changed,0)
        self.assertAlmostEqual(rotated[0][0],-.115)
        self.assertAlmostEqual(rotated[0][1],-.05)


if __name__=='__main__':unittest.main()
