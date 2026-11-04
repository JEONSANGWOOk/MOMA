import math
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock

from seer_control.runtime_feedback import RuntimeArmFeedback, parse_joints
from seer_control.spatial_ui import SpatialMixin
from seer_control.geometry3d import RobotDescription
from seer_control.world3d import WorldView3D


def payload(values):
    return {'robot': {'joint': dict(zip(('J1','J2','J3','J4','J5','J6'), values))}}


class RuntimeFeedbackTests(unittest.TestCase):
    def test_degrees_and_model_joint_names(self):
        feedback=RuntimeArmFeedback('http://localhost:8000/runtime.html')
        feedback._get=Mock(side_effect=[{'connected': True}, payload([0,90,-90,180,45,-45])])
        feedback.poll()
        names=[f'joint{i}' for i in range(6)]
        joints=feedback.joints(names)
        self.assertAlmostEqual(joints['joint1'], math.pi/2)
        self.assertAlmostEqual(joints['joint2'], -math.pi/2)
        self.assertEqual(feedback._get.call_args_list[1].args, ('/api/moma_status',))
        self.assertEqual(feedback.joints(names[:5]), {})
        feedback._sample=(time.monotonic()-3, (0,)*6)
        self.assertEqual(feedback.joints(names), {})

    def test_invalid_partial_and_nonfinite_angles(self):
        for values in ([0]*5, [0]*5+[float('nan')], [0]*5+[True], [0]*5+['1']):
            with self.assertRaises((KeyError, ValueError)):
                parse_joints(payload(values))

    def test_disconnect_does_not_read_cached_robot_status(self):
        feedback=RuntimeArmFeedback('http://localhost:8000')
        feedback._get=Mock(return_value={'connected': False})
        with self.assertRaises(ConnectionError):feedback.poll()
        feedback._get.assert_called_once_with('/api/connection_status')

    def test_polling_failure_invalidates_previous_pose(self):
        feedback=RuntimeArmFeedback('http://localhost:8000')
        feedback._sample=(time.monotonic(), (1,)*6)
        stop=threading.Event()
        def fail():
            stop.set()
            raise TimeoutError('late')
        feedback.poll=fail
        feedback.run(stop)
        self.assertEqual(feedback.joints([f'J{i}' for i in range(1,7)]), {})
        self.assertEqual(feedback.error, 'late')

    def test_real_render_passes_feedback_into_arm_model(self):
        feedback=RuntimeArmFeedback('http://localhost:8000')
        feedback._sample=(time.monotonic(), parse_joints(payload([10,20,30,40,50,60])))
        names=[f'joint{i}' for i in range(6)]
        asset=RobotDescription('test', {}, [dict(name=n,type='revolute') for n in names], [])
        view=NS(arm_asset=asset,draw_scene=Mock())
        app=NS(real=True,current_state=lambda: {}, world3d=view,
               studio_config={'arm': {'driver': 'none'}},runtime_arm_feedback=feedback,
               real_laser_points=[],lidar_source='NONE',connected=True,last_laser_rx=time.monotonic(),
               layers={'LiDAR': NS(get=lambda: False)},map=object())
        SpatialMixin._spatial_render(app)
        args=view.draw_scene.call_args.args
        state=NS(arm_asset=asset,positions={'arm': {}},app=app,arm_follow=NS(get=lambda: True))
        actual=WorldView3D._arm_positions(state,args[2],args[4])
        self.assertAlmostEqual(actual['joint5'],math.radians(60))
        self.assertIn('Runtime',state.arm_source)
        feedback._sample=(time.monotonic(), parse_joints(payload([15,25,35,45,55,65])))
        SpatialMixin._spatial_render(app)
        args=view.draw_scene.call_args.args
        updated=WorldView3D._arm_positions(state,args[2],args[4])
        self.assertAlmostEqual(updated['joint5'],math.radians(65))


if __name__=='__main__':unittest.main()
