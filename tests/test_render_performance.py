"""Rendering optimizations must preserve projection, pixels and arm state."""
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock

from PIL import Image
from seer_control.arm_workspace import ArmWorkspaceMixin
from seer_control.smooth_renderer import SmoothRenderer, flat_normal, smooth_normals
from seer_control.world3d import Camera
from seer_control.studio_ui import ConsoleAdapter


class RenderPerformanceTests(unittest.TestCase):
    def test_real_mission_start_without_simulator(self):
        ConsoleAdapter(NS(real=True)).mission_started(0)

    def test_real_mission_start_clears_existing_skip_markers(self):
        sim = NS(skipped_goals={'goal': 'blocked'}, skip_result='skipped')
        ConsoleAdapter(NS(real=True, sim=sim)).mission_started(0)
        self.assertEqual(sim.skipped_goals, {})
        self.assertIsNone(sim.skip_result)

    def test_camera_cache_tracks_mutation_and_projection_mode(self):
        camera = Camera()
        first = camera.basis()
        self.assertIs(camera.basis(), first)
        camera.target[0] += 1
        self.assertIsNot(camera.basis(), first)
        for attribute, value in [('yaw', 30), ('pitch', 20), ('distance', 5)]:
            previous = camera.basis()
            setattr(camera, attribute, value)
            self.assertIsNot(camera.basis(), previous)
        for perspective in (True, False):
            camera.perspective = perspective
            screen = camera.project((1.2, -.8, 0), 800, 600)
            ground = camera.ground(*screen[:2], 800, 600)
            self.assertAlmostEqual(ground[0], 1.2)
            self.assertAlmostEqual(ground[1], -.8)

    def test_flat_generated_faces_keep_previous_shading_normals(self):
        for face in [((0,0,0),(1,0,0),(0,1,0)),
                     ((0,0,0),(1,0,1),(1,1,1),(0,1,0)),
                     ((0,0,0),(0,0,0),(0,0,0))]:
            for expected in smooth_normals([face])[0]:
                for actual, value in zip(flat_normal(face), expected):
                    self.assertAlmostEqual(actual, value)

    def test_renderer_readback_uses_native_viewport_and_preserves_output_size(self):
        # Exercise render() without a driver: only GL calls are replaced.
        renderer = object.__new__(SmoothRenderer)
        renderer.Image = Image
        renderer.dc = renderer.context = renderer.window = None
        renderer.size = None
        renderer.make_current = Mock(return_value=True)
        renderer.resize = Mock()
        for name in ('glViewport', 'glClearColor', 'glClear', 'glMatrixMode',
                     'glLoadIdentity', 'glFrustum', 'glOrtho', 'glLoadMatrixf',
                     'glLightfv', 'glEnable', 'glDepthMask', 'glDisable',
                     'glFinish', 'glReadBuffer', 'glReadPixels'):
            setattr(renderer, name, Mock())
        for width, height in [(800,600), (1,8)]:
            image = renderer.render(Camera(), width, height)
            self.assertEqual(image.size, (width, height))
            self.assertEqual(renderer.size, (max(16,width), max(16,height)))
            self.assertEqual(renderer.glReadPixels.call_args.args[2:4], renderer.size)

    def test_state_publication_does_not_draw_a_second_world_frame(self):
        values = {'joint1': .3}
        view = NS(arm_asset=None, scales={'arm': 1}, positions={'arm': {}},
                  winfo_ismapped=lambda: True)
        simulator = NS(kin=NS(asset=NS(), positions=lambda q: values),
                       q=[.3], state='RUNNING', index=1, actions=[{}, {}])
        app = NS(arm_dev_sim=simulator, world3d=view, real=False, sim=NS(arm={}),
                 view_mode=NS(get=lambda: '3D'), _spatial_render=Mock())
        ArmWorkspaceMixin._aw_sync_main(app, render=False)
        self.assertEqual(app.sim.arm['joint_positions'], values)
        self.assertEqual(view.positions['arm'], values)
        self.assertEqual(app.sim.arm['progress'], .5)
        app._spatial_render.assert_not_called()
        ArmWorkspaceMixin._aw_sync_main(app)
        app._spatial_render.assert_called_once()


if __name__ == '__main__':
    unittest.main()
