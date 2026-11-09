import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
from seer_control.app import Console

class LidarDisplayAlignmentTests(unittest.TestCase):
    def test_map_reload_does_not_run_automatic_alignment(self):
        Console.align_lidar_to_map(NS(),auto=True)

    def test_low_confidence_blocks_manual_display_fit(self):
        app=NS(_lidar_alignment_running=False,_confidence_value=lambda:.1058)
        with patch('seer_control.app.messagebox.showinfo') as notice:
            Console.align_lidar_to_map(app)
        notice.assert_called_once()

    def test_low_score_or_nonfinite_saved_fit_is_ignored(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'align.json'
            app=NS(current_robot_map='map',map=NS(name='map'),host=NS(get=lambda:'host'),_lidar_alignment_file=lambda:path,log=Mock())
            for score,tr in ((.453125,[.12,.72,.2007]),(.9,[float('nan'),0,0]),(.9,[.01,.02,.03])):
                path.write_text(json.dumps({'host':{'map':{'score':score,'transform':tr}}}),encoding='utf-8')
                Console._load_lidar_alignment(app)
                self.assertEqual(app.lidar_map_alignment,(0,0,0))

    def test_legacy_offset_cannot_shift_robot_nodes_or_lidar(self):
        app=NS(lidar_map_alignment=(.12,.72,.2))
        self.assertEqual(Console._lidar_map_xy(app,2.,3.),(2.,3.))

    def test_reset_invalidates_pending_fit_and_persists_zero(self):
        app=NS(_lidar_alignment_epoch=3,lidar_map_alignment=(.12,.72,.2),lidar_alignment_score=.45,_save_lidar_alignment=Mock(),draw_map=Mock(),log=Mock())
        Console.reset_lidar_alignment(app)
        self.assertEqual(app._lidar_alignment_epoch,4)
        self.assertEqual(app.lidar_map_alignment,(0,0,0));app._save_lidar_alignment.assert_called_once()

class MapFrameRenderTests(unittest.TestCase):
    def test_3d_uses_same_raw_pose_and_scan_for_both_scan_formats(self):
        import time
        from seer_control.spatial_ui import SpatialMixin
        for source in ('1101.laser_beams(WORLD)','1009:lasers[0].beams'):
            state=dict(x=2.,y=3.,theta=.5)
            points=[(4.,5.)]
            world=NS(arm_asset=NS(movable=lambda:[]),draw_scene=Mock())
            app=NS(world3d=world,current_state=lambda:state,real=True,fr5_feedback={},studio_config={'arm':{}},runtime_arm_feedback=None,
                real_laser_points=points,lidar_source=source,connected=True,last_laser_rx=time.monotonic(),map=NS(),layers={'LiDAR':NS(get=lambda:True)},
                lidar_map_alignment=(.12,.72,.2),_lidar_map_xy=Mock(side_effect=AssertionError('Display correction must not be applied')))
            SpatialMixin._spatial_render(app)
            args=world.draw_scene.call_args.args
            self.assertEqual(args[1],state);self.assertEqual(args[-1],points)
            self.assertEqual(world._isaac_lidar,points)

if __name__=='__main__':unittest.main()
