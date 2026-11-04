import math
import os
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
from seer_control.door_sync import publish,read_snapshot
from web_control import server
from web_control.server import Control,Doors

class DoorSyncTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();root=Path(self.tmp.name)
        self.env=patch.dict(os.environ,{'MOMA_DOOR_SYNC_PATH':str(root/'live.json')});self.env.start()
        self.settings=patch.object(server,'SETTINGS',root/'web.json');self.settings.start()
        self.angles={'left':.2,'right':.5}
        parts=[{'name':'door:1'},{'name':'door:2'}]
        view=NS(cad_asset=NS(name='SIMPLE_WHOLE.glb',door_parts=parts),manual_real_door_display=None,
                real_doors_follow=NS(get=lambda:True),webcam_doors_enabled=NS(get=lambda:False))
        view.cad_door_angle=lambda p:self.angles['left' if p['name'].endswith(':1') else 'right']
        self.desktop=NS(world3d=view,real=True)
        self.control=Control()
    def tearDown(self):self.settings.stop();self.env.stop();self.tmp.cleanup()
    def test_manual_snapshot_and_continuous_angles_reach_web_without_new_revision(self):
        self.assertTrue(publish(self.desktop,command=True))
        self.assertTrue(self.control.sync_desktop_doors())
        self.assertAlmostEqual(self.control.doors['left'],math.degrees(.2))
        revision=self.control.desktop_revision
        self.angles['left']=1.;self.desktop._door_sync_at=0
        self.assertTrue(publish(self.desktop))
        self.control.sync_desktop_doors()
        self.assertEqual(self.control.desktop_revision,revision)
        self.assertAlmostEqual(self.control.doors['left'],math.degrees(1))
        self.assertTrue(self.control.snapshot()['door_sync']['connected'])
    def test_web_override_ignores_old_frames_and_next_desktop_action_resumes_sync(self):
        publish(self.desktop,True);self.control.sync_desktop_doors()
        self.control.set_doors(Doors(left=5,right=10))
        self.assertFalse(self.control.sync_desktop_doors())
        self.assertEqual(self.control.doors['left'],5)
        self.angles['left']=math.pi/2;publish(self.desktop,True)
        self.assertTrue(self.control.sync_desktop_doors())
        self.assertAlmostEqual(self.control.doors['left'],90)
        self.assertEqual(self.control.door_source,'desktop')
    def test_stale_frames_and_invalid_angles_are_ignored(self):
        publish(self.desktop,True)
        self.assertIsNone(read_snapshot(now=time.time()+3))
        self.angles['left']=float('nan')
        self.assertFalse(publish(self.desktop,True))
    def test_manual_override_disables_web_kinematic_follow(self):
        self.desktop.world3d.manual_real_door_display=(self.desktop.world3d.cad_asset,{})
        publish(self.desktop,True);self.control.sync_desktop_doors()
        self.assertFalse(self.control.door_follow)
        self.assertEqual(self.control.desktop_door_mode,'manual')
    def test_different_equipment_is_not_synchronized(self):
        self.desktop.world3d.cad_asset.name='other.glb'
        publish(self.desktop,True)
        self.assertFalse(self.control.sync_desktop_doors())
    def test_desktop_source_is_not_overwritten_by_web_fk(self):
        publish(self.desktop,True);self.control.sync_desktop_doors()
        before=dict(self.control.doors)
        self.control.arm_model=Mock();self.control.update_doors()
        self.control.arm_model.link_transforms.assert_not_called()
        self.assertEqual(self.control.doors,before)

if __name__=='__main__':unittest.main()
