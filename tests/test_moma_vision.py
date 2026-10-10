import json,tempfile,time,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
from seer_control.camera_lease import camera_lease
from seer_control.vision_camera_ui import preview_record
from seer_control.moma_vision_ui import ensure_arm_available,close_panel,stop_panel


class VisionIntegrationTests(unittest.TestCase):
    def test_preview_stale_and_future_frames_are_hidden(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'preview.json'
            for stamp,accepted in [(100,True),(98,False),(101,False)]:
                p.write_text(json.dumps(dict(timestamp=stamp,jpeg_base64='test')),encoding='utf-8')
                self.assertEqual(preview_record(p,100) is not None,accepted)
            p.write_text('{',encoding='utf-8');self.assertIsNone(preview_record(p,100))

    def test_camera_lock_rejects_duplicate_and_releases(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'camera.lock'
            with camera_lease(path):
                with self.assertRaises(RuntimeError):
                    with camera_lease(path):pass
            with camera_lease(path):pass

    def test_main_arm_rejects_panel_owner_connection_or_pending_stop(self):
        link=NS(client=NS(connected=True),busy=False,plan=None)
        app=NS(vision_panel_workspace=NS(key_panel_link=link))
        with self.assertRaises(ValueError):ensure_arm_available(app)
        link.client.connected=False;link.busy=True
        with self.assertRaises(ValueError):ensure_arm_available(app)
        link.busy=False;ensure_arm_available(app)

    def test_main_close_waits_for_panel_stop_before_destroying(self):
        workspace=NS(key_panel_close=Mock(side_effect=[False,True]),key_panel_stop=Mock())
        app=NS(vision_panel_workspace=workspace)
        stop_panel(app);workspace.key_panel_stop.assert_called_once()
        self.assertFalse(close_panel(app));self.assertIs(app.vision_panel_workspace,workspace)
        self.assertTrue(close_panel(app));self.assertIsNone(app.vision_panel_workspace)


if __name__=='__main__':unittest.main()
