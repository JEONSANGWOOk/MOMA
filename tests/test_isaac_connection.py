import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock,patch
from urllib.request import urlopen
from seer_control.isaac_connection import IsaacConnection,find_launcher

class IsaacConnectionTests(unittest.TestCase):
    def test_embedded_bridge_and_heartbeat(self):
        with tempfile.TemporaryDirectory() as folder,patch('seer_control.isaac_connection.directory',return_value=Path(folder)):
            manager=IsaacConnection(port=0);manager.start_bridge();server=manager.server
            try:
                self.assertIsNone(manager.health()['client_age_s'])
                with urlopen(manager.url+'/client-ready') as response:response.read()
                self.assertLess(manager.health()['client_age_s'],1)
                manager.start_bridge();self.assertIs(manager.server,server)
            finally:server.shutdown();server.server_close();manager.server=None
    def test_launch_uses_bootstrap_without_robot_commands(self):
        with tempfile.TemporaryDirectory() as folder,patch('seer_control.isaac_connection.directory',return_value=Path(folder)):
            launcher=Path(folder)/'isaac-sim.bat';launcher.write_text('@echo off')
            manager=IsaacConnection();manager.health=Mock(return_value=None)
            with patch('seer_control.isaac_connection.subprocess.Popen') as process:
                process.return_value.poll.return_value=None
                self.assertTrue(manager.launch(launcher))
                args=process.call_args
                self.assertEqual(args.args[0][1],'--exec')
                self.assertTrue(Path(args.args[0][2]).is_file())
                self.assertIn('MOMA_MIRROR_CLIENT',args.kwargs['env'])
                self.assertFalse(manager.launch(launcher))
                self.assertEqual(process.call_count,1)
                manager.close();process.return_value.terminate.assert_not_called()
    def test_reuses_a_live_receiver(self):
        with tempfile.TemporaryDirectory() as folder:
            launcher=Path(folder)/'isaac-sim.bat';launcher.write_text('@echo off')
            manager=IsaacConnection();manager.launch_mode=False;manager.health=Mock(return_value={'client_age_s':.1})
            with patch('seer_control.isaac_connection.subprocess.Popen') as process:
                self.assertFalse(manager.launch(launcher));process.assert_not_called()
    def test_invalid_launcher_rejected(self):
        with self.assertRaises(ValueError):IsaacConnection().launch('missing.bat')

    def test_known_extracted_install_auto_detected(self):
        with tempfile.TemporaryDirectory() as folder:
            launcher=Path(folder)/'isaac-sim-standalone-6.2.0-windows-x86_64'/'isaac-sim.bat'
            launcher.parent.mkdir();launcher.write_text('@echo off')
            self.assertEqual(find_launcher(roots=[folder]),str(launcher))
    def test_saved_valid_path_preferred_and_invalid_path_falls_back(self):
        with tempfile.TemporaryDirectory() as folder:
            launcher=Path(folder)/'IsaacSim'/'isaac-sim.bat';launcher.parent.mkdir();launcher.write_text('@echo off')
            self.assertEqual(find_launcher('missing/isaac-sim.bat',roots=[folder]),str(launcher))
            self.assertEqual(find_launcher(str(launcher),roots=[]),str(launcher))
    def test_missing_install_does_not_select_nonexistent_path(self):
        with tempfile.TemporaryDirectory() as folder:self.assertEqual(find_launcher(roots=[folder]),'')
