import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
from seer_control.app import Console as App

class AutoMapRecoveryTests(unittest.TestCase):
    def view(self):
        return NS(real=True,connected=True,_auto_map_pending=True,_auto_map_retry_at=0.,
                  downloading_map=False,loading_map=False,map_dirty=False,
                  pull_robot_map=Mock(),log=Mock())
    def test_recovers_current_map_and_throttles_retries(self):
        view=self.view()
        App._auto_refresh_recovered_map(view,10.)
        view.pull_robot_map.assert_called_once_with(refresh_current=True)
        App._auto_refresh_recovered_map(view,11.)
        self.assertEqual(view.pull_robot_map.call_count,1)
        App._auto_refresh_recovered_map(view,15.)
        self.assertEqual(view.pull_robot_map.call_count,2)
    def test_does_not_replace_unsaved_edits(self):
        view=self.view();view.map_dirty=True
        App._auto_refresh_recovered_map(view,10.)
        view.pull_robot_map.assert_not_called()
        self.assertFalse(view._auto_map_pending)
    def test_no_duplicate_download_or_download_while_disconnected(self):
        for key,value in [('connected',False),('downloading_map',True),('loading_map',True),('_auto_map_pending',False),('real',False)]:
            view=self.view();setattr(view,key,value)
            App._auto_refresh_recovered_map(view,10.)
            view.pull_robot_map.assert_not_called()
