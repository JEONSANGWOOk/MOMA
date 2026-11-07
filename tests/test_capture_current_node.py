import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
from seer_control.app import Console

class CaptureNodeTests(unittest.TestCase):
    def view(self):
        return NS(editable=Mock(),task_running=False,held=None,real=False,
            current_state=Mock(return_value=dict(x=1.2,y=-.3,theta=.7)),
            map=NS(nodes={}),_next_lm_id=Mock(return_value='LM3'),
            selected=None,target=Mock(),refresh_nodes=Mock(),draw_map=Mock(),log=Mock(),
            open_point_properties=Mock(),_sync_paths_after_point_edit=Mock())
    def test_capture_preserves_heading_and_does_not_upload(self):
        view=self.view()
        with patch('tkinter.simpledialog.askstring',return_value='LM3'):
            Console.capture_current_node(view)
        node=view.map.nodes['LM3']
        self.assertEqual((node['x'],node['y'],node['r']),(1.2,-.3,.7))
        self.assertTrue(view.map_dirty)
    def test_cancel_leaves_no_draft(self):
        view=self.view()
        with patch('tkinter.simpledialog.askstring',return_value=None):Console.capture_current_node(view)
        self.assertFalse(view.map.nodes)
    def test_replace_updates_path_endpoints(self):
        view=self.view();view.selected='LM1';view.map.nodes['LM1']=dict(id='LM1',x=0,y=0)
        with patch('seer_control.app.messagebox.askyesno',return_value=True):Console.capture_current_node(view,True)
        view._sync_paths_after_point_edit.assert_called_once_with('LM1','LM1',0,0,1.2,-.3)
    def test_stale_real_pose_rejected(self):
        view=self.view();view.real=True;view.connected=True;view.last_state=0
        with patch('seer_control.app.messagebox.showwarning') as warning:
            Console.capture_current_node(view)
            warning.assert_called_once()
        self.assertFalse(view.map.nodes)

    def test_pose_loader_returns_all_three_coordinates(self):
        view=self.view()
        self.assertEqual(Console.current_pose_for_node(view),(1.2,-.3,.7))
    def test_pose_loader_rejects_nan_theta(self):
        view=self.view();view.current_state.return_value['theta']=float('nan')
        with self.assertRaises(ValueError):Console.current_pose_for_node(view)
