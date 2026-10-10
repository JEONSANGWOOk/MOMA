import copy,unittest
from unittest.mock import Mock
from seer_control.hybrid_key_target import HybridKeyTarget
from tests import test_key_panel as fixtures


def record():return dict(timestamp=100,handle_target=dict(valid=True,confidence=.95,profile_revision='hole',board_revision='board',board_xyz_mm=[0.,100.,32.]))


class HybridTests(unittest.TestCase):
    def test_missing_stale_wrong_board_or_new_profile_cannot_advance(self):
        fusion=HybridKeyTarget();r=record();self.assertTrue(fusion.update(r,'board',100))
        for mode in ('missing','stale','board','profile','jump'):
            s=copy.deepcopy(r)
            if mode=='missing':s['handle_target']['valid']=False
            if mode=='stale':s['timestamp']=99
            if mode=='board':s['handle_target']['board_revision']='other'
            if mode=='profile':s['handle_target']['profile_revision']='other'
            if mode=='jump':s['handle_target']['board_xyz_mm'][0]=5
            self.assertFalse(fusion.update(s,'board',100),mode)
        self.assertEqual(fusion.offset,[0,100])
    def test_missing_hole_pauses_insert_and_turn_but_keeps_marker_tracking(self):
        case=fixtures.KeyTests();case.setUp();s=case.servo;s.start_key();s.require_visual_target=True;s.visual_target_ready=False;s.move_tip=Mock()
        s.stage='INSERT';s.distance=case.scene.mouth_mm+5
        r=case.scene.nominal();p=case.scene.world(case.scene.offset+[s.distance])
        pose=fixtures.pose_from_matrix(r,[p[i]-r[i][2]*case.tool.offset_mm for i in range(3)])
        case.sim.set_joints(case.sim.kin.ik(pose,case.sim.q,position_tolerance=.000002,rotation_tolerance=.0001))
        before=s.distance;case.frame()
        self.assertTrue(s.enabled);self.assertEqual(s.distance,before);s.move_tip.assert_called_once()
        s.visual_target_ready=True;case.frame();self.assertLess(s.distance,before)
        s.stage='TURN';s.turn=10;s.visual_target_ready=False;case.frame();self.assertEqual(s.turn,10)
        self.assertTrue(s.enabled);self.assertIn('보류',s.status)
    def test_retarget_preserves_panel_and_moves_handle_and_socket(self):
        case=fixtures.KeyTests();case.setUp();scene=case.scene
        panel=next(local for o,local,_ in scene.meshes if o['name']=='판넬 문');before=list(panel)
        scene.retarget([2,101]);scene.sync()
        self.assertEqual(panel,before);self.assertEqual(scene.offset,[2,101])
        f=scene.feedback();self.assertGreater(f['lateral_mm'],0)


if __name__=='__main__':unittest.main()
