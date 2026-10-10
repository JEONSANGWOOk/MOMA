import json,tempfile,unittest,math
from pathlib import Path
from seer_control.panel_handle_model import settings,key_meshes,handle_meshes,tube
from tests import test_key_panel as fixtures
from seer_control.aruco_board import pose_from_matrix


class HandleModelTests(unittest.TestCase):
    def test_tube_end_has_an_open_center(self):
        faces=tube(.004,.003,.03)
        front=[f for f in faces if all(abs(p[2])<1e-12 for p in f)]
        self.assertTrue(front)
        self.assertTrue(all(math.hypot(p[0],p[1])>=.003-1e-10 for f in front for p in f))

    def test_handle_and_key_are_shared_photo_model(self):
        c=settings();self.assertEqual(c['dimension_source'],'photo_estimate')
        self.assertIn('chrome_tapered_handle',[name for _,_,name in handle_meshes(c)])
        self.assertIn('hollow_tubular_key',[name for _,_,name in key_meshes(c)])
        self.assertNotIn('key_blade',[name for _,_,name in key_meshes(c)])

    def test_incompatible_annular_dimensions_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'model.json';path.write_text(json.dumps(dict(key_inner_radius_mm=5)),encoding='utf-8')
            with self.assertRaises(ValueError):settings(path)

    def test_inserted_tube_clears_center_pin_but_off_axis_is_blocked(self):
        case=fixtures.KeyTests();case.setUp();s=case.scene;r=s.nominal()
        for lateral,blocked in [(0,False),(1.5,True)]:
            p=s.world([s.offset[0]+lateral,s.offset[1],s.mouth_mm-s.insertion_mm])
            q=case.sim.kin.ik(pose_from_matrix(r,[p[i]-r[i][2]*case.tool.offset_mm for i in range(3)]),case.sim.q)
            risks,_=case.sim.physics.contacts(q)
            bore=[pair for pair in risks if pair[0]==case.tool.key['name'] and '슬롯' in pair[1]]
            self.assertEqual(bool(bore),blocked,risks)


if __name__=='__main__':unittest.main()
