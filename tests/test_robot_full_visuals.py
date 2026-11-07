import unittest
from unittest.mock import patch
from types import SimpleNamespace as NS
from seer_control.geometry3d import identity,RobotDescription,load_glb_static_robot

class FullVisualTests(unittest.TestCase):
    def test_static_loader_preserves_all_faces_before_lod(self):
        face=((0.,0.,0.),(1.,0.,0.),(0.,1.,0.))
        mesh=NS(faces=[face]*20,face_materials=['m']*20,face_nodes=['body']*20,material_colors={'m':'#ffffff'},total=20)
        with patch('seer_control.geometry3d.load_glb',return_value=mesh) as load:
            asset=load_glb_static_robot('robot.glb',max_faces=2)
        self.assertIsNone(load.call_args.kwargs['max_faces'])
        self.assertEqual(sum(map(len,asset.full_visuals.values())),20)
        self.assertEqual(len(asset.draw_faces({},full_resolution=True)),20)

    def test_full_software_and_legacy_lod_are_separate(self):
        face=((0.,0.,0.),(1.,0.,0.),(0.,1.,0.))
        asset=RobotDescription('test',{'base':[([face],identity(),'#ffffff')]},[],['base'])
        asset.full_visuals={('base',0):[face,face]}
        self.assertEqual(len(asset.draw_faces({})),1)
        self.assertEqual(len(asset.draw_faces({},full_resolution=True)),2)
