import math
import struct
import tempfile
import unittest
from pathlib import Path
from seer_control.geometry3d import RobotDescription, load_mesh, transform, point
from seer_control.world3d import Camera, PRESETS


class GeometryTests(unittest.TestCase):
    def write(self, directory, name, content):
        path=Path(directory)/name;path.write_text(content,encoding='utf-8');return path

    def test_transform_rotation_then_translation(self):
        p=point(transform((3,4,5),(0,0,math.pi/2)),(1,0,0))
        for actual,expected in zip(p,(3,5,5)):self.assertAlmostEqual(actual,expected)

    def test_joint_origin_axis_limit_and_mimic(self):
        with tempfile.TemporaryDirectory() as d:
            path=self.write(d,'robot.urdf','''<robot name="test"><link name="base"/><link name="slide"/><link name="tool"/>
            <joint name="slide" type="prismatic"><parent link="base"/><child link="slide"/><origin xyz="1 0 0"/><axis xyz="0 0 2"/><limit lower="0" upper="2"/></joint>
            <joint name="rotate" type="continuous"><parent link="slide"/><child link="tool"/><axis xyz="0 0 1"/><mimic joint="slide" multiplier="0.5"/></joint></robot>''')
            asset=RobotDescription.load(path);poses=asset.link_transforms({'slide':10})
            self.assertEqual(point(poses['slide'],(0,0,0)),(1,0,2))
            p=point(poses['tool'],(1,0,0))
            self.assertAlmostEqual(p[0],1+math.cos(1));self.assertAlmostEqual(p[1],math.sin(1))
            self.assertEqual(len(asset.movable()),1)

    def test_obj_negative_indices_and_local_scale(self):
        with tempfile.TemporaryDirectory() as d:
            path=self.write(d,'mesh.obj','v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf -4 -3 -2 -1\n')
            mesh=load_mesh(path);self.assertEqual(len(mesh.faces),2)
            faces=mesh.draw_faces(transform((10,0,0)),(.001,)*3)
            self.assertAlmostEqual(faces[0][0][1][0],10.001)

    def test_ascii_and_binary_stl(self):
        with tempfile.TemporaryDirectory() as d:
            ascii=self.write(d,'ascii.stl','solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid t')
            binary=Path(d)/'binary.stl';binary.write_bytes(b' '*80+struct.pack('<I',1)+struct.pack('<12fH',0,0,1,0,0,0,1,0,0,0,1,0,0))
            self.assertEqual(load_mesh(ascii).faces,load_mesh(binary).faces)

    def test_package_mesh_and_missing_visual(self):
        with tempfile.TemporaryDirectory() as d:
            folder=Path(d)/'pkg';folder.mkdir();self.write(folder,'mesh.obj','v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3')
            path=self.write(d,'robot.urdf','<robot name="test"><link name="base"><visual><geometry><mesh filename="package://pkg/mesh.obj" scale="2 2 2"/></geometry></visual><visual><geometry><mesh filename="missing.stl"/></geometry></visual></link></robot>')
            asset=RobotDescription.load(path,d);self.assertEqual(len(asset.draw_faces({})),1)
            self.assertEqual(asset.draw_faces({})[0][0][1],(2,0,0));self.assertEqual(len(asset.warnings),1)

    def test_invalid_xml_and_axis(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ValueError):RobotDescription.load(self.write(d,'bad.urdf','<robot>'))
            path=self.write(d,'axis.urdf','<robot><link name="a"/><link name="b"/><joint name="j" type="continuous"><parent link="a"/><child link="b"/><axis xyz="0 0 0"/></joint></robot>')
            with self.assertRaises(ValueError):RobotDescription.load(path)

    def test_camera_presets_projection(self):
        camera=Camera()
        for yaw,pitch in PRESETS.values():
            camera.yaw,camera.pitch=yaw,pitch
            for perspective in (True,False):
                camera.perspective=perspective;p=camera.project(camera.target,800,600)
                self.assertAlmostEqual(p[0],400);self.assertAlmostEqual(p[1],300)
                self.assertTrue(all(math.isfinite(v) for v in p))

    def test_demo_arm_fk_changes_tool(self):
        asset=RobotDescription.load(Path(__file__).resolve().parents[1]/'examples/demo_mobile_arm.urdf')
        self.assertEqual(len(asset.movable()),6)
        self.assertNotEqual(asset.link_transforms({})['tool'],asset.link_transforms({'joint_2':1})['tool'])

    def test_ground_projection_roundtrip(self):
        camera=Camera()
        for yaw,pitch in PRESETS.values():
            camera.yaw,camera.pitch=yaw,pitch
            for perspective in (True,False):
                camera.perspective=perspective
                screen=camera.project((1.2,-.8,0),800,600)
                ground=camera.ground(*screen[:2],800,600)
                self.assertAlmostEqual(ground[0],1.2);self.assertAlmostEqual(ground[1],-.8)

    def test_default_seer_fr5_models(self):
        folder=Path(__file__).resolve().parents[1]/'examples'
        amr=RobotDescription.load(folder/'seer_amb_csw04_ce.urdf')
        arm=RobotDescription.load(folder/'fairino_fr5.urdf')
        self.assertFalse(amr.warnings);self.assertFalse(arm.warnings)
        self.assertEqual(len(arm.movable()),6)
        self.assertTrue(amr.name.startswith('SEER AMB-CSW04-CE'))
        self.assertTrue(arm.name.startswith('FAIRINO FR5'))
        base=amr.draw_faces({})[0][0]
        self.assertAlmostEqual(max(p[0] for p in base)-min(p[0] for p in base),.9582)
        self.assertAlmostEqual(max(p[1] for p in base)-min(p[1] for p in base),.6314)
        safe=dict(zip(('joint_'+str(i) for i in range(1,7)),(0,-1.57,1.57,-1.57,-1.57,0)))
        self.assertTrue(all(p[2]>-.001 for face,_,_ in arm.draw_faces(safe) for p in face))
