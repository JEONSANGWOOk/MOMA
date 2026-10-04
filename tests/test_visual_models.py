import unittest,math,json
from pathlib import Path
from types import SimpleNamespace
from seer_control.geometry3d import RobotDescription
from seer_control.pose_display import real_arm_positions
from seer_control.smooth_renderer import smooth_normals
ROOT=Path(__file__).resolve().parents[1]
class VisualModels(unittest.TestCase):
 def test_sba_model_frames_and_materials(self):
  asset=RobotDescription.load(ROOT/'models/seer_sba400eu_description/urdf/sba400eu.urdf')
  self.assertEqual(asset.name,'SEER SBA-400EU')
  self.assertIn('arm_mount_link',asset.links)
  poses=asset.link_transforms({})
  self.assertAlmostEqual(poses['arm_mount_link'][2][3],.182)
  for name in ('lidar_1_link','lidar_2_link'):self.assertAlmostEqual(poses[name][2][3],.1965)
  colors={color for rows in asset.links.values() for faces,local,color in rows}
  self.assertGreaterEqual(len(colors),7)
 def test_estimated_extrinsics_not_measured(self):
  data=json.loads((ROOT/'models/seer_sba400eu_description/config/model.json').read_text())
  self.assertEqual(data['dimensions_m'],[.9582,.6314,.182])
  self.assertFalse(data['factory_cad'])
  self.assertTrue(all(not p['verified'] for p in data['extrinsics']))
 def test_fresh_real_feedback_only(self):
  joints=[dict(name='j'+str(i)) for i in range(6)]
  app=SimpleNamespace(fr5_client=SimpleNamespace(connected=True),fr5_feedback=dict(joints_rad=[.2]*6),fr5_rx=10)
  self.assertEqual(real_arm_positions(app,joints,11)['j0'],.2)
  self.assertIsNone(real_arm_positions(app,joints,13))
  app.fr5_client.connected=False;self.assertIsNone(real_arm_positions(app,joints,10))
  app.fr5_client.connected=True;app.fr5_feedback['joints_rad'][0]=math.nan;self.assertIsNone(real_arm_positions(app,joints,10))
 def test_normals_preserve_sharp_edges(self):
  faces=[((0,0,0),(1,0,0),(0,1,0)),((0,0,0),(0,0,1),(1,0,0))]
  normals=smooth_normals(faces)
  self.assertAlmostEqual(normals[0][0][2],1)
  self.assertAlmostEqual(normals[1][0][1],1)
