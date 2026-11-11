import math
import unittest
from seer_control.manual_relocation_ui import pose_payload

class ManualRelocationTests(unittest.TestCase):
    def test_xy_metres_and_heading_degrees_to_radians(self):
        p=pose_payload('1.25','-2.5','90')
        self.assertEqual((p['x'],p['y']),(1.25,-2.5));self.assertAlmostEqual(p['angle'],math.pi/2)
    def test_equivalent_angles_are_normalized(self):
        self.assertAlmostEqual(pose_payload(0,0,450)['angle'],math.pi/2)
    def test_invalid_or_nonfinite_values_cannot_make_a_payload(self):
        for value in ('nan','inf','-inf','bad',''):
            for index in range(3):
                args=[0,0,0];args[index]=value
                with self.assertRaises(ValueError):pose_payload(*args)
