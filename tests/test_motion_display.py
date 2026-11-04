import math
import unittest
from seer_control.motion_display import MotionDisplay

class MotionDisplayTests(unittest.TestCase):
    def test_bounded_convergence_and_no_overshoot(self):
        f=MotionDisplay();f.update({'x':0},0)
        first=f.update({'x':.5},.033)['x']
        self.assertGreater(first,0);self.assertLess(first,.5)
        for i in range(2,21):value=f.update({'x':.5},i*.033)['x']
        self.assertAlmostEqual(value,.5,places=3)
    def test_heading_wrap_takes_short_arc(self):
        f=MotionDisplay();f.update({'theta':math.radians(179)},0)
        value=f.update({'theta':math.radians(-179)},.033,angular=('theta',))['theta']
        self.assertLess(abs(value-math.pi),.03)
    def test_relocation_gap_and_context_reset(self):
        f=MotionDisplay();f.update({'x':0},0,'a')
        self.assertEqual(f.update({'x':5},.033,'a')['x'],5)
        self.assertEqual(f.update({'x':5.1},1,'a')['x'],5.1)
        self.assertEqual(f.update({'x':5.2},1.033,'b')['x'],5.2)
