import unittest
from seer_control.equipment_placement import polygon_gap,placed
class EquipmentGapTests(unittest.TestCase):
    def test_known_21cm_gap(self):
        square=[(0,0),(1,0),(1,1),(0,1)]
        self.assertAlmostEqual(polygon_gap(square,placed(square,(1.21,0,0))),.21)
    def test_overlap_is_zero(self):
        square=[(0,0),(1,0),(1,1),(0,1)]
        self.assertEqual(polygon_gap(square,placed(square,(.5,0,0))),0)
    def test_distance_invariant_under_common_rotation(self):
        square=[(0,0),(1,0),(1,1),(0,1)]
        other=placed(square,(1.21,0,0))
        self.assertAlmostEqual(polygon_gap(placed(square,(3,4,.7)),placed(other,(3,4,.7))),.21)
