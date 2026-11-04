import unittest
from seer_control.placement_nudge import nudged

class NudgeTests(unittest.TestCase):
    def test_millimetre_and_degree_steps(self):
        self.assertEqual(nudged('1.2','0.001',1),'1.201000')
        self.assertEqual(nudged('-1.2','0.005',-1),'-1.205000')
        self.assertEqual(nudged('90','0.1',-1),'89.900000')
    def test_invalid_inputs(self):
        for value,step in [('nan','1'),('1','inf'),('1','0'),('1','-1'),('abc','1')]:
            with self.assertRaises(ValueError):nudged(value,step,1)
