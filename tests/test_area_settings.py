import unittest
from seer_control.area_settings import parse_properties,LIMITS
class AreaTests(unittest.TestCase):
 def test_empty_and_limits(self):
  self.assertEqual(parse_properties({'maxspeed':''}),{})
  for k,(lo,hi,_,_) in LIMITS.items():
   self.assertEqual(parse_properties({k:str(hi)})[k],hi)
   for v in [str(lo-1),str(hi+1),'nan','inf','abc']:
    with self.assertRaises(ValueError):parse_properties({k:v})
 def test_integer_and_distance(self):
  with self.assertRaises(ValueError):parse_properties({'collisionPointThreshold':'1.5'})
  with self.assertRaises(ValueError):parse_properties({'obsDecDist':'0.1','obsStopDist':'0.2'})
