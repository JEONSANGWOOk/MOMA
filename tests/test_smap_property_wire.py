import unittest,base64
from seer_control.smap import _set_property
class WireTests(unittest.TestCase):
 def test_float_updates_encoded_value_and_clears_old_type(self):
  p=_set_property([{'key':'maxspeed','type':'int32','int32Value':0,'value':'MA=='}],'maxspeed',.2)[0]
  self.assertNotIn('int32Value',p);self.assertEqual(p['doubleValue'],.2)
  self.assertEqual(base64.b64decode(p['value']).decode(),'0.2')
 def test_integral_edit_preserves_original_integer_type(self):
  p=_set_property([{'key':'maxspeed','type':'int32','int32Value':0,'value':'MA=='}],'maxspeed',2.)[0]
  self.assertEqual(p['type'],'int32');self.assertEqual(p['int32Value'],2)
  self.assertEqual(base64.b64decode(p['value']).decode(),'2')
 def test_string_encoding(self):
  p=_set_property([],'name','LM1')[0]
  self.assertEqual(base64.b64decode(p['value']).decode(),'LM1')
