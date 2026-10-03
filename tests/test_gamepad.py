import unittest
from seer_control.gamepad import PadGate,Sample,deadzone,jog_packet,WindowsPad


class PadTests(unittest.TestCase):
    def sample(self,axes=None,buttons=0,stamp=10,identity=('sony',)):
        return Sample(axes or {'X':0.,'Y':0.},buttons,stamp,identity)
    def armed(self):
        gate=PadGate();gate.evaluate(self.sample(),10,True);return gate
    def test_neutral_and_release_required_on_connect(self):
        gate=PadGate()
        self.assertEqual(gate.evaluate(self.sample({'X':0,'Y':-1},16),10,True)[:2],(0,0))
        gate.evaluate(self.sample(),10,True)
        self.assertEqual(gate.evaluate(self.sample({'X':0,'Y':-1},16),10,True)[:2],(1,0))
    def test_deadzones_and_mixed_motion(self):
        self.assertEqual(deadzone(.1,.15),0)
        v,w,_,_=self.armed().evaluate(self.sample({'X':1,'Y':-1},16),10,True)
        self.assertEqual((v,w),(1,-1))
        self.assertEqual(self.armed().evaluate(self.sample({'X':.1,'Y':-.1},16),10,True)[:2],(0,0))
    def test_release_stops(self):
        self.assertEqual(self.armed().evaluate(self.sample({'X':1,'Y':-1},0),10,True)[:2],(0,0))
    def test_disconnect_stale_focus_and_disabled(self):
        for sample,now,allowed in ((None,10,True),(self.sample(),11,True),(self.sample(),10,False)):
            gate=self.armed();self.assertEqual(gate.evaluate(sample,now,allowed)[:2],(0,0));self.assertFalse(gate.armed)
    def test_stop_disarms(self):
        gate=self.armed();result=gate.evaluate(self.sample({'X':1,'Y':-1},20),10,True)
        self.assertEqual(result[:3],(0,0,True));self.assertFalse(gate.armed)
    def test_hotplug_does_not_resume(self):
        gate=self.armed();self.assertEqual(gate.evaluate(self.sample({'X':1,'Y':-1},16,identity=('other',)),10,True)[:2],(0,0))
    def test_poll_gap_requires_rearming(self):
        gate=self.armed();self.assertEqual(gate.evaluate(self.sample({'X':1,'Y':-1},16,stamp=11),11,True)[:2],(0,0))
    def test_wrong_mapping_blocks(self):
        for config in ({'forward':'U'},{'turn':'Y'},{'deadman':2},{'zone':.9}):
            self.assertEqual(self.armed().evaluate(self.sample({'X':1,'Y':-1},16),10,True,**config)[:2],(0,0))
    def test_axis_inversion(self):
        result=self.armed().evaluate(self.sample({'X':1,'Y':1},16),10,True,invert_forward=False,invert_turn=False)
        self.assertEqual(result[:2],(1,1))
    def test_jog_lease_strips_metadata_and_expires(self):
        command=dict(vx=.1,vy=0,w=.2,_expires=10.25)
        self.assertEqual(jog_packet(command,10),dict(vx=.1,vy=0,w=.2))
        self.assertIsNone(jog_packet(command,10.3));self.assertIsNone(jog_packet(None,10))
        self.assertEqual(jog_packet(dict(vx=0,vy=0,w=.2),100),dict(vx=0,vy=0,w=.2))
