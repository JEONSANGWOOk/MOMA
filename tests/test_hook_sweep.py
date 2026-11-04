import unittest
from seer_control.door_feedback import SurfaceDoorFollower

class HookSweepTests(unittest.TestCase):
    def follower(self):
        face=((.8,0,.5),(1.2,0,.5),(1,0,1.5))
        return SurfaceDoorFollower([dict(name='right',pivot=(0,0,0),sign=1,handle=(99,99,99),
            surface_faces=[face],hook_faces=[face],hook_open_only=True,hook_dwell=.4)])
    def test_crossing_between_frames_opens_door(self):
        f=self.follower()
        f.update_contacts((1,-.04,1),None,0)
        f.update_contacts((1,.04,1),None,.1)
        self.assertGreater(f.angles['right'],.03)
        self.assertEqual(f.active,'right')
    def test_gap_or_missed_surface_does_not_open(self):
        for tip,stamp in [((1,.04,1),1),((2,.04,1),.1)]:
            f=self.follower();f.update_contacts((1,-.04,1),None,0)
            f.update_contacts(tip,None,stamp)
            self.assertEqual(f.angles['right'],0)

    def test_contact_then_next_sample_opens_immediately_without_dwell(self):
        import math
        f=self.follower();f.doors['right']['hook_dwell']=0.
        f.update_contacts((1,0,1),None,0)
        self.assertEqual(f.active,'right')
        f.update_contacts((math.cos(.04),math.sin(.04),1),None,.05)
        self.assertAlmostEqual(f.angles['right'],.04)

    def test_sliding_along_frame_keeps_opening_after_seven_cm(self):
        import math
        f=self.follower();f.doors['right']['hook_dwell']=0.
        f.update_contacts((1,0,1),None,0)
        for i in range(1,21):
            a=i*.025;r=1-i*.005
            f.update_contacts((r*math.cos(a),r*math.sin(a),1),None,i*.1)
        self.assertEqual(f.active,'right')
        self.assertAlmostEqual(f.angles['right'],.5)
    def test_leaving_frame_releases_without_inventing_more_opening(self):
        f=self.follower();f.doors['right']['hook_dwell']=0.
        f.update_contacts((1,0,1),None,0)
        f.update_contacts((1,0,1.7),None,.5)
        self.assertIsNone(f.active)
        self.assertEqual(f.angles['right'],0)

    def test_right_requires_inside_then_exit(self):
        import math
        f=self.follower();d=f.doors['right'];d['hook_dwell']=0.;d['hook_inside_exit']=True;d['sign']=-1
        f.update_contacts((1,-.04,1),None,0)
        f.update_contacts((1,0,1),None,.1)
        self.assertIsNone(f.active)
        f.update_contacts((1,.04,1),None,.2)
        self.assertIsNone(f.active)
        f.update_contacts((1,.02,1),None,.3)
        self.assertEqual(f.active,'right')
        f.update_contacts((math.cos(.1),-math.sin(.1),1),None,.5)
        self.assertGreater(f.angles['right'],.08)

    def test_left_hook_hands_off_to_roller_without_reset(self):
        import math
        f=self.follower();d=f.doors['right'];d['hook_dwell']=0.
        f.update_contacts((1,0,1),None,0)
        f.update_contacts((math.cos(.2),math.sin(.2),1),None,.2)
        angle=f.angles['right']
        # Roller is just behind the already rotated surface.
        start=f._rotate('right',(1,-f.roller_radius,1),angle)
        end=f._rotate('right',(1,-f.roller_radius+.012,1),angle)
        f.update_contacts(None,start,.3)
        f.update_contacts(None,end,.4)
        self.assertEqual(f.contact_mode,'push')
        self.assertGreater(f.angles['right'],angle)
