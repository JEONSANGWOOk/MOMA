import math
import unittest
from seer_control.arrival_pose import arrival_pose

class ArrivalPoseTests(unittest.TestCase):
    def test_position_alone_is_not_enough(self):
        self.assertFalse(arrival_pose(dict(x=0,y=0,r=0),dict(x=0,y=0,theta=.1))['ok'])
    def test_heading_alone_is_not_enough(self):
        self.assertFalse(arrival_pose(dict(x=0,y=0,r=0),dict(x=.1,y=0,theta=0))['ok'])
    def test_both_and_wrapped_angle(self):
        self.assertTrue(arrival_pose(dict(x=0,y=0,r=math.pi-.005),dict(x=.01,y=0,theta=-math.pi+.005))['ok'])
    def test_missing_or_invalid_heading_rejected(self):
        self.assertFalse(arrival_pose(dict(x=0,y=0),dict(x=0,y=0,theta=0))['ok'])
        self.assertFalse(arrival_pose(dict(x=0,y=0,r=0),dict(x=0,y=0,theta=float('nan')))['ok'])
