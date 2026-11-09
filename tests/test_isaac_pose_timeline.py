import importlib.util
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('moma_pose_timeline_test',Path(__file__).resolve().parents[1]/'tools/isaac_mirror/pose_timeline.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
PoseTimeline=module.PoseTimeline

class PoseTimelineTests(unittest.TestCase):
    def test_shared_time_for_robot_tool_and_door(self):
        b=PoseTimeline(.08)
        b.push(1.,dict(robot=0,tool=0,door=0))
        b.push(1.04,dict(robot=4,tool=4,door=4))
        before,after,a=b.sample(1.1)
        self.assertAlmostEqual(a,.5)
        self.assertEqual([before[k]*(1-a)+after[k]*a for k in before],[2.,2.,2.])
    def test_lost_packets_freeze_instead_of_extrapolating(self):
        b=PoseTimeline();b.push(1.,{'robot':1});b.push(1.03,{'robot':2})
        self.assertEqual(b.sample(2.),({'robot':2},{'robot':2},1.))
    def test_reconnect_clears_old_pose(self):
        b=PoseTimeline();b.push(1.,{'robot':1});b.push(2.,{'robot':99})
        self.assertEqual(b.sample(2.),({'robot':99},{'robot':99},1.))
    def test_reset_clears_all_samples(self):
        b=PoseTimeline();b.push(1.,{'robot':1});b.clear()
        self.assertEqual(b.sample(1.),({},{},1.))
    def test_irregular_packets_have_monotonic_interpolation(self):
        b=PoseTimeline(.08)
        for t in (1.,1.035,1.08,1.12):b.push(t,{'robot':t})
        values=[]
        for now in (1.08,1.09,1.1,1.11,1.12,1.13,1.14):
            a,z,w=b.sample(now);values.append(a['robot']*(1-w)+z['robot']*w)
        self.assertEqual(values,sorted(values));self.assertLessEqual(max(values),1.12)
