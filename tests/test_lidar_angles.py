import math
import unittest
from seer_control.live import extract_laser_scans, extract_laser_points


class LidarAngleTests(unittest.TestCase):
    def test_degree_scan_keeps_near_zero_beams_in_their_sector(self):
        angles=[-120,-6.5,-5,-1,0,.5,5,6.5,119.5]
        beams=[dict(angle=a,dist=2,valid=True) for a in angles]
        payload={'lasers':[{'beams':beams,'range_max':10}]}
        scan=extract_laser_scans(payload)[0]
        self.assertEqual(len(scan['points']),len(angles))
        for (x,y),angle in zip(scan['points'],angles):
            self.assertAlmostEqual(x,2*math.cos(math.radians(angle)))
            self.assertAlmostEqual(y,2*math.sin(math.radians(angle)))
        fallback,_=extract_laser_points(payload)
        for expected,actual in zip(scan['points'],fallback):
            for a,b in zip(expected,actual):self.assertAlmostEqual(a,b)

    def test_radian_and_explicit_small_degree_scan(self):
        for unit,angles in [('rad',[-1,0,1]),('deg',[-5,0,5])]:
            scan=extract_laser_scans({'lasers':[dict(angle_unit=unit,range_max=10,
                  beams=[dict(angle=a,dist=2) for a in angles])]},pose=(3,4,.2))[0]
            for (x,y),a in zip(scan['points'],angles):
                angle=(a if unit=='rad' else math.radians(a))+.2
                self.assertAlmostEqual(x,3+2*math.cos(angle))
                self.assertAlmostEqual(y,4+2*math.sin(angle))

    def test_invalid_and_no_return_beams_stay_filtered(self):
        scan=extract_laser_scans({'lasers':[dict(range_max=10,beams=[
            dict(angle=-120,dist=10),dict(angle=0,dist=2,valid=False),dict(angle=5,dist=1)])]})[0]
        self.assertEqual(len(scan['points']),1)
        self.assertAlmostEqual(scan['points'][0][1],math.sin(math.radians(5)))


if __name__=='__main__':unittest.main()
