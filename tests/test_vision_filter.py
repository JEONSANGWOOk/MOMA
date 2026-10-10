import math
import unittest
from seer_control.vision_filter import VisionFilter
from seer_control.aruco_arm_follow import rotation


class FilterTests(unittest.TestCase):
    def test_static_noise_is_held(self):
        filt=VisionFilter();filt.vector(('p',),[0,0,.6],0)
        values=[filt.vector(('p',),[0,0,.6+(1 if i%2 else -1)*.0002],i/30) for i in range(1,90)]
        self.assertTrue(all(v==[0,0,.6] for v in values))

    def test_single_outlier_rejected_but_real_motion_followed(self):
        filt=VisionFilter()
        for i in range(10):filt.vector(('p',),[0,0,.6],i/30)
        self.assertEqual(filt.vector(('p',),[0,0,.8],10/30),[0,0,.6])
        for i in range(11,50):value=filt.vector(('p',),[0,0,.65],i/30)
        self.assertGreater(value[2],.648)

    def test_pi_axis_sign_wrap_does_not_average_into_identity(self):
        filt=VisionFilter()
        for i in range(60):
            value=filt.rotation(('r',),[math.pi*(1 if i%2 else -1),0,0],i/30)
            matrix=rotation(value)
            self.assertAlmostEqual(matrix[1][1],-1.,places=6)

    def test_missing_pose_and_new_revision_clear_previous_values(self):
        filt=VisionFilter();orientation=lambda r:{'value':r}
        board=dict(valid=True,revision='a',camera_xyz_m=[0,0,.6],rotation_vector_rad=[math.pi,0,0])
        filt.board(board,0,orientation)
        invalid=filt.board(dict(valid=False,revision='a'),.1,orientation)
        self.assertFalse(invalid['valid'])
        board['camera_xyz_m']=[0,0,.8];self.assertEqual(filt.board(board,.2,orientation)['camera_xyz_m'][2],.8)
        board['revision']='b';board['camera_xyz_m']=[0,0,.9]
        self.assertEqual(filt.board(board,.3,orientation)['camera_xyz_m'][2],.9)

    def test_raw_input_unchanged_and_gap_resets_filter(self):
        filt=VisionFilter();marker=dict(dictionary='DICT_4X4_1000',id=0,camera_xyz_m=[0,0,.6],rotation_vector_rad=[math.pi,0,0])
        filt.markers([marker],0,lambda r:{})
        self.assertNotIn('orientation_deg',marker)
        filt.markers([], .1,lambda r:{})
        marker['camera_xyz_m']=[0,0,.8]
        self.assertEqual(filt.markers([marker],.2,lambda r:{})[0]['camera_xyz_m'][2],.8)
        self.assertEqual(filt.vector(('gap',),[0,0,.5],0),[0,0,.5])
        self.assertEqual(filt.vector(('gap',),[0,0,.7],1),[0,0,.7])


if __name__=='__main__':unittest.main()
