import copy,json,math,tempfile,unittest
from pathlib import Path
from seer_control.photo_screen_stream import photo_record
from seer_control.live_servo_start import LiveServoStart
from tests import test_key_panel as fixtures
from tests import test_key_panel_real as real_fixtures
try:
    import cv2
    import numpy as np
    from seer_control.photo_screen_vision import PhotoScreenDetector
except ImportError:cv2=None


@unittest.skipUnless(cv2 is not None,'Requires the D455 vision environment')
class PhotoVisionTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory();self.addCleanup(self.directory.cleanup);self.root=Path(self.directory.name)
        self.reference=np.random.default_rng(11).integers(0,255,(400,480),dtype=np.uint8)
        cv2.circle(self.reference,(200,150),15,0,4)
        path=self.root/'reference.png';cv2.imwrite(str(path),self.reference)
        self.profile=self.root/'profile.json';self.profile.write_text(json.dumps(dict(version=1,revision='photo',reference_file=str(path),hole_center_px=[200,150],reference_px_per_mm=2.8)),encoding='utf-8')
        self.detector=PhotoScreenDetector(self.profile);self.camera=dict(matrix=[[580,0,320],[0,580,240],[0,0,1]],distortion=[0]*5,model='none')
    def view(self):
        src=np.float32([[0,0],[479,0],[479,399],[0,399]])
        points=np.column_stack(((src[:,0]-200)/2.8,-(src[:,1]-150)/2.8,np.zeros(4)))/1000
        dst=cv2.projectPoints(points,np.asarray([math.pi-.12,.03,.1]),np.asarray([-.02,.01,.25]),np.asarray(self.camera['matrix'],dtype=float),np.zeros(5))[0].reshape(-1,2).astype(np.float32)
        h=cv2.getPerspectiveTransform(src,dst);image=cv2.warpPerspective(self.reference,h,(640,480),borderValue=150)
        return image,cv2.perspectiveTransform(np.float32([[[200,150]]]),h)[0,0]
    def test_photo_scale_perspective_and_hole_center(self):
        image,expected=self.view()
        for i in range(3):r=self.detector.detect(image,self.camera,i)
        self.assertTrue(r['valid'],r);self.assertLess(math.dist(r['center_px'],expected),2)
        self.assertEqual(r['markers_used'],0);self.assertEqual(r['geometry_source'],'photo_screen_estimate')
    def test_blank_repeats_and_missing_reference_do_not_arm(self):
        image,_=self.view()
        for _ in range(5):r=self.detector.detect(image,self.camera,1)
        self.assertFalse(r['valid']);self.assertEqual(r['confirmation_frames'],1)
        self.assertFalse(self.detector.detect(np.full((480,640),150,np.uint8),self.camera,2)['valid'])
        self.profile.unlink();self.assertFalse(self.detector.detect(image,self.camera,3)['valid'])
    def test_supplied_product_photo_center(self):
        profile=Path(__file__).resolve().parents[1]/'.delivery/d455_photo_screen.json'
        if not profile.exists():self.skipTest('Private supplied product photograph is not in the repository')
        p=json.loads(profile.read_text(encoding='utf-8'));ref=cv2.imdecode(np.frombuffer(Path(p['reference_file']).read_bytes(),np.uint8),1)
        image=np.full((480,640,3),150,np.uint8);image[40:424,95:542]=cv2.resize(ref,(447,384));detector=PhotoScreenDetector(profile)
        for i in range(3):r=detector.detect(image,self.camera,i)
        expected=[95+p['hole_center_px'][0]*447/ref.shape[1],40+p['hole_center_px'][1]*384/ref.shape[0]]
        self.assertTrue(r['valid'],r);self.assertLess(math.dist(r['center_px'],expected),2)


class PhotoControlTests(unittest.TestCase):
    def setUp(self):
        self.case=fixtures.KeyTests();self.case.setUp()
        self.record=dict(self.case.record,vision_source='photo_screen_sim',board=dict(self.case.record['board'],markers_used=0,geometry_source='photo_screen_estimate'))
    def test_photo_requires_explicit_sim_mode_and_has_no_markers(self):
        m=self.case.m;m.inspect(self.record,now=self.record['timestamp']);self.assertIsNone(m.latest)
        m.allow_photo_screen=True;m.inspect(self.record,now=self.record['timestamp']);self.assertIsNotNone(m.latest)
        r=photo_record(dict(timestamp=self.record['timestamp'],photo_target=self.record['board']),self.record['board']['revision'])
        self.assertEqual(r['board']['markers_used'],0);gate=LiveServoStart();gate.request('SIM')
        self.assertTrue(gate.tick(r,True,lambda:None,now=r['timestamp']))
        with self.assertRaises(ValueError):gate.request('REAL')
    def test_photo_is_rejected_by_real_even_if_metadata_is_forged(self):
        r=copy.deepcopy(self.record);r['board'].update(geometry_source='measured',markers_used=2,revision='measured')
        with self.assertRaises(ValueError):real_fixtures.socket_frame(real_fixtures.calibrated(),r,now=r['timestamp'])
    def test_changed_photo_revision_is_invalid(self):
        r=photo_record(dict(timestamp=100,photo_target=self.record['board']),'other')
        self.assertFalse(r['board']['valid']);self.assertFalse(r['raw_board']['valid'])

    def test_photo_sim_insert_turn_and_auto_retract(self):
        case=self.case;case.record=self.record;case.m.allow_photo_screen=True
        case.scene.retarget([0,0]);case.cycle()
        self.assertTrue(case.servo.completed);self.assertEqual(case.servo.stage,'HOLD')
        self.assertEqual(case.servo.errors['vision_source'],'photo_screen_sim')


if __name__=='__main__':unittest.main()
