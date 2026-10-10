import base64,copy,json,math,tempfile,unittest
from pathlib import Path
try:
    import cv2
    import numpy as np
    from seer_control.handle_vision import HandleDetector,project,plane_point
except ImportError:cv2=None


@unittest.skipUnless(cv2 is not None,'Requires the D455 vision environment')
class HandleVisionTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory();self.addCleanup(self.directory.cleanup)
        self.camera=dict(matrix=[[580.,0,320],[0,580.,240],[0,0,1]],distortion=[0]*5,model='none')
        self.board=dict(valid=True,markers_used=2,revision='board',camera_xyz_m=[0.,0.,.6],rotation_vector_rad=[math.pi,0.,0.])
        self.texture=np.random.default_rng(7).integers(40,210,(61,61),dtype=np.uint8)
        cv2.circle(self.texture,(30,30),10,20,3)
        image=self.render(self.board)
        _,jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,95])
        self.path=Path(self.directory.name)/'profile.json'
        self.path.write_text(json.dumps(dict(version=1,revision='handle',center_px=[320,240],surface_z_mm=32,
            image_jpeg_base64=base64.b64encode(jpeg).decode(),board=self.board,camera=self.camera)),encoding='utf-8')
        self.detector=HandleDetector(self.path)
    def render(self,board,offset=(0,0)):
        x,y=offset;corners=project([[x-30,y-30,32],[x+30,y-30,32],[x+30,y+30,32],[x-30,y+30,32]],board,self.camera)
        h=cv2.getPerspectiveTransform(np.float32([[0,0],[60,0],[60,60],[0,60]]),corners.astype(np.float32))
        patch=cv2.warpPerspective(self.texture,h,(640,480));mask=cv2.warpPerspective(np.full((61,61),255,np.uint8),h,(640,480))
        result=np.full((480,640),170,np.uint8);result[mask>0]=patch[mask>0]
        return cv2.cvtColor(result,cv2.COLOR_GRAY2BGR)
    def test_marker_movement_and_tilt_preserve_actual_hole_target(self):
        b=copy.deepcopy(self.board);b['camera_xyz_m'][0]=.025;b['rotation_vector_rad'][0]-=.10
        image=self.render(b,(2,-1))
        for i in range(3):result=self.detector.detect(image,dict(timestamp=i,raw_board=b),self.camera)
        self.assertTrue(result['valid'],result);self.assertGreater(result['confidence'],.78)
        self.assertLess(math.dist(result['board_xyz_mm'][:2],[2,-1]),1.5)
        expected=project([[2,-1,32]],b,self.camera)[0]
        self.assertLess(math.dist(result['center_px'],expected),1.6)
    def test_unrelated_image_and_marker_loss_cannot_supply_hole(self):
        image=np.full((480,640,3),170,np.uint8)
        result=self.detector.detect(image,dict(timestamp=1,raw_board=self.board),self.camera)
        self.assertFalse(result['valid'])
        b=dict(self.board,valid=False);result=self.detector.detect(self.render(self.board),dict(timestamp=2,raw_board=b),self.camera)
        self.assertFalse(result['valid'])
    def test_repeat_frame_does_not_confirm_and_registration_revision_matters(self):
        image=self.render(self.board);record=dict(timestamp=1,raw_board=self.board)
        for _ in range(5):result=self.detector.detect(image,record,self.camera)
        self.assertFalse(result['valid']);self.assertEqual(result['confirmation_frames'],1)
        record['raw_board']=dict(self.board,revision='other')
        self.assertFalse(self.detector.detect(image,record,self.camera)['valid'])
    def test_no_registration_and_plane_projection_roundtrip(self):
        self.path.unlink();self.assertFalse(self.detector.detect(self.render(self.board),dict(timestamp=1,raw_board=self.board),self.camera)['valid'])
        point=[14,-8,32];pixel=project([point],self.board,self.camera)[0]
        self.assertLess(math.dist(point,plane_point(pixel,self.board,self.camera,32)),1e-6)

    def test_real_d455_inverse_brown_matches_sdk_and_recovers_plane(self):
        self.camera=dict(matrix=[[380.63,0,323.2],[0,380.31,238.48],[0,0,1]],distortion=[-.06122,.07405,-.0004256,-.0001698,-.02369],model='inverse_brown_conrady')
        point=[120,-75,32];pixel=project([point],self.board,self.camera)[0]
        self.assertLess(math.dist(point,plane_point(pixel,self.board,self.camera,32)),.001)
        p=json.loads(self.path.read_text(encoding='utf-8'));p['camera']=self.camera;p['center_px']=project([[0,0,32]],self.board,self.camera)[0].tolist()
        _,jpeg=cv2.imencode('.jpg',self.render(self.board),[cv2.IMWRITE_JPEG_QUALITY,95]);p['image_jpeg_base64']=base64.b64encode(jpeg).decode()
        self.path.write_text(json.dumps(p),encoding='utf-8')
        b=copy.deepcopy(self.board);b['camera_xyz_m'][0]+=.03;b['rotation_vector_rad'][0]-=.07
        image=self.render(b,(1,-1))
        for i in range(3):result=self.detector.detect(image,dict(timestamp=i,raw_board=b),self.camera)
        self.assertTrue(result['valid'],result);self.assertLess(math.dist(result['board_xyz_mm'][:2],[1,-1]),1.5)


if __name__=='__main__':unittest.main()
