import io
import json
import unittest
from PIL import Image
from seer_control.camera_recovery import recover

class Response:
    def __init__(self,data,kind='application/json'):
        self.data=io.BytesIO(data);self.headers={'Content-Type':kind}
    def read(self,*args):return self.data.read(*args)
    def read1(self,*args):return self.data.read(*args)
    def __enter__(self):return self
    def __exit__(self,*args):return False

class Opener:
    def __init__(self,responses):self.responses=list(responses);self.calls=[]
    def open(self,target,timeout):
        self.calls.append(target)
        response=self.responses.pop(0)
        if isinstance(response,Exception):raise response
        return response

class CameraRecoveryTests(unittest.TestCase):
    def test_only_camera_helper_is_posted_and_three_frames_verified(self):
        out=io.BytesIO();Image.new('RGB',(4,3)).save(out,'JPEG')
        op=Opener([Response(b'{"status":"success"}'),Response(out.getvalue()*3,'multipart/x-mixed-replace; boundary=frame')])
        self.assertEqual(recover('http://host:8000/runtime.html','http://host:8000/video_feed',op),((4,3),3))
        req=op.calls[0];self.assertEqual(req.get_method(),'POST')
        self.assertEqual(req.full_url,'http://host:8000/api/execute_scenario')
        self.assertEqual(json.loads(req.data),{'func':'_camera_preview_only'})
    def test_response_success_alone_is_not_video_success(self):
        op=Opener([Response(b'{"status":"success"}'),Response(b'','multipart/x-mixed-replace')])
        with self.assertRaises(RuntimeError):recover('http://host:8000','http://host:8000/video_feed',op)
    def test_failure_or_timeout_never_retries_or_runs_motion(self):
        for response in (Response(b'{"status":"error","message":"not installed"}'),TimeoutError('no response')):
            op=Opener([response])
            with self.assertRaises((RuntimeError,TimeoutError)):recover('http://host:8000','http://host:8000/video_feed',op)
            self.assertEqual(len(op.calls),1)
