import io
import unittest
from unittest.mock import Mock,patch
from PIL import Image
from seer_control.robot_camera import JpegFrames,CameraStream,camera_url


class CameraTests(unittest.TestCase):
    def jpeg(self):
        data=io.BytesIO();Image.new('RGB',(32,24),'green').save(data,format='JPEG');return data.getvalue()

    def test_fragmented_multipart_and_multiple_frames(self):
        frame=self.jpeg();payload=b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+frame+b'\r\n--frame\r\n\r\n'+frame
        parser=JpegFrames();frames=[]
        for i in range(0,len(payload),17):frames.extend(parser.feed(payload[i:i+17]))
        self.assertEqual(frames,[frame,frame])

    def test_invalid_unbounded_frame_rejected(self):
        with self.assertRaises(ValueError):JpegFrames().feed(b'\xff\xd8'+b'a'*(4*1024*1024))

    def test_worker_publishes_decoded_frame(self):
        stream=CameraStream('http://localhost/video_feed');response=Mock()
        response.headers={'Content-Type':'multipart/x-mixed-replace; boundary=frame'}
        def read(size):stream.stop_event.set();return self.jpeg()
        response.read1.side_effect=read
        manager=Mock();manager.__enter__=Mock(return_value=response);manager.__exit__=Mock(return_value=False)
        with patch('seer_control.robot_camera.urllib.request.build_opener') as opener:
            opener.return_value.open.return_value=manager
            stream.run()
        sample,error=stream.latest()
        self.assertEqual(sample[1].size,(32,24));self.assertEqual(error,'')

    def test_url_uses_runtime_host(self):
        app=Mock(studio_config={'viewer3d':{'runtime_url':'http://192.168.30.13:8000/runtime.html'}})
        self.assertEqual(camera_url(app),'http://192.168.30.13:8000/video_feed')


if __name__=='__main__':unittest.main()
