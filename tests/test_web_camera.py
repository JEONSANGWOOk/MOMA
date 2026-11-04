import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from web_control.camera import CameraHub,realsense_url

class CameraTests(unittest.TestCase):
    def test_realsense_url_follows_runtime_and_desktop_override(self):
        self.assertEqual(realsense_url({},'http://192.168.30.13:8000/runtime.html'),'http://192.168.30.13:8000/video_feed')
        self.assertEqual(realsense_url({'camera_stream_url':'http://robot/stream'},'http://other:8000/runtime.html'),'http://robot/stream')
    def test_stale_frame_not_marked_live_and_source_change_clears_cache(self):
        camera=CameraHub('http://one/video_feed')
        camera.publish(b'jpeg','image/jpeg',0)
        self.assertTrue(camera.status()['connected'])
        camera.received=time.monotonic()-3
        self.assertFalse(camera.status()['connected'])
        camera.configure('http://two/video_feed')
        self.assertIsNone(camera.latest()[0])
        camera.publish(b'old source','image/jpeg',0)
        self.assertIsNone(camera.latest()[0])
    def test_one_continuous_upstream_shared_by_multiple_readers(self):
        done=threading.Event();requests=[]
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_GET(self):
                requests.append(self.path);self.send_response(200)
                self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame');self.end_headers()
                i=0
                try:
                    while not done.is_set():
                        jpeg=b'\xff\xd8'+str(i).encode()+b'\xff\xd9'
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+jpeg[:3]);self.wfile.flush()
                        self.wfile.write(jpeg[3:]+b'\r\n');self.wfile.flush();i+=1;time.sleep(.01)
                except (OSError,ConnectionError):pass
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        camera=CameraHub(f'http://127.0.0.1:{server.server_port}/video_feed');camera.start()
        try:
            end=time.monotonic()+3
            while camera.sequence<4 and time.monotonic()<end:time.sleep(.01)
            self.assertGreaterEqual(camera.sequence,4)
            for _ in range(3):self.assertTrue(camera.latest()[0].startswith(b'\xff\xd8'))
            self.assertEqual(len(requests),1)
        finally:
            camera.close();done.set();server.shutdown();server.server_close();camera.thread.join(1)

if __name__=='__main__':unittest.main()
