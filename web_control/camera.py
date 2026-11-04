"""One read-only upstream camera connection shared by all browser viewers."""
import os
import threading
import time
import urllib.request
from urllib.parse import urlsplit


def realsense_url(viewer,runtime_url):
    explicit=viewer.get('camera_stream_url')
    if explicit:return explicit
    source=urlsplit(runtime_url)
    if source.scheme not in ('http','https') or not source.hostname:
        raise ValueError('Runtime HTTP 주소 오류')
    return f'{source.scheme}://{source.netloc}/video_feed'


class CameraHub:
    def __init__(self,url=''):
        self.lock=threading.Lock();self.stop_event=threading.Event();self.thread=None
        self.url=url;self.generation=0;self.frame=None;self.received=0.;self.sequence=0
        self.error='영상 연결 대기';self.content_type='image/jpeg'
    def configure(self,url):
        with self.lock:
            if self.url==url:return
            self.url=url;self.generation+=1;self.frame=None;self.received=0
            self.error='영상 연결 중' if url else '영상 입력 꺼짐'
    def start(self):
        if self.thread is not None:return
        self.thread=threading.Thread(target=self.run,daemon=True,name='web-realsense-stream');self.thread.start()
    def close(self):
        self.stop_event.set()
        if self.thread is not None:self.thread.join(timeout=4)
    def latest(self):
        with self.lock:return self.frame,self.content_type,self.received,self.sequence
    def status(self):
        with self.lock:
            age=time.monotonic()-self.received if self.received else None
            return dict(connected=bool(self.frame and age is not None and age<=2),error=self.error,
                        age_s=round(age,2) if age is not None else None,sequence=self.sequence,url=self.url)
    def publish(self,frame,kind,generation):
        with self.lock:
            if generation!=self.generation:return
            self.frame=frame;self.content_type=kind;self.received=time.monotonic();self.sequence+=1;self.error=''
    def capture_webcam(self,url,generation):
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError('웹캠 사용에는 opencv-python이 필요합니다. web_control/requirements.txt를 설치하세요.') from exc
        index=int(url.split(':',1)[1])
        capture=cv2.VideoCapture(index,cv2.CAP_DSHOW) if os.name=='nt' else cv2.VideoCapture(index)
        try:
            if not capture.isOpened():raise ConnectionError(f'웹캠 {index}을 열 수 없습니다. 연결, Windows 카메라 권한 및 다른 앱 사용 여부를 확인하세요.')
            capture.set(cv2.CAP_PROP_FRAME_WIDTH,640)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
            capture.set(cv2.CAP_PROP_FPS,15)
            while not self.stop_event.is_set() and generation==self.generation:
                started=time.monotonic()
                ok,frame=capture.read()
                if not ok:raise ConnectionError(f'웹캠 {index} 영상 수신 실패')
                ok,jpeg=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,80])
                if not ok:raise ValueError('웹캠 JPEG 변환 실패')
                self.publish(jpeg.tobytes(),'image/jpeg',generation)
                self.stop_event.wait(max(0,1/15-(time.monotonic()-started)))
        finally:
            capture.release()

    def run(self):
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        while not self.stop_event.is_set():
            with self.lock:url,generation=self.url,self.generation
            if not url:self.stop_event.wait(.2);continue
            try:
                if url.startswith('webcam:'):
                    self.capture_webcam(url,generation)
                    continue
                with opener.open(url,timeout=3) as response:
                    content=response.headers.get('Content-Type','').split(';')[0].lower()
                    if content.startswith('multipart/'):
                        buf=bytearray()
                        while not self.stop_event.is_set() and generation==self.generation:
                            chunk=response.read1(65536)
                            if not chunk:raise ConnectionError('카메라 스트림 종료')
                            buf.extend(chunk)
                            if len(buf)>8*1024*1024:raise ValueError('카메라 프레임 크기 초과')
                            while True:
                                start=buf.find(b'\xff\xd8')
                                if start<0:buf[:]=buf[-1:];break
                                if start:del buf[:start]
                                end=buf.find(b'\xff\xd9',2)
                                if end<0:break
                                frame=bytes(buf[:end+2]);del buf[:end+2]
                                self.publish(frame,'image/jpeg',generation)
                    elif content in ('image/jpeg','image/png'):
                        frame=response.read(8*1024*1024+1)
                        if len(frame)>8*1024*1024:raise ValueError('카메라 프레임 크기 초과')
                        self.publish(frame,content,generation);self.stop_event.wait(.1)
                    else:raise ValueError('RealSense 영상 서버의 MJPEG/JPEG/PNG 주소가 필요합니다.')
            except Exception as e:
                with self.lock:
                    if generation==self.generation:self.error=str(e)
                self.stop_event.wait(.5)
