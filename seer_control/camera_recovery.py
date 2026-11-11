"""Explicit camera-only Runtime recovery. Never initializes or moves the robot."""
import io
import json
import queue
import threading
import time
import urllib.request
from urllib.parse import urlsplit
from PIL import Image


def recover(runtime_url,stream_url,opener=None):
    from .robot_camera import JpegFrames
    parts=urlsplit(runtime_url)
    if parts.scheme not in ('http','https') or not parts.hostname:raise ValueError('Runtime 주소를 확인하세요.')
    base=parts.scheme+'://'+parts.netloc
    opener=opener or urllib.request.build_opener(urllib.request.ProxyHandler({}))
    request=urllib.request.Request(base+'/api/execute_scenario',data=json.dumps({'func':'_camera_preview_only'}).encode('utf-8'),headers={'Content-Type':'application/json'},method='POST')
    # One explicit request only. A lost response must not cause automatic retry.
    with opener.open(request,timeout=20) as response:result=json.load(response)
    if result.get('status')!='success':raise RuntimeError('카메라 복구 함수 실패: '+str(result.get('message',result)))
    parser=JpegFrames();count=0;size=None;deadline=time.monotonic()+8
    with opener.open(stream_url,timeout=3) as response:
        if 'multipart/' not in response.headers.get('Content-Type','').lower():raise ValueError('영상 주소가 MJPEG 스트림이 아닙니다.')
        while time.monotonic()<deadline and count<3:
            chunk=response.read1(65536)
            if not chunk:break
            for packet in parser.feed(chunk):
                with Image.open(io.BytesIO(packet)) as image:image.load();size=image.size
                count+=1
                if count>=3:break
    if count<3:raise RuntimeError('카메라 복구 요청 후 연속 영상 3프레임을 확인하지 못했습니다. USB 연결·다른 카메라 프로그램 점유를 확인하세요.')
    return size,count


def start(app):
    from tkinter import messagebox
    from .robot_camera import camera_url
    if getattr(app,'_camera_recovery_busy',False):return
    manager=getattr(app,'_runtime_scenario',None)
    if manager and manager.future and not manager.future.done():
        messagebox.showwarning('카메라 복구','실행 중인 시나리오가 끝난 뒤 카메라를 복구하세요.',parent=app);return
    cfg=app.studio_config.get('viewer3d',{})
    runtime=cfg.get('runtime_url','http://192.168.30.13:8000/runtime.html');stream=camera_url(app)
    app._camera_recovery_busy=True;button=app.camera_recovery_button
    button.configure(state='disabled',text='카메라 복구 중…')
    results=queue.Queue()
    app.log('CAMERA','RealSense 카메라만 재초기화 요청 · 로봇/AMR/IO 명령 없음')
    def work():
        try:results.put((True,recover(runtime,stream)))
        except Exception as error:results.put((False,str(error)))
    threading.Thread(target=work,daemon=True,name='realsense-camera-recovery').start()
    def poll():
        try:ok,value=results.get_nowait()
        except queue.Empty:app.after(100,poll);return
        app._camera_recovery_busy=False;button.configure(state='normal',text='RealSense 카메라 복구')
        if ok:
            size,count=value;message=f'카메라 복구 완료 · {size[0]}×{size[1]} 영상 {count}프레임 확인'
            app.log('CAMERA',message);messagebox.showinfo('RealSense 카메라 복구',message,parent=app)
        else:
            app.log('CAMERA','카메라 복구 실패 · '+value)
            messagebox.showerror('RealSense 카메라 복구',value+'\n제어 PC에 _camera_preview_only 함수가 설치되어 있어야 합니다.',parent=app)
    app.after(100,poll)
