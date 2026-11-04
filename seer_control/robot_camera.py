"""Read-only latest-frame MJPEG viewer for the camera on the MoMa PC."""
import io
import threading
import time
import urllib.request
from urllib.parse import urlsplit
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk


class JpegFrames:
    def __init__(self):self.buffer=bytearray()
    def feed(self,data):
        self.buffer.extend(data);frames=[]
        while True:
            start=self.buffer.find(b'\xff\xd8')
            if start<0:
                self.buffer[:]=self.buffer[-1:];break
            if start:self.buffer[:start]=b''
            end=self.buffer.find(b'\xff\xd9',2)
            if end<0:
                if len(self.buffer)>4*1024*1024:raise ValueError('Camera frame size limit exceeded')
                break
            frames.append(bytes(self.buffer[:end+2]));del self.buffer[:end+2]
        return frames


class CameraStream:
    def __init__(self,url):
        self.url=url;self.stop_event=threading.Event();self.lock=threading.Lock()
        self.frame=None;self.error='연결 중…';self.response=None
        self.thread=threading.Thread(target=self.run,daemon=True,name='moma-camera')
    def start(self):self.thread.start()
    def latest(self):
        with self.lock:return self.frame,self.error
    def close(self):
        self.stop_event.set()
        # Socket reads have a finite timeout. Tk must never block on shutdown.
    def run(self):
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        while not self.stop_event.is_set():
            try:
                with opener.open(self.url,timeout=3) as response:
                    if 'multipart/' not in response.headers.get('Content-Type','').lower():
                        raise ValueError('MJPEG 영상 스트림이 아닙니다.')
                    parser=JpegFrames()
                    while not self.stop_event.is_set():
                        chunk=response.read1(65536)
                        if not chunk:raise ConnectionError('영상 연결 종료')
                        packets=parser.feed(chunk)
                        if packets:
                            with Image.open(io.BytesIO(packets[-1])) as image:
                                image.load();frame=image.convert('RGB')
                            with self.lock:self.frame=(time.monotonic(),frame);self.error=''
            except Exception as error:
                with self.lock:self.error=str(error)
                self.stop_event.wait(1.)


class WebcamStream:
    """Display-only local capture; no door classification or calibration."""
    def __init__(self,index=0):
        from web_control.camera import CameraHub
        self.hub=CameraHub(f'webcam:{index}')
        self.cached=None;self.sequence=-1
    def start(self):self.hub.start()
    def close(self):self.hub.stop_event.set()
    def latest(self):
        data,kind,received,sequence=self.hub.latest()
        if data and sequence!=self.sequence:
            with Image.open(io.BytesIO(data)) as image:
                self.cached=(received,image.convert('RGB'))
            self.sequence=sequence
        return self.cached,self.hub.status()['error']


def show_webcam(app):
    existing=getattr(app,'_webcam_picker',None)
    if existing is not None and existing.winfo_exists():existing.lift();return existing
    win=tk.Toplevel(app);app._webcam_picker=win
    win.title('PC 웹캠 실시간 보기');win.resizable(False,False)
    ttk.Label(win,text='웹캠 번호 (기본 0)').pack(padx=20,pady=(16,6))
    index=tk.StringVar(win,value='0')
    ttk.Combobox(win,textvariable=index,values=tuple(str(i) for i in range(10)),state='readonly',width=12).pack(padx=20,pady=6)
    ttk.Button(win,text='실시간 영상 팝업 열기',command=lambda:show_camera(app,webcam=int(index.get()))).pack(padx=20,pady=12)
    ttk.Label(win,text='영상 표시 전용 · 문 열림 판단 없음').pack(padx=20,pady=(0,12))
    return win


def camera_url(app):
    cfg=getattr(app,'studio_config',{}).get('viewer3d',{})
    if cfg.get('camera_stream_url'):return cfg['camera_stream_url']
    source=urlsplit(cfg.get('runtime_url','http://192.168.30.13:8000/runtime.html'))
    return f'{source.scheme}://{source.netloc}/video_feed'


def show_camera(app,url=None,on_close=None,webcam=None):
    window_key='_robot_camera_window' if webcam is None else f'_webcam_window_{webcam}'
    existing=getattr(app,window_key,None)
    if existing is not None and existing.winfo_exists():existing.deiconify();existing.lift();return existing
    win=tk.Toplevel(app);setattr(app,window_key,win)
    win.title('RealSense · MoMa 실시간 영상' if webcam is None else f'PC 웹캠 {webcam} · 실시간 영상');win.geometry('960x650');win.minsize(480,320);win.resizable(True,True)
    url=url or camera_url(app) if webcam is None else f'webcam:{webcam}'
    ttk.Label(win,text='RealSense · 엔드이펙터 영상 수신' if webcam is None else 'PC 웹캠 · 영상 표시 전용').pack(pady=(8,2))
    ttk.Label(win,text=url).pack()
    controls=ttk.Frame(win);controls.pack(fill='x',padx=8,pady=6)
    always_on_top=tk.BooleanVar(win,value=False)
    ttk.Checkbutton(controls,text='항상 위',variable=always_on_top,
        command=lambda:win.attributes('-topmost',always_on_top.get())).pack(side='right',padx=8)
    window_size=tk.StringVar(value='960×650')
    ttk.Label(controls,text='창 크기').pack(side='left')
    sizes=ttk.Combobox(controls,textvariable=window_size,values=('640×480','960×650','1280×850'),state='readonly',width=10)
    sizes.pack(side='left',padx=5)
    sizes.bind('<<ComboboxSelected>>',lambda e:win.geometry(window_size.get().replace('×','x')))
    zoom=tk.DoubleVar(value=1.);zoom_text=tk.StringVar(value='100%')
    ttk.Button(controls,text='화면 맞춤',command=lambda:zoom.set(1.)).pack(side='left',padx=5)
    ttk.Label(controls,text='확대').pack(side='left')
    ttk.Scale(controls,from_=.25,to=3.,variable=zoom).pack(side='left',fill='x',expand=True,padx=5)
    ttk.Label(controls,textvariable=zoom_text,width=5).pack(side='left')
    area=ttk.Frame(win);area.pack(fill='both',expand=True,padx=8,pady=4)
    area.rowconfigure(0,weight=1);area.columnconfigure(0,weight=1)
    canvas=tk.Canvas(area,bg='#111820',highlightthickness=0);canvas.grid(row=0,column=0,sticky='nsew')
    vertical=ttk.Scrollbar(area,orient='vertical',command=canvas.yview);vertical.grid(row=0,column=1,sticky='ns')
    horizontal=ttk.Scrollbar(area,orient='horizontal',command=canvas.xview);horizontal.grid(row=1,column=0,sticky='ew')
    canvas.configure(yscrollcommand=vertical.set,xscrollcommand=horizontal.set)
    canvas.bind('<MouseWheel>',lambda e:zoom.set(max(.25,min(3.,zoom.get()*(1.1 if e.delta>0 else 1/1.1)))))
    status=tk.StringVar(value='영상 연결 중…');ttk.Label(win,textvariable=status).pack(pady=(0,8))
    stream=CameraStream(url) if webcam is None else WebcamStream(webcam)
    if webcam is not None:
        # Reuse the web server capture when it already owns this USB camera.
        try:
            import json
            opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
            with opener.open('http://127.0.0.1:8765/api/config',timeout=.4) as response:
                cfg=json.load(response)
            if cfg.get('camera_source')=='webcam' and cfg.get('camera_url')==f'webcam:{webcam}':
                stream=CameraStream('http://127.0.0.1:8765/api/camera/stream')
        except (OSError,ValueError):pass
    stream.start();win.stream=stream
    stamp=None;dimensions=None;picture=None;job=None;closed=False
    def tick():
        nonlocal stamp,dimensions,picture,job
        if closed or not win.winfo_exists():return
        sample,error=stream.latest()
        if sample:
            at,frame=sample;age=time.monotonic()-at
            size=(max(1,canvas.winfo_width()),max(1,canvas.winfo_height()))
            settings=(*size,round(zoom.get(),3))
            if stamp!=at or dimensions!=settings:
                factor=min(size[0]/frame.width,size[1]/frame.height)*zoom.get()
                factor=min(factor,3840/frame.width,2160/frame.height)
                image=frame.resize((max(1,round(frame.width*factor)),max(1,round(frame.height*factor))),Image.Resampling.BILINEAR)
                picture=ImageTk.PhotoImage(image,master=win)
                width,height=max(size[0],image.width),max(size[1],image.height)
                canvas.delete('all');canvas.create_image(width/2,height/2,image=picture)
                canvas.configure(scrollregion=(0,0,width,height))
                if dimensions!=settings:
                    canvas.xview_moveto(0);canvas.yview_moveto(0)
                zoom_text.set(f'{zoom.get()*100:.0f}%')
                stamp=at;dimensions=settings
            if error or age>2:
                canvas.delete('offline');canvas.create_text(16,16,anchor='nw',text='영상 지연 / 재연결 중',fill='#ffb44d',tags='offline')
                status.set(f'마지막 수신 {age:.1f}초 전 · {error or "영상 대기"}')
            else:status.set(f'실시간 · {frame.width}×{frame.height} · 수신 {age:.1f}초 전')
        else:status.set('연결 대기 · '+error)
        job=win.after(50,tick)
    def close():
        nonlocal closed
        if closed:return
        closed=True;stream.close()
        if job is not None:
            try:win.after_cancel(job)
            except tk.TclError:pass
        win.destroy()
        if on_close:on_close()
    win.protocol('WM_DELETE_WINDOW',close)
    win.bind('<Destroy>',lambda e:stream.close() if e.widget==win else None,add='+')
    win.after(50,tick)
    return win


def main():
    root=tk.Tk();root.withdraw()
    show_camera(root,on_close=root.destroy)
    root.mainloop()


if __name__=='__main__':main()
