"""Fixed-webcam door-state observation; never issues motion commands."""
import base64
import json
import threading
import time
from pathlib import Path

DIRECTORY=Path.home()/'.seer_amr_console'
CONFIG=DIRECTORY/'webcam_door_calibration.json'
LIVE=DIRECTORY/'webcam_doors_live.json'


def read_live():
    try:
        data=json.loads(LIVE.read_text(encoding='utf-8'))
        age=time.time()-data['timestamp']
        if not -.1<=age<=1.5:return {'left':{'state':'UNKNOWN'},'right':{'state':'UNKNOWN'}}
        return data['doors']
    except (OSError,ValueError,KeyError,TypeError):
        return {'left':{'state':'UNKNOWN'},'right':{'state':'UNKNOWN'}}


def similarity(a,b):
    import cv2
    import numpy as np
    if a is None or b is None or a.shape!=b.shape:return 0.
    # Preserve structure and brightness; correlation alone can accept a blank ROI.
    af,bf=a.astype('float32'),b.astype('float32')
    sa,sb=float(af.std()),float(bf.std())
    corr=float(cv2.matchTemplate(a,b,cv2.TM_CCOEFF_NORMED)[0,0]) if min(sa,sb)>5 else 0.
    mae=float(np.abs(af-bf).mean())/255
    return max(0.,min(1.,.7*max(0.,corr)+.3*max(0.,1-mae*3)))


def classify(patch,closed,opened):
    a,b=similarity(patch,closed),similarity(patch,opened)
    if max(a,b)<.82 or abs(a-b)<.12:return 'UNKNOWN'
    return 'CLOSED' if a>b else 'OPEN'


class Observer:
    def __init__(self,index=0):
        import cv2
        self.cv=cv2;self.index=index;self.lock=threading.Lock();self.stop=threading.Event()
        self.frame=None;self.frame_at=0.;self.error='웹캠 연결 중…';self.shape=None
        self.profiles={};self.stable={};self.states={s:{'state':'UNKNOWN'} for s in ('left','right')}
        self.thread=threading.Thread(target=self.run,daemon=True,name='webcam-door-observer')

    def patch(self,frame,roi):
        x,y,w,h=roi
        if min(w,h)<16 or x<0 or y<0 or x+w>frame.shape[1] or y+h>frame.shape[0]:raise ValueError('영역 크기나 영상 해상도를 확인하세요.')
        return self.cv.resize(self.cv.cvtColor(frame[y:y+h,x:x+w],self.cv.COLOR_BGR2GRAY),(96,96))

    def snapshot(self):
        with self.lock:return (None if self.frame is None else self.frame.copy()),self.frame_at,dict(self.states),self.error

    def select(self,slot,roi):
        with self.lock:
            if self.frame is None or time.time()-self.frame_at>1:raise ValueError('최신 웹캠 영상이 필요합니다.')
            patch=self.patch(self.frame,roi)
            if patch.std()<8:raise ValueError('윤곽이 보이는 영역을 더 넓게 선택하세요.')
            self.profiles[slot]={'roi':list(roi)}
            if slot=='anchor':self.profiles[slot]['reference']=patch
            self.shape=list(self.frame.shape[:2]);self.stable.clear();self.save()

    def teach(self,side,kind,angle):
        if not 1<=angle<=120:raise ValueError('GUI 열림 표시 각도는 1~120°입니다.')
        with self.lock:
            profile=self.profiles.get(side)
            if not profile:raise ValueError('해당 문 영역을 먼저 선택하세요.')
            if self.frame is None or time.time()-self.frame_at>1:raise ValueError('최신 웹캠 영상이 필요합니다.')
            patch=self.patch(self.frame,profile['roi'])
            opposite=profile.get('open' if kind=='closed' else 'closed')
            if opposite is not None and similarity(patch,opposite)>.78:
                raise ValueError('닫힘·열림 영상이 너무 비슷합니다. 문 전체가 움직이는 영역을 선택하세요.')
            profile[kind]=patch;profile['angle']=angle;self.stable.clear();self.save()

    def save(self):
        # Called with the profile lock held. Images are small grayscale references.
        data={'version':1,'index':self.index,'shape':self.shape,'profiles':{}}
        for side,profile in self.profiles.items():
            row={k:v for k,v in profile.items() if k in ('roi','angle')}
            for key in ('reference','closed','open'):
                if key in profile:
                    row[key]=base64.b64encode(self.cv.imencode('.png',profile[key])[1]).decode('ascii')
            data['profiles'][side]=row
        DIRECTORY.mkdir(exist_ok=True)
        pending=CONFIG.with_suffix('.tmp');pending.write_text(json.dumps(data),encoding='utf-8');pending.replace(CONFIG)

    def load(self):
        import numpy as np
        try:
            data=json.loads(CONFIG.read_text(encoding='utf-8'))
            if data.get('version')!=1 or data.get('index')!=self.index:return
            self.shape=data['shape']
            for side,row in data['profiles'].items():
                profile={k:v for k,v in row.items() if k in ('roi','angle')}
                for key in ('reference','closed','open'):
                    if key in row:profile[key]=self.cv.imdecode(np.frombuffer(base64.b64decode(row[key]),dtype='uint8'),self.cv.IMREAD_GRAYSCALE)
                self.profiles[side]=profile
        except (OSError,ValueError,KeyError,TypeError):self.profiles={}

    def observe(self,frame,now):
        states={s:{'state':'UNKNOWN'} for s in ('left','right')}
        anchor=self.profiles.get('anchor')
        if not anchor or self.shape!=list(frame.shape[:2]):
            self.stable.clear();return states,'고정 프레임 기준 영역 등록 필요'
        if similarity(self.patch(frame,anchor['roi']),anchor['reference'])<.88:
            self.stable.clear();return states,'고정 기준 불일치 · 카메라 이동/가림/조명 확인'
        for side in states:
            p=self.profiles.get(side,{})
            if not all(k in p for k in ('roi','closed','open')):continue
            state=classify(self.patch(frame,p['roi']),p['closed'],p['open'])
            last,since=self.stable.get(side,(None,now))
            if state!=last:self.stable[side]=(state,now);since=now
            if state!='UNKNOWN' and now-since>=.6:
                states[side]={'state':state,'display_angle_deg':p.get('angle',90.) if state=='OPEN' else 0.}
        return states,''

    def publish(self):
        DIRECTORY.mkdir(exist_ok=True)
        tmp=LIVE.with_suffix('.tmp')
        tmp.write_text(json.dumps({'timestamp':time.time(),'doors':self.states}),encoding='utf-8');tmp.replace(LIVE)

    def run(self):
        cap=None
        try:
            self.load();cap=self.cv.VideoCapture(self.index,self.cv.CAP_DSHOW)
            if not cap.isOpened():raise RuntimeError('웹캠을 열지 못했습니다. 다른 앱의 카메라 사용과 번호를 확인하세요.')
            cap.set(self.cv.CAP_PROP_FRAME_WIDTH,640);cap.set(self.cv.CAP_PROP_FRAME_HEIGHT,480)
            cap.set(self.cv.CAP_PROP_BUFFERSIZE,1)
            while not self.stop.is_set():
                started=time.monotonic();ok,frame=cap.read()
                if not ok:raise RuntimeError('웹캠 영상 수신 중단')
                now=time.time()
                with self.lock:
                    self.frame,self.frame_at=frame,now
                    self.states,self.error=self.observe(frame,now)
                    self.publish()
                self.stop.wait(max(0.,.1-(time.monotonic()-started)))
        except Exception as e:
            with self.lock:self.error=str(e);self.frame_at=0.
        finally:
            if cap is not None:cap.release()
            with self.lock:
                self.states={s:{'state':'UNKNOWN'} for s in ('left','right')}
                self.publish()
