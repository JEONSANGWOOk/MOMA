"""LM2->LM1: speed rise runs main2; arrival runs main3. No DO writes."""
import json
import math
import queue
import threading
import time
import urllib.request
from urllib.parse import urlsplit
from .transport import PersistentSeerSession

class DepartureState:
    def __init__(self):
        self.active=False;self.moving=False;self.done=False;self.zero_since=None
    def update(self,station,speed,target,task,now):
        events=[]
        if self.done:return events
        if type(speed) not in (int,float) or not math.isfinite(speed):raise ValueError('Invalid speed')
        if target!='LM1':
            if self.active:raise ValueError('Navigation target changed')
            return events
        if task in (5,6):
            if self.active:raise ValueError('LM1 navigation failed/canceled')
            return events
        if task in (1,2,3) and not self.active:self.active=True
        if self.active and not self.moving and abs(speed)>.015:
            self.moving=True;events.append('main2')
        if self.active:
            if station=='LM1' and abs(speed)<=.01:
                if self.zero_since is None:self.zero_since=now
            else:self.zero_since=None
            arrived=self.moving and self.zero_since is not None and now-self.zero_since>=.3
            if self.moving and (task==4 or arrived):self.done=True;events.append('main3')
        return events

class DepartureLink:
    def __init__(self,host,url,scenario):
        self.host=host;p=urlsplit(url);self.base=p.scheme+'://'+p.netloc;self.scenario=scenario
        self.events=queue.Queue();self.stop=threading.Event();self.nav_sent=threading.Event();self.thread=None
        self.opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def start(self):
        self.thread=threading.Thread(target=self.run,daemon=True,name='LM2-LM1-scenarios');self.thread.start()
    def run(self):
        session=PersistentSeerSession(self.host,19204,timeout=2);state=DepartureState();started=time.monotonic()
        scenario_future=None;scenario_name=None;arrival_pending=False;main3_due=None
        try:
            data=session.request(1100)
            speed=math.hypot(float(data['vx']),float(data['vy']))
            if data.get('current_station')!='LM2' or speed>.01:raise ValueError('LM2 정지 상태에서 시작하세요.')
            before=session.request(1020)
            if before.get('task_status') in (1,2,3):raise ValueError('기존 주행 작업을 먼저 종료하세요.')
            with self.opener.open(self.base+'/api/connection_status',timeout=3) as response:
                if json.load(response).get('connected') is not True:raise ValueError('Runtime 로봇 연결이 필요합니다.')
            self.events.put(('navigate','LM1 이동 요청 준비 · 속도 상승 대기'))
            if not self.nav_sent.wait(5):raise TimeoutError('LM1 navigation was not requested')
            while not self.stop.is_set():
                if time.monotonic()-started>300:raise TimeoutError('연동 실행 시간 초과 · 추가 실행하지 않습니다.')
                data=session.request(1100);task=session.request(1020)
                if data.get('emergency') is True:raise ValueError('Robot emergency stop')
                if scenario_future and scenario_future.done() and scenario_future.exception():
                    raise ValueError(str(scenario_name)+' 실행 실패 또는 결과 미확정 · 후속 실행 차단')
                if scenario_name=='main3' and scenario_future.done():
                    self.events.put(('status','main3 정상 종료 응답 · 연동 완료'));return
                if arrival_pending:
                    if task.get('target_id')!='LM1':raise ValueError('목적지 변경 · main3 대기 폐기')
                    if task.get('task_status') in (5,6):raise ValueError('이동 실패/취소 · main3 대기 폐기')
                speed=math.hypot(float(data['vx']),float(data['vy']))
                for action in state.update(data.get('current_station'),speed,task.get('target_id'),task.get('task_status'),time.monotonic()):
                    if action=='main2':
                        scenario_future=self.scenario.start(self.base,'main2');scenario_name='main2'
                        self.events.put(('status','속도 상승 확인 · main2 1회 실행 요청'))
                    elif action=='main3':
                        arrival_pending=True
                        self.events.put(('status','LM1 도착/이동 완료 확인 · main2 종료 후 2초 대기'))
                if arrival_pending and scenario_name=='main2' and scenario_future.done():
                    if main3_due is None:
                        main3_due=time.monotonic()+2.
                        self.events.put(('status','마커 인식 안정화 · main3 실행 전 2초 대기'))
                    if time.monotonic()<main3_due:
                        self.stop.wait(.1);continue
                    scenario_future=self.scenario.start(self.base,'main3');scenario_name='main3';arrival_pending=False
                    self.events.put(('status','LM1 도착 · 2초 대기 완료 후 main3 1회 실행 요청'))
                self.stop.wait(.1)
        except Exception as error:self.events.put(('error',str(error)))
        finally:
            session.close();self.events.put(('finished','연동 감시 종료'))


def show_departure_link(app):
    import tkinter as tk
    from tkinter import ttk
    from .runtime_scenario import RuntimeScenario
    if not hasattr(app,'_runtime_scenario'):app._runtime_scenario=RuntimeScenario()
    win=tk.Toplevel(app);win.title('LM2 → LM1 · main2 → main3 연동');win.geometry('700x240')
    status=tk.StringVar(value='LM2 정지 상태에서 실행하세요. 기존 이동 전 재추정·팔 인터록을 유지합니다.')
    ttk.Label(win,text='속도 상승 → main2 / LM1 도착·정지 또는 이동 완료 → 2초 대기 → main3').pack(pady=12)
    ttk.Label(win,textvariable=status,wraplength=660).pack(pady=10)
    def start():
        old=getattr(app,'_departure_link',None)
        if old and old.thread.is_alive():raise ValueError('연동이 이미 실행 중입니다.')
        if not app.real or not app.connected or not app.control_enabled:raise ValueError('REAL 연결 및 제어 허용이 필요합니다.')
        if time.monotonic()-app.last_state>2:raise ValueError('실시간 상태 수신을 확인하세요.')
        if not app.studio_arm_safe:raise ValueError('팔 안전 자세 확인이 필요합니다.')
        if app.task_running or getattr(app.studio_runner,'active',False):raise ValueError('다른 작업을 먼저 종료하세요.')
        if 'LM1' not in app.robot_stations:raise ValueError('로봇 Stations에서 LM1을 조회하세요.')
        manager=app._runtime_scenario
        if manager.uncertain or (manager.future and not manager.future.done()):raise ValueError('Runtime main2 실행 상태를 먼저 확인하세요.')
        url=app.studio_config.get('viewer3d',{}).get('runtime_url','http://192.168.30.13:8000/runtime.html')
        link=DepartureLink(app.host.get().strip(),url,manager);app._departure_link=link;link.start()
        def poll():
            while True:
                try:kind,message=link.events.get_nowait()
                except queue.Empty:break
                app.log('LINK',message)
                if win.winfo_exists() and kind!='finished':status.set(message)
                if kind=='navigate':
                    if not app.real or not app.connected or not app.control_enabled or not app.studio_arm_safe:
                        link.stop.set();link.nav_sent.set()
                    else:
                        app.target.set('LM1');app.navigate();link.nav_sent.set()
            if link.thread.is_alive():app.after(100,poll)
        app.after(100,poll)
    ttk.Button(win,text='LM2 → LM1 연동 실행',command=lambda:app.guarded(start)).pack(pady=5)
    def release():
        link=getattr(app,'_departure_link',None)
        if link:link.stop.set()
    ttk.Button(win,text='연동 해제 · 후속 실행 차단 (실행 중인 동작 정지 아님)',command=release).pack()
    ttk.Label(win,text='DO7은 사용하지 않습니다. main2 실행 중 도착하면 종료 후 2초 뒤 main3 실행. 실제 정지는 EStop 사용.').pack(pady=8)
