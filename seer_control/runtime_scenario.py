"""Explicit one-shot Runtime main2 execution; no automatic motion retries."""
import json
import threading
import urllib.request
from urllib.parse import urlsplit
from concurrent.futures import Future

class RuntimeScenario:
    def __init__(self,observer=None):
        self.observer=observer;self.recording_error=''
        self.lock=threading.Lock();self.future=None;self.uncertain=False
        self.opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def start(self,url,scenario="main2"):
        if scenario not in ("main1","main2","main3"):raise ValueError("허용된 시나리오는 main1/main2/main3입니다.")
        with self.lock:
            if self.uncertain:raise ValueError('실행 결과 미확정입니다. Runtime에서 실제 실행 상태를 확인하세요. 재전송하지 않습니다.')
            if self.future and not self.future.done():raise ValueError('Runtime 시나리오가 이미 실행 중입니다.')
            parts=urlsplit(url)
            if parts.scheme not in ('http','https') or not parts.hostname:raise ValueError('Runtime URL 오류')
            base=parts.scheme+'://'+parts.netloc
            future=Future();self.future=future
            def work():
                try:
                    if self.observer:self.observer.begin(scenario)
                    result=self.execute(base,scenario)
                    if self.observer:
                        try:
                            if not self.observer.finish(scenario,True):self.recording_error='기록 프레임 부족 · 실기 3D 화면을 열고 실행해야 저장됩니다.'
                        except Exception as error:self.recording_error=str(error)
                    future.set_result(result)
                except Exception as error:
                    if self.observer:self.observer.finish(scenario,False)
                    future.set_exception(error)
            threading.Thread(target=work,daemon=True,name='runtime-scenario').start()
            return future
    def execute(self,base,scenario="main2"):
        with self.opener.open(base+'/api/connection_status',timeout=3) as response:
            connected=json.load(response)
        if connected.get('connected') is not True:raise ValueError('Runtime 로봇 연결이 끊겨 있습니다. Viewer에서 먼저 연결하세요.')
        request=urllib.request.Request(base+'/api/execute_scenario',data=json.dumps({'func':scenario}).encode('utf-8'),headers={'Content-Type':'application/json'},method='POST')
        try:
            with self.opener.open(request,timeout=3600) as response:result=json.load(response)
            if result.get('status')!='success':raise RuntimeError('Runtime 실행 실패: '+str(result))
            return result
        except Exception:
            # A lost HTTP response does not mean the physical scenario stopped.
            self.uncertain=True
            raise


def show_runtime_scenario(app):
    from .offline_scene import archive_for
    import tkinter as tk
    from tkinter import ttk
    import webbrowser
    mode_real=bool(app.real)
    if mode_real:
        if not hasattr(app,'_runtime_scenario'):app._runtime_scenario=RuntimeScenario()
        manager=app._runtime_scenario
        from .offline_scene import archive_for
        manager.observer=archive_for(app)
    else:
        from .sim_runtime import SimScenario
        if not hasattr(app,'_sim_scenario'):app._sim_scenario=SimScenario(app)
        manager=app._sim_scenario
    win=tk.Toplevel(app);win.title('시나리오 main1/2/3 · 실기 / 오프라인');win.geometry('800x500')
    url=app.studio_config.get('viewer3d',{}).get('runtime_url','http://192.168.30.13:8000/runtime.html')
    ttk.Label(win,text='실제 로봇 main2 실행' if mode_real else 'SIM 로컬 main2/main3 · 네트워크 명령 없음',font=('',12,'bold')).pack(pady=12)
    ttk.Label(win,text=url if mode_real else '로컬 로봇팔 개발 프로그램 · main2 / main3').pack()
    ttk.Label(win,text='현재 서버에 로드된 scenario.main2를 실행합니다. 편집 코드는 저장 후 사용하세요.' if mode_real else '기본: 로컬에 저장한 실기 관절/베큠 기록 재생. 기록이 없으면 실행을 차단합니다.').pack(pady=6)
    available=([name for name in ('main1','main2','main3') if archive_for(app).path(name).exists()] if not mode_real else [])
    status=tk.StringVar(value='실행 대기' if mode_real else '저장된 실기 기록: '+(', '.join(available) or '없음 · 예제로 대체하지 않음')); ttk.Label(win,textvariable=status,wraplength=610).pack(pady=6)
    def run(name='main2'):
        if bool(app.real)!=mode_real:raise ValueError('모드가 바뀌었습니다. 실행 창을 다시 여세요.')
        if not mode_real:
            manager.start('',name);return
        if not app.connected or not app.control_enabled:raise ValueError('실기 연결 및 제어 허용을 먼저 켜세요.')
        if getattr(app,'task_running',False) or getattr(getattr(app,'studio_runner',None),'active',False):raise ValueError('다른 미션/로봇팔 작업을 먼저 정지하세요.')
        manager.start(url,name)
    button=ttk.Button(win,text='실제 전체 동작 실행 (main2)' if mode_real else 'SIM main2 실행',command=lambda:app.guarded(run));button.pack(pady=4)
    ttk.Button(win,text='main1 실행' if mode_real else 'SIM main1 재생',command=lambda:app.guarded(lambda:run('main1'))).pack()
    if not mode_real:ttk.Button(win,text='SIM main3 실행',command=lambda:app.guarded(lambda:run('main3'))).pack()
    if mode_real:
        ttk.Button(win,text='Runtime 열기 · 정지는 Viewer EStop 또는 실제 비상정지 사용',command=lambda:webbrowser.open(url)).pack()
    else:
        from .offline_scene import use_last_real_map,archive_for,validate_recording
        from tkinter import filedialog
        def import_record():
            path=filedialog.askopenfilename(filetypes=[('실기 실행 기록','*.json')])
            if not path:return
            from pathlib import Path
            data=json.loads(Path(path).read_text(encoding='utf-8'));validate_recording(data)
            archive=archive_for(app);archive.folder.mkdir(parents=True,exist_ok=True)
            archive.path(data['scenario']).write_text(json.dumps(data),encoding='utf-8')
            status.set(data['scenario']+' 기록을 로컬에 저장했습니다.')
        source=tk.StringVar(value=getattr(app,'sim_scenario_source','recording'))
        ttk.Label(win,text='재생 소스: recording=실기 저장 기록 / program=직접 등록한 로컬 프로그램·예제').pack()
        def change_source(event=None):
            if manager.future and not manager.future.done():
                source.set(getattr(app,'sim_scenario_source','recording'));return
            app.sim_scenario_source=source.get()
        source_box=ttk.Combobox(win,textvariable=source,values=('recording','program'),state='readonly',width=15)
        source_box.pack();source_box.bind('<<ComboboxSelected>>',change_source)
        ttk.Button(win,text='마지막 실기 맵 → SIM',command=lambda:app.guarded(lambda:use_last_real_map(app))).pack()
        ttk.Button(win,text='실기 실행 기록 JSON 가져오기',command=lambda:app.guarded(import_record)).pack()
        ttk.Button(win,text='SIM 프로그램 편집 · 로봇팔 개발',command=lambda:app.tabs.select(app.arm_workspace_page)).pack()
        ttk.Button(win,text='SIM 팔 정지',command=app.arm_dev_sim.stop).pack()
    def poll():
        if not win.winfo_exists():return
        future=manager.future
        busy=bool(future and not future.done())
        button.configure(state='disabled' if busy or manager.uncertain else 'normal')
        if busy:status.set(('Runtime' if mode_real else 'SIM')+' 시나리오 실행 중 · 창 닫기는 정지가 아닙니다.')
        elif future:
            error=future.exception()
            status.set(('실행 결과 미확정 · 실제 정지 여부를 확인하세요. 자동 재전송 없음. ' if manager.uncertain else '실행 실패: ')+str(error) if error else ('Runtime' if mode_real else 'SIM')+' 시나리오 정상 종료'+((' · 기록 주의: '+manager.recording_error) if mode_real and manager.recording_error else ''))
        win.after(200,poll)
    poll()
