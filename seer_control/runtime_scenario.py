"""Explicit one-shot Runtime main2 execution; no automatic motion retries."""
import json
import threading
import urllib.request
from urllib.parse import urlsplit
from concurrent.futures import Future

class RuntimeScenario:
    def __init__(self):
        self.lock=threading.Lock();self.future=None;self.uncertain=False
        self.opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def start(self,url,scenario="main2"):
        if scenario not in ("main2","main3"):raise ValueError("허용된 시나리오는 main2/main3입니다.")
        with self.lock:
            if self.uncertain:raise ValueError('실행 결과 미확정입니다. Runtime에서 실제 실행 상태를 확인하세요. 재전송하지 않습니다.')
            if self.future and not self.future.done():raise ValueError('Runtime 시나리오가 이미 실행 중입니다.')
            parts=urlsplit(url)
            if parts.scheme not in ('http','https') or not parts.hostname:raise ValueError('Runtime URL 오류')
            base=parts.scheme+'://'+parts.netloc
            future=Future();self.future=future
            def work():
                try:future.set_result(self.execute(base,scenario))
                except Exception as error:future.set_exception(error)
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
    import tkinter as tk
    from tkinter import ttk
    import webbrowser
    if not hasattr(app,'_runtime_scenario'):app._runtime_scenario=RuntimeScenario()
    manager=app._runtime_scenario
    win=tk.Toplevel(app);win.title('Runtime 전체 동작 · main2');win.geometry('650x230')
    url=app.studio_config.get('viewer3d',{}).get('runtime_url','http://192.168.30.13:8000/runtime.html')
    ttk.Label(win,text='실제 로봇의 main2 전체 시나리오를 실행합니다.',font=('',12,'bold')).pack(pady=12)
    ttk.Label(win,text=url).pack()
    ttk.Label(win,text='현재 서버에 로드된 scenario.main2를 실행합니다. 편집 중인 코드는 저장 후 사용하세요.').pack(pady=6)
    status=tk.StringVar(value='실행 대기');ttk.Label(win,textvariable=status,wraplength=610).pack(pady=6)
    def run():
        if not app.real:raise ValueError('REAL 모드에서만 Runtime 실제 시나리오를 실행할 수 있습니다.')
        if not app.connected or not app.control_enabled:raise ValueError('실기 연결 및 제어 허용을 먼저 켜세요.')
        if getattr(app,'task_running',False) or getattr(getattr(app,'studio_runner',None),'active',False):raise ValueError('다른 미션/로봇팔 작업을 먼저 정지하세요.')
        manager.start(url)
    button=ttk.Button(win,text='실제 전체 동작 실행 (main2)',command=lambda:app.guarded(run));button.pack(pady=4)
    ttk.Button(win,text='Runtime 열기 · 정지는 Viewer EStop 또는 실제 비상정지 사용',command=lambda:webbrowser.open(url)).pack()
    def poll():
        if not win.winfo_exists():return
        future=manager.future
        busy=bool(future and not future.done())
        button.configure(state='disabled' if busy or manager.uncertain else 'normal')
        if busy:status.set('main2 실행 중 · 창을 닫아도 로봇 동작은 중지되지 않습니다.')
        elif future:
            error=future.exception()
            status.set(('실행 결과 미확정 · 실제 정지 여부를 확인하세요. 자동 재전송 없음. ' if manager.uncertain else '실행 실패: ')+str(error) if error else 'Runtime에서 main2 정상 종료 응답을 받았습니다.')
        win.after(200,poll)
    poll()
