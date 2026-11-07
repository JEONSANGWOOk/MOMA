"""Desktop-owned read-only bridge and optional Isaac Sim process launcher."""
import json
import os
from pathlib import Path
import subprocess
import threading
import urllib.request
from http.server import ThreadingHTTPServer
from .isaac_mirror import directory

ROOT=Path(__file__).resolve().parents[1]
CONFIG=Path.home()/'.seer_amr_console'/'isaac_connection.json'

def find_launcher(saved='',roots=None):
    """Search known install locations only; do not scan whole disks."""
    candidates=[Path(saved)] if saved else []
    roots=[Path('D:/'),Path('C:/')] if roots is None else [Path(p) for p in roots]
    for root in roots:
        candidates.extend([root/'isaac-sim-standalone-6.2.0-windows-x86_64'/'isaac-sim.bat',root/'IsaacSim'/'isaac-sim.bat'])
        try:candidates.extend(sorted(root.glob('isaac-sim-standalone-*-windows-x86_64/isaac-sim.bat'),reverse=True))
        except OSError:pass
    for candidate in candidates:
        if candidate.name.lower()=='isaac-sim.bat' and candidate.is_file():return str(candidate)
    return ''


class IsaacConnection:
    def __init__(self,port=8770):
        self.port=port;self.server=None;self.process=None;self.log_file=None;self.launch_mode=None
    @property
    def url(self):return f'http://127.0.0.1:{self.port}'
    def health(self):
        try:
            opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
            with opener.open(self.url+'/health',timeout=.3) as response:data=json.load(response)
            return data if data.get('service')=='moma-isaac-mirror' else None
        except (OSError,ValueError):return None
    def start_bridge(self):
        if self.server is not None:return
        from tools.isaac_mirror.serve import handler
        try:server=ThreadingHTTPServer(('127.0.0.1',self.port),handler(directory()))
        except OSError:
            if self.health() is not None:return
            raise ValueError(f'{self.port} 포트를 다른 프로그램이 사용 중입니다.')
        self.server=server;self.port=server.server_port
        threading.Thread(target=server.serve_forever,daemon=True,name='isaac-http-bridge').start()
    def launch(self,launcher,door_physics=False):
        launcher=Path(launcher)
        if not launcher.is_file() or launcher.name.lower()!='isaac-sim.bat':
            raise ValueError('설치 폴더의 isaac-sim.bat를 선택하세요.')
        if self.process is not None and self.process.poll() is None:
            if self.launch_mode is not None and self.launch_mode!=bool(door_physics):raise ValueError('문 물리모드를 바꾸려면 Isaac Sim을 종료한 뒤 다시 연결하세요.')
            return False
        health=self.health()
        if health and health.get('client_age_s') is not None and health['client_age_s']<3:
            if self.launch_mode is None or self.launch_mode!=bool(door_physics):raise ValueError('이미 실행 중인 Isaac Sim의 모드를 변경할 수 없습니다. 종료 후 다시 연결하세요.')
            return False
        env=dict(os.environ)
        env['MOMA_DOOR_PHYSICS']='1' if door_physics else '0'
        env['MOMA_DOOR_PHYSICS_CONFIG']=str(ROOT/'config/isaac_door_physics.json')
        env['MOMA_MIRROR_URL']=self.url;env['MOMA_MIRROR_CLIENT']=str(ROOT/'tools/isaac_mirror/isaac_live_view.py')
        # Keep the --exec script path simple, even when the project path has spaces/brackets.
        bootstrap=directory()/'isaac_autostart.py';bootstrap.parent.mkdir(parents=True,exist_ok=True)
        bootstrap.write_bytes((ROOT/'tools/isaac_mirror/isaac_autostart.py').read_bytes())
        self.log_file=open(directory()/'isaac_startup.log','ab')
        try:
            self.launch_mode=bool(door_physics)
            self.process=subprocess.Popen([str(launcher),'--exec',str(bootstrap)],cwd=str(launcher.parent),env=env,
                stdout=self.log_file,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        finally:
            self.log_file.close();self.log_file=None
        return True
    def close(self):
        if self.server is not None:
            server=self.server;self.server=None
            def shutdown():server.shutdown();server.server_close()
            threading.Thread(target=shutdown,daemon=True).start()
        # Do not terminate Isaac Sim: it may contain unsaved user scenes.


def show_connection(app):
    import tkinter as tk
    from tkinter import ttk,filedialog,messagebox
    existing=getattr(app,'_isaac_connection_window',None)
    if existing is not None and existing.winfo_exists():
        existing.lift();existing.after_idle(existing.connect_isaac);return existing
    manager=getattr(app,'_isaac_connection',None)
    if manager is None:manager=IsaacConnection();app._isaac_connection=manager
    win=tk.Toplevel(app);app._isaac_connection_window=win
    win.title('Isaac Sim 실시간 연결');win.geometry('730x390');win.minsize(600,370)
    try:saved=json.loads(CONFIG.read_text(encoding='utf-8')).get('launcher','')
    except (OSError,ValueError):saved=''
    path=tk.StringVar(win,value=find_launcher(saved))
    ttk.Label(win,text='설치 경로를 자동 감지해 연결합니다. 아래 경로는 필요한 경우에만 변경하세요.').pack(anchor='w',padx=14,pady=12)
    row=ttk.Frame(win);row.pack(fill='x',padx=14)
    ttk.Entry(row,textvariable=path).pack(side='left',fill='x',expand=True)
    def browse():
        value=filedialog.askopenfilename(parent=win,title='Isaac Sim 설치 폴더의 isaac-sim.bat 선택',filetypes=[('Isaac Sim launcher','isaac-sim.bat')])
        if value:path.set(value)
    ttk.Button(row,text='찾아보기',command=browse).pack(side='left',padx=5)
    status=tk.StringVar(win,value='연결 대기');ttk.Label(win,textvariable=status,wraplength=650).pack(anchor='w',padx=14,pady=14)
    try:physics_saved=json.loads(CONFIG.read_text(encoding='utf-8')).get('door_physics',False)
    except (OSError,ValueError):physics_saved=False
    CONFIG.parent.mkdir(parents=True,exist_ok=True)
    door_physics=tk.BooleanVar(win,value=bool(physics_saved))
    ttk.Checkbutton(win,text='실험 문 물리 (physics 설정 시 적용 · 기본은 파이썬 문 추종)',variable=door_physics,command=lambda:CONFIG.write_text(json.dumps({'launcher':path.get().strip(),'door_physics':door_physics.get()},ensure_ascii=False),encoding='utf-8')).pack(anchor='w',padx=14,pady=6)
    active=[os.environ.get('MOMA_ISAAC_MIRROR')=='1']
    def enable_bridge():
        manager.start_bridge();os.environ['MOMA_ISAAC_MIRROR']='1';active[0]=True
        app.view_mode.set('3D');app.tabs.select(app.operation_page);app.draw_map()
    def connect():
        try:
            detected=find_launcher(path.get().strip())
            path.set(detected)
            enable_bridge()
            if not detected:
                status.set('설치 경로를 찾지 못했습니다. 찾아보기로 isaac-sim.bat를 한 번 선택하세요.');browse()
                if not path.get().strip():return
            launched=manager.launch(path.get().strip(),door_physics=door_physics.get())
            CONFIG.parent.mkdir(parents=True,exist_ok=True)
            CONFIG.write_text(json.dumps({'launcher':path.get().strip(),'door_physics':door_physics.get()},ensure_ascii=False),encoding='utf-8')
            status.set('Isaac Sim 시작 중 · 첫 실행은 수 분 걸릴 수 있습니다.' if launched else '기존 Isaac Sim 수신기 연결 확인 중')
        except (OSError,ValueError) as error:messagebox.showerror('Isaac Sim 연결',str(error),parent=win)
    def stop():
        os.environ.pop('MOMA_ISAAC_MIRROR',None);active[0]=False;manager.close()
        publisher=getattr(app.world3d,'_isaac_publisher',None)
        if publisher:
            publisher.close();publisher.thread.join(.5);delattr(app.world3d,'_isaac_publisher')
        status.set('송출 중지 · Isaac Sim은 마지막 자세를 유지합니다.')
    buttons=ttk.Frame(win);buttons.pack(fill='x',padx=14)
    ttk.Button(buttons,text='연결 / Isaac Sim 실행',command=connect).pack(side='left',padx=3)
    ttk.Button(buttons,text='브리지만 실행',command=lambda:app.guarded(enable_bridge)).pack(side='left',padx=3)
    ttk.Button(buttons,text='송출 중지',command=stop).pack(side='left',padx=3)
    ttk.Label(win,text='기존에 수동 실행한 Isaac Sim은 최초 1회 수신 스크립트를 실행하세요.\n이 창을 닫아도 송출은 유지됩니다. 로봇 이동 명령은 전송하지 않습니다.',wraplength=650).pack(anchor='w',padx=14,pady=14)
    def poll():
        if not win.winfo_exists():return
        if active[0]:
            data=manager.health();age=data.get('client_age_s') if data else None
            if age is not None and age<3:status.set('Isaac Sim 연결됨 · 실시간 3D 수신 확인')
            elif data:status.set('브리지 송출 중 · Isaac Sim 시작/모델 로딩 또는 수신 대기')
            publisher=getattr(app.world3d,'_isaac_publisher',None)
            if publisher and publisher.error:status.set('미러 내보내기 오류: '+publisher.error)
        win.after(1500,poll)
    win.connect_isaac=connect
    win.after(1500,poll)
    win.after_idle(connect)
    return win
