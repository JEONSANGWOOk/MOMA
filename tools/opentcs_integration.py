"""Launch the existing MOMA Console connected to an openTCS virtual vehicle."""
import argparse
import sys
import time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
WINDOW_TITLE='MOMA · openTCS 가상 ACS 연동 · 실제 장비 통신 없음'


def activate_existing():
    if sys.platform!='win32':return False
    import ctypes
    from ctypes import wintypes
    user32=ctypes.windll.user32
    found=[]
    callback_type=ctypes.WINFUNCTYPE(wintypes.BOOL,wintypes.HWND,wintypes.LPARAM)
    @callback_type
    def visit(handle,param):
        title=ctypes.create_unicode_buffer(512)
        user32.GetWindowTextW(handle,title,len(title))
        if title.value==WINDOW_TITLE:found.append(handle);return False
        return True
    user32.EnumWindows(visit,0)
    if not found:return False
    user32.ShowWindow.argtypes=[wintypes.HWND,ctypes.c_int]
    user32.SetForegroundWindow.argtypes=[wintypes.HWND]
    user32.ShowWindow(found[0],9);user32.SetForegroundWindow(found[0])
    print('Existing MOMA openTCS window activated.')
    return True


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--vehicle',default='AGV-01')
    parser.add_argument('--smoke',action='store_true',help='Verify launcher and vehicle selection without leaving a window open')
    args=parser.parse_args()
    if not args.smoke and activate_existing():return
    import tkinter as tk
    from tkinter import ttk,messagebox
    from seer_control.opentcs_bridge import OpenTCSRobot
    from seer_control.robot_emulator import EmulatorServer
    import seer_control.app as am
    import seer_control.studio_ui as studio

    folder=ROOT/'artifacts'/'opentcs_session'
    folder.mkdir(parents=True,exist_ok=True)
    am.USER_DIR=folder/'settings'
    studio.SETTINGS=am.USER_DIR/'studio_settings.json'
    robot=None; server=None; app=None
    try:
        robot=OpenTCSRobot(vehicle=args.vehicle,log_dir=folder/'bridge_logs').start()
        server=EmulatorServer(robot,host='127.0.0.1').start(physics=False)
        app=am.Console()
        if args.smoke:app.withdraw()
        app.title(WINDOW_TITLE)
        app.pose_autosave.set(False);app.pose_autorecover.set(False);app.pose_autoconfirm.set(False)
        app.host.set('127.0.0.1');app.profile=ROOT/'config'/'api_profile.json';app.mode.set('실기 · 제어')
        app.profile_label.config(text=str(app.profile))
        banner=tk.Frame(app,bg='#fff1ce')
        banner.pack(side='top',fill='x',before=app.winfo_children()[0])
        tk.Label(banner,text='openTCS 가상 ACS · 노드 도착 위치 표시 · 실제 센서/연속 속도 없음',bg='#fff1ce',fg='#553a00').pack(side='left',padx=10,pady=6)
        selected=tk.StringVar(value=robot.vehicle)
        selector=ttk.Combobox(banner,textvariable=selected,values=['AGV-01','AGV-02','AGV-03','AGV-04'],state='readonly',width=12)
        selector.pack(side='left',padx=6)
        def switch_vehicle(event=None):
            previous=robot.vehicle
            try:
                if app.studio_runner.status in ('RUNNING','PAUSED') or robot.order_data.get('state') in ('RAW','ACTIVE','DISPATCHABLE','BEING_PROCESSED','WITHDRAWN'):
                    raise ValueError('진행 중인 미션을 종료한 뒤 차량을 바꾸세요.')
                app.disconnect()
                with robot.lock:
                    robot.vehicle=selected.get()
                    try:robot.verify_virtual();robot.refresh()
                    except Exception:robot.vehicle=previous;robot.refresh();raise
                    robot.order_name='';robot.order_data={};robot.target='';robot.canceled=False;robot.control_owner=''
                    robot.refresh()
                configure_connection()
            except Exception as error:
                selected.set(previous)
                messagebox.showerror('차량 변경 실패',str(error),parent=app)
        selector.bind('<<ComboboxSelected>>',switch_vehicle)
        info=tk.StringVar()
        tk.Label(banner,textvariable=info,bg='#fff1ce',fg='#553a00').pack(side='right',padx=8)
        deadline=0.
        stage=0
        switched=False
        failures=[]
        def configure_connection():
            nonlocal deadline,stage
            app.host.set('127.0.0.1');app.mode.set('실기 · 제어');app.profile=ROOT/'config'/'api_profile.json'
            app.connect();deadline=time.monotonic()+15;stage=0
        def update():
            nonlocal stage,switched
            if app.connected and app.last_state>0:
                if stage==0:
                    app.send_command('lock_control',dict(nick_name=app.control_nick));stage=1
                elif stage==1 and robot.control_owner==app.control_nick:
                    app.pull_robot_map();stage=2
            if time.monotonic()>deadline and stage<2:
                info.set('연결 대기 · 로그 확인')
                if args.smoke:
                    failures.append('Launcher connection timeout');app.close();return
            else:
                external=robot.vehicle_data.get('transportOrder')
                prefix='외부 ACS 작업' if external and external!=robot.order_name else 'ACS'
                info.set(f'{robot.vehicle} · {prefix} {robot.observed_order.get("state","대기")}')
            if args.smoke and stage==2 and not app.downloading_map and app.map.name=='opentcs_virtual':
                if not switched:
                    selected.set('AGV-02');switch_vehicle();switched=True
                elif robot.vehicle=='AGV-02' and robot.control_owner==app.control_nick:
                    print('PASS: integrated launcher, auto connect, map pull, control and AGV selection')
                    app.close();return
            app.after(300,update)
        configure_connection();app.after(300,update)
        app.mainloop()
        if failures:raise RuntimeError('; '.join(failures))
    except Exception as error:
        if app is None:
            root=tk.Tk();root.withdraw();messagebox.showerror('openTCS 연동 실패',str(error),parent=root);root.destroy()
        else:raise
    finally:
        if robot:
            # Only the order created by this connection is withdrawn on exit.
            if robot.order_name and robot.order_data.get('state') in ('RAW','ACTIVE','DISPATCHABLE','BEING_PROCESSED','WITHDRAWN'):
                robot.request(3003)
        if server:server.close()
        if robot:robot.close()


if __name__=='__main__':main()
