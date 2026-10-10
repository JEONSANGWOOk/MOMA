"""FR5 real key panel console. Connection is read-only; motion needs calibration."""
import argparse,json,math,queue,socket,sys,threading,time
from pathlib import Path
from types import SimpleNamespace
import tkinter as tk
from tkinter import ttk,filedialog
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from seer_control.fairino_api import profile,load_sdk
from seer_control.fairino_client import FairinoClient
from seer_control.key_panel_real import template,calibration,RealKeyPlan,socket_frame,pose_matrix
from seer_control.arm_simulation import ArmSimulator
from seer_control.arm_workspace import ArmCanvas
from seer_control.geometry3d import RobotDescription
from seer_control.fairino_model import installed
from seer_control.vision_stream import read_frame
from seer_control.decision_log import DecisionJournal


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--ip',default='192.168.57.2');parser.add_argument('--smoke',action='store_true');parser.add_argument('--probe',action='store_true');args=parser.parse_args()
    sdk_default=ROOT/'.delivery/fairino-sdk/windows'
    if args.probe:
        try:
            with socket.create_connection((args.ip,20003),timeout=2):pass
            cfg=profile();cfg.update(ip=args.ip,sdk_path=str(sdk_default));client=FairinoClient()
            try:print(json.dumps(dict(ok=True,feedback=client.connect(cfg)),ensure_ascii=False))
            finally:client.close()
        except Exception as exc:print(json.dumps(dict(ok=False,ip=args.ip,error=str(exc)),ensure_ascii=False));return
        return
    root=tk.Tk();root.title('MOMA · FR5 전기 판넬 실기 연결');root.geometry('1320x850')
    journal=DecisionJournal(ROOT/'.delivery/d455_key_panel_real');client=FairinoClient();events=queue.Queue()
    ip=tk.StringVar(value=args.ip);sdk=tk.StringVar(value=str(sdk_default));cfg=profile();cfg.update(ip=args.ip,sdk_path=str(sdk_default))
    cal=None;plan=None;busy=False;epoch=0;feedback={};rx=0.;record=None;poll_at=0.;draw_at=0.;closing=False;started=time.time()
    status=tk.StringVar(value='미연결 · 연결 버튼은 상태만 읽습니다.');details=tk.StringVar(value='실제 관절·열쇠 TCP 수신 대기')
    cal_info=tk.StringVar(value='실측 보정 파일 없음 · 실기 명령 차단');contact=tk.BooleanVar(value=False)
    top=ttk.Frame(root,padding=10);top.pack(fill='x')
    ttk.Label(top,text='FR5 실제 로봇 · D455 비주얼 기준 열쇠 작업',font=('맑은 고딕',16,'bold')).pack(anchor='w')
    ttk.Label(top,text='실제 자세 표시 / 실측 보정 경로 / 0.5 mm·0.25° 이내 MoveL 보정 / SIM 좌표 전송 금지').pack(anchor='w',pady=4)
    row=ttk.Frame(root,padding=8);row.pack(fill='x')
    ttk.Label(row,text='FR5 IP').pack(side='left');ttk.Entry(row,textvariable=ip,width=17).pack(side='left',padx=5)
    ttk.Label(row,text='공식 SDK windows 폴더').pack(side='left');ttk.Entry(row,textvariable=sdk,width=65).pack(side='left',padx=5)
    ttk.Button(row,text='SDK 선택',command=lambda:sdk.set(filedialog.askdirectory() or sdk.get())).pack(side='left')
    def log(message,evidence=None,action='상태 표시'):
        status.set(message);journal.emit('REAL.FR5 열쇠',dict(connected=client.connected,active=plan is not None,ip=cfg['ip']),evidence or {},message,action)
    def queue_job(kind,fn):
        nonlocal busy
        if busy:raise ValueError('FR5 요청 처리 중')
        busy=True;token=epoch
        def run():
            try:events.put((token,kind,True,fn()))
            except Exception as exc:events.put((token,kind,False,str(exc)))
        threading.Thread(target=run,daemon=True,name='fr5-panel-'+kind).start()
    def safe(fn):
        try:fn()
        except Exception as exc:log(str(exc),dict(actual_feedback=feedback,calibration_loaded=cal is not None))
    def connect():
        nonlocal cfg,feedback,rx
        if plan:raise ValueError('먼저 실기 작업을 정지하세요.')
        if client.connected or busy:raise ValueError('연결 해제 후 다시 연결하세요.')
        cfg=profile();cfg.update(ip=ip.get().strip(),sdk_path=sdk.get().strip())
        if cal:cfg.update(verified=cal.get('verified',False),version_confirmed=cal.get('version_confirmed',False),controller_version=cal.get('controller_version',''))
        feedback={};rx=0.
        def job():
            with socket.create_connection((cfg['ip'],20003),timeout=2):pass
            load_sdk(cfg['sdk_path']);client.connect(cfg)
            return client.call(cfg,'panel_status')
        queue_job('connect',job);log('FR5 읽기 전용 연결 시험 중')
    def stop(reason='사용자 정지',force=False):
        nonlocal plan,epoch,busy,rx
        if plan is None and not force:return
        if not client.config:log('FR5 연결 정보가 없어 정지 응답을 확인할 수 없습니다.');return
        evidence=dict(actual_feedback=feedback,board=record.get('board') if record else None,
            camera_timestamp=record.get('timestamp') if record else None,last_decision=plan.evidence if plan else None)
        plan=None;epoch+=1;busy=True;rx=0.;token=epoch
        log(reason,evidence,action='독립 채널 StopMotion 요청 · 재연결 필요')
        def run():
            try:events.put((token,'stop',True,client.stop()))
            except Exception as exc:events.put((token,'stop',False,'정지 확인 실패: '+str(exc)))
        threading.Thread(target=run,daemon=False,name='fr5-panel-priority-stop').start()
    ttk.Button(row,text='연결 · 상태 읽기',command=lambda:safe(connect)).pack(side='left',padx=5)
    calibration_row=ttk.Frame(root,padding=8);calibration_row.pack(fill='x')
    def load_cal():
        nonlocal cal
        if busy or plan or client.connected:raise ValueError('연결을 해제한 뒤 보정 파일을 변경하세요.')
        path=filedialog.askopenfilename(filetypes=[('JSON','*.json')])
        if path:
            data=json.loads(Path(path).read_text(encoding='utf-8'));calibration(data);cal=data
            cal_info.set(f"TCP #{cal['tool']} / user 0 / 제어기 {cal['controller_version']} / revision {cal['board_revision']}")
            log('실측 보정 파일 검증 완료',dict(file=path,calibration=cal))
    def create_template():
        path=ROOT/'.delivery/fr5_key_calibration.json'
        if not path.exists():path.write_text(json.dumps(template(),ensure_ascii=False,indent=2),encoding='utf-8')
        log('보정 템플릿: '+str(path))
    def disconnect():
        nonlocal epoch,busy,feedback,rx
        if plan:stop('연결 해제 요청');return
        if busy:log('FR5 요청 처리 완료 후 연결을 해제하세요.');return
        epoch+=1;client.close();busy=False;feedback={};rx=0.;log('연결 해제')
    ttk.Button(calibration_row,text='보정 템플릿 생성',command=lambda:safe(create_template)).pack(side='left',padx=4)
    ttk.Button(calibration_row,text='실측 보정 파일 불러오기',command=lambda:safe(load_cal)).pack(side='left',padx=4)
    ttk.Button(calibration_row,text='연결 해제',command=disconnect).pack(side='left',padx=4)
    ttk.Label(calibration_row,textvariable=cal_info).pack(side='left',padx=8)
    def preview():
        calibration(cal);s=socket_frame(cal,record);log('실제 슬롯 목표 계산 완료 · 명령 전송 없음',dict(socket_base_mm=s))
    def start():
        nonlocal plan
        if busy or plan:raise ValueError('요청/작업 종료 후 시작하세요.')
        if not client.connected or time.monotonic()-rx>.5:raise ValueError('최신 실제 FR5 상태가 필요합니다.')
        if feedback.get('telemetry_verified') is not True:raise ValueError('FR5 실시간 상태 패킷 갱신 확인 필요')
        calibration(cal,contact.get())
        if feedback.get('tool')!=cal.get('tool') or feedback.get('user')!=0:raise ValueError('실제 열쇠 TCP 번호와 user 0 확인 필요')
        if feedback['status'] in ('ERROR','RUNNING','PAUSED','CANCELED') or feedback['motion_done']!=1:raise ValueError('로봇 정지 및 오류 상태 확인 필요')
        calibration(cal,contact.get());socket_frame(cal,record)
        plan=RealKeyPlan(cal,contact.get());log('실기 작업 시작',dict(contact=contact.get()),'실측 좌표의 제한된 MoveL 보정')
    action=ttk.Frame(root,padding=8);action.pack(fill='x')
    ttk.Button(action,text='슬롯 목표 계산 · 전송 없음',command=lambda:safe(preview)).pack(side='left',padx=4)
    ttk.Checkbutton(action,text='삽입·회전 포함 (접촉 보호 검증 필요)',variable=contact).pack(side='left',padx=8)
    ttk.Button(action,text='실기 정렬·접근 시작',command=lambda:safe(start)).pack(side='left',padx=5)
    ttk.Button(action,text='실기 정지',command=lambda:stop(force=True)).pack(side='left',padx=8)
    ttk.Label(root,textvariable=status,font=('맑은 고딕',11,'bold'),padding=10).pack(fill='x')
    sim=ArmSimulator(RobotDescription.load(installed(Path.home()/'.seer_amr_console') or ROOT/'examples/fairino_fr5.urdf'))
    app=SimpleNamespace(font='맑은 고딕',arm_dev_display=tk.StringVar(value='SIM 개발'))
    canvas=ArmCanvas(root,app,sim);canvas.pack(fill='both',expand=True,padx=10)
    canvas.camera.distance=2.;canvas.camera.target=[0,0,.4]
    ttk.Label(root,textvariable=details,wraplength=1260,padding=10).pack(fill='x')
    ttk.Label(root,text='3D: SDK 실측 관절로 갱신하는 로봇 모델 · 실제 TCP 숫자가 제어 기준입니다. 초기 화면 자세는 미연결 모델입니다.\n작업대 고정 카메라 전용 · D455 기준판은 실측 등록 필요 · 실제 키/TCP/슬롯 보정 전 실기 명령 차단 · 로그: .delivery/d455_key_panel_real',padding=10).pack(fill='x')
    failures=[];root.report_callback_exception=lambda typ,val,tb:failures.append(str(val))
    def close_now():client.close();journal.close();root.destroy()
    def close():
        nonlocal closing
        if plan:closing=True;stop('창 닫기 요청');return
        if busy:log('요청 처리 완료 후 창을 닫으세요.');return
        close_now()
    root.protocol('WM_DELETE_WINDOW',close)
    def update():
        nonlocal plan,busy,feedback,rx,poll_at,draw_at,record,closing
        now=time.monotonic();wall=time.time()
        record=read_frame(ROOT/'.delivery/d455_aruco.jsonl',ROOT/'.delivery/d455_aruco_live.json',previous=record)
        while not events.empty():
            token,kind,ok,result=events.get_nowait()
            if token!=epoch:continue
            busy=False
            if not ok:
                if plan:stop('실기 오류: '+str(result))
                else:log(str(result))
                if closing:closing=False
            elif kind in ('connect','poll'):
                feedback=result;rx=now;sim.q=list(result['joints_rad'])
                details.set(f"IP {cfg['ip']} | 상태 {result['status']} | TCP #{result['tool']} / user {result['user']}\n실제 관절°: {result['joints_deg']}\n실제 열쇠 TCP mm/°: {result['tcp_mm_deg']} | 힘/토크: {result['force_torque']}\nD455: {'유효 기준판' if record and record.get('board',{}).get('valid') else '두 마커 인식 대기'}")
                if kind=='connect':log('FR5 실제 상태 수신 완료 · 동작 없음',result)
                if plan:
                    try:
                        if result['status']=='ERROR' or result['tool']!=plan.c['tool'] or result['user']!=0:raise ValueError('로봇 오류/TCP 설정 변경')
                        command=plan.step(record,result,wall);log('실기 '+plan.stage,plan.evidence,'실제 TCP 피드백과 최신 영상으로 판단')
                        if command:queue_job('step',lambda cmd=command:client.call(cfg,'panel_step',cmd))
                    except Exception as exc:stop(str(exc))
            elif kind=='stop':
                log('FR5 정지 명령 확인 · 재연결 필요',result)
                if closing:close_now();return
        if plan:
            try:
                socket_frame(plan.c,record,wall)
                if now-rx>.6:raise ValueError('실기 피드백 600 ms 지연')
            except Exception as exc:stop(str(exc))
        if client.connected and not busy and now-poll_at>=.2:
            poll_at=now;safe(lambda:queue_job('poll',lambda:client.call(cfg,'panel_status')))
        if now-draw_at>=.2:
            canvas.render();draw_at=now;w,h=canvas.winfo_width(),canvas.winfo_height()
            if feedback:
                tip=canvas.camera.project([v/1000 for v in feedback['tcp_mm_deg'][:3]],w,h)
                if tip:
                    x,y=tip[:2];canvas.create_oval(x-5,y-5,x+5,y+5,fill='#e8b431',outline='white');canvas.create_text(x+10,y+12,text='SDK 실제 TCP',anchor='w',fill='#89611b')
            if cal:
                try:
                    target=socket_frame(cal,record,wall);p=canvas.camera.project([r[3]/1000 for r in target[:3]],w,h)
                    if p:
                        x,y=p[:2];canvas.create_line(x-8,y,x+8,y,fill='#d84848',width=2);canvas.create_line(x,y-8,x,y+8,fill='#d84848',width=2);canvas.create_text(x+12,y-12,text='실측 슬롯 원점',anchor='w',fill='#b13838')
                except (ValueError,TypeError,KeyError):pass
        if args.smoke and wall-started>2:
            from window_capture import capture_window
            root.update_idletasks();capture_window(root,ROOT/'.delivery/d455_key_panel_real.png')
            (ROOT/'.delivery/d455_key_panel_real_smoke.json').write_text(json.dumps(dict(gui_errors=failures,connected=client.connected,motion_started=plan is not None,ip=ip.get()),ensure_ascii=False),encoding='utf-8')
            close_now();return
        root.after(30,update)
    root.after(30,update);root.mainloop()
    if failures:raise RuntimeError(';'.join(failures))


if __name__=='__main__':main()
