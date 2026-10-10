"""Register a two-marker board and run offline FR5 align/approach/return."""
import argparse
import json
import math
import queue
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
import tkinter as tk
from tkinter import ttk
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from seer_control.aruco_arm_follow import latest_record
from seer_control.aruco_board import BoardArmMission,make_board,save_board,validate_board,apply,object_points,initialize_panel_arm
from seer_control.visual_servo import VisualServo
from seer_control.arm_simulation import ArmSimulator
from seer_control.arm_workspace import ArmCanvas
from seer_control.geometry3d import RobotDescription
from seer_control.decision_log import DecisionJournal
from seer_control.fairino_model import installed


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--smoke',action='store_true')
    parser.add_argument('--smoke-servo',action='store_true')
    parser.add_argument('--smoke-cycle',action='store_true',help='Validate GUI route planning and complete offline cycle using current camera frames')
    parser.add_argument('--camera-log',type=Path,help='Camera JSONL input (default: live D455 log)')
    args=parser.parse_args();config_path=ROOT/'.delivery/d455_board.json'
    log_path=args.camera_log or ROOT/'.delivery/d455_aruco.jsonl'
    root=tk.Tk();root.title('MOMA · ArUco 기준판 정렬·접근·복귀 SIM');root.geometry('1240x860')
    journal=DecisionJournal(ROOT/'.delivery/d455_board_mission')
    sim=ArmSimulator(RobotDescription.load(installed(Path.home()/'.seer_amr_console') or ROOT/'examples/fairino_fr5.urdf'),decision_journal=journal)
    initialize_panel_arm(sim)
    mission=BoardArmMission(sim,journal);servo=VisualServo(mission);record=None;registered_markers=[];busy=False;epoch=0;results=queue.Queue()
    fields={name:tk.StringVar(value=value) for name,value in [('side_a','100'),('side_b','100'),('spacing','130'),('rot_a','0'),('rot_b','0'),('x','0'),('y','0'),('standby','200'),('approach','100')]}
    source=tk.StringVar(value='실측 입력');info=tk.StringVar(value='카메라 로그 수신 대기');status=tk.StringVar();feedback=tk.StringVar()
    top=ttk.Frame(root,padding=10);top.pack(fill='x')
    ttk.Label(top,text='정면 판넬 · 열쇠구멍 목표 · 비주얼 서보 전진/후진',font=('맑은 고딕',15,'bold')).pack(anchor='w')
    ttk.Label(top,text='SIM 전용 · 실제 로봇 통신 없음 · 등록한 판이 두 마커를 고정한다고 가정 · 카메라도 작업 중 고정').pack(anchor='w',pady=4)
    registration=ttk.LabelFrame(root,text='1. 기준판 등록 — 검은 사각형 한 변 / 중심 간 실제 간격 [mm]',padding=8);registration.pack(fill='x',padx=10)
    row=ttk.Frame(registration);row.pack(fill='x')
    for key,label in [('side_a','왼쪽 한 변'),('side_b','오른쪽 한 변'),('spacing','중심 간격'),('rot_a','왼쪽 회전°'),('rot_b','오른쪽 회전°')]:
        ttk.Label(row,text=label).pack(side='left',padx=(5,2));ttk.Entry(row,textvariable=fields[key],width=7).pack(side='left')
    ttk.Combobox(row,textvariable=source,values=['실측 입력','깊이 추정 · SIM 시험용'],state='readonly',width=23).pack(side='left',padx=8)
    def perform(fn):
        try:fn()
        except (ValueError,TypeError,KeyError,OSError) as error:mission.report(str(error))
    def stop():
        nonlocal epoch,busy
        epoch+=1;busy=False;servo.stop();mission.stop()
    def current_markers():
        if not record or time.time()-record['timestamp']>.7:raise ValueError('현재 카메라 영상이 필요합니다.')
        markers=record['markers']
        if len(markers)!=2:raise ValueError('등록할 마커 두 개만 카메라에 보여주세요.')
        return sorted(markers,key=lambda m:m['center_px'][0])
    def estimate():
        markers=current_markers();pair=record.get('center_pair')
        if not pair or pair.get('source')!='aligned_depth':raise ValueError('두 마커의 유효한 깊이 측정이 필요합니다.')
        for key,m in zip(['side_a','side_b'],markers):
            if not m.get('camera_xyz_m') or not m.get('depth_z_m'):raise ValueError('마커 크기 추정 데이터 없음')
            fields[key].set(f'{m["marker_size_mm"]*m["depth_z_m"]/m["camera_xyz_m"][2]:.1f}')
        fields['spacing'].set(f'{pair["center_distance_m"]*1000:.1f}');source.set('깊이 추정 · SIM 시험용')
        mission.report('깊이 추정 치수 입력 완료 · 등록 버튼을 누르세요',dict(center_pair=pair))
    def register():
        nonlocal registered_markers
        stop();markers=current_markers()
        config=make_board(markers,[float(fields['side_a'].get()),float(fields['side_b'].get())],float(fields['spacing'].get()),
                          'measured' if source.get()=='실측 입력' else 'depth_estimate',
                          [float(fields['rot_a'].get()),float(fields['rot_b'].get())])
        save_board(config,config_path);mission.config=config;mission.reference=None;mission.stable=0
        registered_markers=config['markers'];mission.report('기준판 등록 완료 · 카메라 추정 적용 대기',config)
    buttons=ttk.Frame(registration);buttons.pack(fill='x',pady=(6,0))
    ttk.Button(buttons,text='현재 깊이로 치수 추정',command=lambda:perform(estimate)).pack(side='left',padx=3)
    ttk.Button(buttons,text='두 마커 기준판 등록',command=lambda:perform(register)).pack(side='left',padx=3)
    ttk.Label(buttons,textvariable=info).pack(side='left',padx=10)
    actions=ttk.LabelFrame(root,text='2. 기준 좌표와 작업 목표',padding=8);actions.pack(fill='x',padx=10,pady=6)
    for key,label in [('x','구멍 좌우'),('y','구멍 상하'),('standby','대기 거리'),('approach','접근 거리')]:
        ttk.Label(actions,text=label+' mm').pack(side='left',padx=(6,2));ttk.Entry(actions,textvariable=fields[key],width=6).pack(side='left')
    def reference():
        if busy or mission.running or servo.enabled:raise ValueError('계획/실행을 먼저 정지하세요.')
        servo.stage='IDLE';servo.errors={}
        mission.set_reference(float(fields['standby'].get()));mission.update_scene();canvas.fit()
    ttk.Button(actions,text='가상 좌표 기준 설정',command=lambda:perform(reference)).pack(side='left',padx=8)
    control=ttk.Frame(root,padding=(10,0));control.pack(fill='x')
    def plan(action):
        nonlocal busy,epoch
        if busy or servo.enabled:raise ValueError('현재 계획/서보를 먼저 정지하세요.')
        servo.stage='IDLE';servo.errors={}
        mission.update_scene()
        snapshot=mission.snapshot([float(fields['x'].get()),float(fields['y'].get())],float(fields['standby'].get()),float(fields['approach'].get()),action)
        busy=True;epoch+=1;token=epoch;mission.report('전체 경로 역기구학·충돌 검사 중',dict(targets=snapshot['targets']),action='별도 가상 모델에서 경로 사전 검사')
        def worker():
            try:results.put((token,snapshot,BoardArmMission.prepare(snapshot),None))
            except Exception as error:results.put((token,snapshot,None,str(error)))
        threading.Thread(target=worker,daemon=True,name='board-route-validation').start()
    ttk.Button(control,text='중점·자세 정렬',command=lambda:perform(lambda:plan('align'))).pack(side='left',padx=3)
    ttk.Button(control,text='정렬 → 접근 → 복귀',command=lambda:perform(lambda:plan('cycle'))).pack(side='left',padx=3)
    def visual(cycle):
        if busy:raise ValueError('경로 검증을 먼저 정지하세요.')
        servo.start([float(fields['x'].get()),float(fields['y'].get())],float(fields['standby'].get()),float(fields['approach'].get()),cycle)
    ttk.Button(control,text='비주얼 서보 정렬 유지',command=lambda:perform(lambda:visual(False))).pack(side='left',padx=3)
    ttk.Button(control,text='비주얼 서보 전진 → 후진',command=lambda:perform(lambda:visual(True))).pack(side='left',padx=3)
    ttk.Button(control,text='정지',command=stop).pack(side='left',padx=3)
    ttk.Label(root,textvariable=status,font=('맑은 고딕',11,'bold'),padding=8).pack(fill='x')
    app=SimpleNamespace(font='맑은 고딕',arm_dev_display=tk.StringVar(value='SIM 개발'))
    canvas=ArmCanvas(root,app,sim);canvas.pack(fill='both',expand=True,padx=10,pady=4)
    ttk.Label(root,textvariable=feedback,padding=8).pack(fill='x')
    ttk.Label(root,text='판 X: 좌우 · 판 Y: 상하 · 거리: 판넬 앞뒤 방향 (TCP가 판넬을 향함)\n실측 치수가 없으면 깊이 추정으로 SIM 시험하세요. 영상 소실·판 이동·수신 지연은 정지하며 자동 재개하지 않습니다.',padding=(10,0,10,10)).pack(fill='x')
    try:
        config=validate_board(json.loads(config_path.read_text(encoding='utf-8')));mission.config=config
        for key,m in zip(['side_a','side_b'],config['markers']):fields[key].set(str(m['side_mm']))
        for key,m in zip(['rot_a','rot_b'],config['markers']):fields[key].set(str(m['rotation_deg']))
        fields['spacing'].set(str(config['spacing_mm']));source.set('실측 입력' if config['geometry_source']=='measured' else '깊이 추정 · SIM 시험용')
        registered_markers=config['markers']
    except (OSError,ValueError,KeyError,TypeError):pass
    errors=[];root.report_callback_exception=lambda typ,value,tb:errors.append(str(value))
    previous=time.monotonic();draw_state=None;cycle_requested=False
    def update():
        nonlocal record,previous,busy,epoch,draw_state,cycle_requested
        now=time.monotonic();dt=min(.15,now-previous);previous=now
        record=latest_record(log_path);board=mission.inspect(record)
        if board:
            info.set(f'{board["markers_used"]}개 / 오차 {board["reprojection_px"]:.2f}px / '+('실측 치수' if board['geometry_source']=='measured' else '깊이 추정 치수'))
            if mission.reference:
                try:mission.update_scene()
                except ValueError as error:mission.stop(str(error))
                if not busy and not mission.running:
                    try:
                        snapshot=mission.snapshot([float(fields['x'].get()),float(fields['y'].get())],float(fields['standby'].get()),float(fields['approach'].get()))
                        mission.targets=snapshot['targets']
                    except ValueError:pass
            elif mission.stable==3:mission.report('기준판 인식 완료 · 가상 좌표 기준을 설정하세요')
        else:
            info.set('두 마커를 보여주세요 / 등록 설정 확인')
            if busy:epoch+=1;busy=False
        if (args.smoke or args.smoke_cycle or args.smoke_servo) and mission.latest and mission.stable>=3 and not mission.reference:perform(reference)
        if args.smoke_servo and mission.reference and mission.latest and not cycle_requested:
            cycle_requested=True;perform(lambda:visual(True))
        if args.smoke_cycle and mission.reference and mission.latest and not cycle_requested:
            cycle_requested=True;perform(lambda:plan('cycle'))
        while True:
            try:token,snapshot,config,error=results.get_nowait()
            except queue.Empty:break
            if token!=epoch:continue
            busy=False
            if error:mission.report('전체 경로 검사 실패: '+error,dict(targets=snapshot['targets']))
            else:perform(lambda:mission.launch(snapshot,config))
        mission.tick(dt)
        servo.tick(record)
        if args.smoke_servo and servo.stage=='COMPLETED':
            from window_capture import capture_window
            root.after(0,lambda:(capture_window(root,ROOT/'.delivery/d455_visual_servo.png'),close()))
        if args.smoke_cycle and sim.state=='COMPLETED':
            from window_capture import capture_window
            root.after(0,lambda:(capture_window(root,ROOT/'.delivery/d455_board_mission_cycle.png'),close()))
        state=(tuple(sim.q),mission.stamp,mission.latest is not None,tuple(v.get() for v in fields.values()))
        if state!=draw_state:
            draw_state=state;canvas.render()
            if mission.reference and mission.latest:
                try:center,r=mission.virtual_board()
                except ValueError:center=None
                if center:
                    for index,marker in enumerate(mission.config['markers']):
                        points=[]
                        for corner in object_points(mission.config,index):
                            rotated=apply(r,corner);world=[center[i]/1000+rotated[i] for i in range(3)]
                            points.append(canvas.camera.project(world,max(10,canvas.winfo_width()),max(10,canvas.winfo_height())))
                        if all(points):
                            canvas.create_polygon(*[v for p in points for v in p[:2]],fill='white',outline='#26394d',width=2)
                            x=sum(p[0] for p in points)/4;y=sum(p[1] for p in points)/4
                            canvas.create_text(x,y,text=f'#{index+1}\nID {marker["id"]}',fill='#192c40',font=('맑은 고딕',9,'bold'))
                if center and mission.targets:
                    distance=sum((mission.targets['approach'][i]-center[i])*r[i][2] for i in range(3))
                    hole=[mission.targets['approach'][i]-r[i][2]*distance for i in range(3)]
                    hp=canvas.camera.project([v/1000 for v in hole],max(10,canvas.winfo_width()),max(10,canvas.winfo_height()))
                    ap=canvas.camera.project([v/1000 for v in mission.targets['align'][:3]],max(10,canvas.winfo_width()),max(10,canvas.winfo_height()))
                    if hp:
                        canvas.create_oval(hp[0]-6,hp[1]-6,hp[0]+6,hp[1]+6,outline='#ff9911',width=3)
                        canvas.create_text(hp[0]+10,hp[1]+16,text='열쇠구멍 목표',anchor='w',fill='#dc7600')
                        if ap:canvas.create_line(ap[0],ap[1],hp[0],hp[1],fill='#ff9911',dash=(5,3),arrow='last',width=2)
                targets=[('대기/후퇴',mission.targets['align'][:3],'#2081d5'),('접근',mission.targets['approach'][:3],'#2081d5')] if mission.targets else []
                for name,pose,color in ([('중점',center,'#ef5435')]+targets if center else []):
                    p=canvas.camera.project([v/1000 for v in pose],max(10,canvas.winfo_width()),max(10,canvas.winfo_height()))
                    if p:
                        canvas.create_oval(p[0]-4,p[1]-4,p[0]+4,p[1]+4,fill=color,outline='white')
                        canvas.create_text(p[0]+8,p[1],text=name,anchor='w',fill=color)
        status.set(servo.status if servo.enabled or servo.stage in ('COMPLETED','STOPPED') and servo.last_stamp is not None else mission.status)
        feedback.set('TCP [mm/도]: '+', '.join(f'{v:.1f}' for v in sim.kin.pose(sim.q)) + (' | 좌우·상하 오차 %.2f mm / 각도 %.2f° / 판넬 거리 %.1f mm' % (servo.errors['lateral_mm'],servo.errors['angle_error_deg'],servo.errors['normal_mm']) if servo.errors else ''))
        while True:
            try:journal.pending.get_nowait()
            except queue.Empty:break
        root.after(100,update)
    def close():
        completed=servo.stage=='COMPLETED'
        stop()
        if completed:servo.stage='COMPLETED'
        journal.close();root.destroy()
    root.protocol('WM_DELETE_WINDOW',close);root.update();canvas.fit();root.after(100,update)
    if args.smoke:
        from window_capture import capture_window
        root.after(1200,lambda:capture_window(root,ROOT/'.delivery/d455_board_mission_preview.png'))
        root.after(1600,close)
    if args.smoke_cycle:root.after(25000,close)
    if args.smoke_servo:root.after(65000,close)
    root.mainloop()
    if errors:raise RuntimeError('; '.join(errors))
    if args.smoke_servo:
        if servo.stage!='COMPLETED':raise RuntimeError('PBVS incomplete: '+servo.status)
        print('PASS: camera-frame PBVS align/forward/backward, virtual arm only')
    if args.smoke:print('PASS: board registration, work offsets, offline mission GUI')
    if args.smoke_cycle:
        if sim.state!='COMPLETED':raise RuntimeError('GUI cycle incomplete: '+mission.status+' / '+sim.error)
        print('PASS: GUI background preflight, live board frames, align/approach/retract/home, offline only')


if __name__=='__main__':main()
