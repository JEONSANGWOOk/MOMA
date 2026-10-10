"""Unified cabinet SIM / measured REAL / synchronized SIM+REAL workspace."""
import argparse,json,math,sys,time
from pathlib import Path
from types import SimpleNamespace
import tkinter as tk
from tkinter import ttk
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from seer_control.aruco_board import BoardArmMission,make_board,validate_board,initialize_panel_arm,pose_from_matrix,FRONT,matmul
from seer_control.arm_simulation import ArmSimulator
from seer_control.arm_workspace import ArmCanvas
from seer_control.geometry3d import RobotDescription
from seer_control.fairino_model import installed
from seer_control.decision_log import DecisionJournal
from seer_control.vision_stream import read_frame
from seer_control.key_panel import KeyTool,KeyPanelPhysics,KeyPanelScene,KeyServo
from seer_control.key_panel_twin import KeyTwin,KeyPanelLink,metres
from seer_control.key_panel_link_ui import KeyLinkPane
from seer_control.key_panel_real import socket_frame,pose_matrix


def build_workspace(parent=None,args=None):
    parser=argparse.ArgumentParser();parser.add_argument('--demo',action='store_true');parser.add_argument('--smoke',action='store_true')
    parser.add_argument('--smoke-ui',action='store_true',help='Verify the integrated window without connecting or moving hardware')
    parser.add_argument('--smoke-compare',action='store_true',help='Explicit synthetic feedback for overlay rendering; no network/motion')
    args=parser.parse_args() if args is None else args
    standalone=parent is None
    root=tk.Tk() if standalone else ttk.Frame(parent)
    if standalone:root.title('MOMA · 전기 판넬 SIM·REAL 통합');root.geometry('1500x1000')
    else:root.pack(fill='both',expand=True)
    journal=DecisionJournal(ROOT/'.delivery/d455_key_panel')
    source=tk.StringVar(value='내장 모의 카메라' if standalone else 'D455 실시간');moving=tk.BooleanVar(value=False);lost=tk.BooleanVar(value=False);correct=tk.BooleanVar(value=True)
    status=tk.StringVar(value='기준 마커 준비 중');feedback=tk.StringVar();record=None;requested=False;last_draw=0.;started=time.time()
    fields={k:tk.StringVar(value=v) for k,v in [('x','0'),('y','100'),('speed','120'),('insert','10'),('turn','30')]}
    sim=ArmSimulator(RobotDescription.load(installed(Path.home()/'.seer_amr_console') or ROOT/'examples/fairino_fr5.urdf'),decision_journal=journal)
    initialize_panel_arm(sim);sim.physics=KeyPanelPhysics(sim.kin)
    sim.physics.decision_journal=journal
    mission=BoardArmMission(sim,journal)
    try:mission.config=validate_board(json.loads((ROOT/'.delivery/d455_board.json').read_text(encoding='utf-8')))
    except (OSError,ValueError):mission.config=make_board([dict(dictionary='DICT_4X4_1000',id=0),dict(dictionary='DICT_5X5_1000',id=0)],[100,100],130,'depth_estimate')
    tool=KeyTool(mission);scene=KeyPanelScene(mission,tool);servo=KeyServo(mission,scene)
    twin=KeyTwin(sim.kin.asset);link=KeyPanelLink(twin,journal)
    twin.sim.q=list(sim.q)
    visited_modes=['SIM']
    if args.smoke_compare:
        from seer_control.geometry3d import multiply,transform
        t=multiply(sim.kin.fk(sim.q),transform((0,0,tool.offset_mm/1000)))
        sample=dict(joints_rad=list(sim.q),tcp_mm_deg=pose_from_matrix(t,[r[3]*1000 for r in t[:3]]),
            status='IDLE',motion_done=1,tool=1,user=0,telemetry_verified=False,force_torque=None,simulated_transport=True)
        class TestClient:
            connected=True;config=None
            def call(self,c,kind,op=None):
                if kind!='panel_status':raise ValueError('GUI 시험 데이터 · 실제 동작 전송 불가')
                return dict(sample)
            def close(self):self.connected=False
            def stop(self):self.connected=False;return dict(status='TEST_ONLY')
        link.client=TestClient();link.feedback=sample;link.rx=time.monotonic();twin.fit(sample)
        target=list(sample['tcp_mm_deg']);target[1]+=3;twin.predict(target)
        r=matmul(t,FRONT);origin=[t[i][3]*1000+t[i][2]*100 for i in range(3)]
        twin.socket=[list(r[i])+[origin[i]] for i in range(3)]+[[0,0,0,1]]
    top=ttk.Frame(root,padding=10);top.pack(fill='x')
    ttk.Label(top,text='전기 판넬 · 열쇠 정렬 → 삽입 → 90° 회전 → 잠금해제',font=('맑은 고딕',16,'bold')).pack(anchor='w')
    ttk.Label(top,text='AMR: MOMA 상단 모드 · FR5: 아래 SIM/REAL/SIM+REAL | 파란 반투명: SIM 예측 · 원래 색: REAL 측정 | 실제 동작은 실측 보정값 사용').pack(anchor='w',pady=4)
    if args.smoke_compare:ttk.Label(top,text='GUI 검증용 합성 피드백 · 실제 로봇 연결 없음 · 실기 명령 전송 불가',foreground='#b33d35').pack(anchor='w')
    def mode_changed(mode):
        visited_modes.append(mode)
        if servo.enabled:servo.stop('실행 모드 변경')
        if mode=='SIM':
            canvas.sim=sim;app.arm_scene_geometry=scene.geometry;app.arm_dev_display.set('SIM 개발')
        else:
            source.set('D455 실시간');switch()
            canvas.sim=twin.sim;app.arm_scene_geometry=twin.geometry
            app.arm_dev_display.set('실기 + SIM 비교' if mode=='SIM+REAL' else '실기 자세')
        canvas.render()
    pane=KeyLinkPane(root,link,ROOT,mode_changed);pane.pack(fill='x',padx=10,pady=3)
    controls=ttk.Frame(root,padding=(10,3));controls.pack(fill='x')
    combo=ttk.Combobox(controls,textvariable=source,values=['내장 모의 카메라','D455 실시간'],state='readonly',width=20);combo.pack(side='left',padx=4)
    def perform(fn):
        try:fn()
        except (ValueError,TypeError,KeyError) as error:servo.stop(str(error));status.set(str(error));mission.report(str(error))
    def switch(event=None):
        nonlocal record,requested
        if link.plan or (link.busy and link.pending_kind=='step'):
            source.set('D455 실시간');link.report('실기 작업 정지 후 영상 소스를 변경하세요.');return
        if link.mode!='SIM' and source.get()!='D455 실시간':
            source.set('D455 실시간');link.report('REAL/동시 모드는 실제 D455 영상을 사용합니다.');return
        if servo.enabled:servo.stop('영상 소스 변경')
        sim.physics.objects=[tool.key];scene.parts.clear();scene.meshes.clear();scene.walls.clear();scene.center=None;scene.r=None
        scene.unlocked=False;scene.rotor_deg=0;mission.reference=None;mission.latest=None;mission.stable=0;mission.stamp=None
        initialize_panel_arm(sim);tool.sync();record=None;requested=False;servo.stage='IDLE';servo.errors={}
    combo.bind('<<ComboboxSelected>>',switch)
    ttk.Checkbutton(controls,text='모의 판넬 이동·회전',variable=moving).pack(side='left',padx=6)
    ttk.Checkbutton(controls,text='마커 소실 시험',variable=lost).pack(side='left',padx=6)
    ttk.Checkbutton(controls,text='일치하는 열쇠',variable=correct).pack(side='left',padx=6)
    entryrow=ttk.Frame(root,padding=(10,3));entryrow.pack(fill='x');ttk.Label(entryrow,text='SIM 전용 값',foreground='#315c93').pack(side='left')
    for k,label in [('x','구멍 좌우 mm'),('y','구멍 상하 mm'),('speed','보정 mm/s'),('insert','삽입 mm/s'),('turn','회전 °/s')]:
        ttk.Label(entryrow,text=label).pack(side='left',padx=(8,2));ttk.Entry(entryrow,textvariable=fields[k],width=7).pack(side='left')
    def begin():
        if link.mode!='SIM':
            if servo.enabled:servo.stop('실측 목표의 통합 작업 시작')
            link.start(record,pane.contact.get());return
        if not mission.reference:raise ValueError('두 마커 인식과 가상 기준 설정을 기다리세요.')
        if servo.enabled:raise ValueError('진행 중입니다. 먼저 정지하세요.')
        if scene.feedback()['depth_mm']>0:raise ValueError('먼저 열쇠를 후퇴하세요.')
        x,y=float(fields['x'].get()),float(fields['y'].get());insert=float(fields['insert'].get());turn=float(fields['turn'].get())
        if not all(math.isfinite(v) for v in (x,y,insert,turn)) or not -50<=x<=50 or not 80<=y<=160:raise ValueError('구멍 좌우 ±50 / 상하 80~160 mm')
        if not 1<=insert<=20 or not 5<=turn<=45:raise ValueError('삽입 1~20 mm/s / 회전 5~45 °/s')
        # Rebuild the physical handle at the configured calibrated board offset.
        sim.physics.objects=[tool.key];scene.parts.clear();scene.meshes.clear();scene.walls.clear();scene.offset=[x,y]
        scene.sync();servo.configure_speed(float(fields['speed'].get()),60);servo.insert_speed=insert;servo.turn_speed=turn;servo.start_key()
    buttons=ttk.Frame(root,padding=(10,6));buttons.pack(fill='x')
    ttk.Button(buttons,text='선택 모드 작업 시작',command=lambda:perform(begin)).pack(side='left',padx=4)
    def withdraw():
        if link.mode!='SIM':raise ValueError('실기 자동 후퇴는 지원하지 않습니다. 정지 후 현장 제어기로 후퇴하세요.')
        servo.withdraw()
    ttk.Button(buttons,text='SIM 열쇠 후퇴',command=lambda:perform(withdraw)).pack(side='left',padx=4)
    def stop_all():
        servo.stop('사용자 통합 정지')
        link.stop('사용자 통합 정지',force=bool(link.client.config))
    ttk.Button(buttons,text='SIM + FR5 정지',command=stop_all).pack(side='left',padx=4)
    def reset():
        if link.mode!='SIM':raise ValueError('SIM 모드에서 가상 판넬을 초기화하세요.')
        if servo.enabled:raise ValueError('먼저 정지하세요.')
        scene.reset_lock();switch()
    ttk.Button(buttons,text='초기화',command=lambda:perform(reset)).pack(side='left',padx=4)
    ttk.Label(buttons,text='동시 실행: 동일 실측 TCP 목표를 가상 계산 → FR5 전송 → 실제 도달 확인').pack(side='left',padx=10)
    ttk.Label(root,textvariable=status,padding=(12,4),font=('맑은 고딕',12,'bold')).pack(fill='x')
    body=ttk.Frame(root);body.pack(fill='both',expand=True,padx=10)
    app=SimpleNamespace(font='맑은 고딕',arm_dev_display=tk.StringVar(value='SIM 개발'),arm_scene_geometry=scene.geometry)
    app.fr5_client=link.client;app.fr5_feedback={};app.fr5_rx=0.
    app.arm_real_scene_geometry=lambda faces,lines:twin.geometry(faces,lines,metres(pose_matrix(link.feedback['tcp_mm_deg'])))
    canvas=ArmCanvas(body,app,sim);canvas.camera.distance=1.3;canvas.camera.target=[-.25,0,.35];canvas.pack(side='left',fill='both',expand=True)
    right=ttk.Frame(body,width=350);right.pack(side='right',fill='y',padx=(10,0));right.pack_propagate(False)
    side_tabs=ttk.Notebook(right);side_tabs.pack(fill='both',expand=True)
    camera_page=ttk.Frame(side_tabs);detail_page=ttk.Frame(side_tabs)
    side_tabs.add(camera_page,text='카메라 · ArUco');side_tabs.add(detail_page,text='작업 상세')
    from seer_control.vision_camera_ui import VisionCameraPane
    camera=VisionCameraPane(camera_page,ROOT);camera.pack(fill='both',expand=True)
    detail_scroll=tk.Canvas(detail_page,highlightthickness=0);detail_scroll.pack(side='left',fill='both',expand=True)
    detail_bar=ttk.Scrollbar(detail_page,command=detail_scroll.yview);detail_bar.pack(side='right',fill='y');detail_scroll.configure(yscrollcommand=detail_bar.set)
    right=ttk.Frame(detail_scroll);detail_window=detail_scroll.create_window(0,0,window=right,anchor='nw')
    right.bind('<Configure>',lambda e:detail_scroll.configure(scrollregion=detail_scroll.bbox('all')))
    detail_scroll.bind('<Configure>',lambda e:detail_scroll.itemconfigure(detail_window,width=e.width))
    if standalone and (args.smoke or args.smoke_compare):side_tabs.select(detail_page)
    ttk.Label(right,text='판넬 형상 참고 이미지 · 실측 모델 아님').pack(anchor='w')
    from PIL import Image,ImageTk
    image=Image.open(ROOT/'artifacts/electrical_panel/panel_reference.png');image.thumbnail((340,170))
    photo=ImageTk.PhotoImage(image,master=root);root.panel_reference_photo=photo;ttk.Label(right,image=photo).pack(pady=4)
    detail=tk.Canvas(right,width=340,height=230,bg='white',highlightthickness=1,highlightbackground='#b2bfcb');detail.pack(fill='x',pady=6)
    ttk.Label(right,textvariable=feedback,justify='left',wraplength=335,font=('맑은 고딕',9)).pack(anchor='w')
    def view(close=False):
        if link.mode!='SIM':
            if twin.socket:canvas.camera.target=[r[3]/1000 for r in twin.socket[:3]] if close else [0,0,.4]
            canvas.camera.distance=.5 if close else 1.5;canvas.camera.yaw=-35;canvas.camera.pitch=16;canvas.render();return
        if scene.r:
            h=scene.hole();canvas.camera.target=[v/1000 for v in h] if close else [-.35,-.15,.43]
            canvas.camera.distance=.48 if close else 1.5
        canvas.camera.yaw=-35;canvas.camera.pitch=16;canvas.render()
    ttk.Button(right,text='손잡이·열쇠 확대',command=lambda:view(True)).pack(fill='x',pady=3)
    ttk.Button(right,text='로봇 + 판넬 전체 보기',command=lambda:view(False)).pack(fill='x')
    ttk.Label(root,text='두 마커는 판넬에 함께 고정 · 구멍 위치는 두 마커 중간점 기준 · 황금색 열쇠 / 은색 실린더 / 검정 손잡이\n왼쪽 드래그: 회전 · 오른쪽 드래그: 이동 · 휠: 확대 | 로그: .delivery/d455_key_panel | 잠금기구·삽입 접촉은 기하학적 SIM 모델',padding=10).pack(fill='x')
    failures=[];root.report_callback_exception=lambda typ,value,tb:failures.append(str(value))
    def draw_detail(f):
        detail.delete('all');detail.create_text(12,12,anchor='nw',text='SIM FK 피드백 · 슬롯 정면 / 삽입 단면',fill='#24394e')
        a=-math.radians(scene.rotor_deg);cx,cy=85,90
        detail.create_oval(cx-38,cy-38,cx+38,cy+38,fill='#e0e5ea',outline='#728494',width=3)
        corners=[(-6.4,-20),(6.4,-20),(6.4,20),(-6.4,20)]
        xy=[(cx+x*math.cos(a)-y*math.sin(a),cy+x*math.sin(a)+y*math.cos(a)) for x,y in corners]
        detail.create_polygon(*[v for p in xy for v in p],fill='#17222d')
        detail.create_text(175,68,anchor='w',text=f"실린더 {scene.rotor_deg:.1f}°")
        detail.create_text(175,94,anchor='w',text='잠금해제' if scene.unlocked else '잠김',fill='#159c61' if scene.unlocked else '#d47d1b',font=('맑은 고딕',13,'bold'))
        mouth=180;scale=3.;depth=f['depth_mm'];tip=mouth+max(-45,min(24,depth))*scale
        detail.create_rectangle(mouth,155,mouth+22*scale+8,192,fill='#c1ccd7',outline='#6b7c8d')
        detail.create_rectangle(mouth,166,mouth+22*scale,181,fill='#202b36',outline='')
        detail.create_rectangle(tip-70,169,tip,178,fill='#dfac36',outline='#94651d')
        detail.create_line(mouth,150,mouth,200,fill='#e44b4b',dash=(3,2));detail.create_text(12,212,anchor='w',text=f'침투 깊이 {depth:+.2f} / 22 mm · 금색: 실제 열쇠 끝')
    closing=False;closed=False;timer=None
    def close():
        nonlocal closing,closed
        if closed:return True
        if servo.enabled:servo.stop('창 닫기')
        if link.plan:closing=standalone;link.stop('통합 창 닫기');return False
        if link.busy:link.report('FR5 요청/정지 응답을 기다리세요.');return False
        link.client.close()
        camera.close();closed=True
        if timer:root.after_cancel(timer)
        journal.close();root.destroy();return True
    if standalone:root.protocol('WM_DELETE_WINDOW',close)
    root.key_panel_close=close;root.key_panel_stop=stop_all;root.key_panel_link=link;root.vision_camera=camera;root.key_panel_modes=pane;root.key_panel_status=status
    def update():
        nonlocal record,requested,last_draw,timer
        if closed:return
        now=time.time();scene.correct_key=correct.get()
        if source.get()=='내장 모의 카메라':
            elapsed=now-started;motion=math.sin(elapsed*.35) if moving.get() else 0.
            record=dict(timestamp=now,vision_source='built_in_demo',board=dict(valid=not lost.get(),revision=mission.config['revision'],markers_used=2,
                geometry_source='depth_estimate',camera_xyz_m=[motion*.006,0,.6],rotation_vector_rad=[math.pi-motion*.01,0,0],reprojection_px=.1))
        else:record=read_frame(ROOT/'.delivery/d455_aruco.jsonl',ROOT/'.delivery/d455_aruco_live.json',previous=record)
        link.tick(record);pane.refresh();camera.show_record(record)
        fresh=link.client.connected and time.monotonic()-link.rx<=.6
        app.fr5_feedback=link.feedback;app.fr5_rx=link.rx if fresh else 0.
        if (args.smoke_ui or args.smoke_compare) and now-started>2.5:
            from tools.window_capture import capture_window
            root.update_idletasks();capture_window(root,ROOT/'.delivery/d455_key_panel_integrated.png')
            (ROOT/'.delivery/d455_key_panel_integrated_verification.json').write_text(json.dumps(dict(gui_errors=failures,
                visited_modes=visited_modes,mode=link.mode,synthetic_feedback=args.smoke_compare,
                real_motion_active=link.plan is not None,physical_connected=False if args.smoke_compare else link.client.connected,
                comparison=twin.comparison(link.feedback) if args.smoke_compare else {}),ensure_ascii=False,indent=2),encoding='utf-8')
            close();return
        if closing and not link.busy:
            if link.status.startswith('FR5 정지 응답 확인'):close();return
            status.set(link.status)
        if link.mode!='SIM':
            if link.cal and record:
                try:twin.socket=socket_frame(link.cal,record)
                except (ValueError,KeyError,TypeError):pass
            c=twin.comparison(link.feedback) if fresh else {}
            status.set(link.status)
            feedback.set(f"실행: {link.mode}\nREAL: {'시험 피드백' if args.smoke_compare else '최신 실제 측정' if fresh else '미연결/지연 · 표시 제외'}\n단계: {link.plan.stage if link.plan else '정지/관찰'}\n"+(f"TCP 차이: {c['tcp_difference_mm']:.3f} mm\n자세 차이: {c['angle_difference_deg']:.3f}°\nSIM XYZ: {[round(v,2) for v in c['sim_tcp_mm_deg'][:3]]}\nREAL XYZ: {[round(v,2) for v in c['real_tcp_mm_deg'][:3]]}\n실제 힘/토크: {link.feedback.get('force_torque')}\n" if c else '실제 TCP 수신 대기\n')+'판넬/그리퍼 형상: 참고 모델\n실기 치수: 보정 파일의 값 사용')
            if now-last_draw>=.1 and root.winfo_viewable():
                last_draw=now;canvas.render();detail.delete('all')
                detail.create_text(12,15,anchor='nw',text='SIM 예측 / REAL 측정 · 같은 베이스 좌표',fill='#24394e')
                detail.create_text(12,50,anchor='nw',text='파란 반투명: 가상 목표 자세\n원래 색상: 실제 관절 자세\n노랑: SIM 열쇠 끝 · 초록: REAL TCP',fill='#24394e')
                if c:detail.create_text(12,125,anchor='nw',text=f"TCP 차이 {c['tcp_difference_mm']:.3f} mm\n자세 차이 {c['angle_difference_deg']:.3f}°",fill='#24795a')
                w,h=canvas.winfo_width(),canvas.winfo_height()
                positions=[]
                if twin.fitted:positions.append(('SIM 예측', [r[3] for r in twin.tcp()[:3]],'#edb833'))
                if fresh:positions.append(('REAL 측정',[v/1000 for v in link.feedback['tcp_mm_deg'][:3]],'#20ab72'))
                for label,p,color in positions:
                    xy=canvas.camera.project(p,w,h)
                    if xy:
                        x,y=xy[:2];canvas.create_oval(x-5,y-5,x+5,y+5,outline=color,width=3);canvas.create_text(x+10,y+(16 if label=='SIM 예측' else -16),text=label,anchor='w',fill=color)
            timer=root.after(33,update);return
        mission.inspect(record,now=now)
        if mission.latest and mission.stable>=3 and not mission.reference:
            perform(lambda:mission.set_reference(132));perform(scene.sync);view(False)
        if mission.reference and mission.latest:perform(scene.sync)
        if not (args.smoke_ui or args.smoke_compare) and (args.demo or args.smoke) and mission.reference and not requested:requested=True;perform(begin)
        servo.tick(record,now=now);tool.sync()
        if scene.r:
            f=scene.feedback()
            feedback.set(f"영상: {source.get()}\n단계: {servo.stage}\n좌우·상하 오차: {f['lateral_mm']:.3f} mm\n삽입 깊이: {f['depth_mm']:+.3f} / 22 mm\n열쇠 SIM 회전: {f['turn_deg']:.2f} / 90°\n축 기울기: {f['axis_error_deg']:.3f}°\n잠금 상태: {'해제' if scene.unlocked else '잠김'}\n열쇠 코드: {'일치' if correct.get() else '불일치'}\n그리퍼 앞 TCP: +{tool.offset_mm:.0f} mm\n접촉·토크: 기하학 모델, 센서 측정 아님")
            if now-last_draw>=.1 and root.winfo_viewable():
                canvas.render();draw_detail(f);last_draw=now
                w,h=canvas.winfo_width(),canvas.winfo_height()
                hole=canvas.camera.project([v/1000 for v in scene.hole()],w,h)
                tip=canvas.camera.project([v/1000 for v in tool.tip_mm()],w,h)
                if hole:
                    x,y=hole[:2];canvas.create_line(x-7,y,x+7,y,fill='#e44747',width=2);canvas.create_line(x,y-7,x,y+7,fill='#e44747',width=2)
                if tip:
                    x,y=tip[:2];canvas.create_oval(x-4,y-4,x+4,y+4,outline='#f7c447',width=2)
                    canvas.create_text(x+12,y+18,text='SIM 열쇠 끝',anchor='w',fill='#98601c')
        status.set(servo.status if servo.stage!='IDLE' else ('준비 완료 · 시작 버튼을 누르세요' if mission.reference else '두 마커 인식 대기'))
        if (args.smoke_ui and now-started>2) or (args.smoke and (scene.unlocked or failures or now-started>90 or (requested and not servo.enabled))):
            from tools.window_capture import capture_window
            root.update_idletasks()
            capture_window(root,ROOT/'.delivery/d455_key_panel.png')
            (ROOT/'.delivery/d455_key_panel_verification.json').write_text(json.dumps(dict(unlocked=scene.unlocked,stage=servo.stage,status=servo.status,feedback=servo.errors,gui_errors=failures,mode=link.mode,real_connected=link.client.connected,real_motion_active=link.plan is not None),ensure_ascii=False,indent=2),encoding='utf-8')
            close();return
        timer=root.after(33,update)
    if args.smoke_ui or args.smoke_compare:
        def choose(mode):pane.mode.set(mode);pane.change()
        root.after(500,lambda:choose('REAL'));root.after(1000,lambda:choose('SIM+REAL'))
    timer=root.after(33,update)
    if standalone:
        root.mainloop()
        if failures:raise RuntimeError('; '.join(failures))
    return root


def main():build_workspace()


if __name__=='__main__':main()
