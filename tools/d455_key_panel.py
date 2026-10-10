"""Electrical cabinet key insertion/turning demonstration; never robot transport."""
import argparse,json,math,sys,time
from pathlib import Path
from types import SimpleNamespace
import tkinter as tk
from tkinter import ttk
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from seer_control.aruco_board import BoardArmMission,make_board,validate_board,initialize_panel_arm
from seer_control.arm_simulation import ArmSimulator
from seer_control.arm_workspace import ArmCanvas
from seer_control.geometry3d import RobotDescription
from seer_control.fairino_model import installed
from seer_control.decision_log import DecisionJournal
from seer_control.vision_stream import read_frame
from seer_control.key_panel import KeyTool,KeyPanelPhysics,KeyPanelScene,KeyServo


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--demo',action='store_true');parser.add_argument('--smoke',action='store_true')
    args=parser.parse_args();root=tk.Tk();root.title('MOMA · 전기 판넬 열쇠 삽입·회전 SIM');root.geometry('1400x900')
    journal=DecisionJournal(ROOT/'.delivery/d455_key_panel')
    source=tk.StringVar(value='내장 모의 카메라');moving=tk.BooleanVar(value=False);lost=tk.BooleanVar(value=False);correct=tk.BooleanVar(value=True)
    status=tk.StringVar(value='기준 마커 준비 중');feedback=tk.StringVar();record=None;requested=False;last_draw=0.;started=time.time()
    fields={k:tk.StringVar(value=v) for k,v in [('x','0'),('y','100'),('speed','120'),('insert','10'),('turn','30')]}
    sim=ArmSimulator(RobotDescription.load(installed(Path.home()/'.seer_amr_console') or ROOT/'examples/fairino_fr5.urdf'),decision_journal=journal)
    initialize_panel_arm(sim);sim.physics=KeyPanelPhysics(sim.kin)
    sim.physics.decision_journal=journal
    mission=BoardArmMission(sim,journal)
    try:mission.config=validate_board(json.loads((ROOT/'.delivery/d455_board.json').read_text(encoding='utf-8')))
    except (OSError,ValueError):mission.config=make_board([dict(dictionary='DICT_4X4_1000',id=0),dict(dictionary='DICT_5X5_1000',id=0)],[100,100],130,'depth_estimate')
    tool=KeyTool(mission);scene=KeyPanelScene(mission,tool);servo=KeyServo(mission,scene)
    top=ttk.Frame(root,padding=10);top.pack(fill='x')
    ttk.Label(top,text='전기 판넬 · 열쇠 정렬 → 삽입 → 90° 회전 → 잠금해제',font=('맑은 고딕',16,'bold')).pack(anchor='w')
    ttk.Label(top,text='SIM 전용 | 일반 판넬 600×800×150 mm | 열쇠 초기 파지 | 슬롯 3.2×10 mm | 삽입 22 mm | 실물 치수·좌표 보정 전').pack(anchor='w',pady=4)
    controls=ttk.Frame(root,padding=(10,3));controls.pack(fill='x')
    combo=ttk.Combobox(controls,textvariable=source,values=['내장 모의 카메라','D455 실시간'],state='readonly',width=20);combo.pack(side='left',padx=4)
    def perform(fn):
        try:fn()
        except (ValueError,TypeError,KeyError) as error:servo.stop(str(error));status.set(str(error));mission.report(str(error))
    def switch(event=None):
        nonlocal record,requested
        if servo.enabled:servo.stop('영상 소스 변경')
        sim.physics.objects=[tool.key];scene.parts.clear();scene.meshes.clear();scene.walls.clear();scene.center=None;scene.r=None
        scene.unlocked=False;scene.rotor_deg=0;mission.reference=None;mission.latest=None;mission.stable=0;mission.stamp=None
        initialize_panel_arm(sim);tool.sync();record=None;requested=False;servo.stage='IDLE';servo.errors={}
    combo.bind('<<ComboboxSelected>>',switch)
    ttk.Checkbutton(controls,text='모의 판넬 이동·회전',variable=moving).pack(side='left',padx=6)
    ttk.Checkbutton(controls,text='마커 소실 시험',variable=lost).pack(side='left',padx=6)
    ttk.Checkbutton(controls,text='일치하는 열쇠',variable=correct).pack(side='left',padx=6)
    entryrow=ttk.Frame(root,padding=(10,3));entryrow.pack(fill='x')
    for k,label in [('x','구멍 좌우 mm'),('y','구멍 상하 mm'),('speed','보정 mm/s'),('insert','삽입 mm/s'),('turn','회전 °/s')]:
        ttk.Label(entryrow,text=label).pack(side='left',padx=(8,2));ttk.Entry(entryrow,textvariable=fields[k],width=7).pack(side='left')
    def begin():
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
    ttk.Button(buttons,text='열쇠 삽입 → 90° 잠금해제',command=lambda:perform(begin)).pack(side='left',padx=4)
    ttk.Button(buttons,text='열쇠 후퇴',command=lambda:perform(servo.withdraw)).pack(side='left',padx=4)
    ttk.Button(buttons,text='정지',command=lambda:servo.stop('사용자 정지')).pack(side='left',padx=4)
    def reset():
        if servo.enabled:raise ValueError('먼저 정지하세요.')
        scene.reset_lock();switch()
    ttk.Button(buttons,text='초기화',command=lambda:perform(reset)).pack(side='left',padx=4)
    def real_console():
        import subprocess
        servo.stop('실기 연결 화면으로 전환 · SIM 동작 정지')
        executable=Path(sys.executable).with_name('pythonw.exe')
        subprocess.Popen([str(executable if executable.exists() else sys.executable),str(ROOT/'tools/d455_key_panel_real.py'),'--ip','192.168.57.2'],cwd=str(ROOT),creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    ttk.Button(buttons,text='FR5 실기 연결 · 192.168.57.2',command=real_console).pack(side='left',padx=10)
    ttk.Label(root,textvariable=status,padding=(12,4),font=('맑은 고딕',12,'bold')).pack(fill='x')
    body=ttk.Frame(root);body.pack(fill='both',expand=True,padx=10)
    app=SimpleNamespace(font='맑은 고딕',arm_dev_display=tk.StringVar(value='SIM 개발'),arm_scene_geometry=scene.geometry)
    canvas=ArmCanvas(body,app,sim);canvas.pack(side='left',fill='both',expand=True)
    right=ttk.Frame(body,width=350);right.pack(side='right',fill='y',padx=(10,0));right.pack_propagate(False)
    ttk.Label(right,text='판넬 형상 참고 이미지 · 실측 모델 아님').pack(anchor='w')
    from PIL import Image,ImageTk
    image=Image.open(ROOT/'artifacts/electrical_panel/panel_reference.png');image.thumbnail((340,240))
    photo=ImageTk.PhotoImage(image);ttk.Label(right,image=photo).pack(pady=4)
    detail=tk.Canvas(right,width=340,height=230,bg='white',highlightthickness=1,highlightbackground='#b2bfcb');detail.pack(fill='x',pady=6)
    ttk.Label(right,textvariable=feedback,justify='left',wraplength=335,font=('맑은 고딕',10)).pack(anchor='w')
    def view(close=False):
        if scene.r:
            h=scene.hole();canvas.camera.target=[v/1000 for v in h] if close else [-.35,-.15,.43]
            canvas.camera.distance=.48 if close else 2.
        canvas.camera.yaw=-35;canvas.camera.pitch=16;canvas.render()
    ttk.Button(right,text='손잡이·열쇠 확대',command=lambda:view(True)).pack(fill='x',pady=3)
    ttk.Button(right,text='로봇 + 판넬 전체 보기',command=lambda:view(False)).pack(fill='x')
    ttk.Label(root,text='두 마커는 판넬에 함께 고정 · 구멍 위치는 두 마커 중간점 기준 · 황금색 열쇠 / 은색 실린더 / 검정 손잡이\n왼쪽 드래그: 회전 · 오른쪽 드래그: 이동 · 휠: 확대 | 로그: .delivery/d455_key_panel | 잠금기구·삽입 접촉은 기하학적 SIM 모델',padding=10).pack(fill='x')
    failures=[];root.report_callback_exception=lambda typ,value,tb:failures.append(str(value))
    def draw_detail(f):
        detail.delete('all');detail.create_text(12,12,anchor='nw',text='실제 FK 피드백 · 슬롯 정면 / 삽입 단면',fill='#24394e')
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
    def close():
        if servo.enabled:servo.stop('창 닫기')
        journal.close();root.destroy()
    root.protocol('WM_DELETE_WINDOW',close)
    def update():
        nonlocal record,requested,last_draw
        now=time.time();scene.correct_key=correct.get()
        if source.get()=='내장 모의 카메라':
            elapsed=now-started;motion=math.sin(elapsed*.35) if moving.get() else 0.
            record=dict(timestamp=now,vision_source='built_in_demo',board=dict(valid=not lost.get(),revision=mission.config['revision'],markers_used=2,
                geometry_source='depth_estimate',camera_xyz_m=[motion*.006,0,.6],rotation_vector_rad=[math.pi-motion*.01,0,0],reprojection_px=.1))
        else:record=read_frame(ROOT/'.delivery/d455_aruco.jsonl',ROOT/'.delivery/d455_aruco_live.json',previous=record)
        mission.inspect(record,now=now)
        if mission.latest and mission.stable>=3 and not mission.reference:
            perform(lambda:mission.set_reference(132));perform(scene.sync);view(False)
        if mission.reference and mission.latest:perform(scene.sync)
        if (args.demo or args.smoke) and mission.reference and not requested:requested=True;perform(begin)
        servo.tick(record,now=now);tool.sync()
        if scene.r:
            f=scene.feedback()
            feedback.set(f"영상: {source.get()}\n단계: {servo.stage}\n좌우·상하 오차: {f['lateral_mm']:.3f} mm\n삽입 깊이: {f['depth_mm']:+.3f} / 22 mm\n열쇠 실제 회전: {f['turn_deg']:.2f} / 90°\n축 기울기: {f['axis_error_deg']:.3f}°\n잠금 상태: {'해제' if scene.unlocked else '잠김'}\n열쇠 코드: {'일치' if correct.get() else '불일치'}\n그리퍼 앞 TCP: +{tool.offset_mm:.0f} mm\n접촉·토크: 기하학 모델, 센서 측정 아님")
            if now-last_draw>=.1:
                canvas.render();draw_detail(f);last_draw=now
                w,h=canvas.winfo_width(),canvas.winfo_height()
                hole=canvas.camera.project([v/1000 for v in scene.hole()],w,h)
                tip=canvas.camera.project([v/1000 for v in tool.tip_mm()],w,h)
                if hole:
                    x,y=hole[:2];canvas.create_line(x-7,y,x+7,y,fill='#e44747',width=2);canvas.create_line(x,y-7,x,y+7,fill='#e44747',width=2)
                if tip:
                    x,y=tip[:2];canvas.create_oval(x-4,y-4,x+4,y+4,outline='#f7c447',width=2)
                    canvas.create_text(x+12,y+18,text='실제 열쇠 끝',anchor='w',fill='#98601c')
        status.set(servo.status if servo.stage!='IDLE' else ('준비 완료 · 시작 버튼을 누르세요' if mission.reference else '두 마커 인식 대기'))
        if args.smoke and (scene.unlocked or failures or now-started>90 or (requested and not servo.enabled)):
            from window_capture import capture_window
            root.update_idletasks()
            capture_window(root,ROOT/'.delivery/d455_key_panel.png')
            (ROOT/'.delivery/d455_key_panel_verification.json').write_text(json.dumps(dict(unlocked=scene.unlocked,stage=servo.stage,status=servo.status,feedback=servo.errors,gui_errors=failures),ensure_ascii=False,indent=2),encoding='utf-8')
            close();return
        root.after(33,update)
    root.after(33,update);root.mainloop()
    if failures:raise RuntimeError('; '.join(failures))


if __name__=='__main__':main()
