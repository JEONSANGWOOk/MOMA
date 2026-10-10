"""D455 camera motion -> existing FR5 ArmSimulator/ArmCanvas. Offline only."""
import argparse
import math
import queue
import sys
import time
from pathlib import Path
from types import SimpleNamespace
import tkinter as tk
from tkinter import ttk

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from seer_control.aruco_arm_follow import CameraArmFollower,latest_record
from seer_control.arm_simulation import ArmSimulator
from seer_control.arm_workspace import ArmCanvas
from seer_control.decision_log import DecisionJournal
from seer_control.geometry3d import RobotDescription
from seer_control.fairino_model import installed


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--log',type=Path,default=ROOT/'.delivery/d455_aruco.jsonl')
    parser.add_argument('--smoke',action='store_true')
    args=parser.parse_args()
    root=tk.Tk();root.title('MOMA · D455 카메라 이동 → 가상 로봇팔');root.geometry('1160x760')
    journal=DecisionJournal(ROOT/'.delivery/d455_arm_sim')
    path=installed(Path.home()/'.seer_amr_console') or ROOT/'examples/fairino_fr5.urdf'
    sim=ArmSimulator(RobotDescription.load(path),decision_journal=journal)
    sim.q=sim.kin.clamp([math.radians(v) for v in [0,-90,90,-90,-90,0]])
    follower=CameraArmFollower(sim,journal)
    header=ttk.Frame(root,padding=10);header.pack(fill='x')
    ttk.Label(header,text='카메라를 손으로 움직여 가상 로봇팔 테스트',font=('맑은 고딕',14,'bold')).pack(anchor='w')
    ttk.Label(header,text='마커는 고정하세요 · 기준 대비 이동량 50% 반영 · TCP 자세 유지 · 실제 로봇 통신 없음').pack(anchor='w',pady=4)
    controls=ttk.Frame(root,padding=(10,0));controls.pack(fill='x')
    ttk.Label(controls,text='기준 마커').pack(side='left')
    selected=tk.StringVar();selector=ttk.Combobox(controls,textvariable=selected,state='readonly',width=28)
    selector.pack(side='left',padx=8)
    status=tk.StringVar(value='카메라 영상 수신 대기');tcp=tk.StringVar();movement=tk.StringVar()
    automatic=True;mapping={};previous=time.monotonic()
    def command(fn):
        nonlocal automatic
        automatic=False
        try:fn()
        except ValueError as error:status.set(str(error));follower.report(str(error))
    def start():
        follower.reference();follower.start()
    ttk.Button(controls,text='기준 위치 설정',command=lambda:command(follower.reference)).pack(side='left',padx=3)
    ttk.Button(controls,text='추적 시작',command=lambda:command(start)).pack(side='left',padx=3)
    ttk.Button(controls,text='추적 정지',command=lambda:command(follower.stop)).pack(side='left',padx=3)
    def change(event=None):
        nonlocal automatic
        automatic=False;follower.select(mapping.get(selected.get()))
    selector.bind('<<ComboboxSelected>>',change)
    ttk.Label(root,textvariable=status,font=('맑은 고딕',12,'bold'),padding=10).pack(fill='x')
    ttk.Label(root,textvariable=movement,padding=(10,0)).pack(fill='x')
    app=SimpleNamespace(font='맑은 고딕',arm_dev_display=tk.StringVar(value='SIM 개발'))
    canvas=ArmCanvas(root,app,sim);canvas.pack(fill='both',expand=True,padx=10,pady=8)
    ttk.Label(root,textvariable=tcp,padding=10).pack(fill='x')
    ttk.Label(root,text='미검출·수신 지연·범위 초과 시 정지합니다. 마커를 다시 보여준 뒤 추적 시작을 누르세요.\n시뮬레이션 축: 카메라 앞/뒤 → 팔 X, 좌/우 → 팔 Y, 위/아래 → 팔 Z. 마커 이동도 상대 이동으로 반영됩니다.',padding=(10,0,10,10)).pack(fill='x')
    errors=[]
    root.report_callback_exception=lambda typ,value,tb:errors.append(str(value))
    def close():
        follower.stop();journal.close();root.destroy()
    root.protocol('WM_DELETE_WINDOW',close)
    def update():
        nonlocal previous,automatic
        now=time.monotonic();dt=now-previous;previous=now
        record=latest_record(args.log)
        fresh=record and -.1<=time.time()-record.get('timestamp',0)<=.7
        if fresh:
            for marker in record.get('markers',[]):
                key=(marker.get('dictionary'),marker.get('id'))
                name=f'{str(key[0]).removeprefix("DICT_")} / ID {key[1]}'
                mapping[name]=key
            selector.configure(values=sorted(mapping))
            if not selected.get() and mapping:
                selected.set(sorted(mapping)[0]);follower.select(mapping[selected.get()])
        changed=follower.tick(record,dt)
        if automatic and follower.stable>=3:
            automatic=False
            try:start()
            except ValueError as error:follower.report(str(error))
        status.set(follower.status)
        movement.set('가상 팔 목표 이동량 [mm]: '+', '.join(f'{v:+.1f}' for v in follower.delta_mm)+' · 범위 150mm · 최대 40mm/s')
        tcp.set('TCP [mm / 도]: '+', '.join(f'{v:.1f}' for v in sim.kin.pose(sim.q)))
        if changed:canvas.render()
        while True:
            try:journal.pending.get_nowait()
            except queue.Empty:break
        root.after(100,update)
    root.update();canvas.fit();root.after(100,update)
    if args.smoke:
        from window_capture import capture_window
        root.after(1200,lambda:capture_window(root,ROOT/'.delivery/d455_arm_sim_preview.png'))
        root.after(1600,close)
    root.mainloop()
    if errors:raise RuntimeError('; '.join(errors))
    if args.smoke:print('PASS: offline camera arm simulator GUI, existing FR5 model and ArmCanvas')


if __name__=='__main__':main()
