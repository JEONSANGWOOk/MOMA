"""Small XY/yaw adjustments in map coordinates (metres/degrees)."""
import math
from tkinter import ttk, StringVar


def nudged(value,step,direction):
    value,step=float(value),float(step)
    if not math.isfinite(value) or not math.isfinite(step) or step<=0:
        raise ValueError('좌표와 증분에 유한한 숫자를 입력하세요. 증분은 0보다 커야 합니다.')
    result=value+step*direction
    if not math.isfinite(result):raise ValueError('좌표 범위를 확인하세요.')
    return f'{result:.6f}'


def add_nudge_controls(parent,variables,on_change,safe,enabled=lambda:True):
    frame=ttk.LabelFrame(parent,text='설비 미세조정 · 지도 X/Y, 회전 Yaw');frame.pack(fill='x',padx=10,pady=6)
    move=StringVar(frame,value='0.001');angle=StringVar(frame,value='0.1')
    steps=ttk.Frame(frame);steps.pack(fill='x',padx=6,pady=4)
    ttk.Label(steps,text='이동 증분 (m)').pack(side='left')
    ttk.Combobox(steps,textvariable=move,values=('0.001','0.005','0.01','0.05'),width=8).pack(side='left',padx=4)
    ttk.Label(steps,text='각도 증분 (°)').pack(side='left')
    ttk.Combobox(steps,textvariable=angle,values=('0.1','0.5','1','5'),width=8).pack(side='left',padx=4)
    row=ttk.Frame(frame);row.pack(fill='x',padx=6,pady=4)
    def change(index,direction):
        if not enabled():return
        values=[float(v.get()) for v in variables]
        if not all(math.isfinite(v) for v in values):raise ValueError('설비 좌표를 확인하세요.')
        var=variables[index];previous=var.get()
        var.set(nudged(previous,angle.get() if index==2 else move.get(),direction))
        try:on_change()
        except Exception:
            var.set(previous);raise
    for i,name in enumerate(('X','Y','Yaw')):
        for direction,label in ((-1,'−'),(1,'＋')):
            ttk.Button(row,text=f'{name} {label}',width=8,command=lambda i=i,d=direction:safe(lambda:change(i,d))).pack(side='left',padx=2)
    return frame
