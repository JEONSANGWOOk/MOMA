"""Explicit operator-selected localization reset; never a navigation command."""
import math
import tkinter as tk
from tkinter import ttk,messagebox


def pose_payload(x,y,degrees):
    values=[float(v) for v in (x,y,degrees)]
    if not all(math.isfinite(v) for v in values):raise ValueError('X, Y, 방향에 유효한 숫자를 입력하세요.')
    angle=math.radians(values[2]);angle=math.atan2(math.sin(angle),math.cos(angle))
    return dict(x=values[0],y=values[1],angle=angle)


def show(app):
    if not app._reloc_ready():return
    previous=getattr(app,'_manual_reloc_window',None)
    if previous is not None and previous.winfo_exists():previous.lift();return
    win=tk.Toplevel(app);app._manual_reloc_window=win
    win.title('실기 수동 재배치 · 로봇 위치/방향 재추정');win.geometry('550x400');win.transient(app)
    body=ttk.Frame(win,padding=16);body.pack(fill='both',expand=True)
    ttk.Label(body,text='실제 로봇이 있는 위치와 방향을 지정하세요.',font=('맑은 고딕',12,'bold')).pack(anchor='w')
    ttk.Label(body,text='화면 표시만 이동하는 기능이 아니라 로봇의 자기위치를 재설정합니다. 로봇이 정지한 상태에서 지정하고, LiDAR·지도 정합 확인 후 확정하세요.',wraplength=510).pack(anchor='w',pady=(6,12))
    state=app.current_state();fields={}
    def initial(key):
        v=state.get(key,0.)
        return float(v) if type(v) in (int,float) and math.isfinite(v) else 0.
    form=ttk.Frame(body);form.pack(fill='x')
    for index,(key,label,value) in enumerate((('x','X (m)',initial('x')),('y','Y (m)',initial('y')),('angle','방향 θ (°)',math.degrees(initial('theta'))))):
        ttk.Label(form,text=label).grid(row=0,column=index,sticky='w',padx=4)
        fields[key]=tk.StringVar(win,value=f'{value:.3f}')
        ttk.Entry(form,textvariable=fields[key],width=17).grid(row=1,column=index,sticky='ew',padx=4,pady=4)
        form.columnconfigure(index,weight=1)
    def apply():
        try:payload=pose_payload(*(fields[k].get() for k in ('x','y','angle')))
        except (ValueError,TypeError) as error:messagebox.showerror('수동 재배치',str(error),parent=win);return
        app._send_reloc(payload['x'],payload['y'],payload['angle'])
    def pick():
        if not app._reloc_ready():return
        app.tabs.select(app.operation_page);app.view_mode.set('2D');app.start_manual_reloc();app.draw_map()
        win.destroy()
    row=ttk.Frame(body);row.pack(fill='x',pady=12)
    ttk.Button(row,text='입력 좌표로 재배치 요청',command=apply).pack(side='left',expand=True,fill='x',padx=(0,4))
    ttk.Button(row,text='지도에서 위치·방향 지정',command=pick).pack(side='left',expand=True,fill='x')
    ttk.Label(body,text='지도 지정: 실제 위치에서 누른 채 로봇이 향하는 방향으로 드래그합니다.',wraplength=510).pack(anchor='w')
    status=tk.StringVar(win);ttk.Label(body,textvariable=status,wraplength=510).pack(fill='x',pady=12)
    row=ttk.Frame(body);row.pack(fill='x')
    ttk.Button(row,text='정합 확인 후 위치 확정',command=app.confirm_relocation).pack(side='left',expand=True,fill='x',padx=(0,4))
    ttk.Button(row,text='재배치 취소',command=app.cancel_relocation).pack(side='left',expand=True,fill='x')
    def refresh():
        if not win.winfo_exists():return
        status.set(app.reloc_info.cget('text'))
        win.after(250,refresh)
    refresh()
