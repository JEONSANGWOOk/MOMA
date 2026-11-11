"""Floating one-touch launcher for the existing departure sequence."""
import math
import tkinter as tk
from tkinter import ttk


def link_busy(app):
    link=getattr(app,'_departure_link',None)
    thread=getattr(link,'thread',None)
    return bool(thread and thread.is_alive())


def show_start_popup(app):
    previous=getattr(app,'_link_start_popup',None)
    if previous is not None and previous.winfo_exists():
        previous.deiconify();previous.lift();return previous
    win=tk.Toplevel(app);app._link_start_popup=win
    win.title('자동 연동 Start');win.geometry('350x400');win.minsize(300,370)
    win.configure(bg='#edf2f7');win.attributes('-topmost',True)
    topmost=tk.BooleanVar(win,value=True)
    ttk.Checkbutton(win,text='항상 위',variable=topmost,
                    command=lambda:win.attributes('-topmost',topmost.get())).pack(anchor='e',padx=16,pady=10)
    tk.Label(win,text='LM2 → LM1 → main2·main3 → LM2',bg='#edf2f7',fg='#24374c',
             font=('맑은 고딕',11,'bold')).pack(pady=4)
    canvas=tk.Canvas(win,width=260,height=250,bg='#edf2f7',highlightthickness=0,takefocus=True)
    canvas.pack(pady=4)
    status=tk.StringVar(win,value='Start를 누르면 기존 연동을 바로 실행합니다.')
    tk.Label(win,textvariable=status,wraplength=310,bg='#edf2f7',fg='#384c63',
             font=('맑은 고딕',10)).pack(padx=12,pady=6)
    pressed=False;launching=False;job=None
    def paint(down=False):
        canvas.delete('all');busy=launching or link_busy(app)
        canvas.configure(cursor='arrow' if busy else 'hand2')
        canvas.create_oval(35,40,235,240,fill='#bac5d0',outline='')
        canvas.create_oval(30,24,230,224,fill='#123d27' if not busy else '#53616c',outline='#ffffff',width=3)
        offset=7 if down else 0
        for i in range(28):
            color=('#%02x%02x%02x'%(29+i//2,112+i*2,63+i)) if not busy else '#7b8b98'
            canvas.create_oval(35+i*.65,29+offset+i*.65,225-i*.65,213+offset-i*.65,
                               fill=color,outline='')
        canvas.create_arc(46,38+offset,214,201+offset,start=35,extent=110,style='arc',outline='#a6efba',width=3)
        canvas.create_text(130,122+offset,text='Start',fill='white',font=('Segoe UI',30,'bold'))
    def inside(event):return math.hypot(event.x-130,event.y-124)<=96
    def launch():
        nonlocal launching
        if launching or link_busy(app):return
        launching=True;paint(True)
        try:
            from .departure_link import show_departure_link
            app.guarded(lambda:show_departure_link(app,autostart=True))
        finally:
            launching=False
            if win.winfo_exists():paint()
    def press(event):
        nonlocal pressed
        pressed=inside(event) and not link_busy(app)
        if pressed:canvas.focus_set();paint(True)
    def release(event):
        nonlocal pressed
        run=pressed and inside(event);pressed=False;paint()
        if run:launch()
    canvas.bind('<ButtonPress-1>',press);canvas.bind('<ButtonRelease-1>',release)
    canvas.bind('<Return>',lambda event:launch());canvas.bind('<space>',lambda event:launch())
    def update():
        nonlocal job
        if not win.winfo_exists():return
        status.set(getattr(app,'_departure_link_message','Start를 누르면 기존 연동을 바로 실행합니다.'))
        if not pressed:paint()
        job=win.after(250,update)
    def close():
        if job is not None:win.after_cancel(job)
        app._link_start_popup=None;win.destroy()
    win.protocol('WM_DELETE_WINDOW',close);update()
    return win
