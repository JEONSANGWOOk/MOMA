"""Local webcam preview and explicit closed/open reference registration."""
import time
import tkinter as tk
from tkinter import ttk,messagebox
from PIL import Image,ImageTk
from .webcam_doors import Observer


def show_webcam_doors(app,on_close=None):
    existing=getattr(app,'_webcam_doors_window',None)
    if existing is not None and existing.winfo_exists():existing.lift();return
    win=tk.Toplevel(app);app._webcam_doors_window=win
    win.title('내 PC 웹캠 · 설비 문 상태');win.geometry('960x780');win.minsize(720,600)
    ttk.Label(win,text='고정 웹캠: 고정 프레임 영역 → 문 영역 → 닫힘 등록 → 실제로 문을 열고 열림 등록').pack(pady=8)
    ttk.Label(win,text='문 전체의 이동이 보이는 영역을 선택하세요. 좌우 문은 CAD의 :1 / :2에 대응합니다.').pack()
    bar=ttk.Frame(win);bar.pack(fill='x',padx=10,pady=6)
    index=tk.StringVar(value='0');side=tk.StringVar(value='왼쪽문 :1');angle=tk.StringVar(value='90')
    ttk.Label(bar,text='웹캠 번호').pack(side='left')
    ttk.Combobox(bar,textvariable=index,values=('0','1','2','3'),state='readonly',width=3).pack(side='left',padx=4)
    ttk.Combobox(bar,textvariable=side,values=('왼쪽문 :1','오른쪽문 :2'),state='readonly',width=13).pack(side='left',padx=10)
    ttk.Label(bar,text='GUI 열림 표시 각도').pack(side='left')
    ttk.Spinbox(bar,textvariable=angle,from_=1,to=120,width=5).pack(side='left',padx=4)
    ttk.Label(bar,text='° (영상으로 측정한 각도가 아닙니다)').pack(side='left')
    canvas=tk.Canvas(win,bg='#17222d',highlightthickness=0);canvas.pack(fill='both',expand=True,padx=10,pady=6)
    status=tk.StringVar(value='웹캠 연결 중…');ttk.Label(win,textvariable=status).pack(pady=3)
    label=tk.StringVar(value='문 상태 확인 불가 · 기준 등록 필요');ttk.Label(win,textvariable=label).pack()
    state=dict(observer=None,select=None,start=None,mapping=None,photo=None,closed=False,job=None)

    def safe(fn):
        try:fn()
        except (ValueError,RuntimeError,ImportError) as e:messagebox.showerror('웹캠 문 판단',str(e),parent=win)

    def start():
        old=state['observer']
        if old and old.thread.is_alive():
            old.stop.set();win.after(150,lambda:safe(start));return
        observer=Observer(int(index.get()));state['observer']=observer;observer.thread.start()

    def select(slot):
        state['select']=slot;status.set('영상에서 영역의 한 모서리부터 반대 모서리까지 드래그하세요.')

    def selected_side():return 'left' if side.get().startswith('왼쪽') else 'right'

    def teach(kind):
        observer=state['observer']
        if observer is None:raise ValueError('웹캠을 먼저 연결하세요.')
        observer.teach(selected_side(),kind,float(angle.get()))
        status.set(('닫힘' if kind=='closed' else '열림')+' 기준 저장 완료')

    def drag_start(e):
        if state['select'] and state['mapping']:
            state['start']=(e.x,e.y)

    def drag(e):
        if state['start']:
            canvas.delete('selection');canvas.create_rectangle(*state['start'],e.x,e.y,outline='#ffcc55',width=3,tags='selection')

    def end(e):
        if not state['start'] or not state['mapping']:return
        ax,ay=state['start'];ox,oy,scale,w,h=state['mapping']
        x0,y0=max(0,min(w,round((min(ax,e.x)-ox)/scale))),max(0,min(h,round((min(ay,e.y)-oy)/scale)))
        x1,y1=max(0,min(w,round((max(ax,e.x)-ox)/scale))),max(0,min(h,round((max(ay,e.y)-oy)/scale)))
        def apply():
            state['observer'].select(state['select'],(x0,y0,x1-x0,y1-y0))
            state['select']=None;status.set('영역 등록 완료')
        safe(apply);state['start']=None;canvas.delete('selection')

    def tick():
        if state['closed'] or not win.winfo_exists():return
        observer=state['observer']
        if observer:
            frame,at,states,error=observer.snapshot()
            if frame is not None:
                image=Image.fromarray(observer.cv.cvtColor(frame,observer.cv.COLOR_BGR2RGB))
                w,h=image.size;cw,ch=max(1,canvas.winfo_width()),max(1,canvas.winfo_height());scale=min(cw/w,ch/h)
                size=(max(1,round(w*scale)),max(1,round(h*scale)));ox,oy=(cw-size[0])/2,(ch-size[1])/2
                state['mapping']=(ox,oy,scale,w,h);state['photo']=ImageTk.PhotoImage(image.resize(size),master=win)
                canvas.delete('frame');canvas.create_image(ox,oy,anchor='nw',image=state['photo'],tags='frame');canvas.tag_lower('frame')
                canvas.delete('regions')
                with observer.lock:regions={key:list(p['roi']) for key,p in observer.profiles.items()}
                for key,(x,y,rw,rh) in regions.items():
                    color='#55d7ff' if key=='anchor' else '#ffb54d'
                    canvas.create_rectangle(ox+x*scale,oy+y*scale,ox+(x+rw)*scale,oy+(y+rh)*scale,outline=color,width=2,tags='regions')
                    canvas.create_text(ox+x*scale+3,oy+y*scale+3,text=key,anchor='nw',fill=color,tags='regions')
            fresh=at and 0<=time.time()-at<1.5
            names={'OPEN':'열림','CLOSED':'닫힘','UNKNOWN':'확인 불가'}
            label.set(' / '.join(('왼쪽문' if s=='left' else '오른쪽문')+': '+names.get(states[s]['state'],'확인 불가') if fresh else ('왼쪽문' if s=='left' else '오른쪽문')+': 확인 불가' for s in ('left','right')))
            if error and not state['select']:status.set(error)
        state['job']=win.after(100,tick)

    buttons=ttk.Frame(win);buttons.pack(pady=8)
    for text,fn in [('웹캠 연결',start),('고정 프레임 영역',lambda:select('anchor')),('문 영역 선택',lambda:select(selected_side())),('닫힘 등록',lambda:teach('closed')),('열림 등록',lambda:teach('open'))]:
        ttk.Button(buttons,text=text,command=lambda fn=fn:safe(fn)).pack(side='left',padx=4)
    ttk.Label(win,text='등록한 상태가 0.6초 이상 유지될 때 반영합니다. 가림·이동·수신 지연 시 CAD는 마지막 각도를 유지합니다.').pack(pady=(0,8))
    def close():
        state['closed']=True
        if state['observer']:state['observer'].stop.set()
        if state['job']:win.after_cancel(state['job'])
        win.destroy()
        if on_close:on_close()
    win.protocol('WM_DELETE_WINDOW',close)
    win.bind('<Destroy>',lambda e:state['observer'].stop.set() if e.widget==win and state['observer'] else None,add='+')
    canvas.bind('<ButtonPress-1>',drag_start);canvas.bind('<B1-Motion>',drag);canvas.bind('<ButtonRelease-1>',end)
    app.world3d.webcam_doors_enabled.set(True);app._spatial_save()
    win.after(100,lambda:safe(start));win.after(150,tick)
