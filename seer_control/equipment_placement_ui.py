"""Explicit preview and confirmation of equipment placement from live LiDAR."""
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from .equipment_placement import context, signature, section, placed, quality, refine


def show_placement(app):
    view=app.world3d
    if not app.real or not app.connected:raise ValueError('실기 AMR 조회 연결 후 설비 배치를 시작하세요.')
    if view.cad_asset is None:raise ValueError('설비 CAD를 먼저 불러오세요.')
    existing=getattr(app,'_equipment_placement_window',None)
    if existing is not None and existing.winfo_exists():existing.lift();return
    win=tk.Toplevel(app);app._equipment_placement_window=win
    win.title('LiDAR 설비 배치 · 미리보기 → 확정');win.geometry('960x720');win.transient(app)
    ttk.Label(win,text='1. 점군 갱신 → 2. 설비 위치를 클릭하고 방향 조절 → 3. 근처 점군에 맞춤 → 4. 배치 확정').pack(pady=8)
    ttk.Label(win,text='초록: 실제 LiDAR / 주황: 설비 단면. 휠 확대 · 우드래그 이동. Z·Roll·Pitch는 유지됩니다.').pack()
    canvas=tk.Canvas(win,bg='#17222d',highlightthickness=0);canvas.pack(fill='both',expand=True,padx=10,pady=8)
    row=ttk.Frame(win);row.pack(fill='x',padx=10)
    variables=[tk.StringVar(value=f'{view.cad_origin[i] if i<2 else math.degrees(view.cad_origin[5]):.3f}') for i in range(3)]
    for label,var in zip(('X (m)','Y (m)','Yaw (°)'),variables):
        ttk.Label(row,text=label).pack(side='left',padx=4);ttk.Entry(row,textvariable=var,width=10).pack(side='left')
    state=dict(cloud=[],outline=[],context=None,signature=None,scan_at=0.,quality=None,busy=False)
    status=tk.StringVar(value='실제 점군을 읽는 중…');ttk.Label(win,textvariable=status).pack(pady=5)
    jobs=queue.Queue()

    def pose():
        values=[float(v.get()) for v in variables]
        if not all(math.isfinite(v) for v in values):raise ValueError('위치·방향에 유한한 숫자를 입력하세요.')
        return values[0],values[1],math.radians(values[2])

    def redraw():
        if not win.winfo_exists():return
        canvas.delete('all')
        try:p=pose()
        except ValueError:return
        polygon=placed(state['outline'],p) if state['outline'] else []
        points=state['cloud']+polygon
        if not points:return
        xs,ys=zip(*points);cx,cy=(min(xs)+max(xs))/2,(min(ys)+max(ys))/2
        scale=min(max(1,canvas.winfo_width()-40)/max(1,max(xs)-min(xs)),max(1,canvas.winfo_height()-40)/max(1,max(ys)-min(ys)))
        if 'camera' in state:cx,cy,scale=state['camera']
        state['screen']=(cx,cy,scale)
        def xy(a):return canvas.winfo_width()/2+(a[0]-cx)*scale,canvas.winfo_height()/2-(a[1]-cy)*scale
        for a in state['cloud']:
            x,y=xy(a);canvas.create_oval(x-1,y-1,x+1,y+1,fill='#42dd91',outline='')
        if polygon:
            canvas.create_line(*[v for a in polygon+[polygon[0]] for v in xy(a)],fill='#ffb54d',width=3)
            x,y=xy(p[:2]);canvas.create_text(x,y,text='설비 CAD',fill='#ffcf87',anchor='s')
        if polygon:
            q=quality(state['outline'],state['cloud'],p);state['quality']=q
            status.set(f"{'확정 가능' if q['ok'] else '위치·방향 조정 필요'} · 일치 {q['matches']}점 / 근접점 {q['coverage']:.0%} · 오차 {q['error']*1000:.0f} mm · 점군은 스냅샷입니다")

    def launch(work):
        if state['busy']:return
        state['busy']=True;status.set('계산 중…')
        def run():
            try:jobs.put(('ok',work()))
            except Exception as e:jobs.put(('error',str(e)))
        threading.Thread(target=run,daemon=True,name='equipment-placement').start()

    def refresh():
        if not app.connected or time.monotonic()-app.last_laser_rx>2:raise ValueError('최신 실제 LiDAR 수신이 필요합니다.')
        raw=list(app.real_laser_points)
        cloud=raw if 'laser_beams(WORLD)' in app.lidar_source else [app._lidar_map_xy(x,y) for x,y in raw]
        cloud=[(float(x),float(y)) for x,y in cloud if math.isfinite(x) and math.isfinite(y)]
        if len(cloud)<12:raise ValueError('실제 LiDAR 점이 부족합니다.')
        origin=list(view.cad_origin);asset=view.cad_asset;scale=view.scales['cad']
        ctx=context(app);sig=signature(app,view);stamp=app.last_laser_rx
        launch(lambda:dict(outline=section(asset,origin,scale),cloud=cloud[::max(1,len(cloud)//1500)],context=ctx,signature=sig,scan_at=stamp))

    def fit():
        if not state['outline']:raise ValueError('점군 갱신을 먼저 실행하세요.')
        p=pose();outline=list(state['outline']);cloud=list(state['cloud'])
        launch(lambda:dict(pose=refine(outline,cloud,p)))

    def confirm():
        if state['busy']:raise ValueError('계산 완료 후 확정하세요.')
        if not app.real:raise ValueError('실기 모드에서만 확정할 수 있습니다.')
        if state['signature']!=signature(app,view):raise ValueError('지도·CAD·좌표 설정이 바뀌었습니다. 점군을 갱신하세요.')
        if not app.connected or time.monotonic()-app.last_laser_rx>2:raise ValueError('LiDAR 수신이 끊겼습니다.')
        if time.monotonic()-state['scan_at']>30:raise ValueError('점군이 오래되었습니다. 점군 갱신 후 다시 확인하세요.')
        p=pose();q=quality(state['outline'],state['cloud'],p)
        if not q['ok']:raise ValueError('설비 외곽과 일치하는 LiDAR 점이 부족합니다. 위치·방향을 조정하세요.')
        view.cad_origin[0],view.cad_origin[1],view.cad_origin[5]=p
        view.cad_placement_confirmation=dict(signature=signature(app,view),time=time.time(),quality=q,source='real_lidar_operator_confirmed')
        view.reset_real_doors();app._spatial_save();view.refresh()
        app.log('3D',f'LiDAR 설비 배치 확정 · x={p[0]:.3f} y={p[1]:.3f} yaw={math.degrees(p[2]):.2f}°')
        win.destroy()

    def click(event):
        if state['busy'] or 'screen' not in state:return
        cx,cy,scale=state['screen']
        variables[0].set(f'{cx+(event.x-canvas.winfo_width()/2)/scale:.3f}')
        variables[1].set(f'{cy-(event.y-canvas.winfo_height()/2)/scale:.3f}');redraw()

    def zoom(event):
        if 'screen' not in state:return
        cx,cy,scale=state['screen'];factor=1.2 if event.delta>0 else 1/1.2
        state['camera']=(cx,cy,max(1,min(5000,scale*factor)));redraw()

    def pan_start(event):
        if 'screen' in state:state['pan']=(event.x,event.y,state['screen'])

    def pan(event):
        if 'pan' not in state:return
        x,y,(cx,cy,scale)=state['pan']
        state['camera']=(cx-(event.x-x)/scale,cy+(event.y-y)/scale,scale);redraw()

    def safe(fn):
        try:fn()
        except (ValueError,TypeError) as e:messagebox.showerror('설비 배치',str(e),parent=win)

    def results():
        if not win.winfo_exists():return
        try:
            kind,data=jobs.get_nowait();state['busy']=False
            if kind=='error':status.set(data)
            elif 'pose' in data:
                for var,value in zip(variables,(data['pose'][0],data['pose'][1],math.degrees(data['pose'][2]))):var.set(f'{value:.3f}')
                redraw()
            else:state.update(data);redraw()
        except queue.Empty:pass
        win.after(100,results)

    from .placement_nudge import add_nudge_controls
    add_nudge_controls(win,variables,redraw,safe,enabled=lambda:not state['busy'])
    buttons=ttk.Frame(win);buttons.pack(pady=8)
    for text,fn in [('점군 갱신',refresh),('미리보기 적용',redraw),('근처 점군에 맞춤',fit),('배치 확정',confirm),('취소',win.destroy)]:
        ttk.Button(buttons,text=text,command=lambda fn=fn:safe(fn)).pack(side='left',padx=4)
    canvas.bind('<Button-1>',click);canvas.bind('<Configure>',lambda e:redraw())
    canvas.bind('<MouseWheel>',zoom);canvas.bind('<ButtonPress-3>',pan_start);canvas.bind('<B3-Motion>',pan)
    win.after(100,lambda:safe(refresh));win.after(100,results)
