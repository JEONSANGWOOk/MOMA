"""Explicit preview and confirmation of equipment placement from live LiDAR."""
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from .equipment_placement import context, signature, section, placed, quality, refine, body_outline, polygon_gap


def show_placement(app):
    view=app.world3d
    if app.real and not app.connected:raise ValueError('실기 AMR 조회 연결 후 설비 배치를 시작하세요.')
    mode_real=bool(app.real)
    def guard_mode():
        if bool(app.real)!=mode_real:raise ValueError('운전 모드가 바뀌었습니다. 배치 창을 다시 여세요.')
    if view.cad_asset is None:raise ValueError('설비 CAD를 먼저 불러오세요.')
    existing=getattr(app,'_equipment_placement_window',None)
    if existing is not None and existing.winfo_exists():existing.lift();return
    win=tk.Toplevel(app);app._equipment_placement_window=win
    win.title(('실기 LiDAR' if mode_real else 'SIM 수동/가상 LiDAR')+' 설비 배치 · 미리보기 → 확정');win.geometry('960x720');win.transient(app)
    ttk.Label(win,text='1. 점군 갱신 → 2. 설비 위치를 클릭하고 방향 조절 → 3. 근처 점군에 맞춤 → 4. 배치 확정').pack(pady=8)
    ttk.Label(win,text='초록: '+('실제' if mode_real else '시뮬레이션')+' LiDAR / 주황: 설비 단면. SIM은 점군 없이 수동 확정 가능. Z·Roll·Pitch 유지.').pack()
    canvas=tk.Canvas(win,bg='#17222d',highlightthickness=0);canvas.pack(fill='both',expand=True,padx=10,pady=8)
    row=ttk.Frame(win);row.pack(fill='x',padx=10)
    variables=[tk.StringVar(value=f'{view.cad_origin[i] if i<2 else math.degrees(view.cad_origin[5]):.3f}') for i in range(3)]
    for label,var in zip(('X (m)','Y (m)','Yaw (°)'),variables):
        ttk.Label(row,text=label).pack(side='left',padx=4);ttk.Entry(row,textvariable=var,width=10).pack(side='left')
    gap_row=ttk.Frame(win);gap_row.pack(fill='x',padx=10,pady=5)
    target_gap=tk.StringVar(value=str(getattr(view,'equipment_gap_target_m',.21)*100))
    ttk.Label(gap_row,text='목표 간격 (cm)').pack(side='left')
    ttk.Entry(gap_row,textvariable=target_gap,width=8).pack(side='left',padx=6)
    gap_text=tk.StringVar(value='현재 간격: 계산 대기 · 차체 외곽 ↔ 설비 단면')
    ttk.Label(gap_row,textvariable=gap_text).pack(side='left',padx=8)
    robot_outline=body_outline(app,view)

    def update_gap():
        try:
            target=float(target_gap.get())/100
            if not math.isfinite(target) or target<=0:raise ValueError()
            live=app.current_state();rx,ry,theta=(live.get(k) for k in ('x','y','theta'))
            if (mode_real and (not app.connected or time.monotonic()-app.last_state>2)) or not all(type(v) in (int,float) and math.isfinite(v) for v in (rx,ry,theta)):
                gap_text.set('현재 간격: 최신 로봇 위치 대기');return
            if not state['outline']:return
            robot=placed(robot_outline,(rx,ry,theta))
            gap=polygon_gap(robot,placed(state['outline'],pose()))
            gap_text.set(f'현재 약 {gap*100:.1f} cm · 목표 {target*100:.1f} cm · 차이 {(gap-target)*100:+.1f} cm (모델 추정)')
        except (ValueError,TypeError):gap_text.set('목표 간격/배치 좌표를 확인하세요.')

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
        update_gap()
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
            if not mode_real and not state['cloud']:
                status.set('SIM 수동 배치 · 점군 없이 좌표/방향을 조절하고 확정할 수 있습니다.');return
            status.set(f"{'SIM 수동 확정 가능' if not mode_real else '확정 가능' if q['ok'] else '위치·방향 조정 필요'} · 일치 {q['matches']}점 / 근접점 {q['coverage']:.0%} · 오차 {q['error']*1000:.0f} mm · 점군은 스냅샷입니다")

    def launch(work):
        if state['busy']:return
        state['busy']=True;status.set('계산 중…')
        def run():
            try:jobs.put(('ok',work()))
            except Exception as e:jobs.put(('error',str(e)))
        threading.Thread(target=run,daemon=True,name='equipment-placement').start()

    def refresh():
        guard_mode()
        if mode_real:
            if not app.connected or time.monotonic()-app.last_laser_rx>2:raise ValueError('최신 실제 LiDAR 수신이 필요합니다.')
            raw=list(app.real_laser_points)
            cloud=list(raw)  # same MAP frame as robot pose and CAD placement
        else:cloud=list(getattr(app,'scan_points',[]))
        cloud=[(float(x),float(y)) for x,y in cloud if math.isfinite(x) and math.isfinite(y)]
        if mode_real and len(cloud)<12:raise ValueError('실제 LiDAR 점이 부족합니다.')
        origin=list(view.cad_origin);asset=view.cad_asset;scale=view.scales['cad']
        ctx=context(app);sig=signature(app,view);stamp=app.last_laser_rx if mode_real else time.monotonic()
        launch(lambda:dict(outline=section(asset,origin,scale),cloud=cloud[::max(1,len(cloud)//1500)],context=ctx,signature=sig,scan_at=stamp))

    def fit():
        if not state['outline']:raise ValueError('점군 갱신을 먼저 실행하세요.')
        if len(state['cloud'])<12:raise ValueError('맞춤에는 점군이 필요합니다. SIM은 좌표 조정 후 수동 확정할 수 있습니다.')
        p=pose();outline=list(state['outline']);cloud=list(state['cloud'])
        launch(lambda:dict(pose=refine(outline,cloud,p)))

    def confirm():
        if state['busy']:raise ValueError('계산 완료 후 확정하세요.')
        guard_mode()
        if state['signature']!=signature(app,view):raise ValueError('지도·CAD·좌표 설정이 바뀌었습니다. 점군을 갱신하세요.')
        if mode_real and (not app.connected or time.monotonic()-app.last_laser_rx>2):raise ValueError('LiDAR 수신이 끊겼습니다.')
        if mode_real and time.monotonic()-state['scan_at']>30:raise ValueError('점군이 오래되었습니다. 점군 갱신 후 다시 확인하세요.')
        target=float(target_gap.get())/100
        if not math.isfinite(target) or target<=0:raise ValueError('목표 간격에 0보다 큰 숫자를 입력하세요.')
        p=pose();q=quality(state['outline'],state['cloud'],p)
        if mode_real and not q['ok']:raise ValueError('설비 외곽과 일치하는 LiDAR 점이 부족합니다. 위치·방향을 조정하세요.')
        view.equipment_gap_target_m=target
        view.cad_origin[0],view.cad_origin[1],view.cad_origin[5]=p
        view.cad_placement_confirmation=dict(signature=signature(app,view),time=time.time(),quality=q,source='real_lidar_operator_confirmed' if mode_real else 'sim_operator_confirmed')
        view.reset_real_doors();app._spatial_save();
        if not mode_real:app._spatial_sync_cad_lidar()
        view.refresh()
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
        update_gap()
        win.after(500,results)

    from .placement_nudge import add_nudge_controls
    add_nudge_controls(win,variables,redraw,safe,enabled=lambda:not state['busy'])
    from .marker_placement import show as show_marker_placement
    ttk.Button(win,text='마커 자동 배치 설정',command=lambda:show_marker_placement(app)).pack(pady=4)
    buttons=ttk.Frame(win);buttons.pack(pady=8)
    for text,fn in [('점군 갱신',refresh),('미리보기 적용',redraw),('근처 점군에 맞춤',fit),('배치 확정',confirm),('취소',win.destroy)]:
        ttk.Button(buttons,text=text,command=lambda fn=fn:safe(fn)).pack(side='left',padx=4)
    canvas.bind('<Button-1>',click);canvas.bind('<Configure>',lambda e:redraw())
    canvas.bind('<MouseWheel>',zoom);canvas.bind('<ButtonPress-3>',pan_start);canvas.bind('<B3-Motion>',pan)
    win.after(100,lambda:safe(refresh));win.after(100,results)
