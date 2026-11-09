"""Marker registration for display only. Never sends robot commands."""
import math
import time
from .geometry3d import transform,multiply
from .arm_physics import inverse
from .equipment_placement import context,signature


def rigid(value):
    if not isinstance(value,(list,tuple)) or len(value)!=4:raise ValueError('4x4 marker matrix required')
    if any(not isinstance(row,(list,tuple)) or len(row)!=4 for row in value):raise ValueError('4x4 marker matrix required')
    if any(type(v) not in (int,float) or not math.isfinite(v) for row in value for v in row):raise ValueError('Invalid marker matrix')
    if any(abs(a-b)>1e-6 for a,b in zip(value[3],(0,0,0,1))):raise ValueError('Invalid homogeneous matrix')
    for i in range(3):
        for j in range(3):
            if abs(sum(value[k][i]*value[k][j] for k in range(3))-(i==j))>1e-4:raise ValueError('Marker rotation is not rigid')
    a,b,c=value[:3]
    det=a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0])
    if abs(det-1)>1e-4:raise ValueError('Reflected marker frame')
    return value


def solve(reference,measured,origin):
    delta=multiply(rigid(measured),inverse(rigid(reference)))
    # One closed-door marker defines equipment XY/yaw. Keep authored Z/tilt.
    if abs(delta[2][2]-1)>.015:raise ValueError('Marker tilt changed too far; re-detect with door closed')
    result=multiply(delta,transform(origin[:3],origin[3:]))
    return [result[0][3],result[1][3],origin[2],origin[3],origin[4],math.atan2(result[1][0],result[0][0])]


def solve_anchor(measured,center,origin,scale=1.):
    """Place CAD from this marker measurement, independent of a previous pose.

    Teaching uses marker X across the door and marker Y as the hinge axis
    (Rotate(0,angle,0, LOCAL)). CAD Z is vertical. Use the measured X heading,
    with near-horizontal X and near-vertical Y required; never guess yaw from
    the marker's normal. Keep authored CAD floor height/tilt.
    """
    measured=rigid(measured)
    if type(scale) not in (int,float) or not math.isfinite(scale) or scale<=0:raise ValueError('Invalid CAD scale')
    if abs(measured[2][0])>.15 or abs(measured[2][1])<.98:raise ValueError('마커 축이 티칭 기준(X 가로/Y 수직)과 다릅니다. 닫힌 문과 마커 방향을 확인하세요.')
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in center):raise ValueError('Invalid CAD marker center')
    if abs(origin[3])>1e-4 or abs(origin[4])>1e-4:raise ValueError('마커 직접 배치는 수직 설비 CAD가 필요합니다. CAD roll/pitch를 확인하세요.')
    yaw=math.atan2(measured[1][0],measured[0][0])
    c,s=math.cos(yaw),math.sin(yaw);x,y,z=(v*scale for v in center)
    return [measured[0][3]-c*x+s*y,measured[1][3]-s*x-c*y,origin[2],0.,0.,yaw]


def enable_anchor(app,view):
    if not app.real:raise ValueError('실기 모드에서 마커 기준 배치를 켜세요.')
    from .door_marker import marker_center
    part=next((p for p in getattr(view.cad_asset,'door_parts',()) if p['name'].endswith(':1')),None)
    if part is None:raise ValueError('왼쪽 문이 포함된 설비 CAD를 먼저 불러오세요.')
    marker_center(part)
    view._marker_register_pending=False
    view.marker_placement=dict(enabled=True,mode='marker_anchor',scope=scope(app,view))
    view._marker_applied=None
    view._marker_observation=None
    app._spatial_save()
    view.marker_placement_status='마커 기준 배치 켜짐 · 다음 main4 측정으로 설비 위치/방향 확정 (수동 기준 등록 불필요)'


def enable_hook(app,view):
    enable_anchor(app,view)
    view.marker_placement['mode']='hook_teaching'
    from .door_teaching import reference
    view.marker_placement['teaching_sha256']=reference()['source_sha256']
    app._spatial_save()
    view.marker_placement_status='후크 파지 티칭 기준 배치 켜짐'+' · 다음 main4에서 손잡이 앞면과 후크 형상 깊이를 정렬'


def scope(app,view):
    return dict(context=context(app),asset=getattr(view.cad_asset,'name',''),
                path=getattr(app,'spatial_assets',{}).get('cad',{}).get('path',''),scale=view.scales['cad'],mount=list(view.mount))


def observation(app,view,state):
    if not app.real or not app.connected or time.monotonic()-app.last_state>1:raise ValueError('실기 위치 수신을 확인하세요.')
    if type(state.get('speed')) not in (int,float) or not math.isfinite(state['speed']) or abs(state['speed'])>.01:raise ValueError('AMR 정지 후 마커를 인식하세요.')
    runtime=getattr(app,'runtime_arm_feedback',None)
    data=runtime.marker_observation() if runtime else None
    if not data:raise ValueError('새 마커 측정값 없음 · 제어 PC 연동 패치와 main3 인식을 확인하세요.')
    if data.get('marker_id')!=0 or data.get('frame')!='rbase_m':raise ValueError('왼쪽 마커 ID 0 / rbase_m 좌표가 필요합니다.')
    # The remote exporter samples before any arm door motion. Do not reinterpret
    # that observation after the mobile base moves.
    key=(runtime.base,data.get('capture_id'))
    cached=getattr(view,'_marker_observation',None)
    if cached and cached[0]==key:return cached[1],key
    if not isinstance(data.get('capture_id'),str) or not data['capture_id']:raise ValueError('Missing capture identity')
    x,y,a=(state.get(k) for k in ('x','y','theta'))
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in (x,y,a)):raise ValueError('Invalid AMR pose')
    if data.get('map_frame')=='controller_world_m' and data.get('map_matrix') is not None:
        matrix=rigid(data['map_matrix'])
    else:
        matrix=multiply(multiply(transform((x,y,0),(0,0,a)),transform(view.mount[:3],view.mount[3:])),rigid(data.get('matrix')))
    view._marker_observation=(key,matrix)
    return matrix,key


def calibrate(app,view):
    matrix,key=observation(app,view,app.current_state())
    config=dict(enabled=True,reference=matrix,origin=list(view.cad_origin),scope=scope(app,view))
    view.marker_placement=config;view._marker_applied=key
    view.cad_placement_confirmation=dict(source='real_marker_registered',signature=signature(app,view))
    view.reset_real_doors()
    app._spatial_save()
    view.marker_placement_status='기준 등록 완료 · 다음 마커 인식부터 자동 배치'


def update(app,view,state):
    config=getattr(view,'marker_placement',{})
    if getattr(view,'_marker_register_pending',False) and app.real:
        try:
            if view._marker_register_scope!=scope(app,view):
                view._marker_register_pending=False
                raise ValueError('기준 등록 중 지도/CAD 설정 변경: 다시 예약하세요.')
            matrix,key=observation(app,view,state)
            if key!=getattr(view,'_marker_register_skip',None):
                calibrate(app,view);view._marker_register_pending=False
        except (ValueError,TypeError,KeyError) as error:view.marker_placement_status=str(error)
        return
    if not config.get('enabled') or not app.real:return
    try:
        if config.get('scope')!=scope(app,view):raise ValueError('지도·호스트·CAD·장착 설정 변경: 마커 기준 재등록 필요')
        anchor=config.get('mode')=='marker_anchor'
        if anchor or config.get('mode') in ('trajectory_registered','hook_teaching'):
            confidence=app._confidence_value(state)
            if confidence is None or not math.isfinite(confidence) or confidence<.6:raise ValueError('AMR 위치 신뢰도 부족: 지도 위치를 먼저 확인하세요. 마커 기준 배치 대기 중')
        matrix,key=observation(app,view,state)
        if key==getattr(view,'_marker_applied',None):return
        if config.get('mode')=='hook_teaching':
            from .door_teaching import place_handle,reference
            if config.get('teaching_sha256')!=reference()['source_sha256']:raise ValueError('후크 티칭 자료가 변경됐습니다. 후크 정렬을 다시 켜세요.')
            part=next((p for p in getattr(view.cad_asset,'door_parts',()) if p['name'].endswith(':1')),None)
            if part is None:raise ValueError('왼쪽 문 CAD 없음')
            origin=place_handle(matrix,part,view.cad_origin,view.scales['cad'])
        elif anchor:
            from .door_marker import marker_center
            part=next((p for p in getattr(view.cad_asset,'door_parts',()) if p['name'].endswith(':1')),None)
            if part is None:raise ValueError('왼쪽 문 CAD가 없습니다.')
            origin=solve_anchor(matrix,marker_center(part),view.cad_origin,view.scales['cad'])
        else:origin=solve(config['reference'],matrix,config['origin'])
        if config.get('mode') in ('marker_anchor','hook_teaching'):
            shift=math.hypot(origin[0]-view.cad_origin[0],origin[1]-view.cad_origin[1])
            turn=abs(math.atan2(math.sin(origin[5]-view.cad_origin[5]),math.cos(origin[5]-view.cad_origin[5])))
            if shift>.05 or turn>math.radians(5) or abs(origin[2]-view.cad_origin[2])>.05:
                raise ValueError(f'마커 배치 급변 보류: 이동 {shift*100:.1f}cm / 회전 {math.degrees(turn):.1f}° · 기존 설비 위치 유지')
        previous=(config.get('mode') not in ('marker_anchor','hook_teaching')) or getattr(view,'_marker_applied',None) is not None
        if previous and math.hypot(origin[0]-view.cad_origin[0],origin[1]-view.cad_origin[1])>.5:raise ValueError('설비 보정이 50cm 초과: 기준 배치를 확인하세요.')
        angle=math.atan2(math.sin(origin[5]-view.cad_origin[5]),math.cos(origin[5]-view.cad_origin[5]))
        if previous and abs(angle)>math.radians(25):raise ValueError('설비 보정이 25도 초과: 기준 배치를 확인하세요.')
        view.cad_origin[:]=origin
        view.reset_real_doors()
        view.cad_placement_confirmation=dict(source='real_marker_registered',signature=signature(app,view))
        view._marker_applied=key
        app._spatial_save()
        view.marker_placement_status=f'작업 좌표 기준 설비 배치 x={origin[0]:.3f} y={origin[1]:.3f} z={origin[2]:.3f} θ={math.degrees(origin[5]):.2f}°'
        app.log('3D',view.marker_placement_status)
    except (ValueError,TypeError,KeyError) as error:
        view.marker_placement_status=str(error)


def show(app):
    import tkinter as tk
    from tkinter import ttk,messagebox
    view=app.world3d
    win=tk.Toplevel(app);win.title('마커 기반 설비 자동 배치');win.geometry('750x560')
    ttk.Label(win,text='닫힌 왼쪽 문 기준 · 마커 ID 0',font=('',12,'bold')).pack(pady=10)
    ttk.Label(win,text='후크 우선: 제어 PC 파지 티칭 TCP와 CAD 손잡이 내부 중심을 XYZ·각도로 맞춥니다.\n후크 → 왼쪽 손잡이 파지 위치 정렬 켜기 → 다음 main4 측정에서 자동 반영.\n이전 측정값과의 차이나 최초 수동 설비 배치를 기준으로 하지 않습니다.\n실제 로봇 위치·티칭·이동 명령은 변경하지 않습니다.\n닫힌 문 / 마커 ID 0 / 손잡이 왼쪽 끝에서 30mm, 중앙 높이 52.5mm.\n지도 위치 신뢰도가 낮으면 배치하지 않습니다. Python 3D 화면을 열어 두세요.',justify='left').pack(padx=15)
    status=tk.StringVar();ttk.Label(win,textvariable=status,wraplength=620).pack(pady=12)
    def action(fn):
        try:fn()
        except (ValueError,TypeError,KeyError) as error:messagebox.showerror('마커 설비 배치',str(error),parent=win)
    def disable():
        view._marker_register_pending=False
        view.marker_placement=dict(getattr(view,'marker_placement',{}),enabled=False);app._spatial_save()
        view.marker_placement_status='자동 보정 중지 · 현재 설비 배치 유지'
    def queue_reference():
        if not app.real:raise ValueError('실기 모드에서 기준 등록하세요.')
        if view.cad_asset is None:raise ValueError('설비 CAD를 먼저 불러오세요.')
        runtime=getattr(app,'runtime_arm_feedback',None)
        data=runtime.marker_observation() if runtime else None
        view._marker_register_skip=(runtime.base,data.get('capture_id')) if data else None
        view._marker_register_scope=scope(app,view)
        view._marker_register_pending=True
        view.marker_placement_status='다음 새 마커 인식 시 현재 CAD 배치로 기준 등록 · 3D 화면을 열어 두세요.'
    ttk.Button(win,text='후크 → 왼쪽 손잡이 파지 위치 정렬 켜기 (티칭 기준)',command=lambda:action(lambda:enable_hook(app,view))).pack(pady=8)
    ttk.Button(win,text='마커 기준 설비 배치 켜기 (수동 기준 등록 불필요)',command=lambda:action(lambda:enable_anchor(app,view))).pack(pady=8)
    ttk.Button(win,text='기존 수동 배치 기준 등록 예약 (선택)',command=lambda:action(queue_reference)).pack(pady=4)
    from .door_io7 import save_motion_profile
    ttk.Button(win,text='최근 전체 녹화로 DO7 열림 각도 기준 다시 맞추기',command=lambda:action(lambda:save_motion_profile(app))).pack(pady=5)
    ttk.Button(win,text='자동 보정 끄기',command=disable).pack()
    def tick():
        if not win.winfo_exists():return
        status.set(getattr(view,'marker_placement_status','마커 기준 배치를 켠 뒤 다음 main4 측정을 기다리세요.'));win.after(250,tick)
    tick()
