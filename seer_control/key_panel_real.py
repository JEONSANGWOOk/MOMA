"""Calibrated key-tip targets for real FR5. No virtual-reference fallbacks."""
import copy,math,time
from .fairino_api import vector
from .geometry3d import transform,multiply
from .aruco_board import pose_from_matrix,transpose,apply
from .aruco_arm_follow import rotation
from .arm_simulation import rotation_error
from .visual_servo import norm,limited


def rigid(value,name):
    if not isinstance(value,list) or len(value)!=4 or any(not isinstance(r,list) or len(r)!=4 for r in value):raise ValueError(name+': 4×4 행렬 필요')
    if any(type(x) not in (int,float) or not math.isfinite(x) for row in value for x in row):raise ValueError(name+': 유한 숫자 필요')
    if any(abs(a-b)>1e-6 for a,b in zip(value[3],[0,0,0,1])):raise ValueError(name+': 마지막 행 오류')
    r=[row[:3] for row in value[:3]]
    for i in range(3):
        for j in range(3):
            if abs(sum(r[k][i]*r[k][j] for k in range(3))-(i==j))>1e-5:raise ValueError(name+': 회전 직교성 오류')
    cross=[r[0][1]*r[1][2]-r[0][2]*r[1][1],r[0][2]*r[1][0]-r[0][0]*r[1][2],r[0][0]*r[1][1]-r[0][1]*r[1][0]]
    if abs(sum(cross[i]*r[2][i] for i in range(3))-1)>1e-5:raise ValueError(name+': 반사 행렬 불가')
    return value


def template():
    return dict(version=1,verified=False,camera_mount='fixed_external',board_revision=None,tool=None,user=0,
        key_tcp_confirmed=False,T_base_camera_mm=None,T_board_socket_mm=None,T_socket_key_zero_mm=None,
        workspace_min_mm=None,workspace_max_mm=None,standby_mm=None,insert_mm=None,turn_deg=30.,
        controller_version='',version_confirmed=False,contact_verified=False,force_sensor_verified=False,
        force_limit_n=None,torque_limit_nm=None,controller_guard_verified=False,hybrid_target_verified=False,hybrid_target_revision=None)


def calibration(c,contact=False):
    if not isinstance(c,dict) or c.get('version')!=1 or c.get('verified') is not True or c.get('key_tcp_confirmed') is not True:raise ValueError('실측 카메라·열쇠 TCP 보정을 먼저 완료하세요.')
    if c.get('camera_mount')!='fixed_external':raise ValueError('실기 모드는 작업대에 고정한 외부 카메라 보정을 사용합니다.')
    if not isinstance(c.get('board_revision'),str) or not c['board_revision']:raise ValueError('기준판 revision 필요')
    if type(c.get('tool')) is not int or not 1<=c['tool']<=14 or c.get('user')!=0:raise ValueError('열쇠 끝 TCP 번호 1~14 / 베이스 user=0 필요')
    for name in ('T_base_camera_mm','T_board_socket_mm','T_socket_key_zero_mm'):rigid(c.get(name),name)
    z=[c['T_socket_key_zero_mm'][i][2] for i in range(3)]
    if math.dist(z,[0,0,-1])>1e-4 or any(abs(c['T_socket_key_zero_mm'][i][3])>1e-6 for i in range(3)):raise ValueError('열쇠 끝 원점과 슬롯 원점 일치 / 열쇠 +Z는 삽입 방향이어야 합니다.')
    for name in ('workspace_min_mm','workspace_max_mm'):
        if not isinstance(c.get(name),list) or len(c[name])!=3 or any(type(x) not in (int,float) or not math.isfinite(x) for x in c[name]):raise ValueError('검증된 작업 범위 필요')
    if any(a>=b for a,b in zip(c['workspace_min_mm'],c['workspace_max_mm'])):raise ValueError('작업 범위 오류')
    for name,lo,hi in [('standby_mm',5,100),('insert_mm',.1,30),('turn_deg',-30,30)]:
        v=c.get(name)
        if type(v) not in (int,float) or not math.isfinite(v) or not lo<=v<=hi:raise ValueError(name+': 실측 범위 오류')
    if c.get('version_confirmed') is not True or not c.get('controller_version','').strip():raise ValueError('제어기·SDK 호환 버전 확인 필요')
    if contact:
        if any(c.get(k) is not True for k in ('contact_verified','force_sensor_verified','controller_guard_verified')):raise ValueError('접촉 작업에는 실물 검증·힘센서·제어기 접촉 보호 설정이 필요합니다.')
        for key,maxval in [('force_limit_n',50),('torque_limit_nm',5)]:
            v=c.get(key)
            if type(v) not in (int,float) or not math.isfinite(v) or not 0<v<=maxval:raise ValueError(key+': 실물에 맞는 보호 임계값 필요')
    return c


def pose_matrix(pose):
    p=vector(pose,'실제 TCP');return transform(p[:3],[math.radians(v) for v in p[3:]])


def socket_frame(c,record,now=None):
    now=time.time() if now is None else now
    if not record or record.get('vision_source') in ('built_in_demo','photo_screen_sim'):raise ValueError('실기 모드에는 실제 D455 영상과 실측 대상이 필요합니다.')
    age=now-float(record['timestamp'])
    if not math.isfinite(age) or not -.1<=age<=.25:raise ValueError('D455 영상 소실/250 ms 지연')
    b=record['board'];raw=record.get('raw_board',b)
    for item in (b,raw):
        if not item.get('valid') or item.get('revision')!=c['board_revision'] or item.get('geometry_source')!='measured' or item.get('markers_used')!=2:raise ValueError('실측 등록한 두 마커와 보정 revision 불일치')
        xyz=item['camera_xyz_m'];rv=item['rotation_vector_rad']
        if len(xyz)!=3 or len(rv)!=3 or any(not math.isfinite(v) for v in xyz+rv) or xyz[2]<=0 or not 0<=item['reprojection_px']<=1.5:raise ValueError('영상 자세/재투영 품질 미충족')
    r=rotation(b['rotation_vector_rad']);t=[v*1000 for v in b['camera_xyz_m']]
    cam=tuple(tuple(r[i])+(t[i],) for i in range(3))+((0,0,0,1),)
    target=copy.deepcopy(c['T_board_socket_mm'])
    if c.get('hybrid_target_verified') is True:
        h=record.get('handle_target') or {};xyz=h.get('board_xyz_mm')
        if not h.get('valid') or h.get('profile_revision')!=c.get('hybrid_target_revision') or not c.get('hybrid_target_revision') or h.get('board_revision')!=c['board_revision'] or h.get('dimension_source')!='measured' or h.get('confidence',0)<.78:raise ValueError('실기 결합 추종: 실측 확인한 구멍 검출 필요')
        if not isinstance(xyz,list) or len(xyz)!=3 or any(type(v) not in (float,int) or not math.isfinite(v) for v in xyz):raise ValueError('실기 구멍 좌표 오류')
        if math.dist(xyz[:2],[target[0][3],target[1][3]])>10 or abs(xyz[2]-target[2][3])>.2:raise ValueError('실측 구멍 보정 범위 초과')
        target[0][3],target[1][3]=xyz[:2]
    return multiply(multiply(c['T_base_camera_mm'],cam),target)


def in_workspace(c,p):return all(lo<=v<=hi for lo,v,hi in zip(c['workspace_min_mm'],p,c['workspace_max_mm']))


class RealKeyPlan:
    """5 Hz bounded MoveL increments; controller TCP is the physical key tip."""
    def __init__(self,c,contact=False):
        self.c=copy.deepcopy(calibration(c,contact));self.contact=contact
        self.stage='ALIGN';self.depth=-c['standby_mm'];self.turn=0.;self.lock=0
        self.last_socket=None;self.last_stamp=None;self.evidence={};self.started=None;self.last_raw=None;self.completed=False

    def step(self,record,feedback,now=None):
        now=time.time() if now is None else now
        if self.started is None:self.started=now
        if now-self.started>300:raise ValueError('실기 작업 300초 제한 초과')
        socket=socket_frame(self.c,record,now);stamp=record['timestamp']
        raw=record.get('raw_board',record['board'])
        if self.last_raw:
            if math.dist(raw['camera_xyz_m'],self.last_raw['camera_xyz_m'])*1000>2 or math.degrees(norm(rotation_error(rotation(raw['rotation_vector_rad']),rotation(self.last_raw['rotation_vector_rad']))))>2:raise ValueError('원본 마커 급변: 실기 작업 정지')
        self.last_raw=copy.deepcopy(raw)
        if self.last_socket:
            if math.dist([r[3] for r in socket[:3]],[r[3] for r in self.last_socket[:3]])>2 or math.degrees(norm(rotation_error(socket,self.last_socket)))>2:raise ValueError('마커 급변: 2 mm / 2° 초과')
        self.last_socket=socket
        p=vector(feedback['tcp_mm_deg'],'실제 열쇠 TCP');current=pose_matrix(p)
        r=[list(row[:3]) for row in socket[:3]];origin=[row[3] for row in socket[:3]]
        local=apply(transpose(r),[p[i]-origin[i] for i in range(3)])
        lateral=norm(local[:2]);actual_depth=-local[2]
        zero=multiply(socket,self.c['T_socket_key_zero_mm'])
        target_r=multiply(zero,transform(rpy=(0,0,math.radians(self.turn))))
        rr=[[sum(zero[k][i]*current[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        actual_turn=math.degrees(math.atan2(rr[1][0],rr[0][0]))
        angle=math.degrees(norm(rotation_error(target_r,current)))
        tilt=math.degrees(math.acos(max(-1,min(1,sum(current[i][2]*(-socket[i][2]) for i in range(3))))))
        if actual_depth>self.c['insert_mm']+.2:raise ValueError('실제 열쇠 삽입 깊이 초과')
        if actual_depth>0 and (not self.contact or lateral>.2 or tilt>.5):raise ValueError('접촉 중 실제 열쇠 정렬 조건 미충족')
        if self.stage in ('INSERT','TURN','RETRACT','HOLD') and self.contact:
            f=vector(feedback.get('force_torque'),'힘/토크 센서')
            if norm(f[:3])>=self.c['force_limit_n'] or norm(f[3:])>=self.c['torque_limit_nm']:raise ValueError('접촉 힘/토크 한계 초과')
        if self.stage=='TURN' and (lateral>.2 or tilt>.5 or actual_depth<self.c['insert_mm']-.2):raise ValueError('회전 중 실제 깊이·축 정렬 조건 미충족')
        target=[origin[i]-socket[i][2]*self.depth for i in range(3)]
        converged=math.dist(p[:3],target)<=.1 and angle<=.3
        fresh=stamp!=self.last_stamp;self.last_stamp=stamp
        if fresh:self.lock=self.lock+1 if converged and feedback['motion_done']==1 and feedback.get('status') in ('IDLE','COMPLETED') else 0
        if self.lock>=3:
            if self.stage=='ALIGN':self.stage='APPROACH'
            elif self.stage=='APPROACH' and self.depth>=-5:self.stage='INSERT' if self.contact else 'HOLD'
            elif self.stage=='INSERT' and self.depth>=self.c['insert_mm']:self.stage='TURN'
            elif self.stage=='TURN' and abs(actual_turn-self.c['turn_deg'])<=.3 and abs(self.turn-self.c['turn_deg'])<1e-6:self.stage='RETRACT'
            elif self.stage=='RETRACT' and self.depth<=-self.c['standby_mm']:self.stage='HOLD';self.completed=True
            self.lock=0
        if not fresh or feedback['motion_done']!=1 or feedback.get('status') not in ('IDLE','COMPLETED'):return None
        if lateral<=.2 and angle<=.3:
            if self.stage=='APPROACH':self.depth=min(-5.,self.depth+.5)
            elif self.stage=='INSERT':self.depth=min(self.c['insert_mm'],self.depth+.1)
            elif self.stage=='RETRACT':self.depth=max(-self.c['standby_mm'],self.depth-(.1 if actual_depth>0 else .5))
        if self.stage=='TURN':
            delta=max(-.25,min(.25,self.c['turn_deg']-self.turn));self.turn+=delta
        target=[origin[i]-socket[i][2]*self.depth for i in range(3)]
        target_r=multiply(zero,transform(rpy=(0,0,math.radians(self.turn))))
        delta=limited([target[i]-p[i] for i in range(3)],.5)
        dr=limited(rotation_error(target_r,current),math.radians(.25));nr=multiply(tuple(tuple(rotation(dr)[i])+(0,) for i in range(3))+((0,0,0,1),),current)
        command=pose_from_matrix(nr,[p[i]+delta[i] for i in range(3)])
        if not in_workspace(self.c,p[:3]) or not in_workspace(self.c,command[:3]) or not in_workspace(self.c,target):raise ValueError('검증된 실기 작업 범위 밖')
        self.evidence=dict(stage=self.stage,actual_tcp=p,target_tcp=command,goal_tcp=target,lateral_mm=lateral,
            actual_depth_mm=actual_depth,command_depth_mm=self.depth,actual_turn_deg=actual_turn,command_turn_deg=self.turn,
            axis_error_deg=tilt,sample_timestamp=stamp,board_revision=self.c['board_revision'],completed=self.completed,mode='REAL')
        if norm(delta)<.002 and norm(dr)<math.radians(.005):return None
        return dict(target=command,expires=time.monotonic()+.25,stage=self.stage,calibration=self.c,
                    socket_base_mm=[list(row) for row in socket])
