"""Registered two-marker board geometry and offline arm missions. No transport."""
import copy
import json
import math
import time
import uuid
from pathlib import Path
from .aruco_arm_follow import rotation
from .arm_simulation import ArmSimulator,rotation_error
from .fairino_api import profile
from .geometry3d import transform


def matmul(a,b):return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
def transpose(a):return list(map(list,zip(*a)))
def apply(r,p):return [sum(r[i][j]*p[j] for j in range(3)) for i in range(3)]
FRONT=[[1.,0.,0.],[0.,-1.,0.],[0.,0.,-1.]]


def validate_board(config):
    if config.get('version')!=1 or not isinstance(config.get('revision'),str):raise ValueError('기준판 설정 형식 오류')
    markers=config.get('markers',[])
    if len(markers)!=2:raise ValueError('기준판에는 마커 두 개가 필요합니다.')
    keys=set()
    for marker in markers:
        key=(marker.get('dictionary'),marker.get('id'))
        if key in keys:raise ValueError('두 기준 마커의 종류/ID가 중복됩니다.')
        keys.add(key)
        if not isinstance(key[0],str) or type(key[1]) is not int or key[1]<0:raise ValueError('마커 종류/ID 오류')
        for field in ('side_mm','rotation_deg'):
            if type(marker.get(field)) not in (int,float) or not math.isfinite(marker[field]):raise ValueError('마커 치수/회전 오류')
        if not 10<=marker['side_mm']<=1000:raise ValueError('마커 한 변은 10~1000mm입니다.')
    spacing=config.get('spacing_mm')
    if type(spacing) not in (int,float) or not math.isfinite(spacing) or not 10<=spacing<=2000:raise ValueError('중심 간격은 10~2000mm입니다.')
    if spacing<(markers[0]['side_mm']+markers[1]['side_mm'])/2:raise ValueError('중심 간격이 마커 크기보다 작습니다.')
    if config.get('geometry_source') not in ('measured','depth_estimate'):raise ValueError('치수 출처 오류')
    return config


def make_board(markers,sides,spacing,source='measured',rotations=(0.,0.)):
    return validate_board(dict(version=1,revision=uuid.uuid4().hex,geometry_source=source,
        spacing_mm=float(spacing),markers=[dict(dictionary=m['dictionary'],id=m['id'],side_mm=float(s),rotation_deg=float(r))
        for m,s,r in zip(markers,sides,rotations)]))


def save_board(config,path):
    validate_board(config);path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    pending=path.with_suffix('.tmp');pending.write_text(json.dumps(config,ensure_ascii=False,indent=2),encoding='utf-8');pending.replace(path)


def object_points(config,index):
    marker=config['markers'][index];half=marker['side_mm']/2000
    center=(-1 if index==0 else 1)*config['spacing_mm']/2000
    angle=math.radians(marker['rotation_deg']);c,s=math.cos(angle),math.sin(angle)
    return [[center+c*x-s*y,s*x+c*y,0.] for x,y in [(-half,half),(half,half),(half,-half),(-half,-half)]]


def pose_from_matrix(r,p):
    pitch=math.asin(max(-1.,min(1.,-r[2][0])))
    if abs(math.cos(pitch))>1e-7:roll=math.atan2(r[2][1],r[2][2]);yaw=math.atan2(r[1][0],r[0][0])
    else:roll=0.;yaw=math.atan2(-r[0][1],r[1][1])
    return list(p)+[math.degrees(v) for v in (roll,pitch,yaw)]


def initialize_panel_arm(sim):
    """Offline initial pose: upright panel in front, tool Z points toward -base X."""
    seed=[math.radians(v) for v in [0,-90,90,-90,-90,0]]
    q=sim.kin.ik([-350,-100,350,-90,0,90],seed,iterations=500)
    sim.physics.check_motion(q,q)
    sim.q=q


class BoardArmMission:
    def __init__(self,sim,journal=None):
        self.sim=sim;self.journal=journal;self.config=None;self.latest=None
        self.stable=0;self.stamp=None;self.reference=None;self.running=False
        self.status='기준판 등록 대기';self.targets={};self.goal_sample=None
        self.last_step=None;self.tool_offset_mm=0.

    def report(self,conclusion,evidence=None,action='가상 팔 현재 위치 유지'):
        self.status=conclusion
        if self.journal:self.journal.emit('SIM.ArUco 기준판 작업',
            dict(running=self.running,step=self.sim.index,geometry_source=(self.config or {}).get('geometry_source')),
            evidence or {},conclusion,action,identity=(conclusion,self.sim.index))

    def stop(self,reason='사용자 정지',evidence=None):
        if self.running:self.sim.stop()
        self.running=False;self.goal_sample=None;self.report(reason,evidence)

    def inspect(self,record,now=None):
        now=time.time() if now is None else now
        try:
            stamp=record['timestamp'];board=record['board']
            if not math.isfinite(stamp) or not -.1<=now-stamp<=.7:raise ValueError('영상 수신 지연')
            if not board.get('valid'):raise ValueError(board.get('reason','기준판 미검출'))
            if self.config is None or board['revision']!=self.config['revision']:raise ValueError('등록 설정 적용 대기')
            photo=getattr(self,'allow_photo_screen',False) and record.get('vision_source')=='photo_screen_sim' and board.get('geometry_source')=='photo_screen_estimate' and board.get('markers_used')==0
            if board['markers_used']!=2 and not photo:raise ValueError('기준 마커 두 개 필요')
            if not math.isfinite(board['reprojection_px']) or board['reprojection_px']>2:raise ValueError('기준판 재투영 오차 초과')
            for field in ('camera_xyz_m','rotation_vector_rad'):
                if len(board[field])!=3 or any(not math.isfinite(v) for v in board[field]):raise ValueError('기준판 좌표 오류')
            if board['camera_xyz_m'][2]<=0:raise ValueError('기준판 깊이 오류')
        except (KeyError,TypeError,ValueError) as exc:
            self.latest=None;self.stable=0;self.stamp=None
            self.stop(str(exc),dict(frame=record));return None
        if self.stamp!=stamp:self.stable+=1;self.stamp=stamp
        self.latest=board
        if self.running and self.goal_sample:
            raw=record.get('raw_board',board)
            distance=math.dist(raw['camera_xyz_m'],self.goal_sample['camera_xyz_m'])*1000
            angle=math.degrees(math.sqrt(sum(v*v for v in rotation_error(rotation(raw['rotation_vector_rad']),rotation(self.goal_sample['rotation_vector_rad'])))))
            if distance>10 or angle>3:self.stop('실행 중 기준판/카메라 이동 · 재계획 필요',dict(translation_mm=distance,rotation_deg=angle))
        return board

    def set_reference(self,standby_mm=200.):
        if not math.isfinite(standby_mm) or not 20<=standby_mm<=300:raise ValueError('대기 거리는 20~300mm입니다.')
        if self.latest is None or self.stable<3:raise ValueError('유효한 기준판 3프레임을 기다리세요.')
        if self.running:raise ValueError('현재 작업을 먼저 정지하세요.')
        fk=self.sim.kin.fk(self.sim.q);tool=[list(row[:3]) for row in fk[:3]]
        board_r=matmul(tool,FRONT);normal=[row[2] for row in board_r]
        tcp=self.sim.kin.pose(self.sim.q)
        tip=apply(tool,[0.,0.,self.tool_offset_mm])
        tcp[:3]=[tcp[i]+tip[i] for i in range(3)]
        self.reference=dict(camera=copy.deepcopy(self.latest),q=list(self.sim.q),
            board_rotation=board_r,center_mm=[tcp[i]-normal[i]*standby_mm for i in range(3)],
            transform=matmul(board_r,transpose(rotation(self.latest['rotation_vector_rad']))),asset=self.sim.kin.asset)
        self.report('가상 좌표 기준 설정 완료',dict(standby_mm=standby_mm,home_tcp=tcp),action='기준판을 현재 TCP 앞에 가상 배치')

    def virtual_board(self):
        if not self.reference or not self.latest:raise ValueError('가상 좌표 기준을 먼저 설정하세요.')
        if self.sim.kin.asset is not self.reference['asset']:raise ValueError('모델 변경 후 기준을 다시 설정하세요.')
        delta=[(a-b)*1000 for a,b in zip(self.latest['camera_xyz_m'],self.reference['camera']['camera_xyz_m'])]
        if math.sqrt(sum(v*v for v in delta))>150:raise ValueError('기준 대비 이동 150mm 초과 · 기준을 다시 설정하세요.')
        q=self.reference['transform'];r=matmul(q,rotation(self.latest['rotation_vector_rad']))
        moved=apply(q,delta);center=[a+b for a,b in zip(self.reference['center_mm'],moved)]
        return center,r

    def snapshot(self,offset,standby,approach,action='cycle'):
        if self.running:raise ValueError('실행 중입니다.')
        if self.latest is None or self.stable<3:raise ValueError('유효한 기준판 3프레임을 기다리세요.')
        if len(offset)!=2 or any(not math.isfinite(v) or abs(v)>100 for v in offset):raise ValueError('작업 X/Y 오프셋은 ±100mm입니다.')
        if not 20<=approach<standby<=300:raise ValueError('20mm ≤ 접근 거리 < 대기 거리 ≤ 300mm이어야 합니다.')
        center,r=self.virtual_board();tool=matmul(r,FRONT)
        target_center=[center[i]+r[i][0]*offset[0]+r[i][1]*offset[1] for i in range(3)]
        targets={name:pose_from_matrix(tool,[target_center[i]+r[i][2]*distance-tool[i][2]*self.tool_offset_mm for i in range(3)])
                 for name,distance in [('align',standby),('approach',approach),('retract',standby)]}
        return dict(asset=self.sim.kin.asset,q=list(self.sim.q),physics=self.sim.physics.snapshot(),
                    targets=targets,home_q=list(self.reference['q']),action=action,sample=copy.deepcopy(self.latest))

    def update_scene(self):
        center,r=self.virtual_board()
        objects=self.sim.physics.objects
        board=next((o for o in objects if o['name']=='ArUco 기준판'),None)
        size=[(self.config['spacing_mm']+sum(m['side_mm'] for m in self.config['markers'])/2+15)/1000,
              (max(m['side_mm'] for m in self.config['markers'])+15)/1000,.004]
        if board is None:board=self.sim.physics.add_object('ArUco 기준판','panel',size,1.,[0,0,0,0,0,0],static=True)
        board['size']=size
        board['matrix']=tuple(tuple(center[i]/1000 if j==3 else r[i][j] for j in range(4)) if i<3 else (0.,0.,0.,1.) for i in range(4))
        return center,r

    @staticmethod
    def prepare(snapshot):
        # Validate the complete route on an independent simulator before moving.
        sim=ArmSimulator(snapshot['asset']);sim.q=list(snapshot['q']);sim.physics.restore(snapshot['physics'])
        cfg=profile();cfg['operations']={name:dict(method='MoveL',target=pose,tool=0,user=0,vel=12)
                                        for name,pose in snapshot['targets'].items()}
        cfg['operations']['home']=dict(method='MoveJ',target=[math.degrees(v) for v in snapshot['home_q']],vel=12,tool=0,user=0)
        names=['align'] if snapshot['action']=='align' else ['align','approach','retract','home']
        cfg['programs']={'board_cycle':[dict(operation=name,timeout_s=60) for name in names]}
        sim.start(cfg,program='board_cycle')
        for action in sim.actions:
            sim._enter(action)
            for a,b in zip(sim.path,sim.path[1:]):sim.physics.check_motion(a,b)
            if sim.path:sim.q=list(sim.path[-1])
        return cfg

    def launch(self,snapshot,config):
        if self.latest is None:raise ValueError('계획 중 기준판 소실')
        if self.latest['revision']!=snapshot['sample']['revision']:raise ValueError('계획 중 기준판 설정 변경')
        if math.dist(self.latest['camera_xyz_m'],snapshot['sample']['camera_xyz_m'])>.01:raise ValueError('계획 중 기준판/카메라 이동')
        angle=math.sqrt(sum(v*v for v in rotation_error(rotation(self.latest['rotation_vector_rad']),rotation(snapshot['sample']['rotation_vector_rad']))))
        if angle>math.radians(3):raise ValueError('계획 중 기준판/카메라 회전')
        if any(abs(a-b)>1e-6 for a,b in zip(self.sim.q,snapshot['q'])):raise ValueError('계획 중 가상 팔 자세 변경')
        self.targets=snapshot['targets'];self.goal_sample=snapshot['sample'];self.last_step=None
        self.sim.start(config,program='board_cycle');self.running=True
        self.report('전체 경로 검증 완료 · 실행 시작',dict(targets=self.targets,action=snapshot['action']),action='가상 팔 정렬·접근 경로 실행')

    def tick(self,dt):
        if not self.running:return
        if self.last_step!=self.sim.index:
            self.last_step=self.sim.index
            names=dict(align='중점·자세 정렬',approach='지정 거리 접근',retract='대기 거리 후퇴',home='원래 자세 복귀')
            self.report('단계 실행: '+names[self.sim.actions[self.sim.index]['operation']],dict(targets=self.targets),action='검증한 가상 팔 단계 실행')
        self.sim.tick(dt)
        if self.sim.state in ('COMPLETED','FAILED','CANCELED'):
            self.running=False;self.report('작업 '+self.sim.state,dict(error=self.sim.error,events=self.sim.events),action='가상 팔 작업 종료')
