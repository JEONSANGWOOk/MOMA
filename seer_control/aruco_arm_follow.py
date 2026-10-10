"""Camera-relative demonstration using ArmSimulator only; no robot transport."""
import json
import math
import time
from pathlib import Path


def latest_record(path):
    try:
        with Path(path).open('rb') as stream:
            stream.seek(0,2);length=stream.tell();stream.seek(max(0,length-65536))
            lines=stream.read().split(b'\n')
        # A writer may currently be appending the final (incomplete) line.
        for line in reversed(lines[:-1]):
            try:return json.loads(line)
            except (ValueError,UnicodeError):continue
    except OSError:pass
    return None


def rotation(vector):
    angle=math.sqrt(sum(v*v for v in vector))
    if angle<1e-12:return [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
    x,y,z=[v/angle for v in vector];c=math.cos(angle);s=math.sin(angle);d=1-c
    return [[c+x*x*d,x*y*d-z*s,x*z*d+y*s],
            [y*x*d+z*s,c+y*y*d,y*z*d-x*s],
            [z*x*d-y*s,z*y*d+x*s,c+z*z*d]]


def camera_origin(marker):
    r=rotation(marker['rotation_vector_rad']);t=marker['camera_xyz_m']
    return [-sum(r[j][i]*t[j] for j in range(3)) for i in range(3)],r


class CameraArmFollower:
    def __init__(self,sim,journal=None):
        self.sim=sim;self.journal=journal;self.key=None;self.enabled=False
        self.baseline=None;self.status='마커 선택 대기';self.stable=0
        self.last_stamp=None;self.reference_pose=None;self.filtered=None
        self.speed_mm_s=40.;self.gain=.5;self.limit_mm=150.
        self.last_applied=None;self.latest=None;self.delta_mm=[0.,0.,0.]

    def report(self,conclusion,evidence=None):
        self.status=conclusion
        if self.journal:
            self.journal.emit('SIM.ArUco 카메라 추종',
                dict(marker=self.key,tracking=self.enabled,reference_tcp=self.reference_pose),
                evidence or {},conclusion,'가상 팔 갱신' if conclusion=='카메라 이동 추종 중' else '가상 팔 현재 위치 유지',
                identity=conclusion)

    def select(self,key):
        if key==self.key:return
        self.stop('기준 마커 변경 · 기준 위치를 설정하세요')
        self.key=key;self.baseline=None;self.stable=0;self.last_stamp=None;self.latest=None

    def stop(self,reason='추적 정지',evidence=None):
        self.enabled=False;self.report(reason,evidence)

    def inspect(self,record,now=None):
        now=time.time() if now is None else now
        try:
            stamp=float(record['timestamp'])
            if not math.isfinite(stamp) or not -.1<=now-stamp<=.7:
                raise ValueError('영상 수신 지연 · 추적 정지')
            matches=[m for m in record['markers'] if (m.get('dictionary'),m.get('id'))==self.key]
            if len(matches)!=1:raise ValueError('기준 마커 미검출 또는 동일 ID 중복 · 추적 정지')
            marker=matches[0]
            if marker.get('pose_valid') is not True:raise ValueError('마커 위치 추정 실패 · 추적 정지')
            for name in ('camera_xyz_m','rotation_vector_rad'):
                values=marker[name]
                if len(values)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
                    raise ValueError('마커 좌표 오류 · 추적 정지')
            error=marker['reprojection_px']
            if type(error) not in (int,float) or not math.isfinite(error) or not 0<=error<=2.:
                raise ValueError('마커 인식 오차 초과 · 추적 정지')
            if marker['camera_xyz_m'][2]<=0:raise ValueError('마커 거리 오류 · 추적 정지')
        except (KeyError,TypeError,ValueError) as exc:
            self.stable=0;self.latest=None;self.last_stamp=None
            self.stop(str(exc) if isinstance(exc,ValueError) else '마커 데이터 없음 · 추적 정지',
                      dict(frame=record,now=now))
            return None
        if stamp!=self.last_stamp:
            self.stable+=1;self.last_stamp=stamp
        self.latest=marker
        return marker

    def reference(self):
        if self.latest is None or self.stable<3:raise ValueError('기준 마커가 연속 3프레임 이상 보이도록 기다리세요.')
        if self.sim.state in ('RUNNING','PAUSED'):raise ValueError('가상 팔 재생을 먼저 정지하세요.')
        self.enabled=False;self.baseline=camera_origin(self.latest)
        self.reference_asset=self.sim.kin.asset
        self.reference_pose=self.sim.kin.pose(self.sim.q);self.filtered=list(self.reference_pose[:3])
        self.delta_mm=[0.,0.,0.];self.last_applied=None
        self.report('기준 위치 설정 완료',dict(camera_origin_marker_m=self.baseline[0],tcp_mm=self.reference_pose))

    def start(self):
        if self.baseline is None or self.latest is None:raise ValueError('기준 위치를 먼저 설정하세요.')
        self.enabled=True;self.report('추적 시작 · 카메라를 천천히 움직이세요')

    def tick(self,record,dt=.1,now=None):
        marker=self.inspect(record,now)
        if marker is None or not self.enabled:return False
        if self.sim.state in ('RUNNING','PAUSED'):
            self.stop('다른 가상 팔 프로그램 실행 중 · 추적 정지');return False
        if self.sim.kin.asset is not getattr(self,'reference_asset',self.sim.kin.asset):
            self.stop('로봇 모델 변경 · 기준 위치를 다시 설정하세요');return False
        origin,_=camera_origin(marker);initial,r0=self.baseline
        # Express camera translation in the reference camera's optical axes.
        optical=[sum(r0[i][j]*(origin[j]-initial[j])*1000 for j in range(3)) for i in range(3)]
        self.delta_mm=[optical[2]*self.gain,-optical[0]*self.gain,-optical[1]*self.gain]
        if math.sqrt(sum(v*v for v in self.delta_mm))>self.limit_mm:
            self.stop('기준 위치에서 150mm 범위 초과 · 추적 정지',dict(delta_mm=self.delta_mm,limit_mm=self.limit_mm));return False
        target=[a+b for a,b in zip(self.reference_pose[:3],self.delta_mm)]
        self.filtered=[a+(b-a)*.25 for a,b in zip(self.filtered,target)]
        current=self.sim.kin.pose(self.sim.q);distance=math.dist(current[:3],self.filtered)
        if distance<1.5:
            self.report('미세 흔들림 · 현재 위치 유지',dict(delta_mm=self.delta_mm));return False
        ratio=min(1.,self.speed_mm_s*max(0,min(.15,dt))/distance)
        pose=[a+(b-a)*ratio for a,b in zip(current[:3],self.filtered)]+self.reference_pose[3:]
        try:
            q=self.sim.kin.ik(pose,self.sim.q,iterations=50)
            angular=max(abs(a-b) for a,b in zip(self.sim.q,q))
            factor=min(1.,math.radians(30)*max(0,min(.15,dt))/max(angular,1e-12))
            q=[a+(b-a)*factor for a,b in zip(self.sim.q,q)]
            self.sim.set_joints(q)
        except (ValueError,RuntimeError) as exc:
            self.stop('역기구학 또는 충돌 검사 실패 · 추적 정지: '+str(exc),dict(target_tcp_mm=pose,error=str(exc)));return False
        tcp=self.sim.kin.pose(self.sim.q);self.sim.trail.append(tuple(v/1000 for v in tcp[:3]));self.sim.trail=self.sim.trail[-300:]
        self.last_applied=tcp
        self.report('카메라 이동 추종 중',dict(marker=marker,delta_mm=self.delta_mm,target_tcp_mm=pose,applied_tcp_mm=tcp,
                   gain=self.gain,speed_limit_mm_s=self.speed_mm_s,demo_axis_mapping='optical Z -> robot X, -X -> Y, -Y -> Z'))
        return True
