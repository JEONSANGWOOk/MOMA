"""Pose-based visual servo demonstration. Camera target + simulated FK feedback.
No robot transport. Camera-to-base mapping is a virtual reference, not calibration.
"""
import copy
import math
import time
from .aruco_arm_follow import rotation
from .aruco_board import FRONT,apply,matmul,pose_from_matrix
from .arm_simulation import rotation_error


def norm(v):return math.sqrt(sum(x*x for x in v))
def limited(v,maximum):
    scale=min(1.,maximum/max(norm(v),1e-12))
    return [x*scale for x in v]


class VisualServo:
    def __init__(self,mission):
        self.mission=mission;self.sim=mission.sim;self.enabled=False
        self.status='비주얼 서보 대기';self.stage='IDLE';self.errors={}
        self.last_stamp=None;self.raw=None;self.lock=0;self.offset=None
        self.reference=None;self.revision=None;self.distance=0.;self.goal=0.;self.completed_cycles=0
        self.position_tolerance_mm=.4;self.angle_tolerance_deg=.3
        self.speed_mm_s=120.;self.approach_mm_s=60.;self.retract_mm_s=100.
        self.angular_deg_s=35.;self.joint_deg_s=120.;self.gain=6.
        self.pen=None;self.marking=False;self.mark_queue=[];self.mark_index=0
        self.mark_results=[];self.completed_marking=0;self.feedforward=None

    def report(self,message,evidence=None,action='가상 TCP 유지'):
        self.status=message
        if self.mission.journal:
            self.mission.journal.emit('SIM.PBVS',dict(stage=self.stage,enabled=self.enabled),
                evidence or {},message,action,identity=(self.stage,self.last_stamp,message))

    def stop(self,reason='비주얼 서보 정지'):
        self.enabled=False;self.stage='STOPPED';self.report(reason,self.errors)

    def configure_speed(self,speed,approach):
        if any(not math.isfinite(v) for v in (speed,approach)) or not 20<=speed<=250 or not 5<=approach<=120:
            raise ValueError('SIM 보정 속도 20~250, 전진 속도 5~120 mm/s')
        self.speed_mm_s=float(speed);self.approach_mm_s=float(approach);self.retract_mm_s=float(speed)
        self.report('서보 속도 설정',dict(correction_mm_s=speed,approach_mm_s=approach))

    def start_marking(self,pen,standby,indices=(0,1)):
        if not indices or any(i not in (None,0,1) for i in indices):raise ValueError('마킹 대상 오류')
        self.start([0,0],standby,max(20.,standby/2),True)
        self.pen=pen;self.marking=True;self.mark_queue=list(indices);self.mark_index=0
        self.mark_results=[];self.completed_marking=0;self.goal=pen.contact_distance_mm
        self.offset=self.marker_offset(indices[0]);self.report('펜 중심점 마킹 시작',dict(targets=list(indices)))

    def marker_offset(self,index):
        return [0.,0.] if index is None else [(-1 if index==0 else 1)*self.mission.config['spacing_mm']/2,0.]

    def start(self,offset,standby,approach,cycle=False):
        if self.enabled or self.mission.running:raise ValueError('현재 작업을 먼저 정지하세요.')
        self.mission.snapshot(offset,standby,approach) # shared geometry/input validation
        self.offset=list(offset);self.distance=standby;self.standby=standby;self.goal=approach
        self.reference=self.mission.reference;self.revision=self.mission.config['revision']
        self.stage='ALIGN';self.cycle=cycle;self.lock=0;self.last_stamp=None;self.raw=None;self.errors={};self.completed_cycles=0;self.marking=False;self.feedforward=None
        self.enabled=True;self.report('PBVS 정면 정렬 시작',action='영상별 위치·각도 오차를 계산해 가상 팔 보정')

    def tick(self,record,now=None):
        if not self.enabled:return
        now=time.time() if now is None else now
        try:
            m=self.mission
            if m.latest is None or not record or not -.1<=now-record['timestamp']<=.7:
                raise ValueError('영상 소실/지연 · 서보 정지')
            if m.reference is not self.reference or m.config['revision']!=self.revision:
                raise ValueError('기준 좌표/등록 변경 · 서보 정지')
            stamp=record['timestamp']
            if stamp==self.last_stamp:return # never advance using repeated camera frames
            raw=record.get('raw_board',record['board'])
            if not raw.get('valid') or raw['revision']!=self.revision:raise ValueError('원본 기준판 측정 무효')
            for field in ('camera_xyz_m','rotation_vector_rad'):
                if len(raw[field])!=3 or any(not math.isfinite(v) for v in raw[field]):raise ValueError('원본 기준판 좌표 오류')
            if raw['camera_xyz_m'][2]<=0 or not math.isfinite(raw['reprojection_px']) or raw['reprojection_px']>2:
                raise ValueError('원본 기준판 깊이/재투영 오류')
            if self.raw:
                jump=math.dist(raw['camera_xyz_m'],self.raw['camera_xyz_m'])*1000
                turn=math.degrees(norm(rotation_error(rotation(raw['rotation_vector_rad']),rotation(self.raw['rotation_vector_rad']))))
                if jump>30 or turn>10:raise ValueError('원본 영상 급변 · 서보 정지')
            dt=1/30 if self.last_stamp is None else min(.25,max(0.,stamp-self.last_stamp))
            if dt<=0:raise ValueError('영상 시간 역전 · 서보 정지')
            self.last_stamp=stamp;self.raw=copy.deepcopy(raw)
            center,r=m.virtual_board();tool=matmul(r,FRONT);n=[row[2] for row in r]
            hole=[center[i]+r[i][0]*self.offset[0]+r[i][1]*self.offset[1] for i in range(3)]
            current=self.sim.kin.fk(self.sim.q)
            tip_offset=self.mission.tool_offset_mm
            p=[current[i][3]*1000+current[i][2]*tip_offset for i in range(3)]
            relative=[p[i]-hole[i] for i in range(3)]
            normal=sum(relative[i]*n[i] for i in range(3))
            lateral=norm([relative[i]-normal*n[i] for i in range(3)])
            angular=rotation_error(tool,current);angle=math.degrees(norm(angular))
            # Forward progress only after image-confirmed transverse/orientation alignment.
            aligned=lateral<=(.6 if self.marking and self.distance<10 else 1.5) and angle<=(.5 if self.marking and self.distance<10 else 1.)
            if self.stage=='APPROACH' and aligned:self.distance=max(self.goal,self.distance-(min(self.approach_mm_s,5.) if self.marking and self.distance<10 else self.approach_mm_s)*dt)
            if self.stage=='RETRACT' and aligned:self.distance=min(self.standby,self.distance+self.retract_mm_s*dt)
            target=[hole[i]+n[i]*self.distance for i in range(3)]
            error=[target[i]-p[i] for i in range(3)]
            self.errors=dict(lateral_mm=lateral,normal_mm=normal,target_distance_mm=self.distance,
                position_error_mm=norm(error),angle_error_deg=angle,revision=self.revision,
                current_tcp_mm=p,target_tcp_mm=target,forward_alignment_allowed=aligned,
                board_camera_xyz_m=list(m.latest['camera_xyz_m']),offset_mm=list(self.offset),
                tip_offset_mm=tip_offset,marker_target=self.mark_queue[self.mark_index] if self.marking else 'custom',
                correction_mm_s=self.speed_mm_s,approach_mm_s=self.approach_mm_s)
            converged=norm(error)<=self.position_tolerance_mm and angle<=self.angle_tolerance_deg
            if self.marking and self.stage=='APPROACH' and self.distance<=self.goal:
                converged=lateral<=.2 and abs(normal-self.goal)<=.04 and angle<=self.angle_tolerance_deg
            self.lock=self.lock+1 if converged else 0
            if self.stage=='ALIGN' and self.lock>=3:
                if self.cycle:self.stage='APPROACH';self.standby=self.distance
                else:self.stage='HOLD'
                self.lock=0
            elif self.stage=='APPROACH' and self.distance<=self.goal and self.lock>=5:
                if self.marking:
                    mark=self.pen.record_mark(self.mark_queue[self.mark_index],self.offset,stamp)
                    if mark is None:raise ValueError('펜 실제 접촉 좌표 불일치 · 마킹 보류')
                    self.mark_results.append(mark);self.report('실제 펜 끝 마킹 확인',mark,'계산된 접촉점을 판넬에 기록')
                self.stage='RETRACT';self.lock=0
            elif self.stage=='RETRACT' and self.distance>=self.standby and self.lock>=5:
                self.completed_cycles+=1;self.lock=0
                if self.marking and self.mark_index+1<len(self.mark_queue):
                    self.mark_index+=1;self.offset=self.marker_offset(self.mark_queue[self.mark_index]);self.stage='ALIGN'
                else:
                    self.stage='HOLD'
                    if self.marking:self.completed_marking+=1;self.marking=False
                    self.report('PBVS 전진·후진 완료 · 실시간 자세 유지 계속',self.errors)
            # Proportional SE(3) correction; norm limits apply to Cartesian commands.
            # Measured target motion feed-forward supports tracking during both phases.
            # It is computed from the moving board at fixed stand-off, excluding commanded advance.
            anchor=[center[i]+n[i]*self.standby for i in range(3)]
            velocity=[0.,0.,0.]
            if self.feedforward is not None:
                velocity=limited([(a-b)/dt for a,b in zip(anchor,self.feedforward)],self.speed_mm_s)
            self.feedforward=anchor
            alpha=1-math.exp(-self.gain*dt)
            step=limited([alpha*v+velocity[i]*dt for i,v in enumerate(error)],self.speed_mm_s*dt)
            delta=limited([alpha*v for v in angular],math.radians(self.angular_deg_s)*dt)
            if not converged:
                next_r=matmul(rotation(delta),[list(row[:3]) for row in current[:3]])
                pose=pose_from_matrix(next_r,[p[i]+step[i]-next_r[i][2]*tip_offset for i in range(3)])
                q=self.sim.kin.ik(pose,self.sim.q,position_tolerance=.000002 if self.marking else .00001,rotation_tolerance=.0001)
                # Reject discontinuous IK branch switches before updating the simulator.
                if max(abs(a-b) for a,b in zip(q,self.sim.q))>math.radians(self.joint_deg_s)*dt:
                    raise ValueError('관절 속도/역기구학 분기 초과 · 서보 정지')
                self.sim.set_joints(q) # swept collision and joint-limit validation
            self.report('PBVS '+dict(ALIGN='정면 정렬',HOLD='실시간 자세 유지',APPROACH='전진 접근',RETRACT='후진 복귀')[self.stage],
                self.errors,'영상 목표와 가상 TCP 오차를 보정')
        except (ValueError,KeyError,TypeError,OverflowError) as exc:self.stop(str(exc))
