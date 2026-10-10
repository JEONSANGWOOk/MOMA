"""Generic cabinet lock and pre-gripped key. All coordinates/control are SIM only."""
import math
from .geometry3d import box,cylinder,multiply,point,transform
from .aruco_board import FRONT,apply,matmul,transpose
from .aruco_arm_follow import rotation
from .arm_physics import ArmPhysics
from .arm_simulation import rotation_error
from .visual_servo import VisualServo,norm,limited


def matrix(r,xyz):
    return tuple(tuple(r[i])+ (xyz[i]/1000,) for i in range(3))+((0,0,0,1),)


class KeyTool:
    def __init__(self,mission):
        self.mission=mission;self.sim=mission.sim;self.length_mm=165.
        p=self.sim.physics;p.configure(dict(kind='finger',force_n=100.))
        self.offset_mm=p.config['mount_offset_m']*1000+self.length_mm
        mission.tool_offset_mm=self.offset_mm
        self.key=p.add_object('파지된 열쇠','box',[.028,.003,.085],.03,[0]*6)
        self.key['collision_parts']=[dict(matrix=transform((0,0,-.0275)),size=[.028,.003,.030]),
                                     dict(matrix=transform((0,0,.015)),size=[.008,.0022,.055])]
        p.held=self.key;p.relative=transform((0,0,.1225));p.opening=.028;p.force=100.
        self.sync()

    def sync(self):self.key['matrix']=multiply(self.sim.physics.tcp(self.sim.q),self.sim.physics.relative)

    def tip_mm(self,q=None):
        return [v*1000 for v in point(self.sim.kin.fk(self.sim.q if q is None else q),(0,0,self.offset_mm/1000))]


class KeyPanelPhysics(ArmPhysics):
    """Slot cheeks mechanically co-rotate with a fully inserted matching key.
    Candidate sweep queries never commit cylinder angle or unlock state.
    """
    scene=None
    def motion_samples(self,source,target,thickness):
        # FR5 wrist 6 rotates only the terminal link and this compact tool.
        # Conservative path-length bounds: <=2 m for proximal joints, <=.35 m
        # for the last wrist/tool. Every sampled point travels <=wall thickness/4.
        # Retain generic checking for any asset/tool outside this validated model.
        if self.scene is None or getattr(self.kin.asset,'name','')!='fairino5_v6_robot' or len(source)!=6:
            return super().motion_samples(source,target,thickness)
        delta=[abs(a-b) for a,b in zip(source,target)]
        return max(1,math.ceil((2*sum(delta[:-1])+.35*delta[-1])/max(.0001,thickness/4)))
    def contacts(self,q):
        scene=self.scene
        if scene is None or not scene.parts:return super().contacts(q)
        old=[o['matrix'] for o in scene.walls]
        feedback=scene.feedback(q)
        if scene.can_turn(feedback):scene.place_walls(feedback['turn_deg'])
        try:return super().contacts(q)
        finally:
            for o,t in zip(scene.walls,old):o['matrix']=t


class KeyPanelScene:
    width_mm=600.;height_mm=800.;depth_mm=150.
    mouth_mm=32.;insertion_mm=22.;slot_mm=(3.2,10.)
    def __init__(self,mission,tool):
        self.mission=mission;self.tool=tool;self.sim=mission.sim
        self.offset=[0.,100.];self.parts=[];self.walls=[];self.rotor_deg=0.;self.unlocked=False;self.correct_key=True
        self.center=None;self.r=None;self.meshes=[]
        self.sim.physics.scene=self

    def nominal(self):return matmul(matmul(self.r,FRONT),rotation([0,0,-math.pi/2]))

    def world(self,local):
        v=apply(self.r,local);return [a+b for a,b in zip(self.center,v)]

    def hole(self):return self.world(self.offset+[self.mouth_mm])

    def feedback(self,q=None):
        p=self.tool.tip_mm(q);local=apply(transpose(self.r),[a-b for a,b in zip(p,self.center)])
        current=self.sim.kin.fk(self.sim.q if q is None else q)
        relative=matmul(transpose(self.nominal()),current)
        turn=math.degrees(math.atan2(relative[1][0],relative[0][0]))
        tilt=math.degrees(math.acos(max(-1.,min(1.,sum(current[i][2]*(-self.r[i][2]) for i in range(3))))))
        return dict(tip_world_mm=p,tip_local_mm=local,lateral_mm=math.dist(local[:2],self.offset),
                    depth_mm=self.mouth_mm-local[2],turn_deg=turn,axis_error_deg=tilt,
                    correct_key=self.correct_key,unlocked=self.unlocked)

    def can_turn(self,f):
        return self.correct_key and f['depth_mm']>=self.insertion_mm-.3 and f['depth_mm']<=self.insertion_mm+.5 and f['lateral_mm']<=.25 and f['axis_error_deg']<=.5

    def add(self,name,size,local,color):
        o=self.sim.physics.add_object(name,'obstacle',[v/1000 for v in size],1.,[0]*6,static=True)
        self.parts.append(o);self.meshes.append((o,local,color));return o

    def build(self):
        self.add('전기 판넬 본체',(600,800,150),[-170,150,-75],'#a8b1b9')
        self.add('판넬 문',(592,792,2),[-170,150,1],'#d0d6da')
        for y in (-100,400):self.add('힌지 '+str(y),(18,70,12),[-455,y,7],'#717d88')
        x,y=self.offset
        self.add('검정 손잡이',(38,118,26),[x,y+76,15],'#202b36')
        self.add('손잡이 받침',(48,165,8),[x,y+53,5],'#37414a')
        # A true rectangular void, not a painted hole on a solid block.
        for name,size,pos in [('좌',(10,30,26),(-6.6,0,19)),('우',(10,30,26),(6.6,0,19)),
                              ('상',(3.2,10,26),(0,10,19)),('하',(3.2,10,26),(0,-10,19))]:
            self.walls.append(self.add('실린더 슬롯 '+name,size,[x+pos[0],y+pos[1],pos[2]],'#bbc7d3'))
        self.add('슬롯 뒷면',(23.2,30,2),[x,y,5],'#687683')

    def place_walls(self,angle):
        rz=rotation([0,0,-math.radians(angle)]);r=matmul(self.r,rz)
        for o,local,_ in self.meshes:
            if o not in self.walls:continue
            delta=[local[0]-self.offset[0],local[1]-self.offset[1],local[2]]
            d=apply(rz,delta);p=self.world([self.offset[0]+d[0],self.offset[1]+d[1],d[2]])
            o['matrix']=matrix(r,p)

    def sync(self):
        self.center,self.r=self.mission.virtual_board()
        if not self.parts:self.build()
        for o,local,_ in self.meshes:o['matrix']=matrix(self.r,self.world(local))
        self.tool.sync();f=self.feedback()
        if self.can_turn(f):self.rotor_deg=f['turn_deg']
        self.place_walls(self.rotor_deg)
        return f

    def reset_lock(self):
        if self.feedback()['depth_mm']>0:raise ValueError('열쇠를 완전히 후퇴한 뒤 초기화하세요.')
        self.rotor_deg=0.;self.unlocked=False;self.place_walls(0.)

    def geometry(self,faces,lines):
        self.tool.sync()
        owned=self.parts+[self.tool.key]
        previous={tuple(point(o['matrix'],p) for p in face) for o in owned for face in box(o['size'])}
        faces=[row for row in faces if row[0] not in previous]
        def mesh(t,polygons,color,name):
            faces.extend((tuple(point(t,p) for p in face),color,name) for face in polygons)
        for o,_,color in self.meshes:
            if o not in self.walls:mesh(o['matrix'],box(o['size']),color,o['name'])
        # Chrome bezel ring leaves the slot unobstructed; polygon annulus in board XY.
        if self.r:
            t=matrix(matmul(self.r,rotation([0,0,-math.radians(self.rotor_deg)])),self.hole())
            for i in range(40):
                a=i*math.tau/40;b=(i+1)*math.tau/40
                def rim(ang,z):return (.019*math.cos(ang),.019*math.sin(ang),z)
                def slot(ang,z):
                    c,s=math.cos(ang),math.sin(ang);rad=min(.0016/max(abs(c),1e-12),.005/max(abs(s),1e-12))
                    return (rad*c,rad*s,z)
                mesh(t,[[rim(a,0),rim(b,0),slot(b,0),slot(a,0)]],'#e3e9ee','chrome_bezel')
                mesh(t,[[rim(a,0),rim(a,-.026),rim(b,-.026),rim(b,0)]],'#9cabb9','chrome_barrel')
                mesh(t,[[slot(a,0),slot(b,0),slot(b,-.026),slot(a,-.026)]],'#546473','slot_inner_wall')
            # Visual marker plates establish their actual metric centers, no invented IDs.
            for index,m in enumerate(self.mission.config['markers']):
                local=[(-1 if index==0 else 1)*self.mission.config['spacing_mm']/2,0,2.2]
                t=matrix(self.r,self.world(local));s=m['side_mm']/1000
                mesh(t,box((s,s,.0004)),'#f2f3f4','marker_paper')
                mesh(multiply(t,transform((0,0,.0003))),box((s*.8,s*.8,.0002)),'#182431','marker_border')
        t=self.sim.physics.tcp(self.sim.q)
        # Flat brass key blade with generic teeth, bow with a visible central hole.
        profile=[(-.004,.110),(.004,.110),(.004,.150),(.002,.152),(.004,.154),(.001,.157),(.004,.159),(.004,.165),(-.004,.165)]
        for yy in (-.0011,.0011):mesh(t,[[(x,yy,z) for x,z in profile]],'#dca936','key_blade')
        for i,(x,z) in enumerate(profile):
            nx,nz=profile[(i+1)%len(profile)];mesh(t,[[(x,-.0011,z),(nx,-.0011,nz),(nx,.0011,nz),(x,.0011,z)]],'#b88623','key_edge')
        for i in range(32):
            a=i*math.tau/32;b=(i+1)*math.tau/32
            for yy in (-.0015,.0015):
                mesh(t,[[(rad*math.cos(ang),yy,.095+rad*math.sin(ang)) for rad,ang in ((.014,a),(.014,b),(.005,b),(.005,a))]],'#dca936','key_bow')
        return faces,lines


class KeyServo(VisualServo):
    def __init__(self,mission,scene):
        super().__init__(mission);self.scene=scene;self.turn=0.;self.insert_speed=10.;self.turn_speed=30.

    def start_key(self):
        if self.scene.unlocked:raise ValueError('이미 잠금해제 상태입니다. 후퇴 후 초기화하세요.')
        self.start([0,0],100,20)
        self.distance=self.scene.mouth_mm+100;self.turn=0.;self.lock=0
        self.report('열쇠 정렬 시작',dict(hole_offset_mm=self.scene.offset,insert_mm=self.scene.insertion_mm))

    def withdraw(self):
        if not self.mission.latest:raise ValueError('유효한 두 마커 영상이 필요합니다.')
        f=self.scene.feedback()
        self.reference=self.mission.reference;self.revision=self.mission.config['revision']
        self.distance=f['tip_local_mm'][2];self.turn=f['turn_deg'];self.stage='RETRACT';self.lock=0
        self.last_stamp=None;self.raw=None;self.feedforward=None;self.enabled=True

    def tick(self,record,now=None):
        if not self.enabled:return
        import time
        try:
            dt=self.read_sample(record,time.time() if now is None else now)
            if dt is None:return
            s=self.scene;f=s.sync();n=[row[2] for row in s.r]
            target_r=matmul(s.nominal(),rotation([0,0,math.radians(self.turn)]))
            angle=math.degrees(norm(rotation_error(target_r,self.sim.kin.fk(self.sim.q))))
            aligned=f['lateral_mm']<=.20 and angle<=.3
            if self.stage=='APPROACH' and aligned:self.distance=max(s.mouth_mm+5,self.distance-self.approach_mm_s*dt)
            if self.stage=='INSERT' and aligned:self.distance=max(s.mouth_mm-s.insertion_mm,self.distance-self.insert_speed*dt)
            if self.stage=='RETRACT' and f['lateral_mm']<=1.5 and angle<=1:self.distance=min(s.mouth_mm+100,self.distance+self.retract_mm_s*dt)
            if self.stage=='TURN':
                if not s.can_turn(f):raise ValueError('삽입 깊이/정렬/열쇠 일치 조건 미충족: 회전 정지')
                self.turn=min(90.,self.turn+self.turn_speed*dt)
                target_r=matmul(s.nominal(),rotation([0,0,math.radians(self.turn)]))
            goal=s.world(s.offset+[self.distance]);error=math.dist(goal,f['tip_world_mm'])
            converged=error<=.15 and angle<=.3
            self.lock=self.lock+1 if converged else 0
            if self.stage=='ALIGN' and self.lock>=3:self.stage='APPROACH';self.lock=0
            elif self.stage=='APPROACH' and self.distance<=s.mouth_mm+5 and self.lock>=3:self.stage='INSERT';self.lock=0
            elif self.stage=='INSERT' and self.distance<=s.mouth_mm-s.insertion_mm and self.lock>=5:
                if not s.can_turn(f):raise ValueError('키가 일치하지 않거나 완전 삽입이 확인되지 않았습니다.')
                self.stage='TURN';self.lock=0
            elif self.stage=='TURN' and self.turn>=90 and abs(f['turn_deg']-90)<=.3 and self.lock>=5:
                if not s.can_turn(f):raise ValueError('잠금해제 조건 미충족')
                s.unlocked=True;self.stage='HOLD_UNLOCKED';self.lock=0
            elif self.stage=='RETRACT' and self.distance>=s.mouth_mm+100 and self.lock>=5:self.stage='HOLD';self.lock=0
            velocity=[0.,0.,0.]
            anchor=s.hole()
            if self.feedforward is not None:velocity=limited([(a-b)/dt for a,b in zip(anchor,self.feedforward)],self.speed_mm_s)
            self.feedforward=anchor
            self.move_tip(target_r,goal,dt,velocity,precision=True)
            f=s.sync();self.errors=dict(f,target_tcp_mm=goal,position_error_mm=math.dist(goal,f['tip_world_mm']),
                angle_error_deg=math.degrees(norm(rotation_error(target_r,self.sim.kin.fk(self.sim.q)))),
                command_depth_mm=s.mouth_mm-self.distance,command_turn_deg=self.turn,
                revision=self.revision,sample_timestamp=self.last_stamp,vision_source=record.get('vision_source','D455'),
                camera_board_xyz_m=list(self.mission.latest['camera_xyz_m']),
                camera_board_rvec_rad=list(self.mission.latest['rotation_vector_rad']),
                alignment_confirmed=aligned,rotation_allowed=s.can_turn(f),consecutive_confirmed_frames=self.lock,
                rotation_requirements=dict(min_depth_mm=s.insertion_mm-.3,max_lateral_mm=.25,max_axis_deg=.5,matching_key=True),
                collision_checks=True,source='simulated_FK_feedback')
            labels=dict(ALIGN='마커 기준 열쇠 위치·자세 정렬',APPROACH='정렬 확인 후 슬롯 앞까지 접근',
                INSERT='횡방향·각도 확인하며 삽입',TURN='완전 삽입·키 일치 확인 후 회전',
                HOLD_UNLOCKED='실제 90° 회전 확인 · 잠금해제 자세 추종',RETRACT='현재 방향 유지하며 후퇴',HOLD='후퇴 완료 · 실시간 추종')
            self.report('열쇠 SIM · '+labels[self.stage],self.errors,'실시간 마커 자세로 열쇠 끝 위치/회전 보정')
        except (ValueError,KeyError,TypeError,OverflowError) as exc:self.stop(str(exc))
