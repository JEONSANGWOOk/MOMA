"""Generic cabinet lock and pre-gripped key. All coordinates/control are SIM only."""
import math
from .panel_handle_model import settings,handle_meshes,socket_meshes,key_meshes
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
        self.model=settings();c=self.model
        extent=c['key_shaft_mm']+20+c['key_bow_length_mm']/2
        center=self.length_mm-extent/2
        self.key=p.add_object('파지된 원통형 열쇠','box',[c['key_bow_width_mm']/1000,c['key_outer_radius_mm']*.002,extent/1000],.03,[0]*6)
        self.key['collision_parts']=[dict(matrix=transform((0,0,(self.length_mm-c['key_shaft_mm']-20-center)/1000)),size=[c['key_bow_width_mm']/1000,.003,c['key_bow_length_mm']/1000]),
                                     dict(matrix=transform((0,0,(self.length_mm-c['key_shaft_mm']/2-center)/1000)),size=[c['key_outer_radius_mm']*.002,c['key_outer_radius_mm']*.002,c['key_shaft_mm']/1000])]
        p.held=self.key;p.relative=transform((0,0,center/1000));p.opening=.028;p.force=100.
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
        try:
            risks,near=super().contacts(q)
            # The tubular key's coarse OBB has a filled center. Replace only its
            # bore contacts with an annular clearance check, preserving robot,
            # gripper, handle and rear-stop collision checks.
            bore={o['name'] for o in scene.walls}
            risks=[pair for pair in risks if not (pair[0]==scene.tool.key['name'] and pair[1] in bore)]
            near=[pair for pair in near if not (pair[0]==scene.tool.key['name'] and pair[1] in bore)]
            if feedback['depth_mm']>0:
                c=scene.model;overlap=min(feedback['depth_mm'],c['key_shaft_mm'])
                eccentric=feedback['lateral_mm']+overlap*math.sin(math.radians(feedback['axis_error_deg']))
                clearance=min(c['socket_radius_mm']-c['key_outer_radius_mm'],c['key_inner_radius_mm']-c['pin_radius_mm'])
                if eccentric>clearance or feedback['depth_mm']>c['barrel_depth_mm']:
                    risks.append((scene.tool.key['name'],'원통 슬롯 내외경 간섭'))
            return risks,near
        finally:
            for o,t in zip(scene.walls,old):o['matrix']=t


class KeyPanelScene:
    width_mm=600.;height_mm=800.;depth_mm=150.
    mouth_mm=32.;insertion_mm=22.
    def __init__(self,mission,tool):
        self.mission=mission;self.tool=tool;self.sim=mission.sim
        self.model=tool.model;self.insertion_mm=self.model['insertion_mm']
        self.offset=[0.,100.];self.parts=[];self.walls=[];self.rotor_deg=0.;self.unlocked=False;self.correct_key=True
        self.center=None;self.r=None;self.meshes=[]
        self.sim.physics.scene=self

    def nominal(self):return matmul(matmul(self.r,FRONT),rotation([0,0,-math.pi/2]))

    def world(self,local):
        v=apply(self.r,local);return [a+b for a,b in zip(self.center,v)]

    def hole(self):return self.world(self.offset+[self.mouth_mm])

    def retarget(self,offset):
        delta=[b-a for a,b in zip(self.offset,offset)]
        for obj,local,_ in self.meshes:
            if obj['name'] in ('전기 판넬 본체','판넬 문') or obj['name'].startswith('힌지 '):continue
            local[0]+=delta[0];local[1]+=delta[1]
        self.offset=list(offset)

    def feedback(self,q=None):
        p=self.tool.tip_mm(q);local=apply(transpose(self.r),[a-b for a,b in zip(p,self.center)])
        current=self.sim.kin.fk(self.sim.q if q is None else q)
        relative=matmul(transpose(self.nominal()),current)
        turn=math.degrees(math.atan2(relative[1][0],relative[0][0]))
        tilt=math.degrees(math.acos(max(-1.,min(1.,sum(current[i][2]*(-self.r[i][2]) for i in range(3))))))
        return dict(tip_world_mm=p,tip_local_mm=local,lateral_mm=math.dist(local[:2],self.offset),
                    depth_mm=self.mouth_mm-local[2],turn_deg=turn,axis_error_deg=tilt,
                    correct_key=self.correct_key,unlocked=self.unlocked,model_product=self.model['product'],dimension_source=self.model['dimension_source'],
                    annular_clearance_mm=min(self.model['socket_radius_mm']-self.model['key_outer_radius_mm'],self.model['key_inner_radius_mm']-self.model['pin_radius_mm']))

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
        c=self.model
        self.add('은색 테이퍼 손잡이',(30,c['handle_length_mm']-35,14),[x,y-(c['handle_length_mm']+15)/2,22],'#cbd6de')
        # Eight bounding cheeks around the circular annulus. The center post is
        # intentionally separate so the empty key center is part of the model.
        radius=(c['barrel_radius_mm']+c['socket_radius_mm'])/2
        for i in range(8):
            angle=i*math.tau/8
            wall=self.add('원통 실린더 슬롯 '+str(i),(c['barrel_radius_mm']-c['socket_radius_mm'],radius*.82,c['barrel_depth_mm']),
                [x+radius*math.cos(angle),y+radius*math.sin(angle),self.mouth_mm-c['barrel_depth_mm']/2],'#bbc7d3')
            wall['slot_angle_rad']=angle;self.walls.append(wall)
        pin=self.add('원통 슬롯 중앙 핀',(c['pin_radius_mm']*2**.5,c['pin_radius_mm']*2**.5,c['barrel_depth_mm']-1),
            [x,y,self.mouth_mm-(c['barrel_depth_mm']-1)/2],'#8798a6');self.walls.append(pin)
        self.add('슬롯 뒷면',(23.2,30,2),[x,y,self.mouth_mm-c['barrel_depth_mm']-1],'#687683')

    def place_walls(self,angle):
        rz=rotation([0,0,-math.radians(angle)]);r=matmul(self.r,rz)
        for o,local,_ in self.meshes:
            if o not in self.walls:continue
            delta=[local[0]-self.offset[0],local[1]-self.offset[1],local[2]]
            d=apply(rz,delta);p=self.world([self.offset[0]+d[0],self.offset[1]+d[1],d[2]])
            o['matrix']=matrix(matmul(r,rotation([0,0,o.get('slot_angle_rad',0)])),p)

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
            if o not in self.walls and o['name']!='은색 테이퍼 손잡이':mesh(o['matrix'],box(o['size']),color,o['name'])
        if self.r:
            t=matrix(self.r,self.hole())
            for polygons,color,name in handle_meshes(self.model)+socket_meshes(self.model,math.radians(self.rotor_deg)):
                mesh(t,polygons,color,name)
            # Visual marker plates establish their actual metric centers, no invented IDs.
            for index,m in enumerate([] if getattr(self.mission,'allow_photo_screen',False) else self.mission.config['markers']):
                local=[(-1 if index==0 else 1)*self.mission.config['spacing_mm']/2,0,2.2]
                t=matrix(self.r,self.world(local));s=m['side_mm']/1000
                mesh(t,box((s,s,.0004)),'#f2f3f4','marker_paper')
                mesh(multiply(t,transform((0,0,.0003))),box((s*.8,s*.8,.0002)),'#182431','marker_border')
        t=multiply(self.sim.kin.fk(self.sim.q),transform((0,0,self.tool.offset_mm/1000)))
        for polygons,color,name in key_meshes(self.model):mesh(t,polygons,color,name)
        return faces,lines


class KeyServo(VisualServo):
    turn_goal_deg=30.
    def __init__(self,mission,scene):
        super().__init__(mission);self.scene=scene;self.turn=0.;self.insert_speed=10.;self.turn_speed=30.;self.updates=0;self.reacquire=None;self.reacquire_count=0;self.completed=False;self.require_visual_target=False;self.visual_target_ready=True

    def start_key(self):
        if self.scene.unlocked:raise ValueError('이미 잠금해제 상태입니다. 후퇴 후 초기화하세요.')
        self.start([0,0],100,20)
        self.distance=self.scene.mouth_mm+100;self.turn=0.;self.lock=0;self.updates=0;self.reacquire=None;self.reacquire_count=0;self.completed=False
        self.report('열쇠 정렬 시작',dict(hole_offset_mm=self.scene.offset,insert_mm=self.scene.insertion_mm))

    def withdraw(self):
        if not self.mission.latest:raise ValueError('유효한 두 마커 영상이 필요합니다.')
        f=self.scene.feedback()
        self.reference=self.mission.reference;self.revision=self.mission.config['revision']
        self.distance=f['tip_local_mm'][2];self.turn=f['turn_deg'];self.stage='RETRACT';self.lock=0
        self.last_stamp=None;self.raw=None;self.feedforward=None;self.enabled=True

    def accept_raw_change(self,raw):
        import copy
        def changed(a,b,mm,deg):
            return math.dist(a['camera_xyz_m'],b['camera_xyz_m'])*1000>mm or math.degrees(norm(rotation_error(rotation(a['rotation_vector_rad']),rotation(b['rotation_vector_rad']))))>deg
        if not changed(raw,self.raw,30,10):
            self.reacquire=None;self.reacquire_count=0
            return True
        # Only a free-space SIM can reacquire a moved target. A key already
        # inserted into the socket retains the strict contact-motion guard.
        if self.scene.feedback()['depth_mm']>0:return super().accept_raw_change(raw)
        self.reacquire_count=self.reacquire_count+1 if self.reacquire and not changed(raw,self.reacquire,15,5) else 1
        self.reacquire=copy.deepcopy(raw)
        self.feedforward=None;self.lock=0
        if self.reacquire_count<3:
            self.report('마커 이동 재확인 · 추종 유지 · 전진 보류',dict(confirmation_frames=self.reacquire_count),action='새 위치 3프레임 확인 대기')
            return False
        self.reacquire=None;self.reacquire_count=0
        return True

    def tick(self,record,now=None):
        if not self.enabled:return
        import time
        try:
            dt=self.read_sample(record,time.time() if now is None else now)
            if dt is None:return
            # A slow GUI/IK cycle must not turn elapsed camera time into a large
            # swept motion and an ever slower next cycle. Drop missed steps.
            sample_dt=dt;dt=min(dt,1/30)
            self.updates+=1
            s=self.scene;f=s.sync();n=[row[2] for row in s.r]
            target_r=matmul(s.nominal(),rotation([0,0,math.radians(self.turn)]))
            angle=math.degrees(norm(rotation_error(target_r,self.sim.kin.fk(self.sim.q))))
            aligned=f['lateral_mm']<=.20 and angle<=.3 and (not self.require_visual_target or self.visual_target_ready)
            if self.stage=='APPROACH' and aligned:self.distance=max(s.mouth_mm+5,self.distance-self.approach_mm_s*dt)
            if self.stage=='INSERT' and aligned:self.distance=max(s.mouth_mm-s.insertion_mm,self.distance-self.insert_speed*dt)
            if self.stage=='RETRACT' and f['lateral_mm']<=1.5 and angle<=1:self.distance=min(s.mouth_mm+100,self.distance+self.retract_mm_s*dt)
            if self.stage=='TURN' and (not self.require_visual_target or self.visual_target_ready):
                if not s.can_turn(f):raise ValueError('삽입 깊이/정렬/열쇠 일치 조건 미충족: 회전 정지')
                self.turn=min(self.turn_goal_deg,self.turn+self.turn_speed*dt)
                target_r=matmul(s.nominal(),rotation([0,0,math.radians(self.turn)]))
            goal=s.world(s.offset+[self.distance]);error=math.dist(goal,f['tip_world_mm'])
            converged=error<=.15 and angle<=.3 and (not self.require_visual_target or self.visual_target_ready or self.stage in ('RETRACT','HOLD'))
            self.lock=self.lock+1 if converged else 0
            if self.stage=='ALIGN' and self.lock>=3:self.stage='APPROACH';self.lock=0
            elif self.stage=='APPROACH' and self.distance<=s.mouth_mm+5 and self.lock>=3:self.stage='INSERT';self.lock=0
            elif self.stage=='INSERT' and self.distance<=s.mouth_mm-s.insertion_mm and self.lock>=5:
                if not s.can_turn(f):raise ValueError('키가 일치하지 않거나 완전 삽입이 확인되지 않았습니다.')
                self.stage='TURN';self.lock=0
            elif self.stage=='TURN' and self.turn>=self.turn_goal_deg and abs(f['turn_deg']-self.turn_goal_deg)<=.3 and self.lock>=5:
                if not s.can_turn(f):raise ValueError('잠금해제 조건 미충족')
                s.unlocked=True;self.stage='RETRACT';self.lock=0
            elif self.stage=='RETRACT' and self.distance>=s.mouth_mm+100 and self.lock>=5:self.stage='HOLD';self.lock=0;self.completed=s.unlocked
            velocity=[0.,0.,0.];angular_velocity=[0.,0.,0.]
            # Compare the two board poses at identical commanded depth so advancing
            # the key cannot masquerade as marker motion or double the insertion speed.
            anchor=s.world(s.offset+[self.distance])
            if self.feedforward is not None:
                old_center,old_r=self.feedforward
                old_offset=apply(old_r,s.offset+[self.distance])
                old_anchor=[a+b for a,b in zip(old_center,old_offset)]
                velocity=limited([(a-b)/sample_dt for a,b in zip(anchor,old_anchor)],self.speed_mm_s)
                angular_velocity=limited([v/sample_dt for v in rotation_error(s.r,old_r)],math.radians(self.angular_deg_s))
            self.feedforward=(list(s.center),[list(row) for row in s.r])
            self.move_tip(target_r,goal,dt,velocity,precision=True,angular_velocity=angular_velocity)
            f=s.sync();self.errors=dict(f,target_tcp_mm=goal,position_error_mm=math.dist(goal,f['tip_world_mm']),
                angle_error_deg=math.degrees(norm(rotation_error(target_r,self.sim.kin.fk(self.sim.q)))),
                command_depth_mm=s.mouth_mm-self.distance,command_turn_deg=self.turn,
                revision=self.revision,sample_timestamp=self.last_stamp,vision_source=record.get('vision_source','D455'),
                camera_board_xyz_m=list(self.mission.latest['camera_xyz_m']),
                camera_board_rvec_rad=list(self.mission.latest['rotation_vector_rad']),tracking_updates=self.updates,
                advance_paused_for_alignment=self.stage in ('APPROACH','INSERT') and not aligned,
                alignment_confirmed=aligned,rotation_allowed=s.can_turn(f),consecutive_confirmed_frames=self.lock,
                rotation_requirements=dict(min_depth_mm=s.insertion_mm-.3,max_lateral_mm=.25,max_axis_deg=.5,matching_key=True),
                collision_checks=True,completed=self.completed,visual_target_required=self.require_visual_target,visual_target_ready=self.visual_target_ready,source='simulated_FK_feedback')
            labels=dict(ALIGN='마커 기준 열쇠 위치·자세 정렬',APPROACH='정렬 확인 후 슬롯 앞까지 접근',
                INSERT='횡방향·각도 확인하며 삽입',TURN='완전 삽입·키 일치 확인 후 회전',
                RETRACT='30° 방향 유지하며 열쇠 자동 후퇴',HOLD='후퇴 완료 · 실시간 추종')
            self.report('열쇠 SIM · '+labels[self.stage]+(' · 실제 구멍 미확인: 전진·회전 보류' if self.require_visual_target and not self.visual_target_ready and self.stage not in ('RETRACT','HOLD') else ''),self.errors,'실시간 마커 자세와 구멍 중심으로 열쇠 끝 위치/회전 보정')
        except (ValueError,KeyError,TypeError,OverflowError) as exc:self.stop(str(exc))
