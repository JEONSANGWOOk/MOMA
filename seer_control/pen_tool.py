"""Pre-gripped virtual pen, explicit tip TCP and actual simulated impact marks."""
import math
from .geometry3d import box,cylinder,multiply,point,transform
from .aruco_board import apply,transpose


class PenTool:
    def __init__(self,mission,length_mm=180.):
        if not math.isfinite(length_mm) or not 150<=length_mm<=250:raise ValueError('펜 끝 TCP는 150~250mm입니다.')
        self.mission=mission;self.sim=mission.sim;self.length_mm=length_mm
        physics=self.sim.physics;physics.configure(dict(kind='finger',force_n=100.))
        self.offset_mm=physics.config['mount_offset_m']*1000+length_mm
        mission.tool_offset_mm=self.offset_mm
        body_length=(length_mm-60)/1000;center=(60+length_mm)/2000
        self.pen=physics.add_object('파지된 펜','box',[.01,.01,body_length],.04,[0]*6)
        physics.held=self.pen;physics.relative=transform((0,0,center));physics.opening=.01
        physics.force=100.;physics.status='펜 초기 파지 상태 · SIM'
        self.sim.io['ToolDO'][physics.config['do_channel']]=1
        self.marks=[];self.contact_distance_mm=2.0
        self.sync()
        mission.report('그리퍼·펜 초기 파지 설정',dict(tip_offset_mm=self.offset_mm,pen_length_mm=length_mm),action='펜 끝 TCP로 가상 서보 제어')

    def sync(self):
        self.pen['matrix']=multiply(self.sim.physics.tcp(self.sim.q),self.sim.physics.relative)
        self.pen['state']='초기 파지 / 펜'

    def tip_mm(self,q=None):
        t=self.sim.kin.fk(self.sim.q if q is None else q)
        return [v*1000 for v in point(t,(0,0,self.offset_mm/1000))]

    def board_tip(self):
        center,r=self.mission.virtual_board()
        return apply(transpose(r),[v-c for v,c in zip(self.tip_mm(),center)])

    def record_mark(self,index,offset,stamp):
        local=self.board_tip();target=[offset[0],offset[1],self.contact_distance_mm]
        planar=math.dist(local[:2],target[:2]);normal=local[2]-target[2]
        # Record only actual simulated near-surface contact; never replace hit with target.
        if abs(normal)>.05 or planar>.25:return None
        marker=None if index is None else dict(self.mission.config['markers'][index])
        mark=dict(marker_index=index,marker=marker,timestamp=stamp,revision=self.mission.config['revision'],
            target_local_mm=target,hit_local_mm=local,hit_world_mm=self.tip_mm(),
            planar_error_mm=planar,normal_error_mm=normal,success=planar<=.25)
        self.marks.append(mark);return mark

    def geometry(self,faces,lines):
        self.sync();matrix=self.pen['matrix']
        previous={tuple(point(matrix,p) for p in face) for face in box(self.pen['size'])}
        faces=[row for row in faces if row[0] not in previous]
        t=self.sim.physics.tcp(self.sim.q);length=self.length_mm/1000
        body=multiply(t,transform((0,0,(.06+length-.012)/2)))
        for face in cylinder(.005,length-.072,steps=20):
            faces.append((tuple(point(body,p) for p in face),'#e7ac32','pen'))
        ring=[(.005*math.cos(i*math.tau/20),.005*math.sin(i*math.tau/20),length-.012) for i in range(20)]
        for i in range(20):
            faces.append((tuple(point(t,p) for p in (ring[i],ring[(i+1)%20],(0,0,length))),'#222b38','pen_tip'))
        return faces,lines

    def draw_overlay(self,canvas,servo):
        center,r=self.mission.virtual_board();width=max(10,canvas.winfo_width());height=max(10,canvas.winfo_height())
        def project(local):
            moved=apply(r,local)
            return canvas.camera.project([(center[i]+moved[i])/1000 for i in range(3)],width,height)
        def dot(p,color,radius=4):
            if p:canvas.create_oval(p[0]-radius,p[1]-radius,p[0]+radius,p[1]+radius,fill=color,outline='white')
        for index in (0,1):
            x=(-1 if index==0 else 1)*self.mission.config['spacing_mm']/2
            target=project([x,0,self.contact_distance_mm])
            if target:
                canvas.create_line(target[0]-7,target[1],target[0]+7,target[1],fill='#e84444',width=2)
                canvas.create_line(target[0],target[1]-7,target[0],target[1]+7,fill='#e84444',width=2)
        for mark in self.marks:
            if mark['revision']==self.mission.config['revision']:dot(project(mark['hit_local_mm']),'#21b969',3)
        actual=canvas.camera.project([v/1000 for v in self.tip_mm()],width,height);dot(actual,'#f8b622',5)
        if actual:canvas.create_text(actual[0]+9,actual[1]-12,text='펜 끝 TCP',anchor='w',fill='#aa6a00')
        # Fixed board-plane inset: target and real impact use the same metric scale.
        left=max(12,width-330);top=14
        canvas.create_rectangle(left,top,width-12,top+185,fill='#ffffff',outline='#91a8bd')
        canvas.create_text(left+10,top+10,anchor='nw',text='판넬 정면 · 빨강 목표 / 초록 실제 접촉',fill='#24394e')
        spacing=self.mission.config['spacing_mm'];side=max(m['side_mm'] for m in self.mission.config['markers'])
        scale=min(280/(spacing+side+20),95/(side+20));cx=(left+width-12)/2;cy=top+94
        def xy(local):return cx+local[0]*scale,cy-local[1]*scale
        for index,marker in enumerate(self.mission.config['markers']):
            x=(-1 if index==0 else 1)*spacing/2;px,py=xy([x,0]);half=marker['side_mm']*scale/2
            canvas.create_rectangle(px-half,py-half,px+half,py+half,fill='#e9eef5',outline='#627b92')
            canvas.create_line(px-6,py,px+6,py,fill='#e84444',width=2)
            canvas.create_line(px,py-6,px,py+6,fill='#e84444',width=2)
            canvas.create_text(px,py-half-9,text=f'#{index+1}',fill='#24394e')
        tip=self.board_tip();tx,ty=xy(tip)
        if left+4<=tx<=width-16 and top+28<=ty<=top+153:
            dot((tx,ty),'#aa65d9',4)
        for mark in self.marks:
            if mark['revision']==self.mission.config['revision']:dot(xy(mark['hit_local_mm']),'#21b969',3)
        canvas.create_text(left+10,top+159,anchor='nw',text=f'보라: 현재 펜 투영 · 표면까지 {tip[2]-self.contact_distance_mm:+.2f}mm',fill='#7542a3')
