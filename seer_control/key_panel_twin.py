"""SIM/REAL coordinator. Measured feedback and predicted joints remain separate."""
import copy,math,queue,socket,threading,time
from .fairino_api import profile,vector
from .fairino_client import FairinoClient
from .key_panel_real import RealKeyPlan,calibration,socket_frame,pose_matrix
from .geometry3d import multiply,transform,box,point
from .arm_physics import inverse
from .arm_simulation import ArmSimulator,rotation_error
from .aruco_board import pose_from_matrix,transpose,apply
from .visual_servo import norm


def metres(t):return tuple(tuple(v/1000 if j==3 and i<3 else v for j,v in enumerate(row)) for i,row in enumerate(t))


class KeyTwin:
    def __init__(self,asset):
        self.sim=ArmSimulator(asset);self.tool=None;self.socket=None;self.c=None
        self.command=None;self.fitted=False

    def fit(self,feedback):
        joints=vector(feedback['joints_rad'],'실제 관절 rad');tcp=metres(pose_matrix(feedback['tcp_mm_deg']))
        self.sim.q=list(joints)
        # Display/FK alignment only. Never used to derive any physical command.
        self.tool=multiply(inverse(self.sim.kin.fk(joints)),tcp);self.fitted=True;self.command=None

    def tcp(self):return multiply(self.sim.kin.fk(self.sim.q),self.tool) if self.fitted else None

    def predict(self,target):
        if not self.fitted:raise ValueError('실제 자세를 수신한 뒤 SIM 비교 기준을 설정하세요.')
        tip=metres(pose_matrix(target));flange=multiply(tip,inverse(self.tool))
        q=self.sim.kin.ik(pose_from_matrix(flange,[row[3]*1000 for row in flange[:3]]),self.sim.q,position_tolerance=.00002,rotation_tolerance=.0002)
        if max(abs(a-b) for a,b in zip(q,self.sim.q))>math.radians(15):raise ValueError('SIM 관절 분기 급변 · 동시 실행 차단')
        self.sim.set_joints(q);self.command=list(target)
        return q

    def comparison(self,feedback):
        t=self.tcp()
        if t is None or not feedback:return {}
        actual=pose_matrix(feedback['tcp_mm_deg']);sim_pose=pose_from_matrix(t,[r[3]*1000 for r in t[:3]])
        return dict(sim_tcp_mm_deg=sim_pose,real_tcp_mm_deg=list(feedback['tcp_mm_deg']),
            tcp_difference_mm=math.dist(sim_pose[:3],[r[3] for r in actual[:3]]),
            angle_difference_deg=math.degrees(norm(rotation_error(t,actual))),
            joint_difference_deg=[math.degrees(a-b) for a,b in zip(self.sim.q,feedback['joints_rad'])],
            last_target_tcp=self.command,alignment='model_display_fit_only')

    def geometry(self,faces,lines,tip=None):
        result=list(faces)
        def mesh(t,size,color,name):result.extend((tuple(point(t,p) for p in f),color,name) for f in box(size))
        if self.socket:
            s=metres(self.socket)
            for local,size,color,name in [((-.17,.05,-.107),(.6,.8,.15),'#a8b1b9','twin_panel'),
                 ((-.17,.05,-.031),(.592,.792,.002),'#d0d6da','twin_door'),
                 ((0,.076,-.017),(.038,.118,.026),'#202b36','twin_handle')]:
                mesh(multiply(s,transform(local)),size,color,name)
            turn=0.
            if self.fitted and self.c:
                z=multiply(self.socket,self.c['T_socket_key_zero_mm']);key=tip if tip is not None else self.tcp()
                r=[[sum(z[k][i]*key[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
                turn=math.atan2(r[1][0],r[0][0])
            cylinder=multiply(s,transform(rpy=(0,0,-turn)))
            for local,size in [((-.0066,0,-.013),(.010,.030,.026)),((.0066,0,-.013),(.010,.030,.026)),((0,.010,-.013),(.0032,.010,.026)),((0,-.010,-.013),(.0032,.010,.026))]:
                mesh(multiply(cylinder,transform(local)),size,'#c7d2dc','twin_slot')
        if self.fitted:
            t=tip if tip is not None else self.tcp()
            for local,size,color,name in [((0,0,-.0275),(.008,.0022,.055),'#dba633','predicted_key'),((0,0,-.070),(.028,.003,.030),'#dba633','key_bow'),((0,0,-.140),(.09,.06,.05),'#49657d','gripper_reference')]:
                mesh(multiply(t,transform(local)),size,color,name)
        return result,lines


class KeyPanelLink:
    def __init__(self,twin,journal=None,client=None):
        self.twin=twin;self.journal=journal;self.client=client or FairinoClient()
        self.config=profile();self.config['ip']='192.168.57.2';self.cal=None
        self.feedback={};self.rx=0.;self.mode='SIM';self.plan=None;self.busy=False
        self.events=queue.Queue();self.epoch=0;self.poll_at=0.;self.status='FR5 미연결';self.last_record=None;self.pending_kind=None

    def report(self,message,evidence=None,action='통합 화면 갱신'):
        self.status=message
        if self.journal:self.journal.emit('TWIN.FR5',dict(mode=self.mode,active=self.plan is not None,connected=self.client.connected),evidence or {},message,action)

    def submit(self,kind,fn):
        if self.busy:raise ValueError('FR5 요청 처리 중')
        token=self.epoch;self.busy=True;self.pending_kind=kind
        def run():
            try:self.events.put((token,kind,True,fn()))
            except Exception as exc:self.events.put((token,kind,False,str(exc)))
        threading.Thread(target=run,daemon=True,name='key-twin-'+kind).start()

    def connect(self,ip,sdk):
        if self.plan or self.busy or self.client.connected:raise ValueError('정지·연결 해제 후 다시 연결하세요.')
        self.config=profile();self.config.update(ip=ip,sdk_path=sdk)
        if self.cal:self.config.update(verified=self.cal['verified'],version_confirmed=self.cal['version_confirmed'],controller_version=self.cal['controller_version'])
        self.feedback={};self.rx=0.;self.twin.fitted=False
        def job():
            with socket.create_connection((ip,20003),timeout=2):pass
            self.client.connect(self.config);return self.client.call(self.config,'panel_status')
        self.submit('connect',job);self.report('FR5 읽기 전용 연결 중')

    def load(self,c):
        if self.plan or self.busy or self.client.connected:raise ValueError('연결 해제 후 보정 파일을 변경하세요.')
        self.cal=copy.deepcopy(calibration(c));self.twin.c=self.cal
        self.report('실측 보정 파일 적용',dict(calibration=self.cal))

    def set_mode(self,mode):
        if mode not in ('SIM','REAL','SIM+REAL'):raise ValueError('실행 모드 오류')
        if self.plan:raise ValueError('작업 정지 확인 후 모드를 변경하세요.')
        if self.busy and self.pending_kind!='poll':raise ValueError('FR5 요청 완료 후 모드를 변경하세요.')
        self.mode=mode;self.report('실행 모드 '+mode)

    def start(self,record,contact=False):
        if self.mode=='SIM':raise ValueError('SIM 모드는 가상 작업 버튼을 사용하세요.')
        if self.plan or self.busy:raise ValueError('FR5 작업/요청 처리 중')
        calibration(self.cal,contact);socket=socket_frame(self.cal,record)
        f=self.feedback
        if not self.client.connected or time.monotonic()-self.rx>.5 or f.get('telemetry_verified') is not True:raise ValueError('최신 실제 FR5 상태 필요')
        if f.get('status') not in ('IDLE','COMPLETED') or f.get('motion_done')!=1:raise ValueError('실제 로봇 정지·정상 상태 필요')
        if f.get('tool')!=self.cal['tool'] or f.get('user')!=0:raise ValueError('실제 열쇠 TCP/user 불일치')
        self.twin.fit(f);self.twin.socket=socket
        self.plan=RealKeyPlan(self.cal,contact);self.last_record=record
        self.report('동일 실측 목표로 '+self.mode+' 작업 시작',dict(contact=contact), '실측 피드백 기반 제한된 명령 생성')

    def stop(self,reason='통합 작업 정지',force=False):
        if not self.plan and not force:return
        evidence=dict(feedback=self.feedback,comparison=self.twin.comparison(self.feedback),decision=self.plan.evidence if self.plan else None)
        self.plan=None;self.epoch+=1;self.rx=0.;self.busy=True;self.pending_kind='stop';token=self.epoch
        self.report(reason,evidence,'독립 StopMotion · SIM 예측 진행 중단')
        def run():
            try:self.events.put((token,'stop',True,self.client.stop()))
            except Exception as exc:self.events.put((token,'stop',False,'정지 확인 실패: '+str(exc)))
        threading.Thread(target=run,daemon=False,name='key-twin-stop').start()

    def disconnect(self):
        if self.plan:self.stop('통합 연결 해제');return
        if self.busy:raise ValueError('FR5 요청 완료 후 연결을 해제하세요.')
        self.epoch+=1;self.client.close();self.feedback={};self.rx=0.;self.twin.fitted=False;self.report('FR5 연결 해제')

    def advance(self,record):
        f=self.feedback
        if f.get('status')=='ERROR' or f.get('tool')!=self.cal['tool'] or f.get('user')!=0:raise ValueError('실제 로봇 오류/TCP 변경')
        command=self.plan.step(record,f)
        self.twin.socket=socket_frame(self.cal,record)
        if command:
            if self.mode=='SIM+REAL':self.twin.predict(command['target'])
            # Expiry is renewed only after revalidating the original camera frame.
            socket_frame(self.cal,record);command['expires']=time.monotonic()+.25
            command['camera_timestamp']=record['timestamp']
            self.submit('step',lambda cmd=command:self.client.call(self.config,'panel_step',cmd))
        self.report(self.mode+' · '+self.plan.stage,dict(decision=self.plan.evidence,comparison=self.twin.comparison(f)),
            '동일 TCP 목표의 SIM 예측 후 FR5 전송' if self.mode=='SIM+REAL' else '실측 TCP 목표의 FR5 전송')

    def tick(self,record):
        self.last_record=record;now=time.monotonic()
        while not self.events.empty():
            token,kind,ok,result=self.events.get_nowait()
            if token!=self.epoch:continue
            self.busy=False
            if not ok:
                if self.plan:self.stop(str(result))
                else:self.report(str(result))
            elif kind in ('connect','poll'):
                self.feedback=result;self.rx=now
                if kind=='connect':self.twin.fit(result);self.report('실제 관절 수신 · SIM 비교 기준 설정',result)
                if self.plan:
                    try:self.advance(record)
                    except Exception as exc:self.stop(str(exc))
            elif kind=='stop':self.report('FR5 정지 응답 확인 · 재연결 필요',result)
        if self.plan:
            try:
                socket_frame(self.cal,record)
                if now-self.rx>.6:raise ValueError('실제 피드백 600 ms 지연')
            except Exception as exc:self.stop(str(exc))
        if self.client.connected and not self.busy and now-self.poll_at>=.2:
            self.poll_at=now;self.submit('poll',lambda:self.client.call(self.config,'panel_status'))
