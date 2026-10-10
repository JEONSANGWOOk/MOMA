"""FAIRINO SDK adapter. No implicit enable, mode change, homing or motion."""
import importlib
import ipaddress
import math
from pathlib import Path
import sys
import time


def profile():
    return dict(driver='fairino', verified=False, ip='192.168.58.2', sdk_path='',
                controller_version='', version_confirmed=False, operations={}, programs={})


def vector(value, name):
    if not isinstance(value, (list, tuple)) or len(value)!=6:
        raise ValueError(name+'에는 숫자 6개가 필요합니다.')
    if any(isinstance(v,bool) for v in value):raise ValueError(name+' 숫자 오류')
    result=[float(v) for v in value]
    if not all(math.isfinite(v) for v in result):raise ValueError(name+' 유한 숫자 오류')
    return result


def validate(config, motion=False):
    ipaddress.IPv4Address(config.get('ip',''))
    if motion and (config.get('verified') is not True or config.get('version_confirmed') is not True
                   or not config.get('controller_version','').strip()):
        raise ValueError('FR5 제어기 버전과 SDK 호환 확인 후 실기 동작을 허용하세요.')
    operations=config.get('operations',{})
    if not isinstance(operations,dict):raise ValueError('operations는 JSON 객체입니다.')
    for name,spec in operations.items():
        if not isinstance(name,str) or not name.strip() or not isinstance(spec,dict):raise ValueError('작업 이름/설정 오류')
        method=spec.get('method')
        if method in ('SetDO','SetToolDO','WaitDI','WaitToolDI'):
            upper=1 if 'Tool' in method else 15
            if type(spec.get('id')) is not int or not 0<=spec['id']<=upper:raise ValueError(name+': I/O 채널 범위 오류')
            if type(spec.get('status')) is not int or spec['status'] not in (0,1):raise ValueError(name+': status는 0 또는 1')
            continue
        if method not in ('MoveJ','MoveL'):
            raise ValueError(name+': 지원하지 않는 FR5 API 작업입니다.')
        vector(spec.get('target'),name+'.target')
        for key in ('tool','user'):
            if type(spec.get(key,0)) is not int or not 0<=spec.get(key,0)<=14:
                raise ValueError(key+': 0~14 정수')
        speed=float(spec.get('vel',10))
        if not math.isfinite(speed) or not 0<speed<=100:raise ValueError('vel: 0 초과 ~100%')
    from .fairino_programs import validate_programs
    validate_programs(config)
    return config


def checked(result, method, data=False):
    if data:
        if not isinstance(result,(list,tuple)) or len(result)<2 or result[0]!=0:
            raise RuntimeError(f'{method} 실패: {result}')
        return result[1] if len(result)==2 else list(result[1:])
    if type(result) is not int or result!=0:raise RuntimeError(f'{method} 실패: {result}')
    return result


def load_sdk(folder):
    if folder:
        path=Path(folder).expanduser().resolve()
        # Select the folder containing fairino, or the fairino folder itself.
        if path.name=='fairino':path=path.parent
        if not (path/'fairino'/'Robot.py').is_file():
            raise ValueError('SDK의 windows 폴더 또는 fairino 폴더를 선택하세요.')
        sys.path.insert(0,str(path))
    try:return importlib.import_module('fairino.Robot')
    except ImportError as e:
        raise RuntimeError('공식 FAIRINO SDK와 해당 SDK의 의존성을 설치하거나 SDK 폴더를 지정하세요: '+str(e)) from e


class SDKEngine:
    """Lives only in an isolated subprocess: some SDKs retry network calls forever."""
    def __init__(self,config,robot=None):
        self.config=validate(config);self.robot=robot or load_sdk(config.get('sdk_path','')).RPC(config['ip'])
        self.target=None;self.operation='';self.command_time=0.;self.canceled=False;self.io_task=None;self.paused=False

    def status(self):
        r=self.robot
        comm=checked(r.GetSDKComState(),'GetSDKComState',True)
        if comm!=0:raise RuntimeError('FR5 SDK 통신 오류')
        joints=vector(checked(r.GetActualJointPosDegree(),'GetActualJointPosDegree',True),'관절')
        tcp=vector(checked(r.GetActualTCPPose(),'GetActualTCPPose',True),'TCP')
        done=checked(r.GetRobotMotionDone(),'GetRobotMotionDone',True)
        emergency=checked(r.GetRobotEmergencyStopState(),'GetRobotEmergencyStopState',True)
        safety=checked(r.GetSafetyStopState(),'GetSafetyStopState',True)
        errors=checked(r.GetRobotErrorCode(),'GetRobotErrorCode',True)
        if done not in (0,1) or emergency not in (0,1) or not isinstance(safety,(list,tuple)) or len(safety)!=2:
            raise RuntimeError('FR5 상태 응답 형식 오류')
        failed=bool(emergency or any(safety) or any(errors))
        state='ERROR' if failed else 'CANCELED' if self.canceled else 'IDLE'
        if self.target and not failed and not self.canceled:
            method,target=self.target
            actual=joints if method=='MoveJ' else tcp
            tolerances=[.5]*6 if method=='MoveJ' else [1.,1.,1.,.5,.5,.5]
            at_goal=all(abs(a-b)<=tol for a,b,tol in zip(actual,target,tolerances))
            state='COMPLETED' if done==1 and at_goal and time.monotonic()-self.command_time>=.5 else 'RUNNING'
        io_value=None
        if self.io_task and not failed and not self.canceled:
            spec=self.io_task
            if spec['method'].startswith('Wait'):
                reader='GetToolDI' if spec['method']=='WaitToolDI' else 'GetDI'
                io_value=checked(getattr(r,reader)(spec['id'],block=1),reader,True)
                if type(io_value) is not int or io_value not in (0,1):raise RuntimeError('FR5 DI 응답 형식 오류')
                state='COMPLETED' if io_value==spec['status'] else 'RUNNING'
            else:state='COMPLETED'
        if self.paused and state not in ('ERROR','CANCELED'):state='PAUSED'
        return dict(driver='fairino',status=state,io_value=io_value,operation=self.operation,joints_deg=joints,
                    joints_rad=[math.radians(v) for v in joints],tcp_mm_deg=tcp,
                    motion_done=done,emergency=bool(emergency),safety_stop=list(safety),errors=list(errors))

    def execute(self,operation,expires=None):
        validate(self.config,True)
        if operation not in self.config['operations']:raise ValueError('등록되지 않은 FR5 작업: '+str(operation))
        state=self.status()
        if state['status'] in ('ERROR','RUNNING','PAUSED') or state['motion_done']!=1:
            raise ValueError('FR5 정지 및 오류 해제를 먼저 확인하세요.')
        spec=self.config['operations'][operation];method=spec['method']
        if expires is not None:
            if method not in ('SetDO','SetToolDO') or not 0<=float(expires)-time.monotonic()<=.25:raise ValueError('그리퍼 입력 유효시간/작업 오류')
        if method in ('SetDO','SetToolDO','WaitDI','WaitToolDI'):
            # DO acceptance is not a grip confirmation: add a WaitDI step for the sensor.
            if method.startswith('Set'):
                checked(getattr(self.robot,method)(spec['id'],spec['status'],smooth=0,block=1),method)
            self.target=None;self.io_task=dict(spec);self.operation=operation;self.canceled=False;self.paused=False
            return dict(accepted=True,operation=operation)
        target=vector(spec['target'],'target')
        kwargs=dict(tool=spec.get('tool',0),user=spec.get('user',0),vel=float(spec.get('vel',10)),ovl=100.)
        # Nonblocking moves permit StopMotion; application waits for measured arrival.
        kwargs['blendT' if method=='MoveJ' else 'blendR']=0.
        checked(getattr(self.robot,method)(target,**kwargs),method)
        self.io_task=None;self.paused=False;self.target=(method,target);self.operation=operation;self.command_time=time.monotonic();self.canceled=False
        return dict(accepted=True,operation=operation)

    def jog(self,spec):
        validate(self.config,True)
        if not isinstance(spec,dict):raise ValueError('JOG 설정 오류')
        ref=spec.get('ref');axis=spec.get('axis');direction=spec.get('direction')
        if type(ref) is not int or ref not in (0,2) or type(axis) is not int or not 1<=axis<=6 or type(direction) is not int or direction not in (0,1):raise ValueError('JOG 축 오류')
        distance=float(spec.get('distance',0));vel=float(spec.get('vel',0));expires=float(spec.get('expires',0))
        maximum=.3 if ref==2 and axis<=3 else .2
        if not all(math.isfinite(x) for x in (distance,vel,expires)) or not 0<distance<=maximum or not 0<vel<=5:raise ValueError('JOG 속도/거리 범위 오류')
        if not 0<=expires-time.monotonic()<=.25:raise ValueError('JOG 입력 유효시간 초과')
        state=self.status()
        if state['status'] in ('ERROR','RUNNING','PAUSED') or state['motion_done']!=1:raise ValueError('FR5 정지 및 안전 상태 확인')
        # State polling can take time: recheck the lease immediately before motion.
        if time.monotonic()>expires:raise ValueError('JOG 입력 유효시간 초과')
        self.target=None;self.io_task=None;self.paused=False;self.canceled=False
        try:
            checked(self.robot.StartJOG(ref,axis,direction,distance,vel=vel,acc=20.),'StartJOG')
            time.sleep(.08)
        finally:checked(self.robot.ImmStopJOG(),'ImmStopJOG')
        return self.status()

    def panel_status(self):
        state=self.status()
        counter=getattr(getattr(self.robot,'robot_state_pkg',None),'frame_cnt',None)
        previous=getattr(self,'_panel_frame',None);now=time.monotonic()
        if type(counter) is int and 0<=counter<=255:
            if previous is None:self._panel_frame=(counter,now);self._panel_live=False
            elif counter!=previous[0]:self._panel_frame=(counter,now);self._panel_live=True
            elif now-previous[1]>.6:raise RuntimeError('FR5 실시간 상태 패킷 600 ms 정체')
        else:self._panel_live=False
        state['telemetry_verified']=self._panel_live;state['frame_counter']=counter
        if self.target and self.operation.startswith('key_panel_') and state['status'] not in ('ERROR','PAUSED','CANCELED'):
            from .key_panel_real import pose_matrix
            from .arm_simulation import rotation_error
            from .visual_servo import norm
            target=self.target[1];actual=state['tcp_mm_deg']
            arrived=math.dist(target[:3],actual[:3])<=.05 and math.degrees(norm(rotation_error(pose_matrix(target),pose_matrix(actual))))<=.05
            state['status']='COMPLETED' if arrived and state['motion_done']==1 and time.monotonic()-self.command_time>=.15 else 'RUNNING'
        state['tool']=checked(self.robot.GetActualTCPNum(),'GetActualTCPNum',True)
        state['user']=checked(self.robot.GetActualWObjNum(),'GetActualWObjNum',True)
        try:state['force_torque']=vector(checked(self.robot.FT_GetForceTorqueRCS(),'FT_GetForceTorqueRCS',True),'力/力矩')
        except (AttributeError,ValueError,RuntimeError):state['force_torque']=None
        return state

    def panel_step(self,spec):
        from .key_panel_real import calibration,pose_matrix,in_workspace,rigid
        from .aruco_board import apply,transpose
        from .arm_simulation import rotation_error
        from .visual_servo import norm
        validate(self.config,True)
        stage=spec.get('stage')
        if stage not in ('ALIGN','APPROACH','INSERT','TURN','RETRACT','HOLD'):raise ValueError('실기 단계 오류')
        c=calibration(spec['calibration'],stage in ('INSERT','TURN','RETRACT') or spec['calibration'].get('contact_verified') is True)
        target=vector(spec['target'],'실기 열쇠 목표');expires=float(spec['expires'])
        if not math.isfinite(expires) or not 0<=expires-time.monotonic()<=.25:raise ValueError('실기 명령 250 ms 유효시간 초과')
        state=self.panel_status()
        if not state['telemetry_verified']:raise ValueError('FR5 실제 상태 패킷 갱신 확인 필요')
        if state['motion_done']!=1 or state['status'] not in ('IDLE','COMPLETED'):raise ValueError('FR5 정지·안전 상태 미충족')
        if state['tool']!=c['tool'] or state['user']!=0:raise ValueError('실제 열쇠 TCP/베이스 좌표 번호 불일치')
        current=state['tcp_mm_deg']
        socket=rigid(spec['socket_base_mm'],'슬롯 실기 좌표')
        local=apply(transpose([row[:3] for row in socket[:3]]),[target[i]-socket[i][3] for i in range(3)])
        if stage in ('ALIGN','APPROACH') and -local[2]>-4.9:raise ValueError('공중 정렬·접근은 슬롯 5 mm 앞에서 제한됩니다.')
        if -local[2]>c['insert_mm']+.2:raise ValueError('실측 삽입 깊이 초과')
        if stage in ('TURN','RETRACT'):
            actual_local=apply(transpose([row[:3] for row in socket[:3]]),[current[i]-socket[i][3] for i in range(3)])
            axis=pose_matrix(current)
            tilt=math.degrees(math.acos(max(-1,min(1,sum(axis[i][2]*(-socket[i][2]) for i in range(3))))))
            if stage=='TURN' and (-actual_local[2]<c['insert_mm']-.2 or norm(actual_local[:2])>.2 or tilt>.5):raise ValueError('실제 삽입·정렬 확인 없이 회전 불가')
            if stage=='RETRACT' and (-local[2]>-actual_local[2]+.02 or (-actual_local[2]>0 and (norm(actual_local[:2])>.2 or norm(local[:2])>.2 or tilt>.5))):raise ValueError('열쇠 후퇴 방향·접촉 정렬 조건 미충족')
        if math.dist(current[:3],target[:3])>.501 or math.degrees(norm(rotation_error(pose_matrix(target),pose_matrix(current))))>.251:raise ValueError('실기 단일 명령 0.5 mm / 0.25° 초과')
        if not in_workspace(c,current[:3]) or not in_workspace(c,target[:3]):raise ValueError('실기 작업 범위 초과')
        if stage in ('INSERT','TURN','RETRACT') or c.get('contact_verified') is True:
            f=vector(state.get('force_torque'),'힘/토크 센서')
            if norm(f[:3])>=c['force_limit_n'] or norm(f[3:])>=c['torque_limit_nm']:raise ValueError('힘/토크 보호 임계값 초과')
        if time.monotonic()>expires:raise ValueError('실기 명령 유효시간 초과')
        camera_stamp=spec.get('camera_timestamp')
        if camera_stamp is not None:
            if type(camera_stamp) not in (int,float) or not math.isfinite(camera_stamp) or not -.1<=time.time()-camera_stamp<=.25:raise ValueError('전송 직전 D455 영상 250 ms 유효시간 초과')
        checked(self.robot.MoveL(target,tool=c['tool'],user=0,vel=1.,acc=10.,ovl=10.,blendR=0.),'MoveL')
        self.target=('MoveL',target);self.operation='key_panel_'+stage;self.command_time=time.monotonic()
        self.canceled=False;self.paused=False
        return dict(accepted=True,operation=self.operation,target=target)

    def call(self,kind,operation=None):
        if kind=='panel_status':return self.panel_status()
        if kind=='panel_step':return self.panel_step(operation)
        if kind=='status':return self.status()
        if kind=='execute':return self.execute(operation)
        if kind=='jog':return self.jog(operation)
        if kind=='pad_io':
            if not isinstance(operation,dict):raise ValueError('그리퍼 입력 오류')
            return self.execute(operation.get('operation'),operation.get('expires',0))
        if kind in ('pause','resume','stop'):
            checked(getattr(self.robot,{'pause':'PauseMotion','resume':'ResumeMotion','stop':'StopMotion'}[kind])(),kind)
            if kind=='stop':self.canceled=True
            if kind=='pause':self.paused=True
            if kind=='resume':self.paused=False
            return dict(status=kind.upper())
        raise ValueError('지원되지 않는 FR5 요청')
