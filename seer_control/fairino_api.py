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

    def execute(self,operation):
        validate(self.config,True)
        if operation not in self.config['operations']:raise ValueError('등록되지 않은 FR5 작업: '+str(operation))
        state=self.status()
        if state['status'] in ('ERROR','RUNNING','PAUSED') or state['motion_done']!=1:
            raise ValueError('FR5 정지 및 오류 해제를 먼저 확인하세요.')
        spec=self.config['operations'][operation];method=spec['method']
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

    def call(self,kind,operation=None):
        if kind=='status':return self.status()
        if kind=='execute':return self.execute(operation)
        if kind=='jog':return self.jog(operation)
        if kind in ('pause','resume','stop'):
            checked(getattr(self.robot,{'pause':'PauseMotion','resume':'ResumeMotion','stop':'StopMotion'}[kind])(),kind)
            if kind=='stop':self.canceled=True
            if kind=='pause':self.paused=True
            if kind=='resume':self.paused=False
            return dict(status=kind.upper())
        raise ValueError('지원되지 않는 FR5 요청')
