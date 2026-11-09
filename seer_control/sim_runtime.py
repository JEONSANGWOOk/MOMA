"""Offline counterparts of Runtime main2/main3 and departure orchestration.

Only local ArmSimulator and Simulator are used; no HTTP/socket/hardware client.
"""
import copy
import math
import queue
import threading
import time
from concurrent.futures import Future
from types import SimpleNamespace
from .departure_link import DepartureState


def sim_programs(config):
    config=copy.deepcopy(config);operations=config.setdefault('operations',{});programs=config.setdefault('programs',{})
    defaults={'main2':('safe_pose','demo_a'),'main3':('demo_b','safe_pose')}
    for name,sequence in defaults.items():
        if name in programs:continue
        missing=[op for op in sequence if op not in operations]
        if missing:raise ValueError('SIM '+name+' 프로그램을 로봇팔 개발 탭에 등록하세요. 기본 예제 작업 누락: '+','.join(missing))
        programs[name]=[dict(type='Arm Action',operation=op,timeout_s=60.) for op in sequence]
    return config


class SimScenario:
    def __init__(self,app):self.app=app;self.future=None;self.uncertain=False
    def start(self,url='',scenario='main2'):
        if scenario not in ('main1','main2','main3'):raise ValueError('main1/main2/main3만 지원합니다.')
        app=self.app
        if app.real:raise ValueError('SIM 실행 중 실기 모드로 바뀌었습니다.')
        if self.future and not self.future.done():raise ValueError('SIM 시나리오가 실행 중입니다.')
        arm=app.arm_dev_sim
        if arm.state in ('RUNNING','PAUSED'):raise ValueError('기존 SIM 팔 작업을 먼저 종료하세요.')
        # Recorded replay is the default. Local example programs require an
        # explicit opt-in and are never presented as the real scenario.
        if getattr(app,'sim_scenario_source','recording')=='recording':
            from .offline_scene import start_replay
            self.future=start_replay(app,scenario)
            return self.future
        config=sim_programs(app.arm_dev_config)
        app.arm_dev_config=config;app.arm_sim_owner='workspace'
        app.world3d.sim_io7_feedback=False
        app.world3d.sim_contact_follow=True
        app.world3d.cad_doors_auto.set(False)
        for key in ('arm_follow','real_doors_follow'):
            var=getattr(app.world3d,key,None)
            if var is not None:var.set(True)
        combo=getattr(app,'arm_dev_program_combo',None)
        if combo is not None:combo.configure(values=list(config['programs']))
        for channel in (2,3,4,5):arm.io['DO'].setdefault(channel,0)
        future=Future();self.future=future
        arm.start(config,program=scenario)
        def poll():
            if future.done():return
            if app.real or app.arm_dev_sim is not arm:
                arm.stop();future.set_exception(ValueError('모드/팔 모델 변경으로 SIM 작업 취소'));return
            if arm.state in ('ERROR','FAILED','CANCELED'):
                future.set_exception(ValueError(arm.error or 'SIM 팔 작업 취소'));return
            if arm.state=='COMPLETED':
                future.set_result({'status':'success','source':'SIM','scenario':scenario});return
            app.after(50,poll)
        app.after(50,poll)
        return future


class SimDepartureLink:
    def __init__(self,app,scenario):
        self.app=app;self.sim=app.sim;self.scenario=scenario;self.events=queue.Queue()
        self.stop=threading.Event();self.active=False;self.thread=SimpleNamespace(is_alive=lambda:self.active)
        self.started=None;self.fsm=DepartureState();self.arm_future=None;self.name=None;self.pending=False;self.due=None
    def start(self):
        app=self.app
        if app.real:raise ValueError('SIM 모드가 필요합니다.')
        for node in ('LM1','LM2'):
            if node not in app.map.nodes:raise ValueError('SIM 지도에 '+node+' 노드가 필요합니다.')
        s=self.sim.state;n=app.map.nodes['LM2']
        if self.sim.route or abs(s.speed)>.01 or math.hypot(s.x-n['x'],s.y-n['y'])>.18:raise ValueError('SIM 로봇을 LM2에 정지시킨 뒤 실행하세요.')
        if getattr(app,'sim_scenario_source','recording')=='recording':
            from .offline_scene import archive_for
            for name in ('main2','main3'):archive_for(app).load(name)
        else:sim_programs(app.arm_dev_config)
        self.active=True;self.started=time.monotonic()
        app.target.set('LM1');app.navigate()
        if not self.sim.route:self.active=False;raise ValueError('SIM LM1 이동 요청이 시작되지 않았습니다.')
        self.events.put(('status','SIM LM1 이동 시작 · main2 속도 트리거 대기'))
        app.after(50,self.tick)
    def tick(self):
        if not self.active:return
        app=self.app
        try:
            if self.stop.is_set() or app.real or app.sim is not self.sim:raise ValueError('SIM 연동 해제/모드 변경 · 후속 실행 취소')
            if time.monotonic()-self.started>300:raise TimeoutError('SIM 연동 시간 초과')
            if self.arm_future and self.arm_future.done() and self.arm_future.exception():raise ValueError(str(self.arm_future.exception()))
            if self.name=='main3' and self.arm_future.done():self.finish('SIM main3 완료');return
            s=self.sim.state
            if s.stopped:raise ValueError('SIM 비상정지 · 후속 실행 차단')
            status=2 if self.sim.route else 4 if s.last_node=='LM1' else 6
            target=s.target if self.sim.route else 'LM1'
            for action in self.fsm.update(s.last_node,s.speed,target,status,time.monotonic()):
                if action=='main2':self.arm_future=self.scenario.start('',action);self.name=action;self.events.put(('status','SIM 속도 상승 → main2'))
                elif action=='main3':self.pending=True
            if self.pending and self.name=='main2' and self.arm_future.done():
                if self.due is None:self.due=time.monotonic()+2.;self.events.put(('status','SIM 도착 · main3 전 2초 대기'))
                if time.monotonic()>=self.due:
                    self.arm_future=self.scenario.start('','main3');self.name='main3';self.pending=False
                    self.events.put(('status','SIM main3 실행'))
            app.after(50,self.tick)
        except Exception as error:self.finish(str(error),True)
    def finish(self,message,error=False):
        self.active=False;self.events.put(('error' if error else 'status',message))
