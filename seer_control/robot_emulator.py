"""LAN SEER-compatible virtual AMR. Owns simulated motion, never contacts hardware."""
import copy
import json
import math
import socket
import socketserver
import threading
import time
from pathlib import Path

from .decision_log import DecisionJournal, decide
from .model import MapModel, Simulator, number
from .smap import build_smap_from_model, load_smap_data, make_path_record
from .transport import HEADER, MAX_PAYLOAD, receive_exact

PORT_APIS={19204:{1000,1002,1007,1009,1020,1021,1022,1100,1101,1300,1301},
           19205:{2000,2002,2003,2004,2010,2022},
           19206:{3001,3002,3003,3050,3051},19207:{4000,4005,4006,4010,4011}}


def prepare_map(model,name='emulator_demo'):
    model=copy.deepcopy(model)
    if not model.nodes:raise ValueError('가상 로봇 지도에는 노드가 필요합니다.')
    if not hasattr(model,'smap_source'):
        cloud=list(model.cloud)
        for x1,y1,x2,y2 in model.walls:
            count=max(1,math.ceil(math.hypot(x2-x1,y2-y1)/.1))
            cloud.extend((x1+(x2-x1)*i/count,y1+(y2-y1)*i/count) for i in range(count+1))
        model.smap_source=dict(header=dict(mapName=name),normalPosList=[dict(x=x,y=y) for x,y in cloud],advancedPointList=[],advancedCurveList=[])
    else:model.smap_source['header']['mapName']=name
    if not getattr(model,'path_records',[]):
        model.path_records=[dict(a=src,b=dst,raw=make_path_record(model.nodes[src],model.nodes[dst]),properties={})
                            for a,b in model.edges for src,dst in ((a,b),(b,a))]
    model.name=name
    return model


class VirtualRobot:
    def __init__(self,model,log_dir=None):
        self.lock=threading.RLock()
        self.decision_journal=DecisionJournal(log_dir)
        self.decision_journal.bind('EMULATOR')
        self.map=prepare_map(model)
        self.sim=Simulator(self.map,self.decision_journal)
        self.mode=1;self.task_status=0;self.control_owner='';self.emergency=False
        self.manual_block=False;self.forced_failure='';self.localization=.99
        self.offline_until=0.;self.delay_s=0.;self.reloc_status=1
        self.last_error='';self._block_point=None;self._last_task=None

    def _decision(self,source,evidence,conclusion,action):
        decide(self,'가상 로봇.'+source,self.status(scan=False),evidence,conclusion,action,force=True)

    def step(self,dt):
        with self.lock:
            if self.emergency or self.manual_block or self.task_status==3:
                self.sim.state.speed=0.;self.sim._velocity=0.
                self.sim.state.blocked=self.manual_block
            else:
                self.sim.tick(dt)
                if self.task_status==2 and not self.sim.route:
                    if self.sim.state.task=='완료':self.task_status=4
                    elif self.sim.skip_result:self.task_status=5;self.forced_failure=self.sim.skip_result['reason']
            if self.task_status!=self._last_task:
                self._last_task=self.task_status
                self._decision('주행 상태',dict(task_status=self.task_status,reason=self.forced_failure),'주행 상태 변경','상태 조회에 현재 결과 반환')

    def status(self,scan=True):
        with self.lock:
            s=self.sim.state
            points=self.sim.navigation_points() if self.sim.route else []
            blocked=bool(s.blocked or self.manual_block)
            point=self._block_point
            if blocked and self.map.obstacles:
                obstacle=min(self.map.obstacles,key=lambda o:math.hypot(o['x']-s.x,o['y']-s.y))
                point=(obstacle['x'],obstacle['y'])
            result=dict(ret_code=0,x=s.x,y=s.y,angle=s.theta,vx=s.speed,vy=0.,w=self.sim.w,
                        confidence=self.localization,battery_level=s.battery/100,charging=s.charging,
                        blocked=blocked,emergency=self.emergency,stopped=self.emergency,motor=s.motor,
                        mode=self.mode,task_status=self.task_status,target_id=s.target,last_station=s.last_node,
                        current_map=self.map.name,path=points,unfinished_path=points,
                        reloc_status=self.reloc_status,loadmap_status=1,slam_status=0,
                        control_owner=self.control_owner,robot_model='LAN Virtual AMR',
                        robokit_version='emulator-1.0',is_emulator=True,
                        errors=([self.forced_failure] if self.forced_failure else []),warnings=[],fatals=[])
            if scan:result['laser_beams']=self.sim.scan()
            if blocked and point:result.update(block_x=point[0],block_y=point[1])
            return result

    def stations(self):
        return [dict(id=k,x=n['x'],y=n['y'],r=n.get('r',n.get('angle',0.)),
                     spin=n.get('spin',True),type='ChargePoint' if n.get('kind')=='dock' else 'LandMark')
                for k,n in self.map.nodes.items()]

    def request(self,api,payload=None,port=None):
        with self.lock:
            try:
                if port is not None and api not in PORT_APIS[port]:raise ValueError('이 포트에서 지원하지 않는 API')
                if api not in set().union(*PORT_APIS.values()):raise ValueError('지원하지 않는 API')
                p=payload if payload is not None else {}
                if not isinstance(p,dict):raise ValueError('요청 본문은 JSON object입니다.')
                if api in (1100,1002,1007,1020):return self.status()
                if api==1000:return dict(ret_code=0,robot_model='LAN Virtual AMR',robot_id='VIRTUAL-AMR-01',robokit_version='emulator-1.0',is_emulator=True)
                if api in (1009,1101):return dict(ret_code=0,laser_beams=self.sim.scan(),is_emulator=True)
                if api==1021:return dict(ret_code=0,reloc_status=self.reloc_status,confidence=self.localization)
                if api==1022:return dict(ret_code=0,loadmap_status=1)
                if api==1300:return dict(ret_code=0,current_map=self.map.name,maps=[self.map.name])
                if api==1301:return dict(ret_code=0,stations=self.stations())
                if api==4011:
                    if p.get('map_name')!=self.map.name:raise ValueError('등록되지 않은 지도 이름')
                    return build_smap_from_model(self.map)
                if api==4005:
                    nickname=p.get('nick_name')
                    if not isinstance(nickname,str) or not nickname.strip():raise ValueError('nick_name이 필요합니다.')
                    if self.control_owner and self.control_owner!=nickname:raise ValueError('다른 제어자가 제어권을 사용 중입니다.')
                    self.control_owner=nickname
                elif api==4006:
                    if not self.control_owner or p.get('nick_name')!=self.control_owner:raise ValueError('현재 제어권 소유자만 해제할 수 있습니다.')
                    self.sim.stop();self.task_status=6;self.control_owner=''
                else:
                    if not self.control_owner:raise ValueError('4005로 제어권을 먼저 획득하세요.')
                    if self.emergency and api not in (2000,3003):raise ValueError('가상 비상정지를 해제하세요.')
                    if api in (3050,3051):
                        if self.mode!=1:raise ValueError('자동 모드에서만 목적지 주행 가능합니다.')
                        if self.task_status in (2,3):raise ValueError('기존 작업을 취소한 뒤 새 목적지를 보내세요.')
                        goal=p.get('id')
                        if goal not in self.map.nodes:raise ValueError('3050/3051은 등록된 Station ID만 지원합니다.')
                        if self.manual_block:raise ValueError('강제 BLOCKED 상태를 해제하세요.')
                        self.sim.state.blocked=False;self.sim._collision_blocked=False
                        self.sim.obstacle_policy='auto' if api==3050 else 'wait'
                        self.sim.auto_wait_s=0.;self.sim.auto_static_s=0.
                        self.sim.prefer_line_rejoin=False;self.sim.blocked_recovery.enabled=api==3050
                        self.sim.navigate(goal);self.task_status=2;self.forced_failure=''
                    elif api==3001:
                        if self.task_status!=2:raise ValueError('실행 중인 주행이 없습니다.')
                        self.sim.command('pause');self.task_status=3
                    elif api==3002:
                        if self.task_status!=3:raise ValueError('일시정지된 주행이 없습니다.')
                        self.sim.command('resume');self.task_status=2
                    elif api in (3003,2000):self.sim.stop();self.task_status=6
                    elif api==2010:
                        if self.mode!=0:raise ValueError('수동 모드에서만 속도 제어 가능합니다.')
                        if self.task_status in (2,3):raise ValueError('주행 Task를 취소하세요.')
                        vx=number(p.get('vx',0));vy=number(p.get('vy',0));w=number(p.get('w',0))
                        if abs(vx)>.3 or abs(w)>.6 or abs(vy)>1e-9:raise ValueError('차동 주행 제한: vx ±0.3, w ±0.6, vy=0')
                        if self.manual_block:raise ValueError('강제 BLOCKED 상태입니다.')
                        self.sim.drive(vx,w);self.task_status=0
                    elif api==4000:
                        if type(p.get('mode')) is not int or p['mode'] not in (0,1):raise ValueError('mode는 0 또는 1입니다.')
                        if self.task_status in (2,3):raise ValueError('모드 변경 전에 주행을 취소하세요.')
                        self.mode=p['mode']
                    elif api==2002:
                        if self.task_status in (2,3):raise ValueError('재위치 지정 전에 주행을 취소하세요.')
                        x=number(p.get('x'));y=number(p.get('y'));angle=number(p.get('angle'))
                        self.sim.stop();self.sim.state.x=x;self.sim.state.y=y;self.sim.state.theta=angle
                        self.sim.state.last_node=self.map.nearest(x,y);self.reloc_status=1;self.task_status=0
                    elif api==2003:self.reloc_status=1
                    elif api==2004:self.reloc_status=0
                    elif api==2022:
                        if p.get('map_name')!=self.map.name:raise ValueError('등록되지 않은 지도')
                        if self.task_status in (2,3):raise ValueError('주행 중 지도 로드 금지')
                    elif api==4010:
                        if self.task_status in (2,3):raise ValueError('주행 중 지도 업로드 금지')
                        model=load_smap_data(p)
                        if not model.nodes:raise ValueError('지도에 노드가 없습니다.')
                        model.walls=copy.deepcopy(self.map.walls)
                        model=prepare_map(model,self.map.name)
                        old=self.sim.state
                        sim=Simulator(model,self.decision_journal);sim.state.x=old.x;sim.state.y=old.y;sim.state.theta=old.theta
                        sim.state.last_node=model.nearest(old.x,old.y)
                        self.map=model;self.sim=sim;self.task_status=0
                self._decision('API 수락',dict(api=api,payload=p),'검증 조건 통과','가상 로봇에 명령 적용')
                return dict(ret_code=0,is_emulator=True)
            except (ValueError,TypeError,KeyError) as error:
                self._decision('API 거부',dict(api=api,reason=str(error)),'요청 조건 불충족','이동 명령 거부')
                return dict(ret_code=9001,err_msg=str(error),is_emulator=True)

    def inject(self,kind,value=None):
        with self.lock:
            if kind=='blocked':
                self.manual_block=not self.manual_block
                self.sim.state.blocked=self.manual_block
                s=self.sim.state;self._block_point=(s.x+.7*math.cos(s.theta),s.y+.7*math.sin(s.theta))
            elif kind=='emergency':
                self.emergency=not self.emergency
                if self.emergency:self.sim.stop();self.task_status=6
                self.sim.state.stopped=self.emergency
            elif kind=='failure':self.sim.stop();self.task_status=5;self.forced_failure='주행 실패 시험 주입'
            elif kind=='localization':self.localization=number(value)
            elif kind=='offline':self.offline_until=time.monotonic()+number(value)
            elif kind=='delay':
                delay=number(value)
                if not 0<=delay<=10:raise ValueError('응답 지연은 0~10초')
                self.delay_s=delay
            elif kind=='obstacle':
                x,y=value;self.map.obstacles.append(dict(id='BOX'+str(time.time_ns()),x=number(x),y=number(y),radius=.35,map_fixed=False))
            elif kind=='person':
                x,y=value
                self.map.obstacles.append(dict(id='PERSON'+str(time.time_ns()),kind='person',dynamic=True,x=number(x),y=number(y),radius=.25,
                                               motion_path=[[x,y-1],[x,y+1]],speed_mps=.4,dwell_s=.5))
            elif kind=='clear_obstacles':self.map.obstacles.clear()
            else:raise ValueError('지원하지 않는 시험 시나리오')
            self._decision('시험 주입',dict(kind=kind,value=value),'시험 조건 변경','상태/장면에 시험 조건 적용')

    def close(self):self.decision_journal.close()


class _Server(socketserver.ThreadingTCPServer):
    allow_reuse_address=True
    daemon_threads=True


class _Handler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(5.)
        robot=self.server.robot
        try:
            while not self.server.group.stop_event.is_set():
                header=receive_exact(self.request,HEADER.size)
                magic,version,sequence,length,api,reserved=HEADER.unpack(header)
                if magic!=0x5A or version!=1 or length>MAX_PAYLOAD or reserved!=b'\0'*6:return
                body=receive_exact(self.request,length)
                with robot.lock:
                    offline=robot.offline_until>time.monotonic();delay=robot.delay_s
                if offline:return
                try:payload=json.loads(body.decode('utf-8')) if body else None
                except (UnicodeError,ValueError):response=dict(ret_code=9002,err_msg='JSON 형식 오류')
                else:response=robot.request(api,payload,self.server.api_port)
                if self.server.group.stop_event.wait(delay):return
                data=json.dumps(response,ensure_ascii=False,allow_nan=False,separators=(',',':')).encode('utf-8')
                response_api=api+10000 if api<=55535 else api
                self.request.sendall(HEADER.pack(0x5A,1,sequence,len(data),response_api,b'\0'*6)+data)
        except (OSError,ConnectionError):return


class EmulatorServer:
    def __init__(self,robot,host='127.0.0.1',ports=None):
        self.robot=robot;self.host=host;self.ports=ports or {p:p for p in PORT_APIS}
        self.servers=[];self.threads=[];self.stop_event=threading.Event();self.bound_ports={}

    def start(self,physics=True):
        try:
            for api_port,port in self.ports.items():
                server=_Server((self.host,port),_Handler)
                server.robot=self.robot;server.api_port=api_port;server.group=self
                self.servers.append(server);self.bound_ports[api_port]=server.server_address[1]
            for server in self.servers:
                thread=threading.Thread(target=server.serve_forever,kwargs=dict(poll_interval=.05),daemon=True)
                thread.start();self.threads.append(thread)
            if physics:
                thread=threading.Thread(target=self._physics,daemon=True);thread.start();self.threads.append(thread)
        except Exception:
            # If binding failed before serve_forever, shutdown would wait forever.
            if self.threads:self.close()
            else:
                for server in self.servers:server.server_close()
                self.servers.clear()
            raise
        return self

    def _physics(self):
        previous=time.monotonic()
        while not self.stop_event.wait(.02):
            now=time.monotonic();dt=min(.1,now-previous);previous=now
            try:self.robot.step(dt)
            except Exception as error:
                with self.robot.lock:
                    self.robot.last_error=str(error);self.robot.sim.stop();self.robot.task_status=5;self.robot.forced_failure=str(error)

    def close(self):
        self.stop_event.set()
        for server in self.servers:server.shutdown();server.server_close()
        for thread in self.threads:thread.join(timeout=1)
        self.servers.clear();self.threads.clear()


def run_window(robot,server):
    import tkinter as tk
    from tkinter import ttk,messagebox
    from .mission_preview import MissionMapPreview
    root=tk.Tk();root.title('LAN 가상 로봇 · 실제 장비 통신 없음');root.geometry('1050x720')
    try:addresses=', '.join(ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith('127.'))
    except OSError:addresses='ipconfig로 Ethernet IPv4 확인'
    status=tk.StringVar();ttk.Label(root,textvariable=status,font=('Malgun Gothic',11)).pack(fill='x',padx=12,pady=8)
    toolbar=ttk.Frame(root);toolbar.pack(fill='x',padx=12)
    def inject(kind,value=None):
        try:robot.inject(kind,value)
        except ValueError as error:messagebox.showerror('시험 조건',str(error),parent=root)
    for title,kind,value in [('BLOCKED 켜기/끄기','blocked',None),('비상정지 켜기/끄기','emergency',None),
                             ('주행 실패','failure',None),('통신 끊김 5초','offline',5),('장애물 전체 삭제','clear_obstacles',None)]:
        ttk.Button(toolbar,text=title,command=lambda k=kind,v=value:inject(k,v)).pack(side='left',padx=3)
    row=ttk.Frame(root);row.pack(fill='x',padx=12,pady=6)
    mode=tk.StringVar(value='장애물 추가');ttk.Combobox(row,textvariable=mode,values=['장애물 추가','사람 왕복 추가','장애물 삭제'],state='readonly',width=18).pack(side='left')
    delay=tk.StringVar(value='0');ttk.Label(row,text='응답 지연(초)').pack(side='left',padx=6);ttk.Entry(row,textvariable=delay,width=5).pack(side='left')
    def apply_delay():
        try:inject('delay',float(delay.get()))
        except ValueError:messagebox.showerror('시험 조건','응답 지연은 숫자입니다.',parent=root)
    ttk.Button(row,text='지연 적용',command=apply_delay).pack(side='left',padx=4)
    ttk.Button(row,text='위치 신뢰도 낮춤',command=lambda:inject('localization',.3)).pack(side='left',padx=4)
    ttk.Button(row,text='위치 신뢰도 복구',command=lambda:inject('localization',.99)).pack(side='left',padx=4)
    class View(MissionMapPreview):
        def click(self,event,add=False):
            if add:return
            scale,ox,oy=self.transform;x=(event.x-ox)/scale;y=(oy-event.y)/scale
            if mode.get()=='장애물 삭제':
                with robot.lock:
                    if robot.map.obstacles:
                        obstacle=min(robot.map.obstacles,key=lambda o:math.hypot(o['x']-x,o['y']-y))
                        if math.hypot(obstacle['x']-x,obstacle['y']-y)<.8:
                            robot.map.obstacles.remove(obstacle);robot._decision('시험 주입',dict(kind='remove_obstacle',id=obstacle.get('id')),'선택 장애물 삭제','다음 관측부터 해제 반영')
            else:inject('person' if mode.get()=='사람 왕복 추가' else 'obstacle',(x,y))
            self.redraw()
        def redraw(self):
            with robot.lock:
                self.model=robot.map
                super().redraw()
                points=robot.sim.navigation_points()
                if len(points)>1:self.create_line(*[v for point in points for v in self.xy(*point)],fill='#08a86b',width=3,arrow='last')
                for obstacle in robot.map.obstacles:
                    x,y=self.xy(obstacle['x'],obstacle['y']);r=max(4,obstacle['radius']*self.transform[0])
                    self.create_oval(x-r,y-r,x+r,y+r,fill='#f7c46a' if not obstacle.get('dynamic') else '#a580f5',outline='#86581b')
                s=robot.sim.state;x,y=self.xy(s.x,s.y)
                self.create_oval(x-9,y-9,x+9,y+9,fill='#287de0',outline='white')
                self.create_line(x,y,x+20*math.cos(s.theta),y-20*math.sin(s.theta),arrow='last',fill='#287de0',width=3)
    view=View(root,robot.map,lambda:[robot.sim.state.last_node]+list(robot.sim.route),lambda:'',
              lambda:robot.sim.status(),lambda key:None,lambda key:None,lambda a,b:True,height=500)
    view.pack(fill='both',expand=True,padx=10,pady=6)
    ttk.Label(root,text='다른 노트북 GUI에서 이 PC의 LAN IP로 실기 · 제어 연결하세요. 지도 클릭으로 시험 장애물을 배치합니다.').pack(fill='x',padx=12,pady=5)
    def refresh():
        state=robot.status(scan=False)
        status.set(f"가상 로봇 · 이 PC의 IP 후보: {addresses} · TCP 19204~19207\nTask {state['task_status']} · 목표 {state['target_id'] or '—'} · X {state['x']:.2f} / Y {state['y']:.2f} · 속도 {state['vx']:.2f}m/s · BLOCKED {state['blocked']} · 비상정지 {state['emergency']}\n제어자 {state['control_owner'] or '없음'} · 오류 {robot.last_error or robot.forced_failure or '없음'}")
        root.after(200,refresh)
    refresh();root.mainloop()


def main(argv=None):
    import argparse
    import sys
    parser=argparse.ArgumentParser(description='LAN virtual AMR; no physical hardware access.')
    parser.add_argument('--host',default='0.0.0.0',help='Bind address; default accepts LAN connections')
    parser.add_argument('--map',type=Path,default=Path(__file__).resolve().parents[1]/'maps/demo.json')
    writable_root=Path(sys.executable).resolve().parent if getattr(sys,'frozen',False) else Path(__file__).resolve().parents[1]
    parser.add_argument('--log-dir',type=Path,default=writable_root/'emulator_logs')
    parser.add_argument('--headless',action='store_true')
    args=parser.parse_args(argv)
    model=load_smap_data(json.loads(args.map.read_text(encoding='utf-8-sig'))) if args.map.suffix.lower()=='.smap' else MapModel.load(args.map)
    robot=VirtualRobot(model,args.log_dir);server=EmulatorServer(robot,args.host)
    try:
        server.start()
        print('LAN 가상 로봇 시작 · 실제 장비 연결 없음',flush=True)
        print('TCP:',server.bound_ports,'로그:',args.log_dir,flush=True)
        if args.headless:
            while not server.stop_event.wait(1):pass
        else:run_window(robot,server)
    except KeyboardInterrupt:pass
    finally:server.close();robot.close()


if __name__=='__main__':main()
