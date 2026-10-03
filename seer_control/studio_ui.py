"""Extended operator tools and adapters, isolated from the original console."""
import copy
import bisect
import json
import math
import queue
import time
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox, simpledialog
from .model import Simulator
from .smap import path_record_geometry, make_path_record, make_area_record
from .studio_core import (EditHistory, Recorder, MissionRunner, ACTION_DEFAULTS,
                          flatten_actions, validate_actions, validate_model,
                          rigid_calibration, finite, point_segment_distance)
from .studio_devices import DeviceBridge, tcp_operation, arm_call, validate_operation, get_field
from .fairino_ui import FairinoUIMixin
from .fairino_api import validate as validate_fr5
from .obstacle_ui import ObstacleUIMixin
from .route_planner import plan_stops, plan_actions, adjacency, upgrade_loop_chain

from .theme import PANEL, INK, MUTED, GREEN, ORANGE, RED
SETTINGS=Path.home()/'.seer_amr_console'/'studio_settings.json'


class ConsoleAdapter:
    def __init__(self,console):self.c=console;self.context={};self.motion=(0.,0.)
    def begin(self,a):
        c=self.c;state=c.current_state();typ=a['type']
        if not c.connected or (not c.real and not c.sim_powered):raise ValueError('로봇 연결/전원이 꺼져 있습니다.')
        if c.real and not c.control_enabled:raise ValueError('실기 제어권이 필요합니다.')
        self.context=dict(start=(state['x'],state['y']),theta=state['theta'],turn=0.,seen=False,token=None,type=typ)
        self.motion=(0.,0.)
        if typ=='Path Nav':
            c.studio_command_error=None
            if not c.studio_arm_safe:raise ValueError('로봇팔 safe_pose 완료 후 AMR를 이동하세요.')
            if a['goal'] not in c.map.nodes:raise ValueError('목적지 Point가 없습니다.')
            if c.real:
                if a['goal'] not in c.robot_stations:raise ValueError('로봇 Station에 없는 목적지입니다.')
                c.send_command('navigate',c._station_nav_payload(c.map.nodes[a['goal']]))
            else:
                if c.sim.obstacle_policy in ('reroute','auto'):
                    a['timeout_s']=max(a['timeout_s'],c.sim.auto_static_s+c.sim.auto_wait_s+c.sim.reroute_wait_s*(c.sim.reroute_attempt_limit+1)+120)
                c.sim.navigate(a['goal'],route_nodes=None if c.sim.obstacle_policy in ('reroute','auto') else a.get('route_nodes'))
        elif typ in ('Translation','Rotation'):
            c.studio_motion_error=None
            if not c.studio_arm_safe:raise ValueError('로봇팔 safe_pose 완료 후 AMR를 이동하세요.')
            if c.real and (state.get('emergency') or state.get('blocked')):raise ValueError('비상정지/장애물 상태입니다.')
            if typ=='Translation':self.motion=(math.copysign(min(.3,a['speed_mps']),a['distance_m']) if a['distance_m'] else 0.,0.)
            else:self.motion=(0.,math.copysign(min(.6,math.radians(a['speed_dps'])),a['angle_deg']) if a['angle_deg'] else 0.)
            self._motion(True)
        elif typ=='Set DO':
            if c.real:self.context['token']=c._studio_device('set_do',dict(channel=a['channel'],value=a['value']))
            else:c.sim.do[a['channel']]=a['value']
        elif typ=='Custom Action':
            name=a['operation']
            if c.real:self.context['token']=c._studio_device(name,a.get('payload',{}))
            else:
                if name=='set_di':c.sim.di[int(a['payload']['channel'])]=bool(a['payload']['value'])
                elif name=='set_battery':c.sim.state.battery=max(0,min(100,finite(a['payload']['value'])))
                elif name=='clear_obstacles':c.map.obstacles=[]
                else:raise ValueError('SIM Custom Action: set_di / set_battery / clear_obstacles')
        elif typ=='Arm Action':
            if abs(float(state.get('speed',0)))>.02 or state.get('emergency') or state.get('stopped') or (not c.real and c.sim.route):
                raise ValueError('로봇팔 작업 전 AMR 정지 확인이 필요합니다.')
            if c.real:
                if c.studio_config['arm'].get('driver')!='fairino' and not c.studio_config['arm'].get('stop_method'):raise ValueError('로봇팔 stop_method 설정이 필요합니다.')
                config=copy.deepcopy(c.studio_config['arm'])
                self.context['token']=c._studio_submit(lambda:c._arm_call(config,'execute',a['operation']))
            else:
                c.sim.arm.update(status='RUNNING',operation=a['operation'],progress=0.)
                cfg=c.studio_config['arm']
                if cfg.get('driver')!='fairino':c.sim.arm.pop('joint_positions',None)
                if cfg.get('driver')=='fairino':
                    spec=cfg.get('operations',{}).get(a['operation'])
                    if not spec:raise ValueError('등록되지 않은 FR5 작업: '+a['operation'])
                    if spec['method']=='MoveL':raise ValueError('SIM MoveL 역기구학은 지원하지 않습니다. MoveJ 작업으로 관절 자세를 지정하세요.')
                    validate_fr5(cfg)
                    names=[j['name'] for j in c.world3d.arm_asset.movable()]
                    if len(names)!=6:raise ValueError('SIM FR5는 6축 URDF 모델이 필요합니다.')
                    current=c.sim.arm.get('joint_positions') or c.world3d.positions['arm']
                    self.context['joint_start']={n:current.get(n,0.) for n in names}
                    self.context['joint_target']=dict(zip(names,[math.radians(v) for v in spec['target']]))
            c.studio_arm_safe=False
        c.log('MISSION',f"{c.studio_runner.index+1}단계 시작 · {typ}")
    def _motion(self,active):
        c=self.c
        if c.real:
            with c._jog_lock:
                c._jog_desired={'vx':self.motion[0],'vy':0.,'w':self.motion[1]} if active else None
            if c._jog_wakeup:c._jog_wakeup.set()
        elif active:c.sim.drive(*self.motion)
        else:c.sim.v=c.sim.w=c.sim.lease=0
    def poll(self,a,dt,elapsed):
        c=self.c;typ=a['type'];state=c.current_state()
        if not c.connected or (not c.real and not c.sim_powered):raise ValueError('미션 중 연결/전원이 끊겼습니다.')
        if c.real and (time.monotonic()-c.last_state>3 or not c.control_enabled):raise ValueError('실기 상태/제어권이 유효하지 않습니다.')
        if c.real and state.get('emergency'):raise ValueError('비상정지가 활성화되었습니다.')
        if typ=='Path Nav':
            if c.real and getattr(c,'studio_command_error',None):raise ValueError(c.studio_command_error)
            if not c.real:
                if c.sim.auto_charge['phase']!='IDLE':return False
                if c.sim.skip_result and c.sim.skip_result['goal']==a['goal']:
                    result=dict(skip=True,**c.sim.skip_result);c.sim.skip_result=None
                    c.log('MISSION','목적지 패스 · '+result['goal']+' · '+result['reason'])
                    return result
                return not c.sim.route and c.sim.state.task=='완료'
            task=state.get('task');status=state.get('task_status')
            if task in ('FAILED','CANCELED') or status in (5,6):raise ValueError(f'주행 실패: {task}')
            if task in ('RUNNING','WAITING') or status in (1,2):self.context['seen']=True
            node=c.map.nodes[a['goal']];distance=math.hypot(state['x']-node['x'],state['y']-node['y'])
            return elapsed>.8 and distance<=.2 and abs(float(state.get('speed',0)))<.02 and (task=='COMPLETED' or status==4)
        if typ=='Translation':
            if c.real and getattr(c,'studio_motion_error',None):raise ValueError(c.studio_motion_error)
            traveled=math.dist(self.context['start'],(state['x'],state['y']))
            done=traveled>=abs(a['distance_m'])-.005
            if done:self._motion(False)
            elif not state.get('blocked'):self._motion(True)
            else:self._motion(False)
            return done
        if typ=='Rotation':
            if c.real and getattr(c,'studio_motion_error',None):raise ValueError(c.studio_motion_error)
            angle=state['theta'];prev=self.context['theta']
            self.context['turn']+=math.atan2(math.sin(angle-prev),math.cos(angle-prev));self.context['theta']=angle
            done=abs(self.context['turn'])>=abs(math.radians(a['angle_deg']))-.005
            if done:self._motion(False)
            elif not state.get('blocked'):self._motion(True)
            else:self._motion(False)
            return done
        if typ=='Wait':return elapsed>=a['duration_s']
        if typ=='Wait DI Trigger':return c._studio_io_value('di',a['channel'])==a['value']
        if typ=='Branch DI':return True
        if typ=='Arm Action':
            if not c.real:
                progress=min(1.,elapsed/max(.001,a['duration_s']));c.sim.arm['progress']=progress
                if 'joint_target' in self.context:
                    c.sim.arm['joint_positions']={n:self.context['joint_start'][n]+(v-self.context['joint_start'][n])*progress for n,v in self.context['joint_target'].items()}
                if progress>=1:
                    c.sim.arm['status']='COMPLETED';c.sim.arm['pose']='SAFE' if a['operation']=='safe_pose' else 'WORK'
                    c.studio_arm_safe=a['operation']=='safe_pose';return True
                return False
            token=self.context['token'];result=c.studio_results.pop(token,None)
            if result:
                ok,value=result
                if not ok:raise ValueError(value)
                self.context['accepted']=True
            if not self.context.get('accepted'):return False
            if not self.context.get('status_token') and elapsed-self.context.get('last_poll',-1)>.5:
                config=copy.deepcopy(c.studio_config['arm'])
                self.context['status_token']=c._studio_submit(lambda:c._arm_call(config,'status'))
                self.context['last_poll']=elapsed
            result=c.studio_results.pop(self.context.get('status_token'),None)
            if result:
                self.context['status_token']=None;ok,value=result
                if not ok:raise ValueError(value)
                cfg=c.studio_config['arm']
                if cfg.get('driver')=='fairino':
                    c._fr5_receive(value)
                    if value.get('status') in ('ERROR','CANCELED'):raise ValueError('FR5 작업 실패: '+str(value))
                    return value.get('status')=='COMPLETED'
                if value in cfg.get('failure_values',['FAILED','ERROR']):raise ValueError(f'로봇팔 작업 실패: {value}')
                done=value in cfg.get('done_values',['COMPLETED'])
                if done:c.studio_arm_safe=a['operation']=='safe_pose'
                return done
            return False
        token=self.context.get('token')
        if token is None:return True
        result=c.studio_results.pop(token,None)
        if result:
            ok,value=result
            if not ok:raise ValueError(value)
            if typ=='Set DO':c.studio_io['do'][a['channel']]=a['value']
            return True
        return False
    def branch_target(self,a):
        value=self.c._studio_io_value('di',a['channel'])
        if value is None:raise ValueError('분기에 필요한 DI 상태가 없습니다.')
        return a['target'] if value==a['value'] else None
    def pause(self):
        self._motion(False)
        if self.c.real and self.context.get('type')=='Arm Action' and self.c.studio_config['arm'].get('driver')=='fairino':
            cfg=copy.deepcopy(self.c.studio_config['arm'])
            self.c._studio_submit(lambda:self.c._arm_call(cfg,'pause'),lambda r:None)
        if self.c.real:self.c.send_command('pause',{})
        else:self.c.sim.command('pause')
    def resume(self):
        if self.c.real and self.context.get('type')=='Arm Action' and self.c.studio_config['arm'].get('driver')=='fairino':
            cfg=copy.deepcopy(self.c.studio_config['arm'])
            self.c._studio_submit(lambda:self.c._arm_call(cfg,'resume'),lambda r:None)
        if self.c.real:self.c.send_command('resume',{})
        else:self.c.sim.command('resume')
        if self.motion!=(0.,0.):self._motion(True)
    def cancel(self):
        c=self.c;self._motion(False)
        c.studio_charge_inhibit=True
        c._studio_real_charge_phase='IDLE'
        c.sim.auto_charge['phase']='IDLE';c.sim.auto_charge['enabled']=False;c.sim._auto_charge_goal=None
        c.studio_bridge.cancel(self.context.get('token'))
        c.studio_bridge.cancel(self.context.get('status_token'))
        if c.real:
            if c.connected and c.control_enabled:c.send_command('cancel',{})
            if self.context.get('type')=='Arm Action':
                config=copy.deepcopy(c.studio_config['arm'])
                if config.get('driver')=='fairino':c._fr5_priority_stop()
                elif config.get('stop_method'):c._studio_submit(lambda:arm_call(config,'stop'),c._studio_show_device)
        else:
            c.sim.stop();c.sim.arm['status']='CANCELED'


class StudioMixin(FairinoUIMixin,ObstacleUIMixin):
    def _studio_init(self):
        self.studio_config=dict(robot_model=validate_model({}),
            peripherals=dict(operations={}),arm=dict(verified=False,endpoint='',execute_method='',
            status_method='',stop_method='',success_value=0,done_values=['COMPLETED'],failure_values=['FAILED','ERROR'],operations={}),
            auto_charge=dict(enabled=False,low=20.,high=80.,rate=2.))
        try:
            if SETTINGS.exists():
                loaded=json.loads(SETTINGS.read_text(encoding='utf-8'));self.studio_config.update(loaded)
                self.studio_config['robot_model']=validate_model(self.studio_config['robot_model'])
        except Exception as e:self.log('ERROR','확장 설정 읽기 실패: '+str(e))
        self.studio_history=EditHistory();self._studio_snapshot=self.studio_history.capture(self.map)
        self._studio_map=self.map;self.studio_bridge=DeviceBridge();self.studio_results={};self.studio_jobs={}
        self.studio_token_generation={}
        self.studio_io=dict(di={},do={});self.studio_recorder=Recorder();self.studio_alarms=[];self._studio_alarm_active={}
        self.studio_arm_safe=True
        self._fr5_init()
        self.studio_charge_inhibit=False
        self.studio_runner=MissionRunner(ConsoleAdapter(self));self._studio_runner_status='IDLE'
        self.studio_snap=tk.BooleanVar(value=True);self.studio_grid=tk.StringVar(value='0.10')
        self.studio_diagnostics=tk.BooleanVar(value=True);self.studio_record=tk.BooleanVar(value=False)
        self.studio_playing=False;self.studio_play_frame=None;self._studio_record_at=0.;self._studio_draw_at=0.
        self._studio_auto_seek=None
        self._studio_io_poll_at=0.;self.studio_polygon=[];self.studio_wall_start=None
        self._studio_edit_pending=None;self._studio_real_charge_phase='IDLE';self._studio_real_charge_saved=None
        self.studio_point_drag=None;self.studio_play_cursor=0.;self.studio_play_speed=tk.StringVar(value='1')
        self._studio_erasing=False
        self._studio_apply_sim_settings()
        self.editor_canvas.bind('<Return>',lambda e:self._studio_finish_polygon() if self.edit_mode.get()=='다각형 영역' else None)
        page=ttk.Frame(self.tabs);self.tabs.add(page,text='확장 도구')
        self._register_navigation(page, '확장 도구', '⊞')
        self._sync_navigation()
        sub=ttk.Notebook(page);sub.pack(fill='both',expand=True,pady=8)
        pages={}
        for key,title in [('edit','지도 도구'),('mission','미션 실행'),('io','DI / DO'),('charge','자동 충전'),
                          ('record','알람 / 운행 기록'),('model','로봇 모델'),('calibration','보정'),('arm','로봇팔 / 장비 연결')]:
            outer=ttk.Frame(sub);sub.add(outer,text=title)
            container,inner,_=self._scrollable_frame(outer,bg=PANEL);container.pack(fill='both',expand=True)
            pages[key]=inner
        self._studio_edit_page(pages['edit']);self._studio_mission_page(pages['mission'])
        self._studio_io_page(pages['io']);self._studio_charge_page(pages['charge'])
        self._studio_record_page(pages['record']);self._studio_model_page(pages['model'])
        self._studio_calibration_page(pages['calibration']);self._studio_devices_page(pages['arm'])
        self.bind('<Control-z>',lambda e:self._studio_undo(False));self.bind('<Control-y>',lambda e:self._studio_undo(True))

    def _studio_apply_sim_settings(self):
        if not self.real:
            self.map.robot_model=copy.deepcopy(self.studio_config['robot_model'])
            self.sim.auto_charge.update(self.studio_config['auto_charge'])
            self.studio_arm_safe=self.sim.arm.get('pose')=='SAFE'

    def _studio_row(self,parent):
        row=tk.Frame(parent,bg=PANEL);row.pack(fill='x',padx=12,pady=5);return row
    def _studio_note(self,parent,text):
        note=self.label(parent,text,9,MUTED,bg=PANEL,wraplength=730,justify='left',anchor='w')
        note.pack(fill='x',padx=12,pady=7)
        note.bind('<Configure>',lambda event:note.configure(wraplength=max(200,event.width-8)))
    def _studio_json_box(self,parent,value,height=12):
        frame=self._studio_row(parent)
        box=tk.Text(frame,height=height,bg='#ffffff',fg=INK,insertbackground=INK,font=('Consolas',10),wrap='none')
        sb=ttk.Scrollbar(frame,command=box.yview);box.configure(yscrollcommand=sb.set)
        sb.pack(side='right',fill='y');box.pack(fill='both',expand=True)
        box.insert('1.0',json.dumps(value,ensure_ascii=False,indent=2));return box
    def _studio_json(self,box):return json.loads(box.get('1.0','end'))
    def _studio_edit_page(self,page):
        row=self._studio_row(page)
        for title,mode in [('Point 이동','Point 이동'),('곡선 조절','곡선 편집'),('벽 만들기','벽 만들기'),('가상벽','가상벽'),
                           ('다각형 영역','다각형 영역'),('점군 지우개','점군 지우개'),('SIM 장애물','SIM 장애물')]:
            self.button(row,title,lambda m=mode:(self.tabs.select(self.nodes_page),self.edit_mode.set(m))).pack(side='left',padx=2)
        row=self._studio_row(page)
        self.button(row,'Undo',lambda:self._studio_undo(False)).pack(side='left',padx=2)
        self.button(row,'Redo',lambda:self._studio_undo(True)).pack(side='left',padx=2)
        tk.Checkbutton(row,text='좌표 스냅',variable=self.studio_snap,bg=PANEL,fg=INK,selectcolor=PANEL).pack(side='left',padx=8)
        ttk.Entry(row,textvariable=self.studio_grid,width=7).pack(side='left');self.label(row,'m',bg=PANEL).pack(side='left')
        self.button(row,'선택 경로 직선화',self._studio_straighten).pack(side='left',padx=6)
        self.button(row,'선택 Point 일괄 변경',self._studio_batch_nodes).pack(side='left',padx=2)
        self.button(row,'선택 Path 일괄 변경',self._studio_batch_paths).pack(side='left',padx=2)
        row=self._studio_row(page)
        self.button(row,'다각형 완성',self._studio_finish_polygon).pack(side='left',padx=2)
        self.button(row,'그리기 취소',lambda:self._studio_cancel_draw()).pack(side='left',padx=2)
        self.button(row,'SIM 장애물 모두 제거',lambda:self._studio_obstacle_edit(lambda:setattr(self.map,'obstacles',[]),'장애물 제거')).pack(side='left',padx=2)
        self.button(row,'가상벽 모두 제거',lambda:self._studio_edit(lambda:setattr(self.map,'virtual_walls',[]),'가상벽 제거')).pack(side='left',padx=2)
        self.button(row,'점군 정리',self._studio_clean_cloud).pack(side='left',padx=2)
        self._studio_note(page,'Point 이동: 노드를 드래그. 가상벽: 두 점 클릭. 다각형: 꼭짓점을 클릭하고 완성. 점군 지우개: 반경 0.3m. SIM 장애물: 클릭으로 추가/삭제. Ctrl+Z / Ctrl+Y로 지도 편집을 되돌립니다.')
        tk.Checkbutton(page,text='경로 진단 표시 (방향 / 좌표 개수 / 적용 속도)',variable=self.studio_diagnostics,
            bg=PANEL,fg=INK,selectcolor=PANEL,command=self.draw_map).pack(anchor='w',padx=12,pady=10)
        self.studio_diag_text=self._studio_json_box(page,{},height=10)

    def _studio_mission_page(self,page):
        row=self._studio_row(page)
        for title,fn in [('선택 Taskchain 실행',self.run_tasks),('일시정지',lambda:self.studio_runner.pause(time.monotonic())),
                         ('재개',lambda:self.studio_runner.resume(time.monotonic())),('취소',self.cancel_tasks),
                         ('실패 단계부터 재개',self._studio_retry)]:self.button(row,title,fn).pack(side='left',padx=3)
        self.studio_mission_label=self.label(page,'미션 대기',12,INK,bg=PANEL);self.studio_mission_label.pack(anchor='w',padx=12,pady=8)
        self.studio_action_tree=ttk.Treeview(page,columns=('step','type','target','status','timeout'),show='headings',height=10)
        for key,title in [('step','단계'),('type','Action'),('target','목적지 / 작업'),('status','상태'),('timeout','시간 제한(s)')]:
            self.studio_action_tree.heading(key,text=title);self.studio_action_tree.column(key,width=140,stretch=True)
        self.studio_action_tree.pack(fill='both',expand=True,padx=12,pady=8)
        self._studio_note(page,'Taskchain의 모든 체크된 Task/Group을 순서대로 실행합니다. Branch DI의 target은 1부터 시작하는 단계 번호입니다. 실패한 장비 명령은 자동 재전송하지 않습니다. 실패 단계 재개 버튼은 사용자가 재실행을 선택하는 기능입니다.')

    def _studio_io_page(self,page):
        row=self._studio_row(page)
        self.studio_channel=tk.StringVar(value='0');self.studio_io_bool=tk.BooleanVar(value=True)
        self.label(row,'채널',bg=PANEL).pack(side='left');tk.Spinbox(row,from_=0,to=63,textvariable=self.studio_channel,width=5).pack(side='left',padx=5)
        tk.Checkbutton(row,text='ON',variable=self.studio_io_bool,bg=PANEL,fg=INK,selectcolor=PANEL).pack(side='left')
        self.button(row,'SIM DI 설정',lambda:self.guarded(self._studio_set_di)).pack(side='left',padx=4)
        self.button(row,'DO 설정',lambda:self.guarded(self._studio_set_do)).pack(side='left',padx=4)
        self.button(row,'실기 I/O 조회',lambda:self.guarded(self._studio_read_io)).pack(side='left',padx=4)
        self.studio_io_tree=ttk.Treeview(page,columns=('channel','di','do'),show='headings',height=15)
        for key,title in [('channel','채널'),('di','DI 입력'),('do','DO 출력')]:self.studio_io_tree.heading(key,text=title)
        for i in range(64):self.studio_io_tree.insert('','end',iid=str(i),values=(i,'—','—'))
        self.studio_io_tree.pack(fill='both',expand=True,padx=12,pady=8)
        self._studio_note(page,'실기 I/O는 장비 연결 탭에 검증된 read_io / set_do API를 등록하면 사용합니다. 실기 미션의 DI 값은 최근 조회값만 사용합니다.')

    def _studio_charge_page(self,page):
        self.studio_charge_enabled=tk.BooleanVar(value=self.studio_config['auto_charge']['enabled'])
        tk.Checkbutton(page,text='자동 충전 사용',variable=self.studio_charge_enabled,bg=PANEL,fg=INK,selectcolor=PANEL).pack(anchor='w',padx=12,pady=12)
        self.studio_charge_vars={}
        for key,title in [('low','충전 시작 배터리 (%)'),('high','미션 재개 배터리 (%)'),('rate','SIM 충전 속도 (%/s)')]:
            row=self._studio_row(page);self.label(row,title,bg=PANEL).pack(side='left')
            var=tk.StringVar(value=str(self.studio_config['auto_charge'][key]));self.studio_charge_vars[key]=var
            ttk.Entry(row,textvariable=var,width=12).pack(side='right')
        row=self._studio_row(page);self.button(row,'정책 적용 / 저장',lambda:self.guarded(self._studio_save_charge)).pack(side='left')
        self.button(row,'SIM 배터리 변경',lambda:self.guarded(self._studio_battery)).pack(side='left',padx=8)
        self.studio_charge_label=self.label(page,'충전 대기',12,INK,bg=PANEL);self.studio_charge_label.pack(anchor='w',padx=12,pady=10)
        self._studio_note(page,'SIM: 가까운 연결 가능한 충전 노드로 이동한 뒤 충전하고 중단된 목적지로 복귀합니다. 실기: Path Nav 사이에서 충전 노드 이동, charging 응답과 배터리 회복을 확인한 뒤 미션을 재개합니다. 충전기에 접근하는 것만으로 충전되는지는 실제 장비에서 확인해야 합니다.')

    def _studio_record_page(self,page):
        row=self._studio_row(page)
        tk.Checkbutton(row,text='운행 기록',variable=self.studio_record,bg=PANEL,fg=INK,selectcolor=PANEL).pack(side='left')
        for title,fn in [('기록 저장',self._studio_save_record),('기록 열기',self._studio_load_record),
                         ('재생 / 정지',self._studio_play),('알람 이력 저장',self._studio_save_alarms)]:self.button(row,title,fn).pack(side='left',padx=3)
        ttk.Combobox(row,textvariable=self.studio_play_speed,values=['0.25','0.5','1','2','4'],state='readonly',width=5).pack(side='left',padx=5)
        self.studio_timeline=tk.Scale(page,from_=0,to=1,orient='horizontal',command=self._studio_seek,bg=PANEL,fg=INK,highlightthickness=0)
        self.studio_timeline.pack(fill='x',padx=12,pady=5)
        self.studio_record_label=self.label(page,'기록 0 프레임',10,INK,bg=PANEL);self.studio_record_label.pack(anchor='w',padx=12)
        self.studio_alarm_tree=ttk.Treeview(page,columns=('time','level','message','end'),show='headings',height=12)
        for key,title in [('time','발생'),('level','종류'),('message','내용'),('end','해제')]:self.studio_alarm_tree.heading(key,text=title)
        self.studio_alarm_tree.column('message',width=450)
        self.studio_alarm_tree.pack(fill='both',expand=True,padx=12,pady=8)
        self._studio_note(page,'운행 기록은 0.2초 간격이며 최대 36,000프레임입니다. 재생 위치는 지도 위 보라색 로봇으로 표시하며 실제 로봇을 움직이지 않습니다.')

    def _studio_model_page(self,page):
        self.studio_model_vars={}
        labels=dict(length='차체 길이(m)',width='차체 폭(m)',radius='충돌 반경(m)',maxspeed='최대 속도(m/s)',maxacc='최대 가속(m/s²)',
                    maxdec='최대 감속(m/s²)',maxrot='최대 회전(rad/s)',wheel_scale='주행 거리 보정 배율',laser_x='LiDAR X(m)',laser_y='LiDAR Y(m)',laser_yaw='LiDAR 방향(rad)')
        for key,title in labels.items():
            row=self._studio_row(page);self.label(row,title,bg=PANEL).pack(side='left')
            var=tk.StringVar(value=str(self.studio_config['robot_model'][key]));self.studio_model_vars[key]=var
            ttk.Entry(row,textvariable=var,width=16).pack(side='right')
        row=self._studio_row(page)
        self.button(row,'SIM 모델 적용 / 저장',lambda:self.guarded(self._studio_save_model)).pack(side='left')
        self.button(row,'모델 JSON 내보내기',self._studio_export_model).pack(side='left',padx=6)
        self.button(row,'모델 JSON 가져오기',self._studio_import_model).pack(side='left',padx=6)
        self.button(row,'실기 모델 조회',lambda:self.guarded(lambda:self._studio_device('read_model',{},self._studio_show_device))).pack(side='left',padx=6)
        self.button(row,'실기 모델 전송',lambda:self.guarded(self._studio_write_model)).pack(side='left',padx=6)
        self._studio_note(page,'모델 값은 SIM에 적용됩니다. 실기 모델은 검증된 read_model / write_model API를 등록해야 조회·전송할 수 있습니다. 본 모델 JSON은 RoboShop 원본 모델 파일과 같은 형식이 아닙니다.')

    def _studio_calibration_page(self,page):
        self._studio_note(page,'LiDAR 대응점 보정: 각 행에 sensor_x,sensor_y,map_x,map_y를 입력하세요. 최소 2개 대응점으로 평행이동·회전을 계산합니다. 결과는 로컬 센서 모델에 적용하며 실기 컨트롤러 자체 보정은 등록된 calibrate API로 별도 요청합니다.')
        self.studio_cal_pairs=tk.Text(page,height=7,bg='#ffffff',fg=INK,insertbackground=INK,font=('Consolas',11))
        self.studio_cal_pairs.pack(fill='x',padx=12,pady=8);self.studio_cal_pairs.insert('1.0','0,0,0,0\n1,0,1,0\n')
        row=self._studio_row(page)
        self.button(row,'대응점 보정 계산',lambda:self.guarded(self._studio_calculate)).pack(side='left')
        self.button(row,'계산 결과 로컬 적용',lambda:self.guarded(self._studio_apply_calibration)).pack(side='left',padx=5)
        self.button(row,'실기 보정 요청',lambda:self.guarded(lambda:self._studio_device('calibrate',{},self._studio_show_device))).pack(side='left',padx=5)
        self.studio_cal_result=self.label(page,'보정 결과 없음',11,INK,bg=PANEL);self.studio_cal_result.pack(anchor='w',padx=12,pady=8)
        row=self._studio_row(page);self.studio_wheel_command=tk.StringVar(value='1');self.studio_wheel_measured=tk.StringVar(value='1')
        for title,var in [('명령 거리(m)',self.studio_wheel_command),('측정 거리(m)',self.studio_wheel_measured)]:
            self.label(row,title,bg=PANEL).pack(side='left',padx=3);ttk.Entry(row,textvariable=var,width=10).pack(side='left')
        self.button(row,'거리 보정 배율 적용',lambda:self.guarded(self._studio_wheel_calibration)).pack(side='left',padx=6)

    def _studio_devices_page(self,page):
        self._studio_note(page,'FR5는 공식 FAIRINO Python SDK로 연결합니다. FR5 연결 설정에서 IP, SDK 폴더, 작업 자세를 등록하세요. Arm Action은 AMR 정지 후 실행하며 실제 목표 도착을 확인합니다. 기타 장비는 아래 JSON으로 설정합니다.')
        self._fr5_controls(page)
        self.studio_device_box=self._studio_json_box(page,dict(peripherals=self.studio_config['peripherals'],arm=self.studio_config['arm']),height=17)
        row=self._studio_row(page);self.button(row,'연결 설정 적용 / 저장',lambda:self.guarded(self._studio_save_devices)).pack(side='left')
        self.button(row,'설정 예제 보기',self._studio_device_example).pack(side='left',padx=5)
        self.button(row,'SIM 로봇팔 시험',lambda:self.guarded(self._studio_test_arm)).pack(side='left',padx=5)
        self.button(row,'로봇팔 상태 조회',lambda:self.guarded(self._studio_arm_status)).pack(side='left',padx=5)
        self.studio_arm_label=self.label(page,'로봇팔 대기',11,INK,bg=PANEL);self.studio_arm_label.pack(anchor='w',padx=12,pady=8)

    def _studio_edit(self,fn,label):
        def edit():
            self.editable();before=self.studio_history.capture(self.map);fn()
            self.studio_history.commit(before,self.map,label);self._studio_snapshot=self.studio_history.capture(self.map)
            self._studio_edit_pending=None
            self.map_dirty=True;self.refresh_nodes();self.draw_map()
        return self.guarded(edit)
    def _studio_obstacle_edit(self,fn,label):
        def edit():
            self.sim_required();before=self.studio_history.capture(self.map);fn()
            self.studio_history.commit(before,self.map,label);self.map_dirty=True;self.draw_map()
        return self.guarded(edit)
    def _studio_can_edit(self):
        try:self.editable();return True
        except ValueError as e:
            self.log('ERROR',str(e));messagebox.showerror('지도 편집',str(e),parent=self);return False
    def _studio_undo(self,redo):
        def run():
            self.editable()
            if self._studio_edit_pending is not None:
                self.studio_history.commit(self._studio_edit_pending,self.map,'지도 편집');self._studio_edit_pending=None
            label=self.studio_history.redo(self.map) if redo else self.studio_history.undo(self.map)
            if label:
                self.curve_edit_record=None;self.curve_drag_index=None;self.map_dirty=True
                self._studio_snapshot=self.studio_history.capture(self.map);self.refresh_nodes();self.draw_map()
                self.log('EDIT',('Redo ' if redo else 'Undo ')+label)
            self._studio_edit_pending=None
        self.guarded(run)
    def _studio_straighten(self):
        rec=self._selected_path_record() or self.curve_edit_record
        if not rec:return
        def edit():
            raw=make_path_record(self.map.nodes[rec['a']],self.map.nodes[rec['b']])
            self._set_curve_controls(rec,[(raw[k]['x'],raw[k]['y']) for k in ('controlPos1','controlPos2')])
        self._studio_edit(edit,'직선화')
    def _studio_batch_nodes(self):
        keys=self.node_tree.selection()
        if not keys:return
        def apply(data):
            allowed={'kind','r','angle','spin','description','maxspeed'}
            if set(data)-allowed:raise ValueError('일괄 변경: kind/r/angle/spin/description/maxspeed만 지원합니다.')
            for key in keys:self.map.nodes[key].update(copy.deepcopy(data))
        self._studio_json_dialog('Point 일괄 속성',{'spin':True},lambda d:self._studio_edit(lambda:apply(d),'Point 일괄 변경'))
    def _studio_batch_paths(self):
        records=getattr(self.map,'path_records',[])
        selected=[records[int(i.split('_')[1])] for i in self.path_tree.selection()]
        if not selected:return
        def apply(data):
            for rec in selected:
                rec.setdefault('properties',{}).update(data);rec['draft']=True
        self._studio_json_dialog('Path 일괄 속성',{'maxspeed':.3},lambda d:self._studio_edit(lambda:apply(d),'Path 일괄 변경'))
    def _studio_json_dialog(self,title,data,apply):
        win=tk.Toplevel(self);win.title(title);self._fit_dialog(win,650,550);win.configure(bg=PANEL)
        box=self._studio_json_box(win,data,height=18)
        def save():
            def work():
                value=self._studio_json(box)
                if not isinstance(value,dict):raise ValueError('JSON object가 필요합니다.')
                apply(value);win.destroy()
            self.guarded(work)
        self.button(win,'적용',save).pack(pady=10)
    def _studio_clean_cloud(self):
        def clean():
            cells={}
            for x,y in getattr(self.map,'cloud',[]):cells[(round(x/.03),round(y/.03))]=(x,y)
            self.map.cloud=list(cells.values());self._studio_sync_cloud()
        self._studio_edit(clean,'점군 정리')
    def _studio_sync_cloud(self):
        if hasattr(self.map,'smap_source'):
            from .smap import _point
            keep=set(map(tuple,self.map.cloud))
            self.map.smap_source['normalPosList']=[p for p in self.map.smap_source.get('normalPosList',[]) if _point(p,None) in keep]
    def _studio_cancel_draw(self):self.studio_polygon=[];self.studio_wall_start=None;self.actor_draft=None;self.draw_map()
    def _studio_finish_polygon(self):
        def create():
            if len(self.studio_polygon)<3:raise ValueError('꼭짓점 3개 이상을 선택하세요.')
            pts=list(self.studio_polygon);aid=self._next_area_id();raw=make_area_record(aid,pts,{'forbidden':True})
            self.map.area_records.append(dict(id=aid,points=pts,properties={'forbidden':True},raw=raw,className='AdvancedArea',draft=True))
            self.studio_polygon=[]
        self._studio_edit(create,'다각형 영역')

    def _studio_snap_xy(self,x,y):
        if self.studio_snap.get():
            cell=finite(self.studio_grid.get())
            if cell<=0:raise ValueError('스냅 간격은 0보다 커야 합니다.')
            return round(x/cell)*cell,round(y/cell)*cell
        return x,y
    def _studio_map_click(self,e):
        mode=self.edit_mode.get()
        if mode not in ('Point 이동','벽 만들기','가상벽','다각형 영역','점군 지우개','SIM 장애물','신규 감지 장애물','동적 장애물 배치'):return False
        def handle():
            if mode in ('SIM 장애물','신규 감지 장애물','동적 장애물 배치'):self.sim_required()
            else:self.editable()
            x,y=self._studio_snap_xy(*self.world(e.x,e.y))
            if mode=='동적 장애물 배치':self._actor_map_click(x,y)
            elif mode=='Point 이동':
                key=self.map.nearest(x,y);n=self.map.nodes[key]
                if math.dist(self.xy(n['x'],n['y']),(e.x,e.y))<22:self.studio_point_drag=key
            elif mode in ('벽 만들기','가상벽'):
                if self.studio_wall_start is None:self.studio_wall_start=(x,y)
                else:
                    a=self.studio_wall_start;self.studio_wall_start=None
                    if math.dist(a,(x,y))<.05:raise ValueError('가상벽 길이가 너무 짧습니다.')
                    target=self.map.walls if mode=='벽 만들기' else self.map.virtual_walls
                    self._studio_edit(lambda:target.append([*a,x,y]),'벽 추가' if mode=='벽 만들기' else '가상벽 추가')
            elif mode=='다각형 영역':self.studio_polygon.append((x,y))
            elif mode=='점군 지우개':self._studio_erasing=True;self._studio_erase(x,y)
            elif mode in ('SIM 장애물','신규 감지 장애물'):
                self.sim_required()
                def change():
                    near=next((o for o in self.map.obstacles if math.hypot(o['x']-x,o['y']-y)<o['radius']+.1),None)
                    if near:self.map.obstacles.remove(near)
                    else:self.map.obstacles.append(dict(x=x,y=y,radius=.25,map_fixed=mode=='SIM 장애물'))
                self._studio_obstacle_edit(change,'장애물 배치')
            self.draw_map()
        self.guarded(handle);return True
    def _studio_erase(self,x,y):
        self.map.cloud=[p for p in self.map.cloud if math.hypot(p[0]-x,p[1]-y)>.3]
        self._studio_sync_cloud();self.map_dirty=True
    def _studio_map_drag(self,e):
        if self.studio_point_drag:
            def move():
                self.editable();key=self.studio_point_drag;n=self.map.nodes[key];ox,oy=n['x'],n['y']
                nx,ny=self._studio_snap_xy(*self.world(e.x,e.y));n.update(x=nx,y=ny)
                self._sync_paths_after_point_edit(key,key,ox,oy,nx,ny);self.map_dirty=True;self.draw_map()
            self.guarded(move);return True
        if self.edit_mode.get()=='점군 지우개':
            self.guarded(lambda:(self.editable(),self._studio_erase(*self.world(e.x,e.y)),self.draw_map()));return True
        return False
    def _studio_map_release(self):
        if self._studio_erasing:
            self._studio_erasing=False;return True
        if self.studio_point_drag:
            self.studio_point_drag=None;self.refresh_nodes();self.draw_map();return True
        return False

    def _studio_device(self,name,values,callback=None):
        if not self.real or not self.connected:raise ValueError('실기 연결이 필요합니다.')
        cfg=copy.deepcopy(self.studio_config['peripherals']);spec=validate_operation(cfg,name)
        if spec['role']=='write' and not self.control_enabled:raise ValueError('실기 제어권이 필요합니다.')
        host=self.host.get();return self._studio_submit(lambda:tcp_operation(host,cfg,name,values),callback)
    def _studio_submit(self,fn,callback=None):
        try:token=self.studio_bridge.submit(fn)
        except queue.Full:raise ValueError('장비 요청 큐가 가득 찼습니다.')
        self.studio_token_generation[token]=self.generation
        if callback:self.studio_jobs[token]=(self.generation,callback)
        return token
    def _studio_io_value(self,kind,channel):
        if not self.real:return getattr(self.sim,kind).get(channel)
        if time.monotonic()-getattr(self,'_studio_io_rx',0)>3:return None
        return self.studio_io[kind].get(channel)
    def _studio_set_di(self):
        self.sim_required();channel=int(self.studio_channel.get())
        if not 0<=channel<64:raise ValueError('I/O 채널: 0~63')
        self.sim.di[channel]=self.studio_io_bool.get()
    def _studio_set_do(self):
        channel=int(self.studio_channel.get());value=self.studio_io_bool.get()
        if not 0<=channel<64:raise ValueError('I/O 채널: 0~63')
        if not self.real:self.sim.do[channel]=value
        else:self._studio_device('set_do',dict(channel=channel,value=value),lambda r:self.studio_io['do'].update({channel:value}))
    def _studio_read_io(self):
        def receive(result):
            spec=self.studio_config['peripherals']['operations']['read_io']
            for kind in ('di','do'):
                value=get_field(result,spec.get(kind+'_path',kind))
                self.studio_io[kind]={int(k):bool(v) for k,v in (value.items() if isinstance(value,dict) else enumerate(value))}
            self._studio_io_rx=time.monotonic()
        return self._studio_device('read_io',{},receive)
    def _studio_save_settings(self):
        SETTINGS.parent.mkdir(parents=True,exist_ok=True)
        SETTINGS.write_text(json.dumps(self.studio_config,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        self.log('INFO','확장 설정 저장')
    def _studio_save_charge(self):
        cfg={k:finite(v.get(),k) for k,v in self.studio_charge_vars.items()}
        if not 0<=cfg['low']<cfg['high']<=100 or cfg['rate']<=0:raise ValueError('충전 기준: 0≤시작<재개≤100, 속도>0')
        cfg['enabled']=self.studio_charge_enabled.get();self.studio_config['auto_charge']=cfg
        self.studio_charge_inhibit=False
        self._studio_apply_sim_settings();self._studio_save_settings()
    def _studio_battery(self):
        self.sim_required();value=simpledialog.askfloat('SIM 배터리','배터리 (%)',minvalue=0,maxvalue=100,parent=self)
        if value is not None:self.sim.state.battery=value
    def _studio_save_model(self):
        cfg=validate_model({k:v.get() for k,v in self.studio_model_vars.items()})
        if self.studio_runner.active or self.sim.route:raise ValueError('주행 종료 후 모델을 변경하세요.')
        self.studio_config['robot_model']=cfg;self._studio_apply_sim_settings();self._studio_save_settings()
    def _studio_export_model(self):
        path=filedialog.asksaveasfilename(defaultextension='.json',initialfile='robot_model.json')
        if path:self.guarded(lambda:Path(path).write_text(json.dumps(self.studio_config['robot_model'],indent=2),encoding='utf-8'))
    def _studio_import_model(self):
        path=filedialog.askopenfilename(filetypes=[('로컬 로봇 모델','*.json')])
        if not path:return
        def load():
            cfg=validate_model(json.loads(Path(path).read_text(encoding='utf-8-sig')))
            for key,value in cfg.items():self.studio_model_vars[key].set(str(value))
            self._studio_save_model()
        self.guarded(load)
    def _studio_write_model(self):
        if abs(float(self.current_state().get('speed',0)))>.02:raise ValueError('AMR 정지 후 모델을 전송하세요.')
        cfg=self.studio_config['robot_model']
        self._studio_device('write_model',dict(model=cfg,**cfg),self._studio_show_device)
    def _studio_show_device(self,result):messagebox.showinfo('장비 응답',json.dumps(result,ensure_ascii=False,indent=2)[:6000],parent=self)
    def _studio_calculate(self):
        pairs=[[float(v.strip()) for v in row.split(',')] for row in self.studio_cal_pairs.get('1.0','end').splitlines() if row.strip()]
        if any(len(row)!=4 for row in pairs):raise ValueError('각 행은 sensor_x,sensor_y,map_x,map_y입니다.')
        self.studio_calibration=rigid_calibration(pairs);r=self.studio_calibration
        self.studio_cal_result.config(text=f"X {r['x']:.4f}m / Y {r['y']:.4f}m / 방향 {math.degrees(r['yaw']):.3f}° / RMS {r['rms']:.4f}m")
    def _studio_apply_calibration(self):
        if not hasattr(self,'studio_calibration'):raise ValueError('보정 계산을 먼저 실행하세요.')
        r=self.studio_calibration
        for k,v in [('laser_x',r['x']),('laser_y',r['y']),('laser_yaw',r['yaw'])]:self.studio_model_vars[k].set(str(v))
        self._studio_save_model()
    def _studio_wheel_calibration(self):
        command=finite(self.studio_wheel_command.get());measured=finite(self.studio_wheel_measured.get())
        if command<=0 or measured<=0:raise ValueError('거리는 0보다 커야 합니다.')
        self.studio_model_vars['wheel_scale'].set(str(command/measured));self._studio_save_model()
    def _studio_save_devices(self):
        if self.studio_runner.active:raise ValueError('미션 종료 후 연결 설정을 변경하세요.')
        value=self._studio_json(self.studio_device_box)
        if not isinstance(value.get('peripherals',{}),dict) or not isinstance(value.get('arm',{}),dict):raise ValueError('peripherals/arm 설정 오류')
        for name,spec in value.get('peripherals',{}).get('operations',{}).items():
            if spec.get('verified'):validate_operation(value['peripherals'],name)
        if value.get('arm',{}).get('driver')=='fairino':validate_fr5(value['arm'],motion=value['arm'].get('verified',False))
        if value.get('arm')!=self.studio_config.get('arm') and self.fr5_client.connected:self._fr5_priority_stop()
        self.studio_config.update(value);self._studio_save_settings()
    def _studio_device_example(self):
        example=dict(peripherals=dict(operations={'read_io':dict(verified=False,role='read',api=0,port=0,response_type=0,payload={},di_path='di',do_path='do'),
             'set_do':dict(verified=False,role='write',api=0,port=0,response_type=0,payload={'channel':'${channel}','value':'${value}'}),
             'read_model':dict(verified=False,role='read',api=0,port=0,response_type=0,payload={}),
             'write_model':dict(verified=False,role='write',api=0,port=0,response_type=0,payload={'model':'${model}'}),
             'calibrate':dict(verified=False,role='write',api=0,port=0,response_type=0,payload={})}),
             arm=dict(verified=False,endpoint='http://ROBOT_IP:PORT/RPC2',execute_method='execute',status_method='status',stop_method='stop',
                 success_value=0,done_values=['COMPLETED'],failure_values=['FAILED','ERROR'],operations={'work':[],'safe_pose':[]}))
        self._studio_json_dialog('장비 설정 예제 (API 번호 입력 필요)',example,lambda value:(self.studio_device_box.delete('1.0','end'),self.studio_device_box.insert('1.0',json.dumps(value,ensure_ascii=False,indent=2))))
    def _studio_test_arm(self):
        self.sim_required()
        if self.studio_runner.active:raise ValueError('실행 중인 미션을 종료하세요.')
        self.studio_runner.start([dict(type='Arm Action',operation='work',duration_s=2),dict(type='Arm Action',operation='safe_pose',duration_s=1)])
        self.task_running=True
    def _studio_arm_status(self):
        if not self.real:self._studio_show_device(self.sim.arm);return
        cfg=copy.deepcopy(self.studio_config['arm'])
        self._studio_submit(lambda:self._arm_call(cfg,'status'),self._studio_show_device)

    def _studio_run_tasks(self):
        def run():
            chain=self._mission_active_chain()
            if not chain:raise ValueError('Taskchain을 선택하세요.')
            if upgrade_loop_chain(self.map,chain):
                self.log('MISSION','기존 순환 미션 변환 · 중간 노드는 통과, 방문 목적지만 정지')
                self.tc_refresh()
            actions=flatten_actions(chain)
            if chain.get('loop_route') and chain.get('move_to_start',True):
                start=chain['loop_route'][0];n=self.map.nodes[start];s=self.current_state()
                if math.hypot(s['x']-n['x'],s['y']-n['y'])>.1:
                    if chain.get('routing_policy'):
                        nearest=self.map.nearest(s['x'],s['y'])
                        plan=plan_stops(self.map,[nearest,start],'time' if chain['routing_policy']=='예상 시간 우선' else 'distance',chain.get('excluded_nodes',[]))
                        setup=plan_actions(plan)
                        actions=(setup or [dict(type='Path Nav',goal=start)])+actions
                    else:actions.insert(0,dict(type='Path Nav',goal=start))
            repeat=chain.get('repeat_count',0 if self.tc_loop.get() else int(self.tc_repeat_count.get()))
            validated=validate_actions(actions)
            if self.studio_runner.active or self.sim.route or self.held:raise ValueError('현재 작업/주행을 종료하세요.')
            if self.real and not (self.connected and self.control_enabled):raise ValueError('실기 제어권이 필요합니다.')
            if self.real:
                state=self.current_state()
                if state.get('task') in ('RUNNING','WAITING','SUSPENDED') or abs(float(state.get('speed',0)))>.02:
                    raise ValueError('진행 중인 실기 주행을 종료한 뒤 미션을 시작하세요.')
            graph=adjacency(self.map)
            for a in validated:
                if a['type']=='Path Nav' and a['goal'] not in self.map.nodes:raise ValueError('목적지가 없습니다: '+a['goal'])
                if a['type']=='Path Nav' and a.get('route_nodes'):
                    if any(src not in graph or not any(dst==nxt for nxt,_,_ in graph[src]) for src,dst in zip(a['route_nodes'],a['route_nodes'][1:])):
                        raise ValueError('계획 경로가 변경되었습니다. 순환 미션을 다시 설정하세요.')
                if self.real and a['type'] in ('Set DO','Custom Action'):validate_operation(self.studio_config['peripherals'],'set_do' if a['type']=='Set DO' else a['operation'])
                if self.real and a['type'] in ('Wait DI Trigger','Branch DI'):validate_operation(self.studio_config['peripherals'],'read_io')
                if self.real and a['type']=='Arm Action' and self.studio_config['arm'].get('verified') is not True:raise ValueError('실기 로봇팔 연결 설정이 필요합니다.')
            self.studio_runner.start(validated,repeat)
            self.studio_charge_inhibit=False
            self.task_running=True;self.studio_playing=False;self.studio_play_frame=None
        self.guarded(run)
    def _studio_retry(self):
        def run():
            runner=self.studio_runner
            if runner.status!='FAILED':raise ValueError('실패한 미션이 없습니다.')
            runner.start(runner.actions,runner.repeat,runner.index);self.task_running=True
        self.guarded(run)
    def _studio_save_record(self):
        path=filedialog.asksaveasfilename(defaultextension='.json',initialfile='run_record.json')
        if path:self.guarded(lambda:self.studio_recorder.save(path))
    def _studio_load_record(self):
        path=filedialog.askopenfilename(filetypes=[('운행 기록','*.json')])
        if path:
            self.guarded(lambda:self.studio_recorder.load(path));self.studio_record.set(False)
            self.studio_timeline.config(to=max(0,len(self.studio_recorder.frames)-1));self.studio_timeline.set(0)
    def _studio_play(self):
        if not self.studio_recorder.frames:return
        self.studio_playing=not self.studio_playing
        if self.studio_playing:self.studio_record.set(False)
    def _studio_seek(self,value):
        frames=self.studio_recorder.frames
        if frames:
            i=max(0,min(len(frames)-1,int(float(value))))
            if self._studio_auto_seek==i:self._studio_auto_seek=None;return
            self.studio_play_cursor=frames[i]['t'];self.studio_play_frame=frames[i]
            self.draw_map()
    def _studio_save_alarms(self):
        path=filedialog.asksaveasfilename(defaultextension='.json',initialfile='alarms.json')
        if path:self.guarded(lambda:Path(path).write_text(json.dumps(self.studio_alarms,ensure_ascii=False,indent=2),encoding='utf-8'))

    def _studio_alarm(self,key,active,message):
        if active and key not in self._studio_alarm_active:
            stamp=time.strftime('%Y-%m-%d %H:%M:%S');record=dict(time=stamp,level=key,message=message,end='')
            self.studio_alarms.append(record);self._studio_alarm_active[key]=record
            if len(self.studio_alarms)>3000:self.studio_alarms=self.studio_alarms[-3000:]
            self.log('ALARM',message)
        elif not active and key in self._studio_alarm_active:self._studio_alarm_active.pop(key)['end']=time.strftime('%Y-%m-%d %H:%M:%S')

    def _studio_charge_tick(self,now):
        runner=self.studio_runner;cfg=self.studio_config['auto_charge'];state=self.current_state()
        phase=self._studio_real_charge_phase
        if phase=='IDLE':
            if not cfg['enabled'] or getattr(self,'studio_charge_inhibit',False) or not self.connected or not self.control_enabled:return False
            if not runner.active or runner.status!='RUNNING' or runner.entered:return False
            if float(state.get('battery',100))>cfg['low']:return False
            if abs(float(state.get('speed',0)))>.02:return False
            docks=[(math.hypot(n['x']-state['x'],n['y']-state['y']),key) for key,n in self.map.nodes.items()
                   if key in self.robot_stations and (n.get('kind')=='dock' or 'charge' in str(n.get('className','')).lower() or key.upper().startswith('CP'))]
            if not docks:
                runner.error='자동 충전: 로봇에서 조회된 충전 노드가 없습니다.';runner.status='FAILED';return False
            goal=min(docks)[1];self._studio_real_charge_saved=goal
            self.send_command('navigate',self._station_nav_payload(self.map.nodes[goal]))
            self._studio_real_charge_phase='TO_DOCK';self._studio_real_charge_at=now
            self.log('CHARGE','자동 충전 이동 → '+goal);return True
        if not self.connected or not self.control_enabled or now-self.last_state>3:
            runner.error='자동 충전 중 연결/제어권 소실';runner.status='FAILED';self._studio_real_charge_phase='IDLE';return False
        if state.get('task') in ('FAILED','CANCELED'):
            runner.error='충전 이동 실패: '+str(state.get('task'));runner.status='FAILED';self._studio_real_charge_phase='IDLE';return False
        if phase=='TO_DOCK' and state.get('charging') is True:
            self._studio_real_charge_phase='CHARGING';self._studio_real_charge_at=now
            self.log('CHARGE','실기 charging 확인')
        elif phase=='CHARGING' and float(state.get('battery',0))>=cfg['high']:
            self._studio_real_charge_phase='IDLE';self.log('CHARGE','충전 완료 · 다음 미션 단계 재개');return False
        if now-self._studio_real_charge_at>(180 if phase=='TO_DOCK' else 3600):
            runner.error='충전 이동/충전 확인 시간 초과';runner.status='FAILED';runner.adapter.cancel()
            self._studio_real_charge_phase='IDLE';return False
        return True

    def _studio_tick(self,now,dt):
        if not hasattr(self,'studio_runner'):return
        self._fr5_tick(now)
        while True:
            try:token,ok,result=self.studio_bridge.results.get_nowait()
            except queue.Empty:break
            if self.studio_token_generation.pop(token,None)!=self.generation:
                self.studio_jobs.pop(token,None);continue
            if token in self.studio_jobs:
                generation,callback=self.studio_jobs.pop(token)
                if generation!=self.generation:continue
                if ok:
                    try:callback(result)
                    except Exception as e:self.log('ERROR','장비 응답 처리 실패: '+str(e))
                else:self.log('ERROR','장비 요청 실패: '+str(result))
            else:self.studio_results[token]=(ok,result)
        while len(self.studio_results)>128:self.studio_results.pop(next(iter(self.studio_results)))
        if self.map is not self._studio_map:
            self.studio_history=EditHistory();self._studio_map=self.map
            self._studio_snapshot=self.studio_history.capture(self.map);self._studio_apply_sim_settings()
            self._studio_edit_pending=None
            self.curve_edit_record=None;self.curve_drag_index=None;self.studio_point_drag=None
            self.studio_polygon=[];self.studio_wall_start=None
        dragging=self.curve_drag_index is not None or self.heading_drag_key or self.studio_point_drag or self._studio_erasing
        if not dragging and self._studio_edit_pending is not None:
            self.studio_history.commit(self._studio_edit_pending,self.map,'지도 편집')
            self._studio_edit_pending=None
        runner=self.studio_runner
        if not self.real:
            allowed=not runner.active or runner.actions[runner.index]['type']=='Path Nav'
            self.sim.auto_charge['enabled']=self.studio_config['auto_charge']['enabled'] and allowed and not self.studio_charge_inhibit
        charge_busy=self._studio_charge_tick(now) if self.real else self.sim.auto_charge['phase']!='IDLE'
        if runner.status=='RUNNING':
            if charge_busy:
                runner.started+=dt
                if runner.dwell_until is not None:runner.dwell_until+=dt
            else:runner.tick(now,dt)
        if runner.status!=self._studio_runner_status:
            self._studio_runner_status=runner.status;self.log('MISSION','미션 '+runner.status+(' · '+runner.error if runner.error else ''))
            if runner.status in ('FAILED','COMPLETED','CANCELED'):self.task_running=False
        self._studio_alarm('MISSION',runner.status=='FAILED',runner.error)
        state=self.current_state()
        self._studio_alarm('BLOCKED',bool(state.get('blocked')),getattr(self.sim,'block_reason','') or '장애물 정지')
        self._studio_alarm('ESTOP',bool(state.get('emergency') or state.get('stopped')),'비상/정지 상태')
        self._studio_alarm('CONNECTION',not self.connected,'로봇 연결 끊김')
        self._studio_alarm('BATTERY',float(state.get('battery',100))<20,'배터리 부족')
        for key,title in [('errors','로봇 오류'),('warnings','로봇 경고'),('fatals','로봇 치명 오류')]:
            value=state.get(key)
            if value is None:
                value=(self.raw.get('all',{}) if isinstance(self.raw,dict) else {}).get(key,[])
            self._studio_alarm(key,bool(value),title+': '+str(value))
        if self.real and now-self._studio_io_poll_at>1 and 'read_io' in self.studio_config['peripherals'].get('operations',{}):
            self._studio_io_poll_at=now
            try:self._studio_read_io()
            except Exception:pass
        self.studio_recorder.recording=self.studio_record.get()
        if now-self._studio_record_at>=.2:
            self._studio_record_at=now
            if all(k in state for k in ('x','y','theta')):self.studio_recorder.append(now,state,dict(map=self.map.name,mission=runner.status,blocked=state.get('blocked')))
        if self.studio_playing and self.studio_recorder.frames:
            frames=self.studio_recorder.frames;self.studio_play_cursor+=dt*float(self.studio_play_speed.get())
            idx=max(0,bisect.bisect_right([f['t'] for f in frames],self.studio_play_cursor)-1)
            idx=min(idx,len(frames)-1);self.studio_play_frame=frames[idx]
            if int(self.studio_timeline.get())!=idx:self._studio_auto_seek=idx;self.studio_timeline.set(idx)
            if idx==len(frames)-1:self.studio_playing=False
        if now-self._studio_draw_at<.3:return
        self._studio_draw_at=now
        total='∞' if runner.repeat==0 else str(runner.repeat)
        text=f'{runner.status} · {runner.cycle+1}/{total}회 · {runner.index+1}단계 · {runner.elapsed:.1f}s'+(' · '+runner.error if runner.error else '')
        if runner.skipped:text+=f' · 목적지 패스 {len(runner.skipped)}건'
        self.studio_mission_label.config(text=text)
        if runner.active or runner.status in ('FAILED','COMPLETED'):self.mission_status.config(text=text[:110])
        self.studio_action_tree.delete(*self.studio_action_tree.get_children())
        for i,a in enumerate(runner.actions):self.studio_action_tree.insert('','end',values=(i+1,a['type'],a.get('goal',a.get('operation','')),a['status'],a['timeout_s']))
        for i in range(64):
            di=self._studio_io_value('di',i);do=self._studio_io_value('do',i)
            self.studio_io_tree.item(str(i),values=(i,'—' if di is None else 'ON' if di else 'OFF','—' if do is None else 'ON' if do else 'OFF'))
        phase=self.sim.auto_charge['phase'] if not self.real else getattr(self,'_studio_real_charge_phase','IDLE')
        self.studio_charge_label.config(text=f"{'REAL' if self.real else 'SIM'} · {phase} · 배터리 {state.get('battery','—')}")
        if not self.real:arm_text=json.dumps(self.sim.arm,ensure_ascii=False)
        elif self.studio_config['arm'].get('driver')=='fairino':
            fb=self.fr5_feedback;fresh=self.fr5_client.connected and now-self.fr5_rx<=2
            arm_text='FR5 · '+('수신 중' if fresh else '연결 / 수신 대기')+' · '+fb.get('status','UNKNOWN')+' · '+self.fr5_error
            if fresh:arm_text+='\nJ1~J6 [도]: '+', '.join(f'{v:.2f}' for v in fb.get('joints_deg',[]))+'\nTCP [mm/도]: '+', '.join(f'{v:.2f}' for v in fb.get('tcp_mm_deg',[]))
        else:arm_text='REAL · 설정된 XML-RPC 완료 상태를 확인합니다.'
        self.studio_arm_label.config(text=arm_text)
        self.studio_record_label.config(text=f'기록 {len(self.studio_recorder.frames)} 프레임 · 재생 {"ON" if self.studio_playing else "OFF"}')
        self.studio_timeline.config(to=max(0,len(self.studio_recorder.frames)-1))
        self.studio_alarm_tree.delete(*self.studio_alarm_tree.get_children())
        for alarm in self.studio_alarms[-300:]:self.studio_alarm_tree.insert('','end',values=tuple(alarm[k] for k in ('time','level','message','end')))
        diag=dict(display_map=self.map.name,simulation_map=self.sim.map.name,same_map=self.map is self.sim.map,
                  route=self.sim.route,navigation_points=len(self.sim.navigation_points()),applied_limits=self.sim.applied_limits,
                  blocked_reason=self.sim.block_reason,source_version='studio-v1.0')
        self.studio_diag_text.delete('1.0','end');self.studio_diag_text.insert('1.0',json.dumps(diag,ensure_ascii=False,indent=2))

    def _studio_draw(self,c):
        if not hasattr(self,'studio_runner'):return
        for goal in (self.sim.skipped_goals if not self.real else []):
            if goal not in self.map.nodes:continue
            node=self.map.nodes[goal];x,y=self.xy(node['x'],node['y'])
            c.create_oval(x-13,y-13,x+13,y+13,outline='#c93043',width=3,tags='skipped_goal')
            c.create_line(x-8,y-8,x+8,y+8,fill='#c93043',width=3,tags='skipped_goal')
            c.create_line(x-8,y+8,x+8,y-8,fill='#c93043',width=3,tags='skipped_goal')
            c.create_text(x,y+23,text=goal+' · 도달 불가 / 패스',fill='#c93043',font=(self.font,8,'bold'),tags='skipped_goal')
        if not self.real:
            s=self.sim.state;cfg=self.map.robot_model;scale=self._view_transform[0]
            points=[]
            for dx,dy in [(cfg['length']/2,cfg['width']/2),(cfg['length']/2,-cfg['width']/2),(-cfg['length']/2,-cfg['width']/2),(-cfg['length']/2,cfg['width']/2)]:
                points.extend(self.xy(s.x+dx*math.cos(s.theta)-dy*math.sin(s.theta),s.y+dx*math.sin(s.theta)+dy*math.cos(s.theta)))
            c.create_polygon(*points,fill='',outline='#2764e7',width=2)
            x,y=self.xy(s.x,s.y);r=cfg['radius']*scale
            c.create_oval(x-r,y-r,x+r,y+r,outline='#799aca',dash=(3,3))
        for x1,y1,x2,y2 in (getattr(self.map,'virtual_walls',[]) if self.layers['벽'].get() else []):c.create_line(*self.xy(x1,y1),*self.xy(x2,y2),fill='#ee4c70',width=4,dash=(6,3),tags='studio_virtual_wall')
        for obs in (getattr(self.map,'obstacles',[]) if self.layers['장애물'].get() else []):
            x,y=self.xy(obs['x'],obs['y']);scale=self._transform()[0];r=obs['radius']*scale
            if obs.get('dynamic'):
                from .dynamic_obstacles import compile_actor_path
                try:path,_=compile_actor_path(self.map,obs)
                except (ValueError,KeyError):path=[]
                if len(path)>1:c.create_line(*[v for p in path for v in self.xy(*p)],fill='#9e96bb',dash=(4,4),tags='studio_actor_route')
                color='#de9535' if obs['kind']=='person' else '#8069bd'
                if obs['kind']=='person':
                    c.create_oval(x-r,y-r,x+r,y+r,fill=color,outline='#ffffff',width=2,tags='studio_obstacle')
                    c.create_oval(x-4,y-8,x+4,y,fill='#fff1d0',outline='',tags='studio_obstacle')
                    c.create_line(x,y,x,y+8,x-5,y+12,x,y+8,x+5,y+12,fill='#fff1d0',width=2,tags='studio_obstacle')
                else:
                    angle=obs.get('_heading',0);points=[]
                    for dx,dy in ((r*.8,r*.55),(r*.8,-r*.55),(-r*.8,-r*.55),(-r*.8,r*.55)):
                        points.extend((x+dx*math.cos(angle)-dy*math.sin(angle),y-dx*math.sin(angle)-dy*math.cos(angle)))
                    c.create_polygon(*points,fill=color,outline='#ffffff',width=2,tags='studio_obstacle')
                c.create_text(x,y-r-9,text=obs.get('id','')+' · '+obs.get('_motion','준비'),fill=color,font=('Malgun Gothic',8),tags='studio_obstacle')
            else:c.create_oval(x-r,y-r,x+r,y+r,fill='#ce4053',outline='#ffffff',width=2,tags='studio_obstacle')
        if not self.real and self.layers['장애물'].get() and self.show_tracks.get():
            from .obstacle_tracking import CLASSES
            for record in self.sim.obstacle_tracker.records():
                color='#84919c' if record['state']=='미관측' else {'dynamic':'#2196c7','static':'#d14b54','unknown':'#e9a237'}[record['kind']]
                trail=list(record['trail'])
                if len(trail)>1:c.create_line(*[v for point in trail for v in self.xy(*point)],fill=color,width=2,dash=(3,3),tags='tracked_obstacle_trail')
                x,y=self.xy(record['x'],record['y']);r=(record['radius']+.10)*self._transform()[0]
                c.create_rectangle(x-r,y-r,x+r,y+r,outline=color,width=2,dash=(5,3),tags='tracked_obstacle')
                c.create_text(x,y-r-24,text=record['id']+' · '+CLASSES[record['kind']]+' · '+record['state'],fill=color,font=(self.font,8,'bold'),tags='tracked_obstacle')
        draft=getattr(self,'actor_draft',None)
        if draft and draft.get('start'):
            x,y=self.xy(*draft['start']);c.create_oval(x-6,y-6,x+6,y+6,outline='#de9535',width=3)
            c.create_text(x,y-16,text='동적 장애물 시작 · 끝점을 클릭',fill='#99691e',anchor='s')
        if len(self.studio_polygon)>1:c.create_line(*[v for p in self.studio_polygon for v in self.xy(*p)],fill='#ffb347',width=3)
        for p in self.studio_polygon:
            x,y=self.xy(*p);c.create_rectangle(x-4,y-4,x+4,y+4,fill='#ffb347')
        if self.studio_wall_start:
            x,y=self.xy(*self.studio_wall_start);c.create_oval(x-5,y-5,x+5,y+5,fill='#ee4c70')
        if self.studio_diagnostics.get():
            # Forward/reverse records share a geometry. Printing every record's
            # text at its midpoint produced overlapping, apparently broken IDs.
            rec=self._selected_path_record() if c is self.editor_canvas and getattr(self,'editor_selection_kind','node')=='path' else None
            if rec:
                text=f"선택 경로 {rec['a']} → {rec['b']} · 최대 속도 {(rec.get('properties') or {}).get('maxspeed',self.map.robot_model['maxspeed'])} m/s"
                self._map_notice(text,'selected_path_diagnostic',30)
            if not self.real:
                self._map_notice(f"SIM 경로 {len(self.sim.navigation_points())}점 · 적용 V≤{self.sim.applied_limits['maxspeed']:.2f}m/s · {self.sim.block_reason}",'sim_diagnostic',20,True)
        if self.studio_play_frame:
            s=self.studio_play_frame['state'];x,y=self.xy(s['x'],s['y'])
            c.create_oval(x-12,y-12,x+12,y+12,fill='#b870ea',outline='#ffffff',width=2)
            c.create_line(x,y,x+25*math.cos(s['theta']),y-25*math.sin(s['theta']),fill='#b870ea',width=3,arrow='last')
            c.create_text(x,y-22,text='REPLAY',fill='#8e39c4',font=(self.font,9,'bold'))
