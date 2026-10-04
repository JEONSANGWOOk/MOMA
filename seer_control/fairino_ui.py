"""FR5 connection, taught tasks, feedback and mission hooks."""
import copy
import json
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from .fairino_api import profile, validate
from .fairino_client import FairinoClient
from .fairino_program_ui import FairinoProgramMixin
from .arm_workspace import ArmWorkspaceMixin
from .fairino_programs import program_actions
from .studio_devices import arm_call
from .theme import PANEL, INK, MUTED, RED


class FairinoUIMixin(FairinoProgramMixin,ArmWorkspaceMixin):
    def _fr5_init(self):
        self.fr5_client=FairinoClient();self.fr5_feedback={};self.fr5_rx=0.;self.fr5_poll_at=0.
        self.fr5_pending=False;self.fr5_events=queue.Queue();self.fr5_epoch=0;self.fr5_error=''
        if self.studio_config['arm'].get('driver')=='fairino':self.studio_arm_safe=False

    def _arm_call(self,config,kind,operation=None):
        if config.get('driver')=='fairino':return self.fr5_client.call(config,kind,operation)
        return arm_call(config,kind,operation)

    def _fr5_receive(self,result):
        self.fr5_feedback=result;self.fr5_rx=time.monotonic();self.fr5_error=''
        cfg=self.studio_config['arm'];safe=cfg.get('operations',{}).get('safe_pose',{})
        self.studio_arm_safe=(result.get('status') not in ('ERROR','RUNNING','CANCELED','PAUSED')
            and result.get('motion_done')==1 and safe.get('method')=='MoveJ'
            and len(safe.get('target',[]))==6
            and all(abs(a-b)<=.5 for a,b in zip(result.get('joints_deg',[]),safe['target']))
            and len(result.get('joints_deg',[]))==6)
        if result.get('status')=='ERROR':self.studio_arm_safe=False

    def _fr5_queue(self,fn,connect=False):
        if self.fr5_pending:raise ValueError('FR5 요청 처리 중입니다.')
        self.fr5_pending=True;epoch=self.fr5_epoch
        def work():
            try:self.fr5_events.put((epoch,True,fn()))
            except Exception as e:self.fr5_events.put((epoch,False,str(e)))
        threading.Thread(target=work,daemon=True,name='fairino-ui-request').start()

    def _fr5_connect(self):
        if not self.real:raise ValueError('REAL 모드로 전환한 뒤 FR5를 연결하세요. SIM에서는 실기로 전송하지 않습니다.')
        if self.studio_runner.active:raise ValueError('미션 종료 후 연결하세요.')
        cfg=copy.deepcopy(self.studio_config['arm'])
        if cfg.get('driver')!='fairino':raise ValueError('FR5 설정을 먼저 저장하세요.')
        self.studio_arm_safe=False;self.fr5_feedback={};self.fr5_rx=0
        self._fr5_queue(lambda:self.fr5_client.connect(cfg),True)

    def _fr5_stop(self):
        if not self.real:return
        arm_active=self.studio_runner.active and (self.studio_runner.adapter.context.get('type')=='Arm Action' or self.studio_runner.adapter.context.get('standalone'))
        if self.studio_runner.active:self.studio_runner.cancel()
        if not arm_active:self._fr5_priority_stop()

    def _fr5_priority_stop(self):
        self.fr5_epoch+=1;self.fr5_pending=True;self.fr5_rx=0;self.studio_arm_safe=False
        epoch=self.fr5_epoch
        def stop():
            try:self.fr5_events.put((epoch,True,self.fr5_client.stop()))
            except Exception as e:self.fr5_events.put((epoch,False,'FR5 정지 확인 실패: '+str(e)))
        threading.Thread(target=stop,daemon=False,name='fairino-stop').start()

    def _fr5_tick(self,now):
        while True:
            try:epoch,ok,result=self.fr5_events.get_nowait()
            except queue.Empty:break
            if epoch!=self.fr5_epoch:continue
            self.fr5_pending=False
            if ok and 'joints_deg' in result:self._fr5_receive(result)
            elif ok:self.fr5_error='정지 명령 완료 · 연결 시험으로 다시 연결하세요.'
            else:
                self.fr5_error=str(result);self.fr5_rx=0;self.studio_arm_safe=False
                self.log('ERROR',self.fr5_error)
        cfg=self.studio_config['arm']
        if cfg.get('driver')!='fairino':return
        if not self.real:
            if self.fr5_client.connected:
                self._fr5_priority_stop()
            return
        if now-self.fr5_rx>2 or not self.fr5_client.connected:self.studio_arm_safe=False
        # Mission polls its own feedback; idle refresh is 5 Hz and never blocks Tk.
        if self.fr5_client.connected and not self.fr5_pending and (self.studio_runner.status!='RUNNING' or self.studio_runner.actions[self.studio_runner.index]['type']=='Wait') and now-self.fr5_poll_at>=.2:
            self.fr5_poll_at=now
            self._fr5_queue(lambda:self._arm_call(copy.deepcopy(cfg),'status'))

    def _fr5_controls(self,page):
        row=self._studio_row(page)
        self.button(row,'FR5 작업 개발',lambda:self.guarded(self._fr5_development)).pack(side='left',padx=3)
        self.button(row,'FR5 연결 설정',lambda:self.guarded(self._fr5_dialog)).pack(side='left',padx=3)
        self.button(row,'FR5 연결 시험 / 관절 수신',lambda:self.guarded(self._fr5_connect)).pack(side='left',padx=3)
        self.button(row,'FR5 작업 실행',lambda:self.guarded(self._fr5_run)).pack(side='left',padx=3)
        self.button(row,'FR5 정지',lambda:self.guarded(self._fr5_stop),RED).pack(side='left',padx=3)

    def _fr5_dialog(self):
        if self.studio_runner.active:raise ValueError('미션 종료 후 설정하세요.')
        cfg=copy.deepcopy(self.studio_config['arm']) if self.studio_config['arm'].get('driver')=='fairino' else profile()
        win=tk.Toplevel(self);win.title('FAIRINO FR5 공식 Python SDK');win.geometry('780x660');win.configure(bg=PANEL)
        vars={k:tk.StringVar(value=cfg.get(k,'')) for k in ('ip','sdk_path','controller_version')}
        for key,title in [('ip','FR5 제어기 IP'),('sdk_path','공식 SDK 폴더 (비우면 설치된 fairino 사용)'),('controller_version','제어기 WebAPP 버전')]:
            row=self._studio_row(win);self.label(row,title,bg=PANEL).pack(side='left',padx=4)
            ttk.Entry(row,textvariable=vars[key],width=48).pack(side='left',fill='x',expand=True)
            if key=='sdk_path':self.button(row,'찾기',lambda:vars['sdk_path'].set(filedialog.askdirectory(parent=win) or vars['sdk_path'].get())).pack(side='left')
        confirmed=tk.BooleanVar(value=cfg.get('version_confirmed',False));enabled=tk.BooleanVar(value=cfg.get('verified',False))
        tk.Checkbutton(win,text='제어기 버전에 맞는 공식 SDK를 설치하고 호환성을 확인함',variable=confirmed,bg=PANEL).pack(anchor='w',padx=12)
        tk.Checkbutton(win,text='등록한 작업의 실기 동작 허용',variable=enabled,bg=PANEL).pack(anchor='w',padx=12)
        self._studio_note(win,'작업 이름은 미션 Arm Action의 operation과 같습니다. MoveJ target: J1~J6 [도]. MoveL target: X,Y,Z [mm], RX,RY,RZ [도]. tool/user: 0~14, vel: 속도 %. 실제 안전 자세를 safe_pose라는 MoveJ 작업으로 등록하세요. 예제 자세를 실기로 자동 전송하지 않습니다.')
        box=self._studio_json_box(win,cfg.get('operations',{}),height=10)
        row=self._studio_row(win);name=tk.StringVar(value='safe_pose');method=tk.StringVar(value='MoveJ')
        ttk.Entry(row,textvariable=name,width=16).pack(side='left',padx=3)
        ttk.Combobox(row,textvariable=method,values=['MoveJ','MoveL'],state='readonly',width=8).pack(side='left')
        def teach():
            if time.monotonic()-self.fr5_rx>2:raise ValueError('FR5 연결 시험 후 현재 자세를 수신하세요.')
            if self.fr5_feedback.get('motion_done')!=1 or self.fr5_feedback.get('status')=='ERROR':raise ValueError('FR5가 정지하고 오류가 없어야 자세를 등록할 수 있습니다.')
            key=name.get().strip()
            if not key:raise ValueError('작업 이름을 입력하세요.')
            ops=self._studio_json(box);target=self.fr5_feedback['joints_deg' if method.get()=='MoveJ' else 'tcp_mm_deg']
            ops[key]=dict(method=method.get(),target=target,tool=0,user=0,vel=10)
            box.delete('1.0','end');box.insert('1.0',json.dumps(ops,ensure_ascii=False,indent=2))
        self.button(row,'수신한 현재 자세로 작업 등록',lambda:self.guarded(teach)).pack(side='left',padx=6)
        def save():
            value=dict(cfg);value.update(driver='fairino',**{k:v.get().strip() for k,v in vars.items()},verified=enabled.get(),version_confirmed=confirmed.get(),operations=self._studio_json(box))
            validate(value,motion=enabled.get())
            if self.fr5_client.connected:self._fr5_priority_stop()
            self.studio_config['arm']=value;self.studio_arm_safe=False;self._studio_save_settings()
            self.studio_device_box.delete('1.0','end');self.studio_device_box.insert('1.0',json.dumps(dict(peripherals=self.studio_config['peripherals'],arm=value),ensure_ascii=False,indent=2))
            win.destroy()
        row=self._studio_row(win);self.button(row,'설정 저장',lambda:self.guarded(save)).pack(side='left')
        self.label(row,'저장 후 연결 시험 → 현재 자세 확인 → 작업 실행',9,MUTED,bg=PANEL).pack(side='left',padx=8)

    def _fr5_run(self):
        if self.studio_runner.active:raise ValueError('미션 실행 중입니다.')
        cfg=self.studio_config['arm'];names=list(cfg.get('operations',{}))+list(cfg.get('programs',{}))
        if cfg.get('driver')!='fairino' or not names:raise ValueError('FR5 작업을 먼저 등록하세요.')
        if self.real and (not self.fr5_client.connected or time.monotonic()-self.fr5_rx>2):raise ValueError('FR5 연결 시험으로 현재 상태를 확인하세요.')
        win=tk.Toplevel(self);win.title('FR5 등록 작업 실행');var=tk.StringVar(value=names[0])
        ttk.Combobox(win,textvariable=var,values=names,state='readonly',width=24).pack(padx=12,pady=12)
        def run():
            operation=var.get()
            if self.real and not messagebox.askokcancel('FR5 실기 동작',f'{operation} 작업으로 FR5가 움직입니다. 등록한 목표와 주변 작업 공간을 확인하세요.',parent=win):return
            actions=program_actions(cfg,operation) if operation in cfg.get('programs',{}) else [dict(type='Arm Action',operation=operation,duration_s=2,timeout_s=60)]
            self.studio_runner.start(actions)
            self.task_running=True;win.destroy()
        self.button(win,'선택 작업 실행',lambda:self.guarded(run)).pack(pady=8)
