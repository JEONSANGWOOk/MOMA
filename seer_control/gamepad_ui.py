"""DualSense manual-control panel; same SIM and SEER jog transports."""
import math
import copy
import threading
import time
import tkinter as tk
from tkinter import ttk
from .gamepad import WindowsPad,PadGate,AXES
from .manual_safety import controller_manual_reason
from .arm_gamepad import GROUPS,sim_jog,sdk_pulse
from .theme import PANEL,MUTED,INK,BLUE


class GamepadMixin:
    def _pad_build(self,body):
        self.pad_backend=WindowsPad();self.pad_gate=PadGate();self.pad_device=None;self.pad_after=None
        self.pad_enabled=tk.BooleanVar(value=False)
        self.pad_config=dict(forward='Y',turn='X',deadman=4,stop=2,zone=.15,invert_forward=True,invert_turn=True)
        self.pad_sample=None;self.pad_arm_active=False;self.pad_arm_active_real=False;self.pad_arm_time=None
        self.pad_target=tk.StringVar(value='AMR');self.pad_arm_group=tk.StringVar(value='J1 / J2');self.pad_arm_real=tk.BooleanVar(value=False)
        tk.Checkbutton(body,text='DualSense 조이스틱 사용 (CFI-ZCT1G)',variable=self.pad_enabled,bg=PANEL,command=self._pad_toggle).pack(anchor='w',pady=(8,0))
        self._pad_mode_controls(body)
        self.pad_status=self.label(body,'컨트롤러 검색 후 수동 조작을 활성화하세요.',8,MUTED,justify='left',wraplength=285)
        self.pad_status.pack(fill='x',pady=3)
        row=tk.Frame(body,bg=PANEL);row.pack(fill='x')
        self.button(row,'컨트롤러 검색 / 설정',self._pad_dialog,BLUE).pack(side='left',fill='x',expand=True)
        self.label(body,'L1: 누르는 동안 조종 · ○: 정지\nAMR: 전후진/회전 · 팔: 선택한 두 축 조절\n팔 SIM: 12°/s · TCP 30mm/s, 10°/s',8,MUTED,justify='left',wraplength=285).pack(fill='x',pady=4)

    def _pad_mode_controls(self,body):
        row=tk.Frame(body,bg=PANEL);row.pack(fill='x',pady=3)
        self.label(row,'조종 대상',8,MUTED).pack(side='left')
        target=ttk.Combobox(row,textvariable=self.pad_target,values=['AMR','로봇팔'],state='readonly',width=7);target.pack(side='left',padx=3)
        target.bind('<<ComboboxSelected>>',lambda e:self._pad_toggle())
        group=ttk.Combobox(row,textvariable=self.pad_arm_group,values=list(GROUPS),state='readonly',width=13);group.pack(side='left')
        group.bind('<<ComboboxSelected>>',lambda e:self._pad_toggle())

    def _pad_arm_stop(self):
        self.pad_arm_time=None
        if not getattr(self,'pad_arm_active',False):return
        self.pad_arm_active=False
        if getattr(self,'pad_arm_active_real',False):
            def stop():
                try:self.fr5_client.jog_stop()
                except Exception as e:self.fr5_events.put((self.fr5_epoch,False,'팔 JOG 정지 확인 실패: '+str(e)))
            threading.Thread(target=stop,daemon=False,name='pad-arm-stop').start()

    def _pad_arm_process(self,sample,now):
        focus=self.focus_get();page=self.tabs.select()
        allowed=(self.pad_enabled.get() and self.manual.get() and focus is not None and focus.winfo_toplevel()==self and
                 page in (str(self.operation_page),str(self.arm_workspace_page)) and not self.reloc_mode and
                 not self.task_running and not self.studio_runner.active)
        if self.real:
            feedback=self.fr5_feedback;cfg=self.studio_config.get('arm',{});amr=self.current_state();speed=amr.get('speed')
            allowed=allowed and self.pad_arm_real.get() and self.connected and self.control_enabled and self.fr5_client.connected and cfg.get('verified') is True and cfg.get('version_confirmed') is True and bool(cfg.get('controller_version')) and now-self.fr5_rx<=2 and not controller_manual_reason(self.live,self.last_state,now) and type(speed) in (int,float) and math.isfinite(speed) and abs(speed)<.01 and feedback.get('status') not in ('ERROR','RUNNING','PAUSED') and not feedback.get('emergency') and not any(feedback.get('safety_stop',[])) and not any(feedback.get('errors',[]))
        else:allowed=allowed and self.sim_powered and not self.sim.route and not self.sim.state.stopped and self.sim.state.motor and self.arm_dev_sim.state not in ('RUNNING','PAUSED')
        try:v,w,stop,status=self.pad_gate.evaluate(sample,now,allowed,**self.pad_config)
        except (ValueError,TypeError):self.pad_gate.reset();v=w=0.;stop=False;status='팔 조이스틱 입력 오류'
        if self.pad_enabled.get() and sample and 0<=now-sample.timestamp<=.25 and focus is not None and focus.winfo_toplevel()==self:
            stop=stop or bool(sample.buttons&(1<<self.pad_config['stop']))
        if stop:
            self.pad_enabled.set(False);self._pad_stop();self.action('stop')
            if self.real:self._fr5_priority_stop()
            else:self.arm_dev_sim.stop();self._aw_sync_main()
        elif not v and not w:self._pad_arm_stop()
        else:
            try:
                if self.held:raise ValueError('AMR 버튼/키보드 조작을 먼저 해제하세요.')
                if self.real:
                    if hasattr(self,'arm_dev_display') and self.arm_dev_display.get()=='SIM 개발':self.arm_dev_display.set('실기 자세')
                    if not self.fr5_pending and feedback.get('motion_done')==1:
                        self.studio_arm_safe=False
                        pulse=sdk_pulse(self.pad_arm_group.get(),v,w,now);cfg=copy.deepcopy(self.studio_config['arm'])
                        self._fr5_queue(lambda:self._arm_call(cfg,'jog',pulse));self.pad_arm_active=True;self.pad_arm_active_real=True
                    status='FR5 단일 축 제한 JOG · '+self.pad_arm_group.get()
                else:
                    dt=.05 if self.pad_arm_time is None else min(.1,max(0,now-self.pad_arm_time))
                    sim_jog(self.arm_dev_sim,self.pad_arm_group.get(),v,w,dt)
                    self.pad_arm_time=now;self.pad_arm_active=True;self.pad_arm_active_real=False;self._aw_sync_main();self.arm_dev_canvas.render()
                    status='팔 SIM 조종 · '+self.pad_arm_group.get()
            except (ValueError,TypeError) as e:self._pad_arm_stop();self.pad_gate.reset();status=str(e)
        if not allowed and not stop:status='팔 조종 대기 · 수동/SIM 정지/팔 재생/실기 허용·수신 상태 확인'
        identity=self.pad_device[1].name if self.pad_device else '미연결'
        raw=' · '.join(f'{k} {value:+.2f}' for k,value in sample.axes.items()) if sample else ''
        self.pad_status.configure(text=f'{identity} · {status}\n{raw}')
        if hasattr(self,'pad_arm_feedback'):self.pad_arm_feedback.set(('REAL · ' if self.real else 'SIM · ')+status)
        if getattr(self,'pad_input_label',None) and self.pad_input_label.winfo_exists():self.pad_input_label.configure(text=self.pad_status.cget('text'))

    def _pad_start(self):
        devices=self.pad_backend.devices()
        sony=[device for device in devices if (device[1].manufacturer,device[1].product)==(0x054c,0x0ce6)]
        if len(sony)==1:self.pad_device=sony[0]
        saved=self.studio_config.get('gamepad',{})
        if isinstance(saved,dict):
            for k in self.pad_config:
                value=saved.get(k)
                if k in ('forward','turn') and value in AXES:self.pad_config[k]=value
                elif k in ('deadman','stop') and type(value) is int and 0<=value<32:self.pad_config[k]=value
                elif k=='zone' and type(value) in (int,float) and 0<=value<=.5:self.pad_config[k]=value
                elif k.startswith('invert') and type(value) is bool:self.pad_config[k]=value
        self.pad_after=self.after(50,self._pad_poll)

    def _pad_toggle(self):
        self._pad_stop();self.release_drive();self.pad_gate.reset()

    def _pad_stop(self):
        self._pad_arm_stop()
        if self.held and self.held[0]=='gamepad':self.release_drive()

    def _pad_poll(self):
        self.pad_after=None
        try:
            sample=None
            if self.pad_device:
                try:sample=self.pad_backend.read(*self.pad_device)
                except (OSError,ValueError):
                    self.pad_device=None;self.pad_enabled.set(False);self._pad_stop()
            self.pad_sample=sample;self._pad_process(sample,time.monotonic())
        finally:
            if self.winfo_exists():self.pad_after=self.after(50,self._pad_poll)

    def _pad_process(self,sample,now):
        if self.pad_target.get()=='로봇팔':return self._pad_arm_process(sample,now)
        focus=self.focus_get()
        allowed=(self.pad_enabled.get() and self.manual.get() and self.connected and
                 self.tabs.select()==str(self.operation_page) and focus is not None and focus.winfo_toplevel()==self and
                 not self.reloc_mode and not self.task_running and not self.studio_runner.active and self.studio_arm_safe)
        allowed=allowed and (not self.real and self.sim_powered or self.real and self.control_enabled and now-self.last_state<=3 and not controller_manual_reason(self.live,self.last_state,now))
        try:v,w,stop,status=self.pad_gate.evaluate(sample,now,allowed,**self.pad_config)
        except (ValueError,TypeError):
            self.pad_gate.reset();v,w,stop,status=0.,0.,False,'컨트롤러 입력 오류'
        if self.pad_enabled.get() and sample and 0<=now-sample.timestamp<=.25 and focus is not None and focus.winfo_toplevel()==self:
            stop=stop or bool(sample.buttons&(1<<self.pad_config['stop']))
        if stop:
            self.pad_enabled.set(False);self._pad_stop();self.action('stop')
        if not v and not w:self._pad_stop()
        else:
            try:
                maximum=float(self.speed.get());rotation=math.radians(float(self.angular.get()))
                if not math.isfinite(maximum+rotation) or not .01<=maximum<=.3 or not 0<rotation<=.6:raise ValueError('속도 설정 범위 확인')
                motion=(v*maximum,w*rotation)
                # A keyboard/mouse jog owns control until explicitly released.
                if self.held and self.held[0]!='gamepad':self.pad_gate.reset();status='키보드 / 버튼 조작 중'
                elif self.real:
                    self.held=('gamepad',motion)
                    with self._jog_lock:self._jog_desired=dict(vx=motion[0],vy=0.,w=motion[1],_expires=now+.25)
                else:
                    self.sim.drive(*motion);self.held=('gamepad',motion)
            except (ValueError,TypeError) as e:self._pad_stop();self.pad_gate.reset();status=str(e)
        if not allowed and sample and not stop:
            if not self.pad_enabled.get():status='조이스틱 사용을 켜세요.'
            elif not self.manual.get():status='수동 조작 활성화를 켜세요.'
            elif not self.connected:status='SIM 또는 실기에 먼저 연결하세요.'
            elif self.studio_runner.active or self.task_running:status='미션 종료 후 조종하세요.'
            elif not self.studio_arm_safe:status='로봇팔 안전 자세 확인'
            else:status='지도 / 제어 화면에서 연결·제어 상태를 확인하세요.'
        identity=self.pad_device[1].name if self.pad_device else '미연결'
        raw=''
        if sample:raw=' · '.join(f'{k} {value:+.2f}' for k,value in sample.axes.items())+'\n버튼: '+(', '.join(str(i+1) for i in range(32) if sample.buttons&(1<<i)) or '없음')
        self.pad_status.configure(text=f'{identity} · {status}\n{raw}')
        if getattr(self,'pad_input_label',None) and self.pad_input_label.winfo_exists():self.pad_input_label.configure(text=self.pad_status.cget('text'))

    def _pad_dialog(self):
        self._pad_stop();self.pad_gate.reset()
        win=tk.Toplevel(self);win.title('DualSense CFI-ZCT1G · 입력 / 설정');self._fit_dialog(win,720,550,620,460);win.configure(bg=PANEL)
        devices=self.pad_backend.devices()
        names=[f'{index}: {caps.name} · VID {caps.manufacturer:04X} / PID {caps.product:04X}' for index,caps in devices]
        selected=tk.StringVar(value=names[0] if names else '')
        self.label(win,'USB 또는 Bluetooth로 연결한 뒤 장치를 선택하세요.',11,INK,True).pack(anchor='w',padx=12,pady=10)
        ttk.Combobox(win,textvariable=selected,values=names,state='readonly',width=70).pack(fill='x',padx=12)
        def choose():
            self._pad_stop();self.pad_gate.reset();self.pad_enabled.set(False)
            self.pad_device=devices[names.index(selected.get())] if selected.get() in names else None
        self.button(win,'선택한 컨트롤러 연결',choose,BLUE).pack(anchor='w',padx=12,pady=5)
        self.pad_input_label=self.label(win,'장치 선택 후 스틱/버튼의 입력 번호를 확인하세요.',9,MUTED,justify='left');self.pad_input_label.pack(fill='x',padx=12,pady=5)
        variables={}
        for key,title,values in (('forward','전후진 축',AXES),('turn','회전 축',AXES),('deadman','주행 허용 L1 버튼 번호',tuple(range(1,33))),('stop','정지 ○ 버튼 번호',tuple(range(1,33)))):
            row=tk.Frame(win,bg=PANEL);row.pack(fill='x',padx=12,pady=3)
            value=self.pad_config[key]+1 if key in ('deadman','stop') else self.pad_config[key]
            variables[key]=tk.StringVar(value=str(value));self.label(row,title,9,INK,width=25,anchor='w').pack(side='left')
            ttk.Combobox(row,textvariable=variables[key],values=values,state='readonly',width=8).pack(side='left')
        row=tk.Frame(win,bg=PANEL);row.pack(fill='x',padx=12,pady=4)
        variables['zone']=tk.StringVar(value=str(self.pad_config['zone']));self.label(row,'데드존 (0~0.5)',9,INK,width=25,anchor='w').pack(side='left');ttk.Entry(row,textvariable=variables['zone'],width=9).pack(side='left')
        for key,title in (('invert_forward','전후진 축 반전'),('invert_turn','회전 축 반전')):
            variables[key]=tk.BooleanVar(value=self.pad_config[key]);tk.Checkbutton(win,text=title,variable=variables[key],bg=PANEL).pack(anchor='w',padx=12)
        def apply():
            config={k:v.get() for k,v in variables.items()};config['zone']=float(config['zone'])
            config['deadman']=int(config['deadman'])-1;config['stop']=int(config['stop'])-1
            if not 0<=config['zone']<=.5 or config['forward']==config['turn'] or config['deadman']==config['stop']:raise ValueError('축은 서로 다르게, 허용/정지 버튼도 다르게 설정하세요.')
            self._pad_stop();self.pad_gate.reset();self.pad_config=config
            self.studio_config['gamepad']=config;self._studio_save_settings();win.destroy()
        self.label(win,'설정 중에는 주행하지 않습니다. 창을 닫고 수동 조작 + 조이스틱 사용을 켠 뒤\n스틱 중앙에서 L1을 놓았다가 다시 누르세요. 재연결 시 다시 활성화해야 합니다.',9,MUTED,justify='left').pack(fill='x',padx=12,pady=7)
        tk.Checkbutton(win,text='이번 실행에서 FR5 실기 조이스틱 JOG 허용 (연결/버전/동작 허용 필요)',variable=self.pad_arm_real,bg=PANEL,command=self._pad_toggle).pack(anchor='w',padx=12)
        self.button(win,'적용 / 저장',lambda:self.guarded(apply),BLUE).pack(pady=4)
        return win
