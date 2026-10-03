"""Main workspace policy and editable moving SIM actors."""
import math
import tkinter as tk
from tkinter import ttk
from .dynamic_obstacles import POLICIES,validate_actor
from .studio_core import collision_reason
from .theme import PANEL,MUTED,ORANGE


class ObstacleUIMixin:
    def _obstacles_controls(self,right):
        frame,body=self.card(right,'주행 장애물 대응 · SIM')
        frame.pack(fill='x',pady=(0,8))
        self.sim_obstacle_policy=tk.StringVar(value=POLICIES['wait'])
        choice=ttk.Combobox(body,textvariable=self.sim_obstacle_policy,values=list(POLICIES.values()),state='readonly')
        choice.pack(fill='x');choice.bind('<<ComboboxSelected>>',lambda event:self._sim_obstacle_change())
        row=tk.Frame(body,bg=PANEL);row.pack(fill='x',pady=2)
        self.reroute_wait=tk.StringVar(value='5');self.reroute_attempts=tk.StringVar(value='3')
        self.label(row,'다른 길 대기(초)',8,bg=PANEL).pack(side='left')
        ttk.Spinbox(row,from_=0,to=60,textvariable=self.reroute_wait,width=4).pack(side='left',padx=2)
        self.label(row,'재탐색',8,bg=PANEL).pack(side='left')
        ttk.Spinbox(row,from_=1,to=10,textvariable=self.reroute_attempts,width=3).pack(side='left',padx=2)
        self.button(row,'적용',lambda:self.guarded(self._reroute_settings)).pack(side='left')
        self.sim_avoidance_label=self.label(body,'벽과 장애물은 통과할 수 없습니다.',8,MUTED,anchor='w',justify='left',wraplength=285)
        self.sim_avoidance_label.pack(fill='x',pady=3)
        self.skipped_label=self.label(body,'패스 목적지: 없음',8,'#c93043',anchor='w',justify='left',wraplength=270)
        self.skipped_label.pack(fill='x')
        self.button(body,'패스 이력 보기 / 마킹 초기화',self._skipped_history).pack(fill='x',pady=2)
        self.button(body,'장애물 정지 해제 / 주행 재개',lambda:self.guarded(self._obstacles_resume)).pack(fill='x')
        row=tk.Frame(body,bg=PANEL);row.pack(fill='x',pady=3)
        for title,kind in [('사람 추가','person'),('AMR 추가','amr')]:
            self.button(row,title,lambda k=kind:self.guarded(lambda:self._actor_dialog(k))).pack(side='left',expand=True,fill='x',padx=1)
        self.button(row,'관리',lambda:self.guarded(self._actors_manage)).pack(side='left',padx=1)
        self.dynamic_paused=tk.BooleanVar(value=False)
        tk.Checkbutton(body,text='동적 장애물 일시정지',variable=self.dynamic_paused,bg=PANEL,command=self._actors_pause).pack(anchor='w')
        self.label(body,'SIM 전용 · 실기 장애물 정책은 제어기에서 설정',8,MUTED,anchor='w').pack(fill='x')
        row=tk.Frame(body,bg=PANEL);row.pack(fill='x',pady=3)
        for title,mode in [('벽 추가','벽 만들기'),('고정 장애물 추가','SIM 장애물')]:
            self.button(row,title,lambda m=mode:(self.tabs.select(self.nodes_page),self.edit_mode.set(m))).pack(side='left',expand=True,fill='x',padx=1)

    def _obstacles_restore(self):
        self.sim_obstacle_policy.set(POLICIES.get(self.studio_config.get('sim_obstacle_policy'),POLICIES['wait']))
        self.dynamic_paused.set(bool(self.studio_config.get('dynamic_paused',False)))
        self.actor_draft=None
        self.reroute_wait.set(str(self.studio_config.get('reroute_wait_s',5)))
        self.reroute_attempts.set(str(self.studio_config.get('reroute_attempts',3)))

    def _reroute_settings(self):
        wait=float(self.reroute_wait.get());attempts=int(self.reroute_attempts.get())
        if not math.isfinite(wait) or not 0<=wait<=60 or not 1<=attempts<=10:raise ValueError('대기 0~60초, 재탐색 1~10회')
        self.studio_config.update(reroute_wait_s=wait,reroute_attempts=attempts)
        self.sim.reroute_wait_s=wait;self.sim.reroute_attempt_limit=attempts;self._studio_save_settings()

    def _skipped_history(self):
        win=tk.Toplevel(self);win.title('도달 불가 목적지 · 패스 이력');win.geometry('740x360')
        tree=ttk.Treeview(win,columns=('goal','reason'),show='headings',height=10)
        tree.heading('goal',text='패스 목적지');tree.heading('reason',text='이유');tree.column('goal',width=100);tree.column('reason',width=580)
        tree.pack(fill='both',expand=True,padx=8,pady=8)
        for record in self.sim.skipped_goals.values():tree.insert('','end',values=(record['goal'],record['reason']))
        def clear():
            self.sim.skipped_goals.clear();self.studio_runner.skipped.clear();self.draw_map();win.destroy()
        self.button(win,'이력 / 지도 마킹 초기화',clear).pack(pady=6)

    def _obstacle_policy_key(self):
        return next((key for key,value in POLICIES.items() if value==self.sim_obstacle_policy.get()),
            {'우회 주행':'avoid','정지 / 대기':'wait'}.get(self.sim_obstacle_policy.get(),'wait'))

    def _actors_pause(self):
        self.map.dynamic_paused=self.dynamic_paused.get()
        self.studio_config['dynamic_paused']=self.dynamic_paused.get();self._studio_save_settings()

    def _obstacles_resume(self):
        self.sim_required()
        self.sim._obstacle_latched=False;self.sim._collision_blocked=False
        self.sim.state.blocked=False;self.sim.block_reason='';self.sim._avoid_next=0.
        self.sim.avoidance_status='재개 요청 · 장애물 다시 확인'

    def _actor_dialog(self,kind='person',start=None,end=None,existing=None):
        self.sim_required()
        state=self.sim.state
        start=start or [state.x+2,state.y-2];end=end or [state.x+2,state.y+2]
        values=dict(x1=start[0],y1=start[1],x2=end[0],y2=end[1],speed_mps=.6 if kind=='person' else .35,
            radius=.3 if kind=='person' else .61,dwell_s=.5)
        if existing:
            kind=existing['kind'];start,end=existing['motion_path']
            values.update(x1=start[0],y1=start[1],x2=end[0],y2=end[1],**{k:existing.get(k,values[k]) for k in ('speed_mps','radius','dwell_s')})
        win=tk.Toplevel(self);win.title('동적 장애물 · '+('사람' if kind=='person' else 'AMR'));win.configure(bg=PANEL)
        self.label(win,'시작 ↔ 끝 왕복 이동 · 벽/로봇 앞에서는 통행 대기',9,MUTED,bg=PANEL).pack(padx=12,pady=10)
        variables={k:tk.StringVar(value=str(v)) for k,v in values.items()}
        for key,title in [('x1','시작 X (m)'),('y1','시작 Y (m)'),('x2','끝 X (m)'),('y2','끝 Y (m)'),('speed_mps','속도 (m/s)'),('radius','충돌 반경 (m)'),('dwell_s','끝점 대기 (초)')]:
            row=self._studio_row(win);self.label(row,title,bg=PANEL,width=18,anchor='w').pack(side='left')
            ttk.Entry(row,textvariable=variables[key],width=16).pack(side='left')
        def save():
            self.sim_required();v={k:float(var.get()) for k,var in variables.items()}
            if not all(math.isfinite(n) for n in v.values()) or not .1<=v['radius']<=2:raise ValueError('유한 숫자와 충돌 반경 0.1~2m가 필요합니다.')
            first=[v['x1'],v['y1']];last=[v['x2'],v['y2']]
            prefix='PERSON' if kind=='person' else 'AMR'
            ids={o.get('id') for o in self.map.obstacles};index=1
            while f'{prefix}{index}' in ids:index+=1
            obs=dict(id=existing['id'] if existing else f'{prefix}{index}',dynamic=True,kind=kind,
                x=first[0],y=first[1],motion_path=[first,last],radius=v['radius'],speed_mps=v['speed_mps'],dwell_s=v['dwell_s'])
            validate_actor(obs)
            from types import SimpleNamespace
            scene=SimpleNamespace(walls=self.map.walls,virtual_walls=self.map.virtual_walls,area_records=self.map.area_records,
                obstacles=[o for o in self.map.obstacles if o is not existing])
            if collision_reason(scene,*first,obs['radius']) or math.dist(first,(state.x,state.y))<=obs['radius']+max(self.sim.collision_radius,self.map.robot_model['radius']):
                raise ValueError('시작점이 벽/장애물/제어 AMR와 겹칩니다. 시작점을 옮기세요.')
            def change():
                if existing:self.map.obstacles.remove(existing)
                self.map.obstacles.append(obs)
            self._studio_obstacle_edit(change,'동적 장애물 설정');self.draw_map();win.destroy()
        row=self._studio_row(win);self.button(row,'저장 / 배치',lambda:self.guarded(save),ORANGE).pack(side='left')
        def pick():
            self.actor_draft=dict(kind=kind,start=None)
            self.tabs.select(self.nodes_page);self.edit_mode.set('동적 장애물 배치');win.destroy()
        self.button(row,'지도에서 시작·끝 선택',pick).pack(side='left',padx=5)

    def _actor_map_click(self,x,y):
        self.sim_required()
        if not self.actor_draft:self.actor_draft=dict(kind='person',start=None)
        if self.actor_draft['start'] is None:self.actor_draft['start']=[x,y]
        else:
            draft=self.actor_draft;self.actor_draft=None
            self._actor_dialog(draft['kind'],draft['start'],[x,y])

    def _actors_manage(self):
        self.sim_required();win=tk.Toplevel(self);win.title('동적 장애물 관리');win.geometry('640x340')
        tree=ttk.Treeview(win,columns=('kind','speed','status'),show='tree headings',height=9)
        tree.heading('#0',text='ID');tree.heading('kind',text='종류');tree.heading('speed',text='m/s');tree.heading('status',text='현재 상태')
        tree.column('#0',width=100);tree.column('kind',width=70);tree.column('speed',width=70);tree.column('status',width=140)
        tree.pack(fill='both',expand=True,padx=8,pady=8)
        def refresh():
            if not win.winfo_exists():return
            selected=tree.selection();tree.delete(*tree.get_children())
            for obs in self.map.obstacles:
                if obs.get('dynamic'):tree.insert('','end',iid=obs['id'],text=obs['id'],values=('사람' if obs['kind']=='person' else 'AMR',obs['speed_mps'],obs.get('_motion','준비')))
            for item in selected:
                if tree.exists(item):tree.selection_add(item)
            win.after(500,refresh)
        def selected():
            keys=tree.selection()
            if not keys:raise ValueError('동적 장애물을 선택하세요.')
            return next(o for o in self.map.obstacles if o.get('id')==keys[0])
        def edit():
            obs=selected();self._actor_dialog(existing=obs)
        def remove():
            obs=selected();self._studio_obstacle_edit(lambda:self.map.obstacles.remove(obs),'동적 장애물 삭제');self.draw_map()
        row=self._studio_row(win)
        self.button(row,'선택 수정',lambda:self.guarded(edit)).pack(side='left',padx=3)
        self.button(row,'선택 삭제',lambda:self.guarded(remove)).pack(side='left',padx=3)
        refresh()
