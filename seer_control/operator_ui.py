"""Role-aware operator orders and developer recipe blocks on the shared engine."""
import copy,json,math,time,re
from pathlib import Path
import tkinter as tk
from tkinter import ttk
from .theme import PANEL,BLUE,RED,INK,MUTED
from .operator_missions import defaults,validate_template,compile_mission
from .voice_ui import VoiceMissionMap
from .operator_recipe_ui import RecipeMixin
from .fairino_api import validate as validate_fr5

class OperatorMixin(RecipeMixin):
 def _operator_build(self):
  self.role_active='개발자';self.ui_role=tk.StringVar(value='개발자');self.operator_plan=None;self.operator_pending=None;self.operator_owned=False;self.operator_after=None
  self.operator_templates=copy.deepcopy(self.studio_config.get('operator_templates') or defaults())
  if not any(t['name']=='현재 노드 왕복 테스트' for t in self.operator_templates):self.operator_templates.append(copy.deepcopy(next(t for t in defaults() if t['name']=='현재 노드 왕복 테스트')))
  self.role_selector=ttk.Combobox(self.role_header,textvariable=self.ui_role,values=['사용자','개발자'],state='readonly',width=9);self.role_selector.pack(side='right',padx=10);self.role_selector.bind('<<ComboboxSelected>>',lambda e:self.guarded(self._role_apply))
  page=ttk.Frame(self.tabs);self.operator_page=page;self.tabs.add(page,text='작업 운영');self._register_navigation(page,'작업 운영','▶')
  split=ttk.Panedwindow(page,orient='horizontal');split.pack(fill='both',expand=True)
  controls=ttk.Frame(split);map_panel=ttk.Frame(split);split.add(controls,weight=1);split.add(map_panel,weight=1)
  def balance(e):
   if e.width>500 and not getattr(split,'balanced',False):split.sashpos(0,int(e.width*.45));split.balanced=True
  split.bind('<Configure>',balance)
  holder,body,_=self._scrollable_frame(controls,bg=PANEL);holder.pack(fill='both',expand=True)
  self._studio_note(body,'작업 종류와 위치를 선택하고 미리보기에서 수행 순서를 확인하세요. 이동과 팔 작업은 기존 실행 엔진을 사용합니다.')
  self.operator_kind=tk.StringVar(value=self.operator_templates[0]['name']);self.operator_pickup=tk.StringVar(value='LM1' if 'LM1' in self.map.nodes else next(iter(self.map.nodes),''));self.operator_destination=tk.StringVar(value='LM3' if 'LM3' in self.map.nodes else next(iter(self.map.nodes),''));self.operator_return=tk.StringVar(value=self.operator_pickup.get());self.operator_repeat=tk.StringVar(value='1')
  self.operator_fields=[];self.operator_location_rows={}
  for label,var,values in [('미션 타입',self.operator_kind,[t['name'] for t in self.operator_templates]),('픽업 위치',self.operator_pickup,list(self.map.nodes)),('목적지',self.operator_destination,list(self.map.nodes)),('복귀 위치',self.operator_return,list(self.map.nodes))]:
   row=self._studio_row(body);ttk.Label(row,text=label,width=12).pack(side='left');combo=ttk.Combobox(row,textvariable=var,values=values,state='readonly',width=28);combo.pack(side='left',fill='x',expand=True);combo.bind('<<ComboboxSelected>>',lambda e:self._operator_invalidate());self.operator_fields.append(combo);self.operator_location_rows[label]=row
  self.operator_cycle_panel=ttk.Frame(body);self.operator_cycle_panel.pack(fill='x',padx=12,pady=4)
  self.operator_cycle_origin=tk.StringVar(value='출발 / 복귀 노드: 현재 위치에서 자동 확인')
  ttk.Label(self.operator_cycle_panel,textvariable=self.operator_cycle_origin).pack(anchor='w')
  self.operator_cycle_nodes=tk.Listbox(self.operator_cycle_panel,height=4,exportselection=False);self.operator_cycle_nodes.pack(fill='x')
  row=ttk.Frame(self.operator_cycle_panel);row.pack(fill='x')
  self.button(row,'선택 목적지 추가',lambda:self.guarded(self._operator_cycle_add)).pack(side='left')
  self.button(row,'선택 삭제',self._operator_cycle_remove).pack(side='left',padx=3)
  ttk.Label(self.operator_cycle_panel,text='위 목적지 목록 또는 지도에서 노드를 선택해 추가하세요. 마지막에는 출발 노드로 자동 복귀합니다.',wraplength=440).pack(fill='x')
  row=self._studio_row(body);self.operator_repeat_row=row;ttk.Label(row,text='실행 횟수',width=12).pack(side='left');ttk.Spinbox(row,textvariable=self.operator_repeat,from_=1,to=100,width=8,command=self._operator_invalidate).pack(side='left');ttk.Label(row,text='1회에 템플릿 전체 수행').pack(side='left',padx=6)
  row=self._studio_row(body)
  self.button(row,'경로 / 작업 미리보기',lambda:self.guarded(self._operator_preview),BLUE).pack(side='left',padx=3);self.operator_start_button=self.button(row,'시작',lambda:self.guarded(self._operator_start),BLUE);self.operator_start_button.pack(side='left',padx=3)
  row=self._studio_row(body)
  for title,action in [('일시정지','pause'),('재개','resume'),('작업 취소','cancel'),('정지','stop')]:self.button(row,title,lambda a=action:self.action(a),RED if action=='stop' else None).pack(side='left',padx=3)
  self.operator_summary=tk.StringVar(value='작업을 선택하고 미리보기를 누르세요.');ttk.Label(body,textvariable=self.operator_summary,wraplength=480).pack(fill='x',padx=12,pady=8)
  self.operator_steps=ttk.Treeview(body,columns=('step','result'),show='headings',height=7);self.operator_steps.heading('step',text='수행 단계');self.operator_steps.column('step',width=280);self.operator_steps.heading('result',text='상태');self.operator_steps.column('result',width=80);self.operator_steps.pack(fill='x',padx=12,pady=4)
  row=self._studio_row(body);self.button(row,'현재 수행 보고서',self._studio_report).pack(side='left',padx=3);self.button(row,'작업 이력',lambda:self.guarded(self._operator_history)).pack(side='left',padx=3)
  ttk.Label(map_panel,text='실시간 지도 · 파랑 계획 / 초록 SIM 실제 경로').pack(anchor='w',padx=8,pady=6)
  self.operator_map=OperatorMap(map_panel,self,width=550,height=400);self.operator_map.pack(fill='both',expand=True,padx=6)
  self.operator_live=tk.StringVar();ttk.Label(map_panel,textvariable=self.operator_live,wraplength=550).pack(fill='x',padx=8,pady=6);self.button(map_panel,'지도 맞춤',self.operator_map.fit).pack(anchor='w',padx=8,pady=5)
  self._developer_menu=self.cget('menu');self._developer_bar_pack=[(w,w.pack_info()) for w in self.connection_bar.winfo_children()]
  self.operator_menu=tk.Menu(self,tearoff=False);self.operator_menu.add_command(label='작업 운영',command=lambda:self.tabs.select(self.operator_page));self.operator_menu.add_command(label='작업 이력',command=lambda:self.guarded(self._operator_history))
  self.button(self.role_header,'사이클 테스트',self._operator_open_cycle).pack(side='right',padx=4)
  self._recipe_build()
  self.ui_role.set(self.studio_config.get('ui_role','개발자'));self._role_apply();self.operator_after=self.after(250,self._operator_tick)
 def _require_developer(self):
  if getattr(self,'role_active','개발자')!='개발자':raise ValueError('개발자 모드에서 사용할 수 있는 기능입니다.')
 def _role_apply(self):
  wanted=self.ui_role.get()
  if wanted not in ('사용자','개발자'):raise ValueError('지원하지 않는 모드')
  if wanted!=self.role_active and (self.studio_runner.active or self.held or self.arm_dev_sim.state in ('RUNNING','PAUSED')):
   self.ui_role.set(self.role_active);raise ValueError('주행과 팔 작업을 종료한 뒤 모드를 변경하세요.')
  self._pad_stop();self.release_drive();self.pad_enabled.set(False);self.manual.set(False);self.role_active=wanted;self.voice_epoch+=1;self.voice_pending=None;self.operator_pending=None
  for button in self.navigation_buttons.values():button.pack_forget()
  ordered=list(self.tabs.tabs()) if wanted=='개발자' else [str(self.operator_page),str(self.voice_page)]+[p for p in self.tabs.tabs() if p not in (str(self.operator_page),str(self.voice_page))]
  for page in ordered:
   allowed=wanted=='개발자' or page in (str(self.operator_page),str(self.voice_page))
   self.tabs.tab(page,state='normal' if allowed else 'hidden')
   button=self.navigation_buttons.get(page)
   if button:
    if allowed:button.pack(fill='x',padx=5,pady=2)
    else:button.pack_forget()
  self.config(menu=self._developer_menu if wanted=='개발자' else self.operator_menu)
  for w,info in self._developer_bar_pack:
   if wanted=='개발자':w.pack(**info)
   elif w is not self.connection_text:w.pack_forget()
  self.tabs.select(self.operator_page);self._sync_navigation();self.studio_config['ui_role']=wanted;self._studio_save_settings()
 def _role_gui_allowed(self,row):
  w=row['widget']
  if w is self.role_selector:return False
  if self.role_active=='개발자':return True
  if row['kind'] in ('map','hold'):return False
  if row['kind']=='screen':return row.get('tab') in (str(self.operator_page),str(self.voice_page))
  current=w
  while current is not None:
   if current in (self.operator_page,self.operator_menu):return True
   if isinstance(current,tk.Toplevel):return current.title() in ('작업 이력','미션 수행 보고서')
   current=getattr(current,'master',None)
  return False
 def _operator_parse_voice(self,text):
  from .voice_commands import normalize,NUM
  text=normalize(text);compact=''.join(text.split())
  recipe=next((r for r in self.operator_templates if ''.join(r['name'].split()) in compact),None)
  ids=re.findall(r'(?<![A-Z0-9])(?:LM|CP)\d+(?![A-Z0-9])',text.upper())
  if not recipe or not ids:return None
  if re.search(r'하지|말고|아니|금지',text):raise ValueError('원하는 운영 미션을 명확히 말하세요.')
  if len(ids)>3:raise ValueError('픽업·목적지·복귀 위치를 최대 3개로 지정하세요.')
  result=dict(action='operator_order',mission=recipe['name'],destination=ids[1] if len(ids)>1 else ids[0])
  if len(ids)>1:result['pickup']=ids[0]
  if len(ids)>2:result['return_to']=ids[2]
  count=re.search(r'([0-9]+|한|두|세|네|다섯)\s*(?:회|번|개)',text)
  result['repeat']=int(NUM.get(count[1],count[1])) if count else 1
  return self.voice_gui.validate_order(result)
 def _operator_arm_config(self):return self.studio_config['arm'] if self.real or self.studio_config['arm'].get('driver')=='fairino' else self.arm_dev_config
 def _operator_open_cycle(self):
  self.operator_kind.set('현재 노드 왕복 테스트');self.tabs.select(self.operator_page);self._operator_invalidate()
  if not self.operator_cycle_panel.winfo_manager():self.operator_cycle_panel.pack(fill='x',padx=12,pady=4,before=self.operator_repeat_row)
 def _operator_cycle_add(self):
  node=self.operator_destination.get()
  if node not in self.map.nodes:raise ValueError('지도에 등록된 목적지 노드를 선택하세요.')
  self.operator_cycle_nodes.insert('end',node);self._operator_invalidate()
 def _operator_cycle_remove(self):
  for index in reversed(self.operator_cycle_nodes.curselection()):self.operator_cycle_nodes.delete(index)
  self._operator_invalidate()
 def _operator_params(self):
  try:repeat=int(self.operator_repeat.get())
  except ValueError:raise ValueError('실행 횟수는 정수입니다.')
  visits=list(self.operator_cycle_nodes.get(0,'end')) if self.operator_kind.get()=='현재 노드 왕복 테스트' else []
  return dict(경유=visits,픽업=self.operator_pickup.get(),목적지=self.operator_destination.get(),복귀=self.operator_return.get(),repeat=repeat)
 def _operator_signature(self):return (self._voice_context(),self.operator_kind.get(),json.dumps(self._operator_params(),sort_keys=True),json.dumps(self.operator_templates,sort_keys=True),json.dumps(self._operator_arm_config(),sort_keys=True))
 def _operator_invalidate(self):self.operator_pending=None;self.operator_summary.set('설정이 변경되었습니다. 미리보기를 다시 확인하세요.')
 def _operator_preview(self):
  self.operator_pending=None
  recipe=next((t for t in self.operator_templates if t['name']==self.operator_kind.get()),None)
  if recipe is None:raise ValueError('미션 타입을 선택하세요.')
  plan=compile_mission(recipe,self._operator_params(),self.map,self._operator_arm_config(),self.current_state());self.operator_plan=plan
  self.operator_pending=(self._operator_signature(),time.monotonic()+60,(self.current_state()['x'],self.current_state()['y']))
  self.operator_summary.set(f"{plan['name']} · {plan['repeat']}회 · {plan['route']['distance']:.1f}m/첫 회\n"+' → '.join(plan['labels'])+'\n미리보기 60초 유효 · 시작 버튼으로 실행')
  self.operator_start_button.configure(state='normal' if not self.studio_runner.active else 'disabled')
  self._operator_steps_refresh(plan['actions']);self.operator_map.fit()
  return plan
 def _operator_steps_refresh(self,actions):
  self.operator_steps.delete(*self.operator_steps.get_children())
  for i,a in enumerate(actions):self.operator_steps.insert('','end',iid=str(i),values=(f"{i+1}. "+a.get('goal',a.get('operation','대기')),a.get('status','대기')))
 def _operator_start(self):
  if not self.operator_pending or self.operator_pending[0]!=self._operator_signature() or time.monotonic()>self.operator_pending[1]:raise ValueError('현재 설정으로 미리보기를 다시 확인하세요.')
  if not self.connected or not self.real and not self.sim_powered:raise ValueError('로봇 연결/전원을 확인하세요.')
  if self.studio_runner.active or self.task_running or self.sim.route or self.held or self.arm_dev_sim.state in ('RUNNING','PAUSED'):raise ValueError('기존 주행/팔 작업을 종료하세요.')
  state=self.current_state()
  if math.hypot(state['x']-self.operator_pending[2][0],state['y']-self.operator_pending[2][1])>.25:raise ValueError('미리보기 후 로봇 위치가 변경되었습니다. 다시 확인하세요.')
  if self.real:
   if not self.studio_arm_safe:raise ValueError('로봇팔 안전 자세를 먼저 확인하세요.')
   if not self.control_enabled or time.monotonic()-self.last_state>3 or state.get('emergency') or state.get('stopped') or state.get('task') in ('RUNNING','WAITING','SUSPENDED') or abs(float(state.get('speed',0)))>.02:raise ValueError('실기 제어권 및 최신 정지/안전 상태가 필요합니다.')
  plan=self.operator_plan
  for action in plan['actions']:
   if action['type']=='Path Nav' and self.real and action['goal'] not in self.robot_stations:raise ValueError('실기에 등록되지 않은 목적지: '+action['goal'])
   if action['type']=='Arm Action' and self.real:
    cfg=self._operator_arm_config();validate_fr5(cfg,True)
    if cfg.get('driver')!='fairino' or not self.fr5_client.connected or time.monotonic()-self.fr5_rx>2:raise ValueError('FR5 실기 연결 및 최신 상태가 필요합니다.')
  self._pad_stop();self.pad_enabled.set(False);self.pad_gate.reset();self.studio_runner.start(copy.deepcopy(plan['actions']),plan['repeat']);self.task_running=True;self.studio_charge_inhibit=False;self.operator_owned=True;self.operator_pending=None
  self.studio_runner.report.metadata.update(operator_recipe=plan['name'],operator_params=self._operator_params(),operator_template=copy.deepcopy(plan['template']),ui_role=self.role_active)
  self.operator_summary.set(plan['name']+' 미션을 시작했습니다.');self._operator_steps_refresh(self.studio_runner.actions)
 def _operator_history(self):
  from .studio_ui import SETTINGS
  win=tk.Toplevel(self);win.title('작업 이력');self._fit_dialog(win,950,600)
  ttk.Label(win,text='작업 단계 성공/실패/패스 횟수 · 일부 목적지 미도착은 부분 완료로 표시').pack(anchor='w',padx=10,pady=8)
  tree=ttk.Treeview(win,columns=('time','name','result','counts'),show='headings');tree.pack(fill='both',expand=True,padx=10,pady=8)
  for k,title in [('time','시작'),('name','미션'),('result','결과'),('counts','단계 성공 / 실패 / 패스')]:tree.heading(k,text=title)
  data={}
  for file in sorted((SETTINGS.parent/'mission_reports').glob('*.json'),reverse=True)[:200]:
   try:d=json.loads(file.read_text(encoding='utf-8'))
   except (OSError,ValueError):continue
   result='부분 완료' if d.get('status')=='COMPLETED' and (d.get('skipped') or d.get('omitted')) else d.get('status','—')
   item=tree.insert('','end',values=(d.get('started'),d.get('metadata',{}).get('operator_recipe','개발 미션'),result,f"{d.get('success',0)} / {d.get('failure',0)} / {d.get('skipped',0)}"));data[item]=d
  text=tk.Text(win,height=9,wrap='word');text.pack(fill='x',padx=10,pady=6)
  def selected(e=None):
   if tree.selection():text.configure(state='normal');text.delete('1.0','end');d=data[tree.selection()[0]];lines=[str(d.get('metadata',{}).get('operator_recipe','개발 미션')),f"완료 반복 {d.get('completed_loops',0)} · 수행 {d.get('seconds',0)}초"]+[f"{r.get('loop',1)}회 / {r.get('step',1)}단계 · {r.get('kind','')} · {r.get('target','')} · {r.get('detail','')}" for r in d.get('records',[])];text.insert('1.0','\n'.join(lines));text.configure(state='disabled')
  tree.bind('<<TreeviewSelect>>',selected);return win
 def _operator_tick(self):
  if self.voice_closed:return
  try:
   allowed={str(self.operator_page),str(self.voice_page)}
   if self.role_active=='사용자' and self.tabs.select() not in allowed:self.tabs.select(self.operator_page)
   nodes=list(self.map.nodes)
   cycle=self.operator_kind.get()=='현재 노드 왕복 테스트'
   if cycle:
    if not self.operator_cycle_panel.winfo_manager():self.operator_cycle_panel.pack(fill='x',padx=12,pady=4,before=self.operator_repeat_row)
    state=self.current_state();origin=self.map.nearest(state['x'],state['y']) if nodes else None
    self.operator_cycle_origin.set('출발 / 복귀 노드: '+(origin or '등록된 노드 없음')+' · 미리보기 시 확정')
   else:self.operator_cycle_panel.pack_forget()
   if nodes and self.operator_destination.get() not in nodes:self.operator_destination.set(nodes[0])
   for combo in self.operator_fields[1:]:combo.configure(values=nodes)
   self.operator_fields[0].configure(values=[t['name'] for t in self.operator_templates])
   recipe=next((t for t in self.operator_templates if t['name']==self.operator_kind.get()),None)
   locations={b.get('value') for b in recipe['blocks'] if b.get('type')=='이동'} if recipe else set()
   for label,token in [('픽업 위치','픽업'),('복귀 위치','복귀')]:
    row=self.operator_location_rows[label]
    if token in locations:
     if not row.winfo_manager():row.pack(fill='x',padx=12,pady=3,before=self.operator_repeat_row)
    else:row.pack_forget()
   self.recipe_destination_combo.configure(values=['모든 위치 (이동 미션)']+nodes)
   runner=self.studio_runner;state=self.current_state();result='부분 완료' if runner.status=='COMPLETED' and runner.report and (runner.report.data()['skipped'] or runner.report.data()['omitted']) else runner.status
   step=runner.actions[runner.index] if runner.active else None
   obstacle=(self.sim.block_reason or self.sim.avoidance_status or self.sim.auto_obstacle_status) if not self.real else ('장애물 정지' if state.get('blocked') else '')
   self.operator_live.set(f"{'REAL' if self.real else 'SIM'} · {result}\n직전 {state.get('last_node') or '—'} → 목표 {state.get('target') or '—'}\n현재 단계 {step.get('goal',step.get('operation',step['type'])) if step else '—'} · 반복 {runner.cycle+1 if runner.active else runner.cycle}/{runner.repeat}\n"+obstacle)
   if self.operator_owned:
    for i,a in enumerate(runner.actions):
     if self.operator_steps.exists(str(i)):self.operator_steps.item(str(i),values=(f"{i+1}. "+a.get('goal',a.get('operation','대기')),a.get('status','대기')))
   valid=self.operator_pending and self.operator_pending[0]==self._operator_signature() and time.monotonic()<=self.operator_pending[1]
   self.operator_start_button.configure(state='normal' if valid and not runner.active else 'disabled')
  except Exception as e:self.operator_live.set(str(e))
  self.operator_after=self.after(250,self._operator_tick)
 def _operator_close(self):
  if getattr(self,'operator_after',None):self.after_cancel(self.operator_after);self.operator_after=None

class OperatorMap(VoiceMissionMap):
 def route(self):return list((self.app.operator_plan or {}).get('stops',[]))
 def click(self,event,add=False):
  candidates=[(math.hypot(event.x-self.xy(n['x'],n['y'])[0],event.y-self.xy(n['x'],n['y'])[1]),key) for key,n in self.model.nodes.items()]
  if candidates:
   distance,key=min(candidates)
   if distance<=18:self.app.operator_destination.set(key);self.app._operator_invalidate();self.redraw()
