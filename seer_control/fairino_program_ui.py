"""FR5 task development workspace; editing/validation never sends commands."""
import copy
import json
import time
import tkinter as tk
from tkinter import ttk,filedialog,messagebox
from .fairino_api import profile,validate,vector
from .fairino_programs import TEMPLATES,template,program_actions,library,FORMAT
from .theme import PANEL,RED

class FairinoProgramMixin:
 def _fr5_development(self):
  if self.studio_runner.active:raise ValueError('미션 종료 후 작업을 편집하세요.')
  cfg=copy.deepcopy(self.studio_config['arm']) if self.studio_config['arm'].get('driver')=='fairino' else profile()
  cfg.setdefault('programs',{});cfg.setdefault('operations',{})
  win=tk.Toplevel(self);win.title('FR5 작업 개발 — API / 티칭 / 작업 순서');win.configure(bg=PANEL);self._fit_dialog(win,1050,800)
  row=self._studio_row(win)
  self.button(row,'연결 설정',lambda:self.guarded(self._fr5_dialog)).pack(side='left',padx=3)
  self.button(row,'연결 / 상태 수신',lambda:self.guarded(self._fr5_connect)).pack(side='left',padx=3)
  self.button(row,'FR5 정지',lambda:self.guarded(self._fr5_stop),RED).pack(side='left',padx=3)
  standalone=tk.BooleanVar(value=False)
  ttk.Checkbutton(win,text='고정형 FR5 단독 시험 — AMR 연결 없이 사용 (AMR 장착 시 사용 금지)',variable=standalone).pack(anchor='w',padx=12,pady=3)
  feedback=tk.StringVar(value='FR5 상태 수신 대기');ttk.Label(win,textvariable=feedback,wraplength=960).pack(fill='x',padx=12,pady=5)
  tabs=ttk.Notebook(win);tabs.pack(fill='both',expand=True,padx=10,pady=5)
  poses=ttk.Frame(tabs);programs=ttk.Frame(tabs);tabs.add(poses,text='위치·그리퍼·입출력 API');tabs.add(programs,text='작업 프로그램')
  self._studio_note(poses,'이동: MoveJ 관절 6개[도], MoveL TCP X/Y/Z[mm]·RX/RY/RZ[도]. I/O: 제어함 0~15, 툴 0~1. 출력 접수는 파지 성공이 아닙니다. 센서 WaitDI로 확인하세요.')
  row=self._studio_row(poses);opname=tk.StringVar(value='safe_pose');method=tk.StringVar(value='MoveJ')
  ttk.Entry(row,textvariable=opname,width=22).pack(side='left',padx=3)
  ttk.Combobox(row,textvariable=method,values=('MoveJ','MoveL','SetDO','SetToolDO','WaitDI','WaitToolDI'),state='readonly',width=15).pack(side='left',padx=3)
  opbox=self._studio_json_box(poses,dict(method='MoveJ',target=[0]*6,tool=0,user=0,vel=10),height=8)
  oplist=tk.Listbox(poses,height=6,exportselection=False);oplist.pack(fill='both',expand=True,padx=12)
  def refresh_ops():
   oplist.delete(0,'end')
   for n in cfg['operations']:oplist.insert('end',n)
   choices.configure(values=list(cfg['operations']))
  def load_op(event=None):
   if not oplist.curselection():return
   n=oplist.get(oplist.curselection()[0]);opname.set(n);spec=cfg['operations'][n];method.set(spec['method']);put(opbox,spec)
  oplist.bind('<<ListboxSelect>>',load_op)
  def api_form(event=None):
   m=method.get();put(opbox,dict(method=m,target=[0]*6,tool=0,user=0,vel=10) if m in ('MoveJ','MoveL') else dict(method=m,id=0,status=1))
  def teach():
   if time.monotonic()-self.fr5_rx>2 or self.fr5_feedback.get('motion_done')!=1 or self.fr5_feedback.get('status') in ('ERROR','RUNNING','CANCELED'):raise ValueError('정지 상태의 최신 FR5 자세를 수신하세요.')
   m=method.get()
   if m not in ('MoveJ','MoveL'):raise ValueError('자세 티칭은 MoveJ/MoveL에서 사용하세요.')
   spec=self._studio_json(opbox);spec.update(method=m,target=vector(self.fr5_feedback['joints_deg' if m=='MoveJ' else 'tcp_mm_deg'],'자세'));put(opbox,spec)
  def save_op():
   n=opname.get().strip();spec=self._studio_json(opbox)
   candidate=copy.deepcopy(cfg);candidate['operations'][n]=spec;validate(candidate)
   cfg['operations']=candidate['operations'];refresh_ops()
  def delete_op():
   cfg['operations'].pop(opname.get().strip(),None);refresh_ops()
  row=self._studio_row(poses)
  for title,fn in [('API 입력 양식',api_form),('현재 자세 티칭',teach),('작업 등록/갱신',save_op),('선택 작업 삭제',delete_op)]:self.button(row,title,lambda f=fn:self.guarded(f)).pack(side='left',padx=3)
  self._studio_note(poses,'초기 숫자는 입력 양식입니다. 실제 작업점 티칭 후 등록하세요. 그리퍼/진공의 개폐 채널·논리와 파지 확인 센서는 사용 장치에 맞게 설정합니다.')
  row=self._studio_row(programs);pname=tk.StringVar(value='pick_place');pchoice=ttk.Combobox(row,textvariable=pname,values=list(cfg['programs']),width=25);pchoice.pack(side='left',padx=3)
  tname=tk.StringVar(value='픽앤플레이스');ttk.Combobox(row,textvariable=tname,values=list(TEMPLATES),state='readonly',width=18).pack(side='left',padx=3)
  steps=[]
  tree=ttk.Treeview(programs,columns=('step','op','timeout'),show='headings',height=11)
  for key,title in [('step','순서'),('op','작업 / 대기'),('timeout','제한(초)')]:tree.heading(key,text=title)
  tree.column('step',width=60,stretch=False);tree.column('timeout',width=100,stretch=False);tree.pack(fill='both',expand=True,padx=12,pady=6)
  row=self._studio_row(programs);selected=tk.StringVar();choices=ttk.Combobox(row,textvariable=selected,values=list(cfg['operations']),width=24);choices.pack(side='left',padx=3)
  timeout=tk.StringVar(value='60');ttk.Label(row,text='제한(초)').pack(side='left');ttk.Entry(row,textvariable=timeout,width=7).pack(side='left')
  delay=tk.StringVar(value='1');ttk.Label(row,text='대기(초)').pack(side='left',padx=3);ttk.Entry(row,textvariable=delay,width=7).pack(side='left')
  def render():
   tree.delete(*tree.get_children())
   for i,s in enumerate(steps):tree.insert('','end',iid=str(i),values=(i+1,s.get('operation','대기 '+str(s.get('wait_s'))+'초'),s.get('timeout_s',60)))
  def load_program(event=None):
   steps[:]=copy.deepcopy(cfg['programs'].get(pname.get(),[]));render()
  def make_template():steps[:]=template(tname.get());render()
  def add(wait=False):
   step=dict(wait_s=float(delay.get()),timeout_s=float(timeout.get())) if wait else dict(operation=selected.get().strip(),timeout_s=float(timeout.get()))
   candidate=dict(cfg,programs={'_draft_':steps+[step]});validate(candidate);steps.append(step);render()
  def edit(delta=None):
   if not tree.selection():return
   i=int(tree.selection()[0])
   if delta is None:steps.pop(i)
   elif 0<=i+delta<len(steps):steps[i],steps[i+delta]=steps[i+delta],steps[i]
   render()
  for title,fn in [('작업 추가',lambda:add()),('대기 추가',lambda:add(True)),('위',lambda:edit(-1)),('아래',lambda:edit(1)),('삭제',lambda:edit())]:self.button(row,title,lambda f=fn:self.guarded(f)).pack(side='left',padx=3)
  status=tk.StringVar(value='템플릿 생성 → 작업점·I/O 등록 → 검증 → 저장 → 실행');ttk.Label(programs,textvariable=status,wraplength=950).pack(fill='x',padx=12,pady=5)
  def draft():
   n=pname.get().strip();candidate=copy.deepcopy(cfg);candidate['programs'][n]=copy.deepcopy(steps);validate(candidate);return candidate,n
  def save_program():
   candidate,n=draft();cfg['programs']=candidate['programs'];pchoice.configure(values=list(cfg['programs']));status.set(n+' 초안 등록')
  def inspect():
   candidate,n=draft();actions=program_actions(candidate,n);status.set(f'{n}: {len(actions)}단계 검증 완료 — 좌표·I/O 동작은 현장 확인 필요')
  pchoice.bind('<<ComboboxSelected>>',load_program)
  row=self._studio_row(programs)
  for title,fn in [('템플릿 생성',make_template),('프로그램 불러오기',load_program),('프로그램 등록',save_program),('실행 전 검증',inspect)]:self.button(row,title,lambda f=fn:self.guarded(f)).pack(side='left',padx=3)
  self._studio_note(programs,'도어 열기는 티칭한 래치·호 궤적 점을 순서대로 이동합니다. 힘 제어/문 형상 자동 추정은 포함하지 않습니다. 미션 Arm Action의 operation에 프로그램 이름을 넣으면 단계별 실행·보고서가 연결됩니다.')
  def put(box,value):box.delete('1.0','end');box.insert('1.0',json.dumps(value,ensure_ascii=False,indent=2))
  def save_all():
   if self.studio_runner.active:raise ValueError('미션 종료 후 저장하세요.')
   validate(cfg)
   # Connection settings edited in the separate dialog take precedence.
   actual=copy.deepcopy(self.studio_config['arm']) if self.studio_config['arm'].get('driver')=='fairino' else profile()
   actual.update(operations=copy.deepcopy(cfg['operations']),programs=copy.deepcopy(cfg['programs']));validate(actual,motion=actual.get('verified',False))
   if self.fr5_client.connected:self._fr5_priority_stop()
   self.studio_config['arm']=actual;self.studio_arm_safe=False;self._studio_save_settings();status.set('라이브러리 저장 완료. 실기 실행 전 다시 연결 / 상태 수신하세요.')
   self.studio_device_box.delete('1.0','end');self.studio_device_box.insert('1.0',json.dumps(dict(peripherals=self.studio_config['peripherals'],arm=actual),ensure_ascii=False,indent=2))
  def export():
   path=filedialog.asksaveasfilename(parent=win,defaultextension='.json',initialfile='fr5_task_library.json')
   if path:Path(path).write_text(json.dumps(library(cfg),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
  def import_file():
   path=filedialog.askopenfilename(parent=win,filetypes=[('FR5 작업 라이브러리','*.json')])
   if not path:return
   value=json.loads(Path(path).read_text(encoding='utf-8'))
   if value.get('format')!=FORMAT:raise ValueError('FR5 작업 라이브러리 형식이 아닙니다.')
   candidate=copy.deepcopy(cfg);candidate.update(operations=value.get('operations',{}),programs=value.get('programs',{}));validate(candidate)
   cfg.update(operations=candidate['operations'],programs=candidate['programs']);refresh_ops();pchoice.configure(values=list(cfg['programs']));steps.clear();render();status.set('가져오기 완료 — 화면 초안만 변경됨. 확인 후 라이브러리 저장하세요.')
  def run(single=False):
   if not self.real:raise ValueError('프로그램은 실기 API 개발용입니다. 실행 전 검증은 SIM에서도 가능합니다. MoveL 시뮬레이션 IK는 미구현입니다.')
   n=pname.get().strip();actual=self.studio_config['arm']
   candidate,name=draft()
   if candidate['operations']!=actual.get('operations'):raise ValueError('위치/I/O 라이브러리 저장 후 실행하세요.')
   if single:
    if not tree.selection():raise ValueError('시험할 단계를 선택하세요.')
    n='선택 단계 '+str(int(tree.selection()[0])+1)
    subset=dict(actual,programs={n:[copy.deepcopy(steps[int(tree.selection()[0])])]})
    actions=program_actions(subset,n)
   else:
    if candidate['programs']!=actual.get('programs'):raise ValueError('프로그램 등록 및 라이브러리 저장 후 실행하세요.')
    actions=program_actions(actual,n)
   validate(actual,True)
   if self.studio_runner.active or self.sim.route or self.held:raise ValueError('현재 작업/주행을 종료하세요.')
   if standalone.get():
    if self.connected:raise ValueError('고정형 FR5 단독 시험은 AMR 연결을 해제한 상태에서만 사용하세요.')
    for action in actions:action['_fr5_standalone']=True
   else:
    if not self.connected or not self.control_enabled or time.monotonic()-self.last_state>3:raise ValueError('AMR 실기 제어 연결과 최신 정지 상태가 필요합니다.')
    state=self.current_state()
    if state.get('task') in ('RUNNING','WAITING','SUSPENDED') or abs(float(state.get('speed',0)))>.02:raise ValueError('AMR 주행 종료 후 실행하세요.')
   if not self.fr5_client.connected or time.monotonic()-self.fr5_rx>2:raise ValueError('FR5 연결 / 상태 수신을 먼저 실행하세요.')
   if not messagebox.askokcancel('FR5 프로그램 실기 실행',f'{n}: {len(actions)}단계를 실제 FR5와 I/O에 전송합니다. 티칭점과 작업 공간을 확인하세요.',parent=win):return
   self.studio_runner.start(actions);self.studio_runner.report.metadata.update(fr5_program=n,fr5_standalone=standalone.get());self.task_running=True;status.set(n+' 실행 중 — 미션 화면에서 단계별 상태·보고서 확인')
  from pathlib import Path
  row=self._studio_row(win)
  for title,fn in [('라이브러리 저장',save_all),('JSON 내보내기',export),('JSON 가져오기',import_file),('선택 단계 시험',lambda:run(True)),('프로그램 실행',run)]:self.button(row,title,lambda f=fn:self.guarded(f)).pack(side='left',padx=3)
  def update_feedback():
   if not win.winfo_exists():return
   fb=self.fr5_feedback;fresh=time.monotonic()-self.fr5_rx<=2
   feedback.set(('실시간' if fresh else '수신 대기 / 오래된 상태')+' · '+fb.get('status','UNKNOWN')+' · J[도] '+str(fb.get('joints_deg',[]))+' · TCP[mm/도] '+str(fb.get('tcp_mm_deg',[])))
   win.after(250,update_feedback)
  refresh_ops();update_feedback()
