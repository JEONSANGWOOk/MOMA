"""Simulation-only tooling, workpieces, collision settings and pick/place demo."""
import copy,json,math
import tkinter as tk
from tkinter import ttk
from .geometry3d import point,transform,multiply,box
from .arm_physics import inverse
class ArmSceneMixin:
 def _aps_build(self,tabs):
  tab=ttk.Frame(tabs);self.arm_physics_tabs=tabs;self.arm_physics_tab=tab;tabs.add(tab,text='그리퍼 / 물체 / 충돌')
  holder,inner,_=self._scrollable_frame(tab,bg='#f4f7fb');holder.pack(fill='both',expand=True)
  self.arm_collision_bounds=tk.BooleanVar(value=True)
  row=self._studio_row(inner);ttk.Checkbutton(row,text='충돌 영역 표시',variable=self.arm_collision_bounds).pack(side='left')
  self.arm_collision_stop=tk.BooleanVar(value=self.arm_dev_sim.physics.config['stop_on_collision'])
  ttk.Checkbutton(row,text='충돌 예상 이동 차단',variable=self.arm_collision_stop,command=lambda:self.guarded(self._aps_apply)).pack(side='left')
  self.arm_gripper_kind=tk.StringVar(value={'none':'없음','finger':'2지 그리퍼','vacuum':'전동 1포트 진공'}[self.arm_dev_sim.physics.config['kind']])
  row=self._studio_row(inner);combo=ttk.Combobox(row,textvariable=self.arm_gripper_kind,values=['없음','2지 그리퍼','전동 1포트 진공'],state='readonly',width=18);combo.pack(side='left');combo.bind('<<ComboboxSelected>>',lambda e:self.guarded(self._aps_apply))
  self.button(row,'물리량 / I/O 설정',lambda:self.guarded(self._aps_settings)).pack(side='left',padx=3)
  self._studio_note(inner,'ON: 닫기/흡착 · OFF: 열기/해제 · 지정 DI: 파지 확인. ToolDO 0 → ToolDI 0 기본값. 신호는 독립 SIM에만 적용합니다.')
  row=self._studio_row(inner)
  for label,value in [('ON',1),('OFF',0)]:self.button(row,'그리퍼 '+label,lambda v=value:self._aps_io(v)).pack(side='left',padx=2)
  self.arm_physics_feedback=tk.StringVar();ttk.Label(inner,textvariable=self.arm_physics_feedback,wraplength=400).pack(fill='x',padx=8,pady=4)
  self._studio_note(inner,'가상 물체: 좌표는 로봇 base 기준 mm/도, 크기는 mm, 질량은 kg. 고정 장애물은 파지되지 않습니다.')
  self.arm_object_name=tk.StringVar(value='box1');self.arm_object_kind=tk.StringVar(value='box')
  row=self._studio_row(inner);ttk.Entry(row,textvariable=self.arm_object_name,width=16).pack(side='left');ttk.Combobox(row,textvariable=self.arm_object_kind,values=['box','panel','obstacle'],state='readonly',width=9).pack(side='left',padx=3)
  self.arm_object_size=tk.StringVar(value='40, 40, 60');self.arm_object_mass=tk.StringVar(value='.2');self.arm_object_pose=tk.StringVar(value='400, 0, 30, 0, 0, 0');self.arm_object_friction=tk.StringVar(value='.5');self.arm_object_sealable=tk.BooleanVar(value=True)
  for label,var in [('크기 X,Y,Z [mm]',self.arm_object_size),('질량 [kg]',self.arm_object_mass),('중심 X,Y,Z / RX,RY,RZ [mm/도]',self.arm_object_pose),('물체 마찰계수',self.arm_object_friction)]:
   ttk.Label(inner,text=label).pack(anchor='w',padx=8);ttk.Entry(inner,textvariable=var).pack(fill='x',padx=8,pady=2)
  ttk.Checkbutton(inner,text='평탄·비다공성 흡착 가능 표면',variable=self.arm_object_sealable).pack(anchor='w',padx=8)
  row=self._studio_row(inner)
  for label,fn in [('물체 추가',self._aps_add),('선택 삭제',self._aps_delete),('픽앤플레이스 데모',self._aps_demo)]:self.button(row,label,lambda f=fn:self.guarded(f)).pack(side='left',padx=2)
  self.arm_object_tree=ttk.Treeview(inner,columns=('kind','mass','state'),show='tree headings',height=5);self.arm_object_tree.heading('#0',text='물체');self.arm_object_tree.column('#0',width=100)
  for key,label in [('kind','종류'),('mass','kg'),('state','상태')]:self.arm_object_tree.heading(key,text=label);self.arm_object_tree.column(key,width=70)
  self.arm_object_tree.pack(fill='x',padx=8,pady=4)
  self._studio_note(inner,'회색: 충돌 검사 영역 · 노란색: 안전 여유 이내 · 빨간색: 충돌. 보수적인 회전 박스 검사이며 실제 로봇 안전장치 대신 사용할 수 없습니다.')
  self._aps_refresh()
 def _aps_apply(self):
  cfg=dict(kind={'없음':'none','2지 그리퍼':'finger','전동 1포트 진공':'vacuum'}[self.arm_gripper_kind.get()],stop_on_collision=self.arm_collision_stop.get())
  self.arm_dev_sim.physics.configure(cfg);self.arm_dev_canvas.render()
 def _aps_settings(self):
  win=tk.Toplevel(self);win.title('SIM 그리퍼 물리량 / I/O');self._fit_dialog(win,650,630)
  ttk.Label(win,text='m / kg / N / s / kPa 단위. force_n은 양쪽 조의 합계 수직력.\n마찰 지지력 = 힘 × 마찰계수. 진공력 = 압력 × 흡착 면적.\nmount_offset_m: URDF 말단→장착면 Z 오프셋(공식 FR5 메시에서 추정).\nsafety_factor: 여유계수, payload_limit_kg: 도구 포함 총 허용하중.\nself_collision: 비인접 링크 보수 검사(초기값 OFF).',wraplength=610).pack(fill='x',padx=12,pady=8)
  text=tk.Text(win,wrap='none');text.pack(fill='both',expand=True,padx=12);text.insert('1.0',json.dumps(self.arm_dev_sim.physics.config,ensure_ascii=False,indent=2))
  def apply():
   self.arm_dev_sim.physics.configure(json.loads(text.get('1.0','end')));self.arm_gripper_kind.set({'none':'없음','finger':'2지 그리퍼','vacuum':'전동 1포트 진공'}[self.arm_dev_sim.physics.config['kind']]);self.arm_collision_stop.set(self.arm_dev_sim.physics.config['stop_on_collision']);win.destroy()
  self.button(win,'적용',lambda:self.guarded(apply)).pack(pady=8)
 def _aps_io(self,value):
  c=self.arm_dev_sim.physics.config;self.arm_dev_sim.io[c['do_bank']][c['do_channel']]=value
 def _aps_add(self):
  if self.arm_dev_sim.state in ('RUNNING','PAUSED'):raise ValueError('재생 정지 후 물체를 추가하세요.')
  size=[float(v.strip())/1000 for v in self.arm_object_size.get().split(',')];p=[float(v.strip()) for v in self.arm_object_pose.get().split(',')]
  if len(p)!=6:raise ValueError('자세 6개 값을 입력하세요.')
  pose=[v/1000 for v in p[:3]]+[math.radians(v) for v in p[3:]]
  self.arm_dev_sim.physics.add_object(self.arm_object_name.get(),self.arm_object_kind.get(),size,float(self.arm_object_mass.get()),pose,self.arm_object_kind.get()=='obstacle',float(self.arm_object_friction.get()),self.arm_object_sealable.get());self._aps_refresh()
 def _aps_delete(self):
  if self.arm_dev_sim.state in ('RUNNING','PAUSED'):raise ValueError('재생 정지 후 삭제하세요.')
  names=set(self.arm_object_tree.selection());p=self.arm_dev_sim.physics
  if p.held and p.held['name'] in names:p.release('삭제')
  p.objects=[o for o in p.objects if o['name'] not in names];self._aps_refresh()
 def _aps_refresh(self):
  if not hasattr(self,'arm_object_tree'):return
  p=self.arm_dev_sim.physics;selected=self.arm_object_tree.selection()
  self.arm_object_tree.delete(*self.arm_object_tree.get_children())
  for o in p.objects:self.arm_object_tree.insert('','end',iid=o['name'],text=o['name'],values=(o['kind'],o['mass'],o['state']))
  for n in selected:
   if self.arm_object_tree.exists(n):self.arm_object_tree.selection_add(n)
  self.arm_physics_feedback.set(p.status+'\n'+('충돌: '+str(p.risk) if p.risk else '근접 경고: '+str(p.near) if p.near else '다음 0.25초 충돌 예상: '+str(p.future_risk) if p.future_risk else '충돌 여유 확보'))
 def _aps_demo(self):
  sim=self.arm_dev_sim;p=sim.physics
  if sim.state in ('RUNNING','PAUSED'):raise ValueError('재생 정지 후 데모를 만드세요.')
  if p.held:raise ValueError('파지 물체를 놓은 후 데모를 만드세요.')
  if p.config['kind']=='none':self.arm_gripper_kind.set('2지 그리퍼');self._aps_apply()
  # Plan all poses before changing the user's scene or library.
  source=sim.kin.pose(sim.q);lift=list(source);lift[2]+=80;q1=sim.kin.ik(lift,sim.q)
  q2=list(q1);q2[0]+=.26;q2=sim.kin.clamp(q2)
  target=sim.kin.pose(q2);target[2]-=80;q3=sim.kin.ik(target,q2)
  tcp=p.tcp(sim.q)
  if tcp[2][2]>-.96:raise ValueError('데모는 TCP가 아래를 보는 자세가 필요합니다. safe_pose를 먼저 실행하세요.')
  vacuum=p.config['kind']=='vacuum';size=[.12,.12,.03] if vacuum else [.04,.04,.06];offset=.14 if vacuum else .095
  matrix=multiply(tcp,transform((0,0,offset)));destination=multiply(p.tcp(q3),transform((0,0,offset)))
  names={'demo_workpiece','demo_pick_table','demo_place_table'};p.objects=[o for o in p.objects if o['name'] not in names]
  o=p.add_object('demo_workpiece','panel' if vacuum else 'box',size,.2,[0]*6);o['matrix']=matrix
  for name,m in [('demo_pick_table',matrix),('demo_place_table',destination)]:
   vertices=[point(m,v) for face in box(size) for v in face];bottom=min(v[2] for v in vertices)
   p.add_object(name,'obstacle',[.16,.16,.04],5,[m[0][3],m[1][3],bottom-.022,0,0,0],True)
  bank=p.config['do_bank'];ibank=p.config['di_bank'];retract=list(target);retract[2]+=80;ops={
   'sim_grip_on':dict(method='Set'+bank,id=p.config['do_channel'],status=1),
   'sim_grip_ok':dict(method='Wait'+ibank,id=p.config['di_channel'],status=1),
   'sim_lift':dict(method='MoveL',target=lift,vel=10),
   'sim_transfer':dict(method='MoveJ',target=[math.degrees(v) for v in q2],vel=10),
   'sim_lower':dict(method='MoveL',target=target,vel=10),
   'sim_grip_off':dict(method='Set'+bank,id=p.config['do_channel'],status=0),
   'sim_release_ok':dict(method='Wait'+ibank,id=p.config['di_channel'],status=0),
   'sim_retract':dict(method='MoveL',target=retract,vel=10)}
  self.arm_dev_config['operations'].update(ops);program='SIM 픽앤플레이스';self.arm_dev_config['programs'][program]=[dict(operation=n,timeout_s=30) for n in ops]
  self.arm_dev_program_combo.configure(values=list(self.arm_dev_config['programs']));self.arm_dev_program.set(program);self._aw_refresh_ops();self._aw_program_load();self._aps_refresh()
