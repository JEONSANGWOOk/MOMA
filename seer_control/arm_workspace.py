"""Arm-only 3D workspace. Simulation state is independent of AMR and SDK."""
import copy,json,math,time
from pathlib import Path
import tkinter as tk
from tkinter import ttk,filedialog
from .geometry3d import RobotDescription,point,identity
from .smooth_renderer import renderer_for
from .pose_display import real_arm_positions
from .world3d import Camera,PRESETS,normalize,cross,subtract,dot
from .arm_simulation import ArmSimulator
from .fairino_api import profile,validate
from .fairino_programs import TEMPLATES,template,library,FORMAT
from .arm_scene_ui import ArmSceneMixin
from .theme import PANEL,INK,BLUE,RED

class ArmCanvas(tk.Canvas):
 def __init__(self,parent,app,sim):
  super().__init__(parent,bg='#edf2f8',highlightthickness=0)
  self.app=app;self.sim=sim;self.camera=Camera();self.camera.distance=2.5;self.camera.target=[0,0,.45];self.drag=None
  self.bind('<Configure>',lambda e:self.render());self.bind('<ButtonPress-1>',lambda e:self.begin(e,'orbit'));self.bind('<ButtonPress-3>',lambda e:self.begin(e,'pan'))
  self.bind('<B1-Motion>',self.motion);self.bind('<B3-Motion>',self.motion);self.bind('<MouseWheel>',self.zoom)
 def begin(self,e,mode):self.drag=(e.x,e.y,self.camera.yaw,self.camera.pitch,list(self.camera.target),mode)
 def motion(self,e):
  if not self.drag:return
  x,y,yaw,pitch,target,mode=self.drag;dx=e.x-x;dy=e.y-y
  if mode=='orbit':self.camera.yaw=yaw-dx*.4;self.camera.pitch=max(3,min(89.9,pitch+dy*.35))
  else:
   _,right,up,_=self.camera.basis();scale=self.camera.distance/max(100,self.winfo_height());self.camera.target=[target[i]-dx*right[i]*scale+dy*up[i]*scale for i in range(3)]
  self.render()
 def zoom(self,e):self.camera.distance=max(.3,min(20,self.camera.distance*(.9 if e.delta>0 else 1.1)));self.render()
 def fit(self):
  points=[p for face,_,_ in self.sim.kin.asset.draw_faces(self.sim.kin.positions(self.sim.q)) for p in face]
  if points:
   lo=[min(p[i] for p in points) for i in range(3)];hi=[max(p[i] for p in points) for i in range(3)]
   self.camera.target=[(a+b)/2 for a,b in zip(lo,hi)];self.camera.distance=max(1.,max(b-a for a,b in zip(lo,hi))*1.9)
  self.render()
 def render(self):
  self.delete('all');w=max(10,self.winfo_width());h=max(10,self.winfo_height())
  def line(a,b,color,width=1):
   p=self.camera.project(a,w,h);q=self.camera.project(b,w,h)
   if p and q:self.create_line(*p[:2],*q[:2],fill=color,width=width)
  for i in range(-10,11):
   value=i*.1;line((value,-1,0),(value,1,0),'#cdd8e4');line((-1,value,0),(1,value,0),'#cdd8e4')
  renderer=renderer_for(self)
  display=getattr(self.app,'arm_dev_display',None);mode=display.get() if display else 'SIM 개발'
  measured=real_arm_positions(self.app,self.sim.kin.joints);assets=[]
  if mode=='SIM 개발':assets.append((self.sim.kin.asset,self.sim.kin.positions(self.sim.q),identity(),None,1))
  elif measured:assets.append((self.sim.kin.asset,measured,identity(),None,1))
  if mode=='실기 + SIM 비교':assets.append((self.sim.kin.asset,self.sim.kin.positions(self.sim.q),identity(),'#319fea',.42))
  show=getattr(self.app,'arm_collision_bounds',None)
  physical_faces,physical_lines=self.sim.physics.geometry(self.sim.q,show.get() if show else False) if mode!='실기 자세' else ([],[])
  if mode!='실기 자세' and self.sim.physics.forecast_q is not None:assets.append((self.sim.kin.asset,self.sim.kin.positions(self.sim.physics.forecast_q),identity(),'#ef4e55',.22))
  if renderer:
   from PIL import ImageTk
   self.render_image=ImageTk.PhotoImage(renderer.render(self.camera,w,h,assets=assets,faces=physical_faces,lines=physical_lines,grid=True))
   self.create_image(0,0,anchor='nw',image=self.render_image,tags='arm_mesh')
  else:
   faces=[]
   for vertices,color,name in physical_faces+[face for asset,positions,world,tint,alpha in assets for face in [(v,tint or c,n) for v,c,n in asset.draw_faces(positions,world)]]:
    projected=[self.camera.project(p,w,h) for p in vertices]
    if all(projected):
     normal=normalize(cross(subtract(vertices[1],vertices[0]),subtract(vertices[2],vertices[0])))
     brightness=.6+.4*abs(dot(normal,normalize((.3,-.5,1))))
     color='#'+''.join(f'{round(int(color[i:i+2],16)*brightness):02x}' for i in (1,3,5))
     faces.append((sum(p[2] for p in projected)/len(projected),projected,color,name))
   for _,projected,color,name in sorted(faces,key=lambda row:row[0],reverse=True):self.create_polygon(*[v for p in projected for v in p[:2]],fill=color,outline=color,width=.5,tags=('arm_mesh',name))
  for a,b,color,width in physical_lines:line(a,b,color,width)
  if mode!='실기 자세':
   for a,b in zip(self.sim.trail,self.sim.trail[1:]):line(a,b,'#319fea',2)
  for axis,color,label in [((.25,0,0),'#dc4c4c','X'),((0,.25,0),'#18a36a','Y'),((0,0,.25),'#267dde','Z')]:
   line((0,0,0),axis,color,3);p=self.camera.project(axis,w,h)
   if p:self.create_text(*p[:2],text=label,fill=color)
  q=[measured[j['name']] for j in self.sim.kin.joints] if mode=='실기 자세' and measured else self.sim.q
  t=self.sim.kin.fk(q);origin=point(t,(0,0,0))
  for axis,color in [((.1,0,0),'#dc4c4c'),((0,.1,0),'#18a36a'),((0,0,.1),'#267dde')]:line(origin,point(t,axis),color,2)
  p=self.camera.project(origin,w,h)
  if p and assets:self.create_oval(p[0]-4,p[1]-4,p[0]+4,p[1]+4,fill='#319fea' if mode!='실기 자세' else '#ef5b4d',outline='white',tags='tcp')
  status='SIM 개발 — 파란 궤적' if mode=='SIM 개발' else ('REAL 수신 자세 — 원래 색상' if measured else 'REAL 자세 없음 / 수신 지연')
  if mode=='실기 + SIM 비교':status+=' · SIM 예측 — 파란 겹쳐보기'
  self.create_text(12,12,anchor='nw',text=status+'\n'+self.sim.kin.asset.name+' · '+('매끄러운 GPU 표면' if renderer else '기본 표면')+'\n좌드래그 회전 · 우드래그 이동 · 휠 확대',fill='#28405d',font=(self.app.font,9))

class ArmWorkspaceMixin(ArmSceneMixin):
 def _arm_workspace_build(self):
  page=ttk.Frame(self.tabs);self.tabs.add(page,text='로봇팔 개발');self._register_navigation(page,'로봇팔 개발','⚙');self._sync_navigation();self.arm_workspace_page=page
  saved=self.studio_config.get('arm_workspace',{})
  cfg=profile();cfg.update(operations={
   'safe_pose':dict(method='MoveJ',target=[0,-90,90,-90,-90,0],vel=20,tool=0,user=0),
   'demo_a':dict(method='MoveJ',target=[30,-100,100,-90,-70,20],vel=20,tool=0,user=0),
   'demo_b':dict(method='MoveJ',target=[-30,-80,70,-100,-100,-20],vel=20,tool=0,user=0)},programs={'관절 이동 예제':[dict(operation=n,timeout_s=60) for n in ('safe_pose','demo_a','demo_b','safe_pose')]})
  selected=saved.get('urdf') or str(self._spatial_demo_path())
  legacy=Path(__file__).resolve().parents[1]/'examples/fairino_fr5.urdf'
  if Path(selected).resolve()==legacy.resolve():selected=str(self._spatial_demo_path())
  try:
   if saved.get('operations') is not None:cfg.update(operations=saved['operations'],programs=saved.get('programs',{}));validate(cfg)
   asset=RobotDescription.load(selected)
  except Exception as e:
   self.log('ERROR','로봇팔 개발 설정 복원 실패: '+str(e));asset=RobotDescription.load(self._spatial_demo_path());cfg=profile()
  self.arm_dev_config=cfg;self.arm_dev_urdf=selected;self.arm_dev_sim=ArmSimulator(asset)
  try:self.arm_dev_sim.physics.restore(saved.get('physics',{}))
  except Exception as e:self.log('WARN','물리 장면 복원 실패: '+str(e))
  initial=saved.get('joints_rad',[math.radians(v) for v in [0,-90,90,-90,-90,0]] if len(asset.movable())==6 else [0]*len(asset.movable()))
  try:self.arm_dev_sim.q=self.arm_dev_sim.kin.clamp(initial)
  except ValueError:self.log('WARN','저장된 관절값을 복원하지 못해 모델 초기값을 사용합니다.')
  row=self._studio_row(page)
  self.label(row,'로봇팔 전용 3D 시뮬레이션',11,INK,True,bg=PANEL).pack(side='left',padx=4)
  padrow=self._studio_row(page)
  tk.Checkbutton(padrow,text='수동 조작',variable=self.manual,bg=PANEL,command=self._pad_toggle).pack(side='left')
  tk.Checkbutton(padrow,text='조이스틱 사용',variable=self.pad_enabled,bg=PANEL,command=self._pad_toggle).pack(side='left')
  self._pad_mode_controls(padrow)
  self.button(padrow,'조이스틱 연결/설정',self._pad_dialog).pack(side='left',padx=3)
  self.pad_arm_feedback=tk.StringVar(value='L1 조종 · ○ 정지 · R1/R2 모드 ± · R3/L3 속도 ± · △ 그리퍼 (버튼 변경 가능)')
  ttk.Label(page,textvariable=self.pad_arm_feedback,wraplength=1100).pack(fill='x',padx=12)
  for title,fn in [('공식 FR5 모델',self._aw_official_model),('URDF 가져오기',self._aw_load_urdf),('API 작업 가져오기',self._aw_copy_api),('실기 API 개발',self._fr5_development),('설정 저장',self._aw_save)]:self.button(row,title,lambda f=fn:self.guarded(f)).pack(side='left',padx=3)
  self._studio_note(page,'AMR 연결 없이 사용 · 관절 슬라이더/재생/I/O는 SIM 전용 · 조이스틱은 상단 SIM/REAL 모드에 따라 조종 · 설치된 공식 FR5 ROS2 모델 사용 가능 · 실기 TCP 좌표·충돌 검증은 별도입니다.')
  pane=ttk.Panedwindow(page,orient='horizontal');pane.pack(fill='both',expand=True,padx=8,pady=4)
  left=ttk.Frame(pane);right=ttk.Frame(pane);pane.add(left,weight=3);pane.add(right,weight=2)
  row=self._studio_row(left);camera=tk.StringVar(value='사선')
  selector=ttk.Combobox(row,textvariable=camera,values=list(PRESETS),state='readonly',width=8);selector.pack(side='left')
  def preset(e=None):self.arm_dev_canvas.camera.yaw,self.arm_dev_canvas.camera.pitch=PRESETS[camera.get()];self.arm_dev_canvas.render()
  selector.bind('<<ComboboxSelected>>',preset)
  projection=tk.StringVar(value='원근');selector=ttk.Combobox(row,textvariable=projection,values=['원근','직교'],state='readonly',width=7);selector.pack(side='left',padx=3)
  def project(e=None):self.arm_dev_canvas.camera.perspective=projection.get()=='원근';self.arm_dev_canvas.render()
  selector.bind('<<ComboboxSelected>>',project)
  self.button(row,'맞춤',lambda:self.arm_dev_canvas.fit()).pack(side='left',padx=3)
  self.arm_dev_display=tk.StringVar(value='SIM 개발')
  ttk.Combobox(row,textvariable=self.arm_dev_display,values=['SIM 개발','실기 자세','실기 + SIM 비교'],state='readonly',width=17).pack(side='left',padx=3)
  self.arm_dev_canvas=ArmCanvas(left,self,self.arm_dev_sim);self.arm_dev_canvas.pack(fill='both',expand=True)
  self.arm_dev_feedback=tk.StringVar();ttk.Label(left,textvariable=self.arm_dev_feedback,wraplength=650).pack(fill='x',padx=5,pady=5)
  tabs=ttk.Notebook(right);tabs.pack(fill='both',expand=True)
  controls=ttk.Frame(tabs);programs=ttk.Frame(tabs);tabs.add(controls,text='관절 / 티칭 / I/O');tabs.add(programs,text='동작 프로그램');self._aps_build(tabs)
  holder,inner,_=self._scrollable_frame(controls,bg=PANEL);holder.pack(fill='both',expand=True);self.arm_dev_joint_frame=tk.Frame(inner,bg=PANEL);self.arm_dev_joint_frame.pack(fill='x');self._aw_joint_controls()
  row=self._studio_row(inner);self.arm_dev_op=tk.StringVar(value='new_pose');self.arm_dev_method=tk.StringVar(value='MoveJ')
  ttk.Entry(row,textvariable=self.arm_dev_op,width=19).pack(side='left');ttk.Combobox(row,textvariable=self.arm_dev_method,values=['MoveJ','MoveL','SetDO','SetToolDO','WaitDI','WaitToolDI'],state='readonly',width=11).pack(side='left',padx=3)
  row=self._studio_row(inner);self.arm_dev_vel=tk.StringVar(value='20');ttk.Label(row,text='속도 %').pack(side='left');ttk.Entry(row,textvariable=self.arm_dev_vel,width=5).pack(side='left')
  self.arm_dev_channel=tk.StringVar(value='0');self.arm_dev_logic=tk.StringVar(value='1');ttk.Label(row,text='I/O 채널 / 값').pack(side='left',padx=5);ttk.Entry(row,textvariable=self.arm_dev_channel,width=4).pack(side='left');ttk.Combobox(row,textvariable=self.arm_dev_logic,values=['0','1'],state='readonly',width=3).pack(side='left',padx=3)
  row=self._studio_row(inner)
  for title,fn in [('현재 자세 / I/O 등록',self._aw_teach),('선택 작업 시험',self._aw_run_op),('작업 삭제',self._aw_delete_op)]:self.button(row,title,lambda f=fn:self.guarded(f)).pack(side='left',padx=2)
  self.arm_dev_ops=tk.Listbox(inner,height=5,exportselection=False);self.arm_dev_ops.pack(fill='x',padx=8)
  self.arm_dev_ops.bind('<<ListboxSelect>>',lambda e:self._aw_select_op())
  self._studio_note(inner,'MoveL 티칭: 현재 모델의 TCP[mm/도] 저장. 시뮬레이션은 모델 base와 tool link 좌표만 사용합니다. 실기 tool/user 좌표와 자동 일치하지 않습니다.')
  ttk.Label(inner,text='TCP 목표 [X,Y,Z mm / RX,RY,RZ 도]').pack(anchor='w',padx=8)
  self.arm_dev_tcp=tk.StringVar(value=', '.join(f'{v:.3f}' for v in self.arm_dev_sim.kin.pose(self.arm_dev_sim.q)))
  ttk.Entry(inner,textvariable=self.arm_dev_tcp).pack(fill='x',padx=8)
  row=self._studio_row(inner)
  self.button(row,'현재 TCP 입력',lambda:self.arm_dev_tcp.set(', '.join(f'{v:.3f}' for v in self.arm_dev_sim.kin.pose(self.arm_dev_sim.q)))).pack(side='left')
  self.button(row,'TCP 목표로 SIM 직선 이동',lambda:self.guarded(self._aw_tcp_move)).pack(side='left',padx=3)
  row=self._studio_row(inner);self.arm_dev_input_kind=tk.StringVar(value='DI');self.arm_dev_input_channel=tk.StringVar(value='0');self.arm_dev_input_value=tk.StringVar(value='1')
  ttk.Combobox(row,textvariable=self.arm_dev_input_kind,values=['DI','ToolDI'],width=7,state='readonly').pack(side='left');ttk.Entry(row,textvariable=self.arm_dev_input_channel,width=4).pack(side='left',padx=3);ttk.Combobox(row,textvariable=self.arm_dev_input_value,values=['0','1'],width=3,state='readonly').pack(side='left');self.button(row,'SIM 입력 변경',lambda:self.guarded(self._aw_input)).pack(side='left',padx=3)
  self.arm_dev_io_text=tk.StringVar();ttk.Label(inner,textvariable=self.arm_dev_io_text,wraplength=400).pack(fill='x',padx=8,pady=3)
  holder,inner,_=self._scrollable_frame(programs,bg=PANEL);holder.pack(fill='both',expand=True)
  row=self._studio_row(inner);self.arm_dev_program=tk.StringVar(value=next(iter(cfg['programs']),'new_program'));self.arm_dev_program_combo=ttk.Combobox(row,textvariable=self.arm_dev_program,values=list(cfg['programs']),width=25);self.arm_dev_program_combo.pack(fill='x',expand=True);self.arm_dev_program_combo.bind('<<ComboboxSelected>>',lambda e:self._aw_program_load())
  row=self._studio_row(inner);self.arm_dev_template=tk.StringVar(value='픽앤플레이스');ttk.Combobox(row,textvariable=self.arm_dev_template,values=list(TEMPLATES),state='readonly',width=17).pack(side='left');self.button(row,'템플릿 생성',lambda:self.guarded(self._aw_template)).pack(side='left',padx=3)
  self.arm_dev_steps=[];self.arm_dev_tree=ttk.Treeview(inner,columns=('num','op','time'),show='headings',height=9)
  for key,title,width in [('num','순서',45),('op','작업',190),('time','제한(s)',60)]:self.arm_dev_tree.heading(key,text=title);self.arm_dev_tree.column(key,width=width,stretch=key=='op')
  self.arm_dev_tree.pack(fill='x',padx=8);self.arm_dev_tree.bind('<<TreeviewSelect>>',lambda e:self._aw_step_select())
  row=self._studio_row(inner);self.arm_dev_step_op=tk.StringVar(value=next(iter(cfg['operations']),''));self.arm_dev_step_combo=ttk.Combobox(row,textvariable=self.arm_dev_step_op,values=list(cfg['operations']),width=20);self.arm_dev_step_combo.pack(side='left');self.arm_dev_timeout=tk.StringVar(value='60');ttk.Entry(row,textvariable=self.arm_dev_timeout,width=5).pack(side='left',padx=3);self.button(row,'작업 추가',lambda:self.guarded(self._aw_add_step)).pack(side='left')
  row=self._studio_row(inner);self.arm_dev_wait=tk.StringVar(value='1');ttk.Entry(row,textvariable=self.arm_dev_wait,width=5).pack(side='left')
  for title,fn in [('대기 추가',lambda:self._aw_add_step(True)),('위',lambda:self._aw_move_step(-1)),('아래',lambda:self._aw_move_step(1)),('삭제',lambda:self._aw_move_step(None))]:self.button(row,title,lambda f=fn:self.guarded(f)).pack(side='left',padx=2)
  row=self._studio_row(inner)
  for title,fn in [('프로그램 등록',self._aw_program_save),('SIM 재생',self._aw_play),('일시정지/재개',self._aw_pause),('정지',self._aw_stop)]:self.button(row,title,lambda f=fn:self.guarded(f)).pack(side='left',padx=2)
  row=self._studio_row(inner)
  for title,fn in [('라이브러리 가져오기',self._aw_import),('API 라이브러리 내보내기',self._aw_export),('SIM 실행 이력',self._aw_report)]:self.button(row,title,lambda f=fn:self.guarded(f)).pack(side='left',padx=2)
  self._aw_refresh_ops();self._aw_program_load();self.arm_dev_time=time.monotonic();self.after(80,self._aw_tick);self.after_idle(self.arm_dev_canvas.fit);self._aw_sync_main()
 def _aw_joint_controls(self):
  for w in self.arm_dev_joint_frame.winfo_children():w.destroy()
  self.arm_dev_vars=[]
  for joint,value in zip(self.arm_dev_sim.kin.joints,self.arm_dev_sim.q):
   row=self._studio_row(self.arm_dev_joint_frame);var=tk.DoubleVar(value=value if joint['type']=='prismatic' else math.degrees(value));self.arm_dev_vars.append(var)
   ttk.Label(row,text=joint['name']+(' [m]' if joint['type']=='prismatic' else ' [°]'),width=12).pack(side='left');ttk.Scale(row,variable=var,from_=joint['lower'] if joint['type']=='prismatic' else math.degrees(joint['lower']),to=joint['upper'] if joint['type']=='prismatic' else math.degrees(joint['upper']),command=lambda v:self._aw_slider()).pack(side='left',fill='x',expand=True)
   entry=ttk.Entry(row,textvariable=var,width=9);entry.pack(side='left');entry.bind('<Return>',lambda e:self.guarded(self._aw_apply_joints))
 def _aw_slider(self):
  if getattr(self,'arm_dev_syncing',False) or self.arm_dev_sim.state in ('RUNNING','PAUSED'):return
  self.guarded(self._aw_apply_joints)
 def _aw_apply_joints(self):self.arm_dev_sim.set_joints([v.get() if joint['type']=='prismatic' else math.radians(v.get()) for joint,v in zip(self.arm_dev_sim.kin.joints,self.arm_dev_vars)]);self._aw_sync_main();self.arm_dev_canvas.render()
 def _aw_refresh_ops(self):
  self.arm_dev_ops.delete(0,'end')
  for name in self.arm_dev_config['operations']:self.arm_dev_ops.insert('end',name)
  self.arm_dev_step_combo.configure(values=list(self.arm_dev_config['operations']))
 def _aw_select_op(self):
  if not self.arm_dev_ops.curselection():return
  name=self.arm_dev_ops.get(self.arm_dev_ops.curselection()[0]);spec=self.arm_dev_config['operations'][name];self.arm_dev_op.set(name);self.arm_dev_method.set(spec['method']);self.arm_dev_vel.set(str(spec.get('vel',20)))
  if 'id' in spec:self.arm_dev_channel.set(str(spec['id']));self.arm_dev_logic.set(str(spec['status']))
 def _aw_teach(self):
  if self.arm_dev_sim.state in ('RUNNING','PAUSED'):raise ValueError('재생 정지 후 티칭하세요.')
  name=self.arm_dev_op.get().strip();method=self.arm_dev_method.get()
  if method in ('MoveJ','MoveL'):
   target=[math.degrees(v) for v in self.arm_dev_sim.q] if method=='MoveJ' else self.arm_dev_sim.kin.pose(self.arm_dev_sim.q)
   spec=dict(method=method,target=target,tool=0,user=0,vel=float(self.arm_dev_vel.get()))
  else:spec=dict(method=method,id=int(self.arm_dev_channel.get()),status=int(self.arm_dev_logic.get()))
  candidate=copy.deepcopy(self.arm_dev_config);candidate['operations'][name]=spec;validate(candidate);self.arm_dev_config=candidate;self._aw_refresh_ops()
 def _aw_run_op(self):self.arm_dev_sim.start(self.arm_dev_config,operation=self.arm_dev_op.get().strip())
 def _aw_delete_op(self):self.arm_dev_config['operations'].pop(self.arm_dev_op.get().strip(),None);self._aw_refresh_ops()
 def _aw_tcp_move(self):
  from .fairino_api import vector
  target=vector([float(v.strip()) for v in self.arm_dev_tcp.get().split(',')],'TCP')
  cfg=profile();cfg['operations']={'tcp_preview':dict(method='MoveL',target=target,tool=0,user=0,vel=float(self.arm_dev_vel.get()))}
  self.arm_dev_sim.start(cfg,operation='tcp_preview')
 def _aw_input(self):
  kind=self.arm_dev_input_kind.get();channel=int(self.arm_dev_input_channel.get());value=int(self.arm_dev_input_value.get())
  if not 0<=channel<=(1 if kind=='ToolDI' else 15):raise ValueError('입력 채널 범위 오류')
  self.arm_dev_sim.io[kind][channel]=value
 def _aw_program_load(self):self.arm_dev_steps=copy.deepcopy(self.arm_dev_config['programs'].get(self.arm_dev_program.get(),[]));self._aw_render_steps()
 def _aw_render_steps(self):
  self.arm_dev_tree.delete(*self.arm_dev_tree.get_children())
  for i,step in enumerate(self.arm_dev_steps):self.arm_dev_tree.insert('','end',iid=str(i),values=(i+1,step.get('operation','대기 '+str(step.get('wait_s'))),step.get('timeout_s',60)))
 def _aw_step_select(self):
  if not self.arm_dev_tree.selection():return
  step=self.arm_dev_steps[int(self.arm_dev_tree.selection()[0])]
  if 'operation' in step:self.arm_dev_op.set(step['operation']);self.arm_dev_step_op.set(step['operation'])
 def _aw_template(self):self.arm_dev_steps=template(self.arm_dev_template.get());self._aw_render_steps()
 def _aw_add_step(self,wait=False):
  step=dict(wait_s=float(self.arm_dev_wait.get()),timeout_s=float(self.arm_dev_timeout.get())) if wait else dict(operation=self.arm_dev_step_op.get().strip(),timeout_s=float(self.arm_dev_timeout.get()))
  candidate=dict(self.arm_dev_config,programs={'_draft_':self.arm_dev_steps+[step]});validate(candidate);self.arm_dev_steps.append(step);self._aw_render_steps()
 def _aw_move_step(self,delta):
  if not self.arm_dev_tree.selection():return
  i=int(self.arm_dev_tree.selection()[0])
  if delta is None:self.arm_dev_steps.pop(i)
  elif 0<=i+delta<len(self.arm_dev_steps):self.arm_dev_steps[i],self.arm_dev_steps[i+delta]=self.arm_dev_steps[i+delta],self.arm_dev_steps[i]
  self._aw_render_steps()
 def _aw_program_save(self):
  candidate=copy.deepcopy(self.arm_dev_config);candidate['programs'][self.arm_dev_program.get().strip()]=copy.deepcopy(self.arm_dev_steps);validate(candidate);self.arm_dev_config=candidate;self.arm_dev_program_combo.configure(values=list(candidate['programs']))
 def _aw_play(self):self._aw_program_save();self.arm_dev_sim.start(self.arm_dev_config,program=self.arm_dev_program.get().strip());self.arm_sim_owner='workspace'
 def _aw_pause(self):
  if getattr(self,'arm_sim_owner','workspace')=='mission' and self.studio_runner.active:
   if self.studio_runner.status=='PAUSED':self.studio_runner.resume(time.monotonic())
   else:self.studio_runner.pause(time.monotonic())
   return
  if self.arm_dev_sim.state=='RUNNING':self.arm_dev_sim.state='PAUSED'
  elif self.arm_dev_sim.state=='PAUSED':self.arm_dev_sim.state='RUNNING'
 def _aw_stop(self):
  if getattr(self,'arm_sim_owner','workspace')=='mission' and self.studio_runner.active:self.studio_runner.cancel()
  else:self.arm_dev_sim.stop()
 def _aw_save(self):
  validate(self.arm_dev_config);self.studio_config['arm_workspace']=dict(urdf=self.arm_dev_urdf,joints_rad=self.arm_dev_sim.q,operations=copy.deepcopy(self.arm_dev_config['operations']),programs=copy.deepcopy(self.arm_dev_config['programs']),physics=self.arm_dev_sim.physics.snapshot());self._studio_save_settings();self.log('SIM','로봇팔 개발 설정 저장')
 def _aw_load_urdf(self):
  if self.arm_dev_sim.state in ('RUNNING','PAUSED'):raise ValueError('재생 정지 후 모델을 변경하세요.')
  path=filedialog.askopenfilename(filetypes=[('URDF','*.urdf')])
  if not path:return
  asset=RobotDescription.load(path);scene=self.arm_dev_sim.physics.snapshot();self.arm_dev_sim=ArmSimulator(asset);self.arm_dev_sim.physics.restore(scene);self.arm_dev_canvas.sim=self.arm_dev_sim;self.arm_dev_urdf=path;self._aw_joint_controls();self.arm_dev_canvas.fit()
  if asset.warnings:self.log('WARN','URDF: '+'; '.join(asset.warnings))
 def _aw_use_official(self,path):
  if self.arm_dev_sim.state in ('RUNNING','PAUSED') or self.studio_runner.active:raise ValueError('현재 작업을 정지한 뒤 공식 모델을 적용하세요.')
  asset=RobotDescription.load(path);previous=list(self.arm_dev_sim.q);scene=self.arm_dev_sim.physics.snapshot()
  self.arm_dev_sim=ArmSimulator(asset);self.arm_dev_sim.physics.restore(scene);self.arm_dev_sim.q=self.arm_dev_sim.kin.clamp(previous if len(previous)==6 else [0]*6)
  self.arm_dev_canvas.sim=self.arm_dev_sim;self.arm_dev_urdf=str(path);self._aw_joint_controls();self.arm_dev_display_q=None;self.arm_dev_canvas.fit();self._aw_save()
  # Apply the same model in the AMR + arm viewer without transmitting hardware commands.
  old_joints=self.world3d.arm_asset.movable();old_values=self.sim.arm.get('joint_positions',{})
  self.world3d.load_asset('arm',path);self.world3d.positions['arm']=self.arm_dev_sim.kin.positions(self.arm_dev_sim.q)
  if old_values and len(old_joints)==6:
   self.sim.arm['joint_positions']={new['name']:old_values.get(old['name'],0.) for old,new in zip(old_joints,asset.movable())}
  self.spatial_assets['arm']=dict(path=str(path),root='',scale=1.)
  self._spatial_save();self.log('MODEL','공식 FR5 ROS2 모델 적용: '+str(path))
 def _aw_official_model(self):
  if self.arm_dev_sim.state in ('RUNNING','PAUSED') or self.studio_runner.active:raise ValueError('작업을 정지한 뒤 모델을 변경하세요.')
  from .fairino_model import installed,install
  from .app import USER_DIR
  path=installed(USER_DIR)
  if path:self._aw_use_official(path);return
  self.log('INFO','FAIRINO 공식 FR5 모델 다운로드 중 · 완료 후 자동 적용')
  self._studio_submit(lambda:install(USER_DIR),self._aw_use_official)
 def _aw_copy_api(self):
  if self.arm_dev_sim.state in ('RUNNING','PAUSED'):raise ValueError('재생 정지 후 가져오세요.')
  cfg=self.studio_config['arm'];validate(cfg);self.arm_dev_config=profile();self.arm_dev_config.update(operations=copy.deepcopy(cfg.get('operations',{})),programs=copy.deepcopy(cfg.get('programs',{})));self._aw_refresh_ops();self.arm_dev_program_combo.configure(values=list(self.arm_dev_config['programs']))
 def _aw_import(self):
  if self.arm_dev_sim.state in ('RUNNING','PAUSED'):raise ValueError('재생 정지 후 가져오세요.')
  path=filedialog.askopenfilename(filetypes=[('로봇팔 작업','*.json')])
  if not path:return
  data=json.loads(Path(path).read_text(encoding='utf-8'))
  if data.get('format')!=FORMAT:raise ValueError('FR5 라이브러리 형식이 아닙니다.')
  cfg=profile();cfg.update(operations=data.get('operations',{}),programs=data.get('programs',{}));validate(cfg);self.arm_dev_config=cfg;self._aw_refresh_ops();self.arm_dev_program_combo.configure(values=list(cfg['programs']));self.arm_dev_steps=[];self._aw_render_steps()
 def _aw_export(self):
  path=filedialog.asksaveasfilename(defaultextension='.json',initialfile='arm_sim_task_library.json')
  if path:
   value=library(self.arm_dev_config);value['simulation_model']=self.arm_dev_urdf;value['hardware_verified']=False;Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
 def _aw_report(self):
  win=tk.Toplevel(self);win.title('로봇팔 SIM 실행 이력');self._fit_dialog(win,650,400);box=tk.Text(win,wrap='word');box.pack(fill='both',expand=True);box.insert('1.0',json.dumps(dict(status=self.arm_dev_sim.state,error=self.arm_dev_sim.error,events=self.arm_dev_sim.events,physics_events=self.arm_dev_sim.physics.events),ensure_ascii=False,indent=2));box.configure(state='disabled')
 def _aw_adopt_loaded_asset(self,asset,path):
  if self.arm_dev_sim.state in ('RUNNING','PAUSED'):raise ValueError('팔 재생을 정지한 뒤 모델을 변경하세요.')
  previous=list(self.arm_dev_sim.q);scene=self.arm_dev_sim.physics.snapshot();sim=ArmSimulator(asset)
  sim.physics.restore(scene);sim.q=sim.kin.clamp(previous if len(previous)==len(sim.kin.joints) else [0]*len(sim.kin.joints))
  self.arm_dev_sim=sim;self.arm_dev_canvas.sim=sim;self.arm_dev_urdf=str(path);self.arm_dev_display_q=None
  self._aw_joint_controls();self._aw_sync_main();self.arm_dev_canvas.fit();self._aw_save()
 def _aw_set_synced_joints(self,q):
  self.arm_dev_sim.set_joints(q);self._aw_sync_main();self.arm_dev_canvas.render()
 def _aw_sync_main(self):
  sim=self.arm_dev_sim;view=self.world3d
  if view.arm_asset is not sim.kin.asset:view.arm_asset=sim.kin.asset;view.arm_asset.synthetic=True;view.scales['arm']=1.
  values=sim.kin.positions(sim.q)
  if not self.real:
   self.sim.arm['joint_positions']=dict(values);view.positions['arm']=dict(values)
   self.sim.arm['status']=sim.state;self.sim.arm['progress']=min(1.,sim.index/max(1,len(sim.actions)))
  if self.view_mode.get()=='3D' and view.winfo_ismapped():self._spatial_render()
 def _aw_tick(self):
  if not self.winfo_exists():return
  now=time.monotonic()
  if getattr(self,'arm_sim_owner','workspace')!='mission':self.arm_dev_sim.tick(min(.15,now-self.arm_dev_time))
  self.arm_dev_time=now;self._aw_sync_main()
  if self.tabs.select()==str(self.arm_workspace_page):
   if tuple(self.arm_dev_sim.q)!=getattr(self,'arm_dev_display_q',None):
    self.arm_dev_syncing=True
    for joint,v,q in zip(self.arm_dev_sim.kin.joints,self.arm_dev_vars,self.arm_dev_sim.q):v.set(round(q if joint['type']=='prismatic' else math.degrees(q),3))
    self.arm_dev_syncing=False;self.arm_dev_display_q=tuple(self.arm_dev_sim.q)
   pose=self.arm_dev_sim.kin.pose(self.arm_dev_sim.q);self.arm_dev_feedback.set(f'{self.arm_dev_sim.state} · 단계 {min(self.arm_dev_sim.index+1,len(self.arm_dev_sim.actions))}/{len(self.arm_dev_sim.actions)} · TCP [mm/도]: '+', '.join(f'{v:.1f}' for v in pose)+(' · '+self.arm_dev_sim.error if self.arm_dev_sim.error else ''))
   self.arm_dev_io_text.set(json.dumps(self.arm_dev_sim.io,ensure_ascii=False));self._aps_refresh();self.arm_dev_canvas.render()
  self.after(80,self._aw_tick)
