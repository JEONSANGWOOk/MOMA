"""2D/3D switching, model imports and preview-only joint controls."""
import math
import time
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from .geometry3d import RobotDescription
from .world3d import WorldView3D
from .theme import BG, PANEL, INK, MUTED, BLUE


class SpatialMixin:
    def _spatial_build(self,toolbar,viewport):
        self.view_mode=tk.StringVar(value='2D');self.camera_view=tk.StringVar(value='사선');self.projection_view=tk.StringVar(value='원근')
        self.world3d=WorldView3D(viewport,self,self._spatial_demo_path())
        self.spatial_assets={}
        for mode in ('2D','3D'):
            tk.Radiobutton(toolbar,text=mode,variable=self.view_mode,value=mode,command=self._spatial_switch,bg=PANEL,activebackground=PANEL,fg=INK,selectcolor='#ffffff').pack(side='left',padx=2)
        camera=ttk.Combobox(toolbar,textvariable=self.camera_view,values=['사선','위','정면','측면','뒤','로봇 추적','뒤따라 보기'],state='readonly',width=10)
        camera.pack(side='left',padx=3);camera.bind('<<ComboboxSelected>>',lambda event:self._spatial_camera())
        projection=ttk.Combobox(toolbar,textvariable=self.projection_view,values=['원근','직교'],state='readonly',width=6)
        projection.pack(side='left',padx=3);projection.bind('<<ComboboxSelected>>',lambda event:self._spatial_projection())
        self.button(toolbar,'3D 모델 / 관절',self._spatial_dialog,BLUE).pack(side='left',padx=3)

    def _spatial_demo_path(self):
        from .app import ROOT
        return ROOT/'examples/fairino_fr5.urdf'

    def _spatial_switch(self):
        self.release_drive()
        if self.view_mode.get()=='3D':
            self.operation_canvas.pack_forget();self.world3d.pack(fill='both',expand=True)
        else:
            self.world3d.pack_forget();self.operation_canvas.pack(fill='both',expand=True)
        self.after_idle(self.draw_map)

    def _spatial_camera(self):
        self.view_mode.set('3D');self._spatial_switch()
        if self.camera_view.get() in ('로봇 추적','뒤따라 보기'):
            self.world3d.chase=self.camera_view.get()=='뒤따라 보기';self.world3d.focus_robot()
        else:self.world3d.chase=False;self.world3d.follow.set(False);self.world3d.preset(self.camera_view.get())

    def _spatial_projection(self):
        self.world3d.camera.perspective=self.projection_view.get()=='원근'
        self.view_mode.set('3D');self._spatial_switch()

    def _spatial_render(self):
        state=dict(self.current_state())
        if self.real and all(type(state.get(k)) in (int,float) and math.isfinite(state[k]) for k in ('x','y','theta')):
            state['x'],state['y']=self._lidar_map_xy(state['x'],state['y']);state['theta']+=self.lidar_map_alignment[2]
        joints=state.get('joint_positions',state.get('joint_states',{}))
        if isinstance(joints,dict) and isinstance(joints.get('name'),list) and isinstance(joints.get('position'),list):
            joints=dict(zip(joints['name'],joints['position']))
        if not isinstance(joints,dict):joints={}
        if not self.real:
            arm=self.sim.arm;joints=arm.get('joint_positions',{})
            route=self.sim.navigation_points()
        else:
            arm=dict(status='REAL',pose='UNKNOWN');route=[]
        if self.real:
            lidar=self.real_laser_points if 'laser_beams(WORLD)' in self.lidar_source else [self._lidar_map_xy(x,y) for x,y in self.real_laser_points]
            if not self.connected or time.monotonic()-self.last_laser_rx>2:lidar=[]
        else:lidar=self.sim.scan() if self.connected and self.sim_powered else []
        self.world3d.draw_scene(self.map,state,arm,route,joints,lidar if self.layers['LiDAR'].get() else [])

    def _spatial_save(self):
        if not hasattr(self,'studio_config'):return
        view=self.world3d
        self.studio_config['viewer3d']=dict(view=self.view_mode.get(),camera=self.camera_view.get(),projection=self.projection_view.get(),
            mount=view.mount,cad_origin=view.cad_origin,wall_height=view.wall_height,layers={k:v.get() for k,v in view.layers.items()},assets=self.spatial_assets)
        self._studio_save_settings()

    def _spatial_restore(self):
        config=self.studio_config.get('viewer3d',{})
        if not isinstance(config,dict):return
        view=self.world3d
        for key in ('mount','cad_origin'):
            values=config.get(key)
            if isinstance(values,list) and len(values)==6 and all(type(v) in (int,float) and math.isfinite(v) for v in values):setattr(view,key,values)
        height=config.get('wall_height')
        if view.mount==[0.,0.,.4285,0.,0.,0.] and not config.get('assets',{}):view.mount=[0.,0.,.308,0.,0.,0.]
        if type(height) in (int,float) and .02<=height<=10:view.wall_height=height
        for key,value in (config.get('layers') if isinstance(config.get('layers'),dict) else {}).items():
            if key in view.layers and type(value) is bool:view.layers[key].set(value)
        for role,spec in (config.get('assets') if isinstance(config.get('assets'),dict) else {}).items():
            if role not in ('arm','amr','cad') or not isinstance(spec,dict):continue
            try:
                scale=float(spec.get('scale',1))
                if not math.isfinite(scale) or scale<=0:raise ValueError('3D 모델 배율이 비정상입니다.')
                view.load_asset(role,spec['path'],spec.get('root'),scale);self.spatial_assets[role]=spec
            except (OSError,ValueError,TypeError,KeyError) as error:self.log('3D',f'{role} 모델 복원 실패: {error}')
        self.camera_view.set(config.get('camera','사선'));view.preset(self.camera_view.get())
        self.projection_view.set('직교' if config.get('projection')=='직교' else '원근');view.camera.perspective=self.projection_view.get()=='원근'
        self.view_mode.set('3D' if config.get('view')=='3D' else '2D');self._spatial_switch()
        if self.camera_view.get() in ('로봇 추적','뒤따라 보기'):
            view.chase=self.camera_view.get()=='뒤따라 보기';view.focus_robot()

    def _spatial_dialog(self):
        self.tabs.select(self.operation_page);self.view_mode.set('3D');self._spatial_switch()
        win=tk.Toplevel(self);win.title('3D 모델 / 관절 / 표시 설정');self._fit_dialog(win,840,680,680,540);win.configure(bg=BG);win.transient(self)
        view=self.world3d
        _,body,_canvas=self._scrollable_frame(win,bg=PANEL)
        container=body.master.master;container.pack(fill='both',expand=True,padx=8,pady=8)
        self.label(body,'3D 공간 · AMR + 로봇팔',14,INK,True).pack(anchor='w',padx=12,pady=(10,5))
        self.label(body,'관절 슬라이더는 화면 미리보기용입니다. 기본 팔은 SIM 미션에 연동됩니다.\nURDF: link/visual/joint 변환 · CAD: STL/OBJ 정적 메시 · STEP/IGES는 STL/OBJ로 내보내세요.',9,MUTED,justify='left').pack(fill='x',padx=12,pady=5)
        row=tk.Frame(body,bg=PANEL);row.pack(fill='x',padx=12,pady=5)
        for key,var in view.layers.items():tk.Checkbutton(row,text=key,variable=var,bg=PANEL,command=view.refresh).pack(side='left')
        row=tk.Frame(body,bg=PANEL);row.pack(fill='x',padx=12,pady=5)
        tk.Checkbutton(row,text='로봇 추적',variable=view.follow,bg=PANEL,command=lambda:view.focus_robot() if view.follow.get() else view.refresh()).pack(side='left')
        tk.Checkbutton(row,text='미션 / 수신 관절 연동',variable=view.arm_follow,bg=PANEL,command=view.refresh).pack(side='left',padx=10)
        self.button(row,'SIM 팔 동작 보기',lambda:self.guarded(self._studio_test_arm),BLUE).pack(side='left')
        self.button(row,'지도 전체 보기',view.fit).pack(side='left',padx=6)
        row=tk.Frame(body,bg=PANEL);row.pack(fill='x',padx=12,pady=5)
        role=tk.StringVar(value='로봇팔');units=tk.StringVar(value='m');root=tk.StringVar(value='')
        ttk.Combobox(row,textvariable=role,values=['AMR','로봇팔','장면 CAD'],state='readonly',width=12).pack(side='left')
        ttk.Combobox(row,textvariable=units,values=['m','mm'],state='readonly',width=5).pack(side='left',padx=5)
        summary=self.label(body,'',9,MUTED,justify='left',wraplength=760);summary.pack(fill='x',padx=12,pady=5)
        joint_frame=tk.Frame(body,bg=PANEL)
        def refresh_summary():
            summary.configure(text=f"AMR: {getattr(view.body_asset,'name','기본 AMR')} · 팔: {view.arm_asset.name} · CAD: {getattr(view.cad_asset,'name','없음')}\n"+(' | '.join(getattr(view.arm_asset,'warnings',[])) or 'URDF 단위는 m/rad. package:// 메시가 있으면 패키지 루트 폴더를 지정하세요.'))
        def joint_controls():
            for child in joint_frame.winfo_children():child.destroy()
            role_key={'AMR':'amr','로봇팔':'arm','장면 CAD':'cad'}[role.get()]
            asset=getattr(view,{'amr':'body_asset','arm':'arm_asset','cad':'cad_asset'}[role_key])
            if not isinstance(asset,RobotDescription):self.label(joint_frame,'정적 메시 모델에는 관절이 없습니다.',9,MUTED).pack(anchor='w');return
            for joint in asset.movable():
                name=joint['name'];prismatic=joint['type']=='prismatic';factor=1 if prismatic else 180/math.pi
                row=tk.Frame(joint_frame,bg=PANEL);row.pack(fill='x',pady=2)
                self.label(row,name+(' (m)' if prismatic else ' (°)'),9,INK,width=22,anchor='w').pack(side='left')
                lower=joint['lower'] if joint['type']!='continuous' else -math.pi;upper=joint['upper'] if joint['type']!='continuous' else math.pi
                variable=tk.DoubleVar(value=view.positions[role_key].get(name,0)*factor)
                def changed(value,key=name,part=role_key,scale=factor):
                    view.arm_follow.set(False);view.positions[part][key]=float(value)/scale;view.refresh()
                tk.Scale(row,from_=lower*factor,to=upper*factor,variable=variable,orient='horizontal',resolution=.001 if prismatic else 1,
                         showvalue=True,bg=PANEL,highlightthickness=0,length=420,command=changed).pack(side='left',fill='x',expand=True)
        def load(kind):
            path=filedialog.askopenfilename(parent=win,filetypes=[('URDF','*.urdf')] if kind=='urdf' else [('3D mesh','*.stl *.obj')])
            if not path:return
            def apply():
                key={'AMR':'amr','로봇팔':'arm','장면 CAD':'cad'}[role.get()];scale=1. if kind=='urdf' or units.get()=='m' else .001
                asset=view.load_asset(key,path,root.get() or None,scale)
                self.spatial_assets[key]=dict(path=path,root=root.get(),scale=scale)
                refresh_summary();joint_controls();self._spatial_save()
                warnings=getattr(asset,'warnings',[])
                if warnings:messagebox.showwarning('URDF 가져오기: 일부 visual 제외','\n'.join(warnings[:15]),parent=win)
            self.guarded(apply)
        self.button(row,'URDF 가져오기',lambda:load('urdf'),BLUE).pack(side='left',padx=5)
        self.button(row,'STL / OBJ 가져오기',lambda:load('mesh')).pack(side='left',padx=5)
        def restore_demo():
            view.arm_asset=RobotDescription.load(self._spatial_demo_path());view.arm_asset.synthetic=True;view.positions['arm']={};view.arm_follow.set(True)
            view.scales['arm']=1.;view.last_arm_operation=None
            self.spatial_assets.pop('arm',None);refresh_summary();joint_controls();view.refresh();self._spatial_save()
        self.button(row,'기본 FR5 복원',restore_demo).pack(side='left',padx=5)
        row=tk.Frame(body,bg=PANEL);row.pack(fill='x',padx=12,pady=3)
        self.label(row,'기본 모델: SEER AMB-CSW04-CE + FAIRINO FR5 · 경량 시각화 모델',9,MUTED).pack(side='left')
        def restore_amr():
            view.body_asset=RobotDescription.load(self._spatial_demo_path().parent/'seer_amb_csw04_ce.urdf')
            view.positions['amr']={};view.scales['amr']=1.;self.spatial_assets.pop('amr',None)
            refresh_summary();joint_controls();view.refresh();self._spatial_save()
        self.button(row,'기본 SEER 복원',restore_amr).pack(side='left',padx=5)
        row=tk.Frame(body,bg=PANEL);row.pack(fill='x',padx=12,pady=5)
        self.label(row,'메시 패키지 루트',9,INK).pack(side='left');ttk.Entry(row,textvariable=root,width=48).pack(side='left',padx=5)
        self.button(row,'폴더',lambda:root.set(filedialog.askdirectory(parent=win) or root.get())).pack(side='left')
        self.label(body,'로봇팔 장착 위치 (AMR 좌표계) · m / °',10,INK,True).pack(anchor='w',padx=12,pady=(10,4))
        mount_values=[]
        for offset in (0,3):
            row=tk.Frame(body,bg=PANEL);row.pack(fill='x',padx=12,pady=3)
            for index,name in enumerate(('X','Y','Z') if offset==0 else ('Roll','Pitch','Yaw')):
                i=offset+index;value=view.mount[i] if i<3 else math.degrees(view.mount[i]);var=tk.StringVar(value=f'{value:.3f}');mount_values.append(var)
                self.label(row,name,9,INK,width=6).pack(side='left');ttk.Entry(row,textvariable=var,width=10).pack(side='left',padx=(0,12))
        row=tk.Frame(body,bg=PANEL);row.pack(fill='x',padx=12,pady=4)
        height=tk.StringVar(value=str(view.wall_height));self.label(row,'2D 벽 표시 높이 (m)',9,INK).pack(side='left');ttk.Entry(row,textvariable=height,width=8).pack(side='left',padx=6)
        def apply_mount():
            values=[float(var.get()) for var in mount_values];wall=float(height.get())
            if not all(math.isfinite(v) for v in values) or not math.isfinite(wall) or not .02<=wall<=10:raise ValueError('위치/벽 높이를 확인하세요.')
            view.mount=values[:3]+[math.radians(v) for v in values[3:]];view.wall_height=wall;view.refresh();self._spatial_save()
        self.button(row,'적용 / 저장',lambda:self.guarded(apply_mount),BLUE).pack(side='left')
        self.label(body,'장면 CAD 배치 (지도 좌표계) · m / °',10,INK,True).pack(anchor='w',padx=12,pady=(10,4))
        cad_values=[]
        for offset in (0,3):
            row=tk.Frame(body,bg=PANEL);row.pack(fill='x',padx=12,pady=3)
            for index,name in enumerate(('X','Y','Z') if offset==0 else ('Roll','Pitch','Yaw')):
                i=offset+index;value=view.cad_origin[i] if i<3 else math.degrees(view.cad_origin[i])
                var=tk.StringVar(value=f'{value:.3f}');cad_values.append(var)
                self.label(row,name,9,INK,width=6).pack(side='left');ttk.Entry(row,textvariable=var,width=10).pack(side='left',padx=(0,12))
        def apply_cad():
            values=[float(var.get()) for var in cad_values]
            if not all(math.isfinite(v) for v in values):raise ValueError('CAD 배치 좌표를 확인하세요.')
            view.cad_origin=values[:3]+[math.radians(v) for v in values[3:]];view.refresh();self._spatial_save()
        self.button(body,'CAD 배치 적용 / 저장',lambda:self.guarded(apply_cad),BLUE).pack(anchor='w',padx=12,pady=4)
        self.label(body,'관절 미리보기 · 수동 조절 시 미션 연동이 해제됩니다.',10,INK,True).pack(anchor='w',padx=12,pady=(10,4))
        joint_frame.pack(fill='x',padx=12,pady=(0,12))
        role.trace_add('write',lambda *args:joint_controls());refresh_summary();joint_controls()
        self.button(win,'닫기',win.destroy).pack(side='bottom',pady=5)
        win.joint_frame=joint_frame
        return win
