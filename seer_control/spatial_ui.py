"""2D/3D switching, model imports and preview-only joint controls."""
import math
import time
import copy
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from .geometry3d import RobotDescription, axis_rotation, box, identity, multiply, point, transform
from .world3d import WorldView3D, SIM_PANEL_CENTER, SIM_PANEL_SIZE
from .theme import BG, PANEL, INK, MUTED, BLUE


def _build_triangle_bvh(faces,leaf_size=12):
    triangles=[]
    for face in faces:
        if len(face)!=3:continue
        lo=tuple(min(p[i] for p in face) for i in range(3));hi=tuple(max(p[i] for p in face) for i in range(3))
        triangles.append((face,lo,hi,tuple(sum(p[i] for p in face)/3 for i in range(3))))
    def build(items):
        lo=tuple(min(item[1][i] for item in items) for i in range(3));hi=tuple(max(item[2][i] for item in items) for i in range(3))
        if len(items)<=leaf_size:return (lo,hi,items,None,None)
        axis=max(range(3),key=lambda i:hi[i]-lo[i]);items=sorted(items,key=lambda item:item[3][axis]);middle=len(items)//2
        return (lo,hi,None,build(items[:middle]),build(items[middle:]))
    return build(triangles) if triangles else None


def _triangle_box_overlap(triangle,half):
    edges=[tuple(triangle[(i+1)%3][k]-triangle[i][k] for k in range(3)) for i in range(3)]
    axes=[(1.,0.,0.),(0.,1.,0.),(0.,0.,1.),
          (edges[0][1]*edges[1][2]-edges[0][2]*edges[1][1],edges[0][2]*edges[1][0]-edges[0][0]*edges[1][2],edges[0][0]*edges[1][1]-edges[0][1]*edges[1][0])]
    unit=((1.,0.,0.),(0.,1.,0.),(0.,0.,1.))
    for edge in edges:
        for axis in unit:axes.append((edge[1]*axis[2]-edge[2]*axis[1],edge[2]*axis[0]-edge[0]*axis[2],edge[0]*axis[1]-edge[1]*axis[0]))
    for axis in axes:
        if sum(v*v for v in axis)<1e-16:continue
        projections=[sum(p[i]*axis[i] for i in range(3)) for p in triangle]
        radius=sum(half[i]*abs(axis[i]) for i in range(3))
        if min(projections)>radius or max(projections)<-radius:return False
    return True


def _obb_intersects_mesh(box,mesh_inverse,tree,margin=.003):
    if tree is None:return False
    center=point(mesh_inverse,box[0])
    axes=tuple(tuple(sum(mesh_inverse[r][k]*axis[k] for k in range(3)) for r in range(3)) for axis in box[1])
    half=tuple(v+margin for v in box[2]);extents=tuple(sum(abs(axes[j][i])*half[j] for j in range(3)) for i in range(3))
    query_lo=tuple(center[i]-extents[i] for i in range(3));query_hi=tuple(center[i]+extents[i] for i in range(3))
    stack=[tree]
    while stack:
        lo,hi,items,left,right=stack.pop()
        if any(hi[i]<query_lo[i] or lo[i]>query_hi[i] for i in range(3)):continue
        if items is None:stack.extend((left,right));continue
        for face,tlo,thi,_centroid in items:
            if any(thi[i]<query_lo[i] or tlo[i]>query_hi[i] for i in range(3)):continue
            local=tuple(tuple(sum(axes[j][k]*(p[k]-center[k]) for k in range(3)) for j in range(3)) for p in face)
            if _triangle_box_overlap(local,half):return True
    return False


class SpatialMixin:
    def _spatial_build(self,toolbar,viewport):
        self.view_mode=tk.StringVar(value='2D');self.camera_view=tk.StringVar(value='사선');self.projection_view=tk.StringVar(value='원근')
        self.world3d=WorldView3D(viewport,self,self._spatial_demo_path())
        self.spatial_assets={}
        for mode in ('2D','3D'):
            tk.Radiobutton(toolbar,text=mode,variable=self.view_mode,value=mode,command=self._spatial_switch,bg=PANEL,activebackground=PANEL,fg=INK,selectcolor='#ffffff').pack(side='left',padx=2)
        self.sim_pose_overlay=tk.BooleanVar(value=True)
        ttk.Checkbutton(toolbar,text='SIM 비교',variable=self.sim_pose_overlay,command=self._spatial_render).pack(side='left',padx=2)
        camera=ttk.Combobox(toolbar,textvariable=self.camera_view,values=['사선','위','정면','측면','뒤','사진형','로봇 추적','뒤따라 보기'],state='readonly',width=10)
        camera.pack(side='left',padx=3);camera.bind('<<ComboboxSelected>>',lambda event:self._spatial_camera())
        projection=ttk.Combobox(toolbar,textvariable=self.projection_view,values=['원근','직교'],state='readonly',width=6)
        projection.pack(side='left',padx=3);projection.bind('<<ComboboxSelected>>',lambda event:self._spatial_projection())
        self.door_demo_speed=tk.StringVar(value='300%')
        ttk.Label(toolbar,text='Speed').pack(side='left',padx=(5,1))
        ttk.Combobox(toolbar,textvariable=self.door_demo_speed,values=('50%','75%','100%','150%','200%','300%'),state='readonly',width=5).pack(side='left',padx=2)
        self.button(toolbar,'3D 모델 / 관절',self._spatial_dialog,BLUE).pack(side='left',padx=3)
        self.button(toolbar,'설비 배치',lambda:self.guarded(self._spatial_equipment_placement),BLUE).pack(side='left',padx=3)
        from .departure_link import show_departure_link
        self.button(toolbar,'LM2→LM1 연동',lambda:show_departure_link(self),BLUE).pack(side='left',padx=3)
        from .runtime_scenario import show_runtime_scenario
        self.button(toolbar,'전체 동작 main2',lambda:show_runtime_scenario(self),BLUE).pack(side='left',padx=3)
        self.button(toolbar,'팔 홈',lambda:self.guarded(self._spatial_home_arm),BLUE).pack(side='left',padx=3)
        self.button(toolbar,'3D 문 열기 / 닫기',lambda:self.guarded(self._spatial_single_door_dialog),BLUE).pack(side='left',padx=3)
        self.button(toolbar,'SIM: Doors 120° + vacuum panel',lambda:self.guarded(self._spatial_run_hook_demo),BLUE).pack(side='left',padx=3)
        self.button(toolbar,'SIM: Vacuum pick / extract panel',lambda:self.guarded(self._spatial_start_panel_pick),BLUE).pack(side='left',padx=3)

    def _spatial_single_door_dialog(self):
        window=tk.Toplevel(self);window.title('3D 표시 · 한쪽 문 열기 / 닫기')
        window.transient(self)
        ttk.Label(window,text='설비 정면 기준 왼쪽 / 오른쪽 · 3D 문 표시만 변경합니다.').pack(padx=16,pady=12)
        for side,title in (('left','왼쪽 문'),('right','오른쪽 문')):
            row=ttk.Frame(window);row.pack(fill='x',padx=16,pady=6)
            ttk.Label(row,text=title,width=12).pack(side='left')
            ttk.Button(row,text='열기 120°',command=lambda side=side:self.guarded(lambda:self._spatial_set_single_door(side,True))).pack(side='left',padx=4)
            ttk.Button(row,text='닫기',command=lambda side=side:self.guarded(lambda:self._spatial_set_single_door(side,False))).pack(side='left',padx=4)
        initial_vars={}
        for side,title in (('left','왼쪽 초기 각도'),('right','오른쪽 초기 각도')):
            row=ttk.Frame(window);row.pack(fill='x',padx=16,pady=4)
            ttk.Label(row,text=title,width=16).pack(side='left')
            value=tk.StringVar(value=str(getattr(self.world3d,'door_initial_angles',{}).get(side,0.)))
            initial_vars[side]=value
            ttk.Spinbox(row,from_=0,to=120,increment=5,textvariable=value,width=8).pack(side='left')
            ttk.Label(row,text='° (닫힘 0 / 열림 120)').pack(side='left',padx=6)
        ttk.Button(window,text='초기 각도 적용 · 저장',command=lambda:self.guarded(lambda:self._spatial_set_initial_doors({k:v.get() for k,v in initial_vars.items()}))).pack(pady=8)
        ttk.Label(window,text='저장값은 다음 실행에도 적용됩니다. 실제 문 상태를 확인한 뒤 설정하세요.').pack(padx=16,pady=4)
        ttk.Label(window,text='반대쪽 문 각도 유지 · 실기 연결 중에도 3D 표시만 변경합니다.').pack(padx=16,pady=12)
        ttk.Button(window,text='로봇 동작 연동 시작',command=lambda:self.guarded(self._spatial_follow_door_display)).pack(pady=(0,8))
        ttk.Button(window,text='실측 문 표시로 복귀',command=lambda:self.guarded(self._spatial_restore_door_display)).pack(pady=(0,12))

    def _spatial_publish_door_state(self,command=False):
        from .door_sync import publish
        return publish(self,command=command)

    def _spatial_set_initial_doors(self,values):
        angles={side:float(values[side]) for side in ('left','right')}
        if not all(math.isfinite(v) and 0<=v<=120 for v in angles.values()):
            raise ValueError('문 초기 각도: 0~120°')
        view=self.world3d
        if view._hook_demo:raise ValueError('문/판넬 동작 종료 후 초기 각도를 설정하세요.')
        view.door_initial_angles=angles
        view.manual_real_door_display=None
        view.webcam_doors_enabled.set(False)
        view.cad_doors_auto.set(False);view.cad_door_hold_angle=None;view.cad_door_hold_angles={}
        follower=view._real_door_follower
        if follower:
            follower.detach()
            for name in follower.angles:
                side='left' if name.endswith(':1') else 'right' if name.endswith(':2') else None
                follower.angles[name]=math.radians(angles.get(side,0.))
        self._spatial_save();self.draw_map()
        self._spatial_publish_door_state(command=True)
        self.log('VIEW',f"문 초기 각도 저장 · 왼쪽 {angles['left']}° / 오른쪽 {angles['right']}° · 사용자 설정")

    def _spatial_follow_door_display(self):
        view=self.world3d
        view.manual_real_door_display=None
        view.webcam_doors_enabled.set(False)
        view.real_doors_follow.set(True);view.arm_follow.set(True)
        self.draw_map()
        self._spatial_publish_door_state(command=True)
        self.log('VIEW','문 표시 · 실제 관절 기반 후크/롤러 접촉 추정 시작 · 힘 미측정')

    def _spatial_restore_door_display(self):
        self.world3d.manual_real_door_display=None
        self.draw_map()
        self._spatial_publish_door_state(command=True)

    def _spatial_set_single_door(self,side,opened):
        if side not in ('left','right'):raise ValueError('왼쪽 또는 오른쪽 문을 선택하세요.')
        view=self.world3d
        if view._hook_demo:raise ValueError('문 열기 또는 판넬 작업 종료 후 변경하세요.')
        doors=list(getattr(view.cad_asset,'door_parts',()))
        suffix=':1' if side=='left' else ':2'
        target=next((part for part in doors if part.get('name','').endswith(suffix)),None)
        if target is None:raise ValueError('설비 SIMPLE_WHOLE 모델의 문을 먼저 불러오세요.')
        # Freeze every leaf at its visible angle before disabling automatic motion.
        angles={part['name']:view.cad_door_angle(part) for part in doors}
        angles[target['name']]=math.radians(120.) if opened else 0.
        if self.real:
            view.manual_real_door_display=(view.cad_asset,angles)
        else:
            view.cad_doors_auto.set(False);view.cad_door_hold_angle=None
            view.cad_door_hold_angles=angles
        self.view_mode.set('3D');self._spatial_switch()
        self.draw_map()
        self._spatial_publish_door_state(command=True)
        self.log('VIEW',('왼쪽' if side=='left' else '오른쪽')+' 문 '+('열기 120°' if opened else '닫기')+' · 3D 표시')

    def _spatial_equipment_placement(self):
        from .equipment_placement_ui import show_placement
        show_placement(self)

    def _spatial_motion_speed(self):
        try:percent=int(str(self.door_demo_speed.get()).rstrip('%'))
        except (AttributeError,TypeError,ValueError):percent=100
        return max(.5,min(3.,percent/100.))

    def _spatial_home_arm(self):
        """Move the arm to its configured safe_pose through the normal runner."""
        runner=getattr(self,'studio_runner',None)
        if runner is None:raise ValueError('로봇팔 작업 실행기가 준비되지 않았습니다.')
        if runner.active or getattr(self,'task_running',False):raise ValueError('현재 미션/로봇팔 작업을 먼저 정지하세요.')
        if getattr(self,'sim',None) is not None and getattr(self.sim,'route',[]):raise ValueError('AMR을 정지한 뒤 팔 홈 동작을 실행하세요.')
        if getattr(self,'held',None):raise ValueError('현재 잡고 있는 물체를 먼저 안전하게 내려놓으세요.')
        if self.real:
            config=copy.deepcopy(self.studio_config.get('arm',{}))
            if not config.get('operations',{}).get('safe_pose'):
                raise ValueError('실기 FR5 설정에 safe_pose MoveJ 작업을 등록한 뒤 실행하세요.')
            if not getattr(getattr(self,'fr5_client',None),'connected',False) or time.monotonic()-getattr(self,'fr5_rx',0.)>2.:
                raise ValueError('최신 FR5 상태를 확인한 뒤 팔 홈 동작을 실행하세요.')
            state=self.current_state()
            if not self.connected or not self.control_enabled or time.monotonic()-self.last_state>3.:
                raise ValueError('AMR 연결과 최신 상태를 확인한 뒤 팔 홈 동작을 실행하세요.')
            if abs(float(state.get('speed',0.)))>.02 or state.get('emergency') or state.get('stopped') or state.get('task') in ('RUNNING','WAITING','SUSPENDED'):
                raise ValueError('AMR 정지 상태에서만 실기 팔 홈 동작을 실행할 수 있습니다.')
        else:
            config=copy.deepcopy(getattr(self,'arm_dev_config',{}))
            if not config.get('operations',{}).get('safe_pose'):
                config.setdefault('operations',{})['safe_pose']=dict(method='MoveJ',target=[0,-90,90,-90,-90,0],vel=20,tool=0,user=0)
                self.arm_dev_config=config
                if hasattr(self,'_aw_refresh_ops'):self._aw_refresh_ops()
        operation=config.get('operations',{}).get('safe_pose',{})
        if operation.get('method')!='MoveJ':raise ValueError('팔 홈은 관절 목표가 등록된 safe_pose MoveJ 작업이어야 합니다.')
        runner.start([dict(type='Arm Action',operation='safe_pose',timeout_s=60.)])
        self.task_running=True
        self.log('ARM','팔 홈 동작 시작 · safe_pose MoveJ')

    def _spatial_run_hook_demo(self):
        """SIM-only DD route and collision-checked 120 degree two-door sequence."""
        view=self.world3d;simulator=getattr(self,'arm_dev_sim',None)
        if self.real:raise ValueError('This motion is a SIM preview only; it never sends commands to hardware.')
        if simulator is None or view.cad_asset is None:raise ValueError('Load SIMPLE_WHOLE and the SIM arm before running this motion.')
        if view.tool_asset is None or not getattr(view.tool_asset,'name','').startswith('EndEffector V3_EndOnly_Change1-org'):
            raise ValueError('Load the EndEffector V3_EndOnly_Change1-org GLB so the door motion uses the real hook assembly.')
        if simulator.state in ('RUNNING','PAUSED') or (getattr(self,'studio_runner',None) and self.studio_runner.active):raise ValueError('Stop the current arm or mission task first.')
        if getattr(self.sim,'route',[]):raise ValueError('Stop AMR navigation before running this motion.')
        if view._hook_demo:raise ValueError('The door hook motion is already running.')
        if len(simulator.q)!=6 or any(j['type'] not in ('revolute','continuous') for j in simulator.kin.joints):raise ValueError('A six-axis revolute SIM arm is required.')
        doors={part.get('name',''):part for part in getattr(view.cad_asset,'door_parts',())}
        left=next((part for name,part in doors.items() if name.endswith(':1')),None)
        right=next((part for name,part in doors.items() if name.endswith(':2')),None)
        if left is None or right is None:raise ValueError('Could not find both front SIMPLE_WHOLE doors.')
        cad_world=transform(view.cad_origin[:3],view.cad_origin[3:]);cad_theta=float(view.cad_origin[5])
        self._spatial_sync_cad_lidar()
        from .arm_physics import inverse
        cad_rotation=transform((0.,0.,0.),view.cad_origin[3:])
        door_mesh_trees={door['name']:_build_triangle_bvh(list(door['asset'].faces)+list(getattr(door['asset'],'transparent_faces',()))) for door in (left,right)}
        tool_components=[]
        tool_names=list(getattr(view.tool_asset,'component_faces',{}))
        for index,(faces,local,_color) in enumerate(getattr(view.tool_asset,'links',{}).get('base_link',())):
            if not faces:continue
            points=[p for face in faces for p in face]
            lo=[min(p[i] for p in points) for i in range(3)];hi=[max(p[i] for p in points) for i in range(3)]
            center=tuple((a+b)/2 for a,b in zip(lo,hi));size=tuple(max(.002,b-a) for a,b in zip(lo,hi))
            tool_components.append((tool_names[index] if index<len(tool_names) else f'tool component {index+1}',local,center,size))
        def rotation_angles(desired,base_theta):
            mount_rotation=multiply(transform((0.,0.,0.),(0.,0.,base_theta)),transform((0.,0.,0.),view.mount[3:]))
            relative=multiply(inverse(mount_rotation),desired)
            pitch=math.asin(max(-1.,min(1.,-relative[2][0])))
            if abs(math.cos(pitch))>1e-7:
                roll=math.atan2(relative[2][1],relative[2][2]);yaw=math.atan2(relative[1][0],relative[0][0])
            else:roll=0.;yaw=math.atan2(-relative[0][1],relative[1][1])
            return [math.degrees(v) for v in (roll,pitch,yaw)]
        def solve_hook_pose(target,desired,base,seed,avoid_door=None,avoid_angle=0.,active_door=None,active_angle=0.,active_contact=None,active_tool_point=(0.,-.14714,.148)):
            pose=target+rotation_angles(desired,base[2]);seeds=[list(seed),[0.,-math.pi/2,math.pi/2,-math.pi/2,-math.pi/2,0.],[0.,-2.,1.,0.,1.,0.],[0.,-1.5,1.5,0.,1.5,0.],[0.,-1.,1.,0.,1.,0.],[0.,1.,-1.,0.,1.,0.]]
            errors=[];solutions=[]
            for index,candidate in enumerate(seeds):
                try:
                    solution=simulator.kin.ik(pose,candidate,iterations=240,position_tolerance=.002,rotation_tolerance=.015)
                    if all(max(abs(a-b) for a,b in zip(solution,known))>1e-4 for known in solutions):solutions.append(solution)
                except ValueError as error:errors.append(error)
            if not solutions:raise errors[0] if errors else ValueError('No collision-safe end-effector contact pose is reachable.')
            checked=[]
            if avoid_door is not None:checked.append((avoid_door,float(avoid_angle),False))
            if active_door is not None and all(active_door['name']!=item[0]['name'] for item in checked):checked.append((active_door,float(active_angle),True))
            if checked:
                arm_world=multiply(transform((base[0],base[1],0.),(0.,0.,base[2])),transform(view.mount[:3],view.mount[3:]))
                obstacles=[]
                for door,door_degrees,_active in checked:
                    faces=list(door['asset'].faces)+list(getattr(door['asset'],'transparent_faces',()))
                    vertices=[p for face in faces for p in face]
                    lo=[min(p[i] for p in vertices) for i in range(3)];hi=[max(p[i] for p in vertices) for i in range(3)]
                    center=tuple((a+b)/2 for a,b in zip(lo,hi));size=tuple(max(.002,b-a) for a,b in zip(lo,hi))
                    angle=math.radians(door_degrees)*door['sign'];pivot=door['pivot']
                    swing=multiply(multiply(transform(pivot),axis_rotation((0.,0.,1.),angle)),transform(tuple(-v for v in pivot)))
                    frame_world=multiply(cad_world,swing);world=multiply(frame_world,transform(center));mesh_inverse=inverse(multiply(inverse(arm_world),frame_world))
                    obstacles.append(dict(name='CAD door '+door['name'],matrix=multiply(inverse(arm_world),world),size=size,static=True,mesh_inverse=mesh_inverse,mesh_tree=door_mesh_trees[door['name']]))
                physics=simulator.physics;original=physics.objects;physics.objects=original+obstacles
                try:
                    safe=[]
                    for solution in solutions:
                        risks,_near=physics.contacts(solution)
                        poses=simulator.kin.asset.link_transforms(simulator.kin.positions(solution),arm_world);tool_link=simulator.kin.tip
                        contact_center=point(poses[tool_link],active_tool_point) if tool_link in poses else None
                        valid=True
                        for link,name in risks:
                            if link=='tool' and name.startswith('CAD '):continue
                            matched=next((item for item in checked if name=='CAD door '+item[0]['name']),None)
                            if matched and matched[2] and link in ('vacuum_body','cup','gripper_body','finger') and contact_center is not None:
                                door,door_degrees,_=matched
                                contact=active_contact
                                if contact is None:
                                    handle_faces=[p for face,material in zip(door['asset'].faces,door['asset'].face_materials) if material==36 for p in face]
                                    contact=tuple((min(p[i] for p in handle_faces)+max(p[i] for p in handle_faces))/2 for i in range(3)) if handle_faces else None
                                if contact is not None:
                                    angle=math.radians(door_degrees)*door['sign'];pivot=door['pivot']
                                    swing=multiply(multiply(transform(pivot),axis_rotation((0.,0.,1.),angle)),transform(tuple(-v for v in pivot)))
                                    if math.dist(contact_center,point(cad_world,point(swing,contact)))<=.03:continue
                            valid=False;break
                        if valid and active_tool_point==(0.,-.14714,.148):
                            from .arm_physics import obb,separation
                            local_poses=simulator.kin.asset.link_transforms(simulator.kin.positions(solution));tool_link=simulator.kin.tip
                            tool_root=multiply(local_poses[tool_link],transform((0.,0.,.1)))
                            arm_inverse=inverse(arm_world)
                            for component_name,component_local,component_center,component_size in tool_components:
                                component_box=obb(multiply(multiply(tool_root,component_local),transform(component_center)),component_size)
                                lower_name=component_name.lower()
                                for obstacle in obstacles:
                                    if obstacle.get('mesh_tree') is not None:
                                        if not _obb_intersects_mesh(component_box,obstacle['mesh_inverse'],obstacle['mesh_tree']):continue
                                    elif separation(component_box,obb(obstacle['matrix'],obstacle['size']))>0.:continue
                                    matched=next((item for item in checked if obstacle['name']=='CAD door '+item[0]['name']),None)
                                    permitted=False
                                    if matched and matched[2]:
                                        door,door_degrees,_=matched
                                        contact=active_contact
                                        if contact is None:
                                            handle_faces=[p for face,material in zip(door['asset'].faces,door['asset'].face_materials) if material==36 for p in face]
                                            contact=tuple((min(p[i] for p in handle_faces)+max(p[i] for p in handle_faces))/2 for i in range(3)) if handle_faces else None
                                        if contact is not None:
                                            angle=math.radians(door_degrees)*door['sign'];pivot=door['pivot']
                                            swing=multiply(multiply(transform(pivot),axis_rotation((0.,0.,1.),angle)),transform(tuple(-v for v in pivot)))
                                            intended=point(arm_inverse,point(cad_world,point(swing,contact)))
                                            contact_point=point(local_poses[tool_link],active_tool_point)
                                            at_contact=math.dist(contact_point,intended)<=.045
                                            if at_contact:
                                                if active_tool_point==(0.,-.14714,.148):permitted=('hook_v2_hook' in lower_name or 'hook_uretan_c' in lower_name)
                                                else:permitted='polymer_roecrha50-10-30-t5' in lower_name
                                    if not permitted:valid=False;break
                                if not valid:break
                        if valid:safe.append(solution)
                    if safe:solutions=safe
                    else:raise ValueError('Every IK branch intersects a door leaf outside the intended contact zone.')
                finally:physics.objects=original
            return min(solutions,key=lambda solution:sum((a-b)**2 for a,b in zip(seed,solution)))
        def plan_door(door,seed,base_at,retract=True,orientation_ratio=1.,mirror_hook=False,avoid_door=None,avoid_angle=0.,contact_local=None,contact_tool_point=None):
            if contact_local is None:
                vertices=[p for face,material in zip(door['asset'].faces,door['asset'].face_materials) if material==36 for p in face]
                if not vertices:raise ValueError('Could not find the chrome handle on '+door['name'])
                contact_local=tuple((min(p[i] for p in vertices)+max(p[i] for p in vertices))/2 for i in range(3))
            q=list(seed);paths=[];pivot=door['pivot'];sign=float(door['sign']);hook_point=(0.,-.14714,.148)
            contact_tool_point=hook_point if contact_tool_point is None else contact_tool_point
            def arm_coordinates(world_point,root):
                arm_world=multiply(transform((root[0],root[1],0.),(0.,0.,root[2])),transform(view.mount[:3],view.mount[3:]))
                delta=[world_point[i]-arm_world[i][3] for i in range(3)]
                return [sum(arm_world[j][i]*delta[j] for j in range(3))*1000 for i in range(3)]
            # Short IK increments keep the elbow/wrist sweep close to the
            # door arc instead of cutting across it between coarse poses.
            for degrees in range(0,121,2):
                base=base_at(degrees);tool_rotation=axis_rotation((0.,0.,1.),math.radians(degrees*orientation_ratio)*sign)
                door_rotation=axis_rotation((0.,0.,1.),math.radians(degrees)*sign)
                desired=multiply(multiply(multiply(cad_rotation,tool_rotation),transform((0.,0.,0.),(math.pi,0.,0.))),
                                 transform((0.,0.,0.),(0.,math.pi,0.)) if mirror_hook else transform((0.,0.,0.),(0.,0.,0.)))
                door_delta=multiply(multiply(transform(pivot),door_rotation),transform(tuple(-v for v in pivot)))
                contact_world=point(cad_world,point(door_delta,contact_local));tool_offset=point(desired,contact_tool_point)
                j6_target=tuple(contact_world[i]-tool_offset[i] for i in range(3))
                try:q=solve_hook_pose(arm_coordinates(j6_target,base),desired,base,q,avoid_door,avoid_angle,door,degrees,contact_local,contact_tool_point)
                except ValueError as error:raise ValueError(f"Cannot reach {door['name']} contact pose at {degrees} degrees from AMR base ({base[0]:.3f}, {base[1]:.3f}, {math.degrees(base[2]):.1f} degrees): {error}") from error
                paths.append((q,degrees,base,door['name']))
            if not retract:return paths
            degrees=120;base=paths[-1][2];tool_rotation=axis_rotation((0.,0.,1.),math.radians(degrees*orientation_ratio)*sign)
            door_rotation=axis_rotation((0.,0.,1.),math.radians(degrees)*sign)
            desired=multiply(multiply(multiply(cad_rotation,tool_rotation),transform((0.,0.,0.),(math.pi,0.,0.))),
                             transform((0.,0.,0.),(0.,math.pi,0.)) if mirror_hook else transform((0.,0.,0.),(0.,0.,0.)))
            door_delta=multiply(multiply(transform(pivot),door_rotation),transform(tuple(-v for v in pivot)))
            final_handle=point(cad_world,point(door_delta,contact_local))
            # Retract normal to the opened leaf itself, not the cabinet's
            # unrotated front plane, before the DD base backs away.
            escape_local=tuple(contact_local[i]+(0.,-.05,0.)[i] for i in range(3))
            escape_tip=point(cad_world,point(door_delta,escape_local));hook_offset=point(desired,contact_tool_point)
            j6_target=tuple(escape_tip[i]-hook_offset[i] for i in range(3))
            try:q=solve_hook_pose(arm_coordinates(j6_target,base),desired,base,q,door,degrees,None,0.,None,contact_tool_point)
            except ValueError as error:raise ValueError(f"Cannot retract from {door['name']} after 120 degrees: {error}") from error
            # Keep the contact state through the short 15 mm unhook motion;
            # the validator permits only the hook/gripper contact at the handle.
            paths.append((q,120,base,door['name']))
            return paths
        def collision_free_approach(start_q,goal_q,base,active_door,avoid_angle=0.,contact_local=None,contact_tool_point=(0.,-.14714,.148),fixed_angles=None):
            """Plan a joint-space detour around the neighboring closed leaf."""
            from .arm_physics import inverse,obb,separation
            physics=simulator.physics;original=physics.objects
            arm_world=multiply(transform((base[0],base[1],0.),(0.,0.,base[2])),transform(view.mount[:3],view.mount[3:]))
            arm_inverse=inverse(arm_world);obstacles=[]
            active_part=next((d for d in (left,right) if d['name']==active_door),None)
            components=[]
            tool_visuals=getattr(view.tool_asset,'links',{}).get('base_link',())
            tool_names=list(getattr(view.tool_asset,'component_faces',{}))
            for index,(faces,local,_color) in enumerate(tool_visuals):
                if not faces:continue
                points=[p for face in faces for p in face]
                lo=[min(p[i] for p in points) for i in range(3)];hi=[max(p[i] for p in points) for i in range(3)]
                center=tuple((a+b)/2 for a,b in zip(lo,hi));size=tuple(max(.002,b-a) for a,b in zip(lo,hi))
                components.append((tool_names[index] if index<len(tool_names) else f'tool component {index+1}',local,center,size))
            for door in (left,right):
                faces=list(door['asset'].faces)+list(getattr(door['asset'],'transparent_faces',()))
                vertices=[p for face in faces for p in face]
                lo=[min(p[i] for p in vertices) for i in range(3)];hi=[max(p[i] for p in vertices) for i in range(3)]
                center=tuple((a+b)/2 for a,b in zip(lo,hi));size=tuple(max(.002,b-a) for a,b in zip(lo,hi))
                door_degrees=(fixed_angles or {}).get(door['name'],0. if door['name']==active_door else avoid_angle)
                angle=math.radians(door_degrees)*door['sign'];pivot=door['pivot']
                swing=multiply(multiply(transform(pivot),axis_rotation((0.,0.,1.),angle)),transform(tuple(-v for v in pivot)))
                frame_world=multiply(cad_world,swing);world=multiply(frame_world,transform(center));mesh_inverse=inverse(multiply(arm_inverse,frame_world))
                obstacles.append(dict(name='CAD door '+door['name'],matrix=multiply(arm_inverse,world),size=size,static=True,mesh_inverse=mesh_inverse,mesh_tree=door_mesh_trees[door['name']]))
            walls=list(getattr(getattr(self,'map',None),'cad_walls',())) if fixed_angles is not None else []
            cad_points=[point(cad_world,p) for face in list(view.cad_asset.faces)+list(getattr(view.cad_asset,'transparent_faces',())) for p in face]
            if walls and cad_points:
                floor_z=min(p[2] for p in cad_points);ceiling_z=max(p[2] for p in cad_points);door_ranges=[]
                for door in (left,right):
                    faces=list(door['asset'].faces)+list(getattr(door['asset'],'transparent_faces',()))
                    pts=[p for face in faces for p in face]
                    if not pts:continue
                    lo=[min(p[i] for p in pts) for i in range(3)];hi=[max(p[i] for p in pts) for i in range(3)]
                    center=tuple((a+b)/2 for a,b in zip(lo,hi));size=tuple(max(.002,b-a) for a,b in zip(lo,hi))
                    corners=[point(cad_world,tuple(center[i]+sign*size[i]/2 for i,sign in enumerate((dx,dy,dz)))) for dx in (-1,1) for dy in (-1,1) for dz in (-1,1)]
                    door_ranges.append((min(p[0] for p in corners),max(p[0] for p in corners),min(p[1] for p in corners),max(p[1] for p in corners)))
                for index,wall in enumerate(walls):
                    x1,y1,x2,y2=wall;length=math.hypot(x2-x1,y2-y1)
                    if length<.02:continue
                    mx,my=(x1+x2)/2,(y1+y2)/2
                    if any(r[0]-.06<=mx<=r[1]+.06 and r[2]-.06<=my<=r[3]+.06 for r in door_ranges):continue
                    matrix=transform((mx,my,(floor_z+ceiling_z)/2),(0.,0.,math.atan2(y2-y1,x2-x1)))
                    obstacles.append(dict(name=f'CAD enclosure wall {index+1}',matrix=multiply(arm_inverse,matrix),size=(length,.025,max(.05,ceiling_z-floor_z)),static=True))
            physics.objects=original+obstacles
            obstacle_boxes=[]
            for obstacle in obstacles:
                bounds=obb(obstacle['matrix'],obstacle['size'])
                radius=math.sqrt(sum(v*v for v in bounds[2]))
                obstacle_boxes.append((obstacle,bounds,radius))
            collision_detail=['arm link']
            def clear(q):
                risks,_near=physics.contacts(q)
                contact=contact_local
                if contact is None and active_part is not None:
                    handle_faces=[p for face,material in zip(active_part['asset'].faces,active_part['asset'].face_materials) if material==36 for p in face]
                    contact=tuple((min(p[i] for p in handle_faces)+max(p[i] for p in handle_faces))/2 for i in range(3)) if handle_faces else None
                for link,name in risks:
                    if link=='tool' and name.startswith('CAD '):continue
                    if active_part is not None and name=='CAD door '+active_door and link in ('vacuum_body','cup','gripper_body','finger') and contact is not None:
                        poses_world=simulator.kin.asset.link_transforms(simulator.kin.positions(q),arm_world);tool_link=simulator.kin.tip
                        contact_center=point(poses_world[tool_link],contact_tool_point) if tool_link in poses_world else None
                        handle_world=point(cad_world,contact) if contact is not None else None
                        if contact_center is not None and handle_world is not None and math.dist(contact_center,handle_world)<=.03:continue
                    collision_detail[0]=f'arm link {link} with {name}'
                    return False
                tool_poses=simulator.kin.asset.link_transforms(simulator.kin.positions(q));tool_link=simulator.kin.tip
                if tool_link not in tool_poses:return False
                tool_root=multiply(tool_poses[tool_link],transform((0.,0.,.1)))
                intended_world=point(cad_world,contact) if contact is not None else None
                intended_local=point(arm_inverse,intended_world) if intended_world is not None else None
                contact_point=point(tool_poses[tool_link],contact_tool_point)
                for component_name,local,center,size in components:
                    component_pose=multiply(multiply(tool_root,local),transform(center))
                    component_box=obb(component_pose,size);component_center=component_box[0]
                    component_radius=math.sqrt(sum(v*v for v in component_box[2]))
                    for obstacle,obstacle_box,obstacle_radius in obstacle_boxes:
                        delta=[component_center[i]-obstacle_box[0][i] for i in range(3)]
                        if sum(v*v for v in delta)>(component_radius+obstacle_radius)**2:continue
                        if obstacle.get('mesh_tree') is not None:
                            if not _obb_intersects_mesh(component_box,obstacle['mesh_inverse'],obstacle['mesh_tree']):continue
                        elif separation(component_box,obstacle_box)>0.:continue
                        allowed=False;lower=component_name.lower()
                        if active_part is not None and obstacle['name']=='CAD door '+active_door and intended_local is not None and math.dist(contact_point,intended_local)<=(.035 if contact_tool_point==roller_tool_point else .045):
                            if contact_tool_point==roller_tool_point:
                                allowed=any(token in lower for token in ('polymer_roecrha50-10-30-t5','core_roecrha50-10-30-t5','bearing-1_roecrha50-10-30-t5'))
                            else:allowed=('hook_v2_hook' in lower or 'hook_uretan_c' in lower)
                        if not allowed:
                            collision_detail[0]=f"end-effector component {component_name} with {obstacle['name']}"
                            return False
                return True
            def distance(a,b):return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
            def edge_clear(a,b):
                count=max(1,math.ceil(max((abs(x-y) for x,y in zip(a,b)),default=0.)/.01))
                return all(clear([x+(y-x)*i/count for x,y in zip(a,b)]) for i in range(1,count+1))
            try:
                phase='fold/stow' if fixed_angles is not None else f'{active_door} contact'
                if not clear(start_q):raise ValueError(f'The start arm pose intersects the fixture during {phase}: {collision_detail[0]}, base={base}, joints={[round(v, 3) for v in start_q]}.')
                if not clear(goal_q):raise ValueError(f'The target arm pose intersects the fixture during {phase}: {collision_detail[0]}, base={base}, joints={[round(v, 3) for v in goal_q]}.')
                if edge_clear(start_q,goal_q):return [list(start_q),list(goal_q)]
                ranges=[(joint['lower'],joint['upper']) if joint['type']!='continuous' else (-math.pi,math.pi) for joint in simulator.kin.joints]
                def extend(tree,target):
                    near=min(range(len(tree[0])),key=lambda i:distance(tree[0][i],target));source=tree[0][near];gap=distance(source,target)
                    candidate=list(target) if gap<=.35 else [a+(b-a)*.35/gap for a,b in zip(source,target)]
                    if not edge_clear(source,candidate):return None,False
                    tree[0].append(candidate);tree[1].append(near)
                    return len(tree[0])-1,gap<=.35
                def route(tree,index):
                    result=[]
                    while index is not None:result.append(tree[0][index]);index=tree[1][index]
                    return result[::-1]
                start_tree=([[list(start_q)],[None]]);goal_tree=([[list(goal_q)],[None]])
                import random
                rng=random.Random(12)
                for _iteration in range(1800):
                    if _iteration%4:
                        fraction=rng.random()
                        sample=[max(lo,min(hi,a+(b-a)*fraction+rng.uniform(-.55,.55))) for a,b,(lo,hi) in zip(start_q,goal_q,ranges)]
                    else:sample=[rng.uniform(*limits) for limits in ranges]
                    start_index,_connected=extend(start_tree,sample)
                    if start_index is None:continue
                    while True:
                        goal_index,connected=extend(goal_tree,start_tree[0][start_index])
                        if goal_index is None:break
                        if connected:
                            return route(start_tree,start_index)+route(goal_tree,goal_index)[-2::-1]
                angles=fixed_angles if fixed_angles is not None else {door['name']:(0. if door['name']==active_door else avoid_angle) for door in (left,right)}
                raise ValueError(f'No collision-free arm route was found (active contact={active_door}, door angles={angles}).')
            finally:physics.objects=original
        state=self.current_state();theta=float(state.get('theta',0.));start_x=float(state.get('x',0.));start_y=float(state.get('y',0.))
        lm6=self.map.nodes.get('LM6',{});lm6_x=float(lm6.get('x',0.));lm6_y=float(lm6.get('y',0.))
        # The operator may dock here manually; accept the measured pose without
        # requiring last_node=LM6 or any specific navigation route history.
        if math.hypot(start_x-lm6_x,start_y-lm6_y)>.08:
            raise ValueError(f'AMR has not settled at LM6: current pose is ({start_x:.3f}, {start_y:.3f}), node is ({lm6_x:.3f}, {lm6_y:.3f}). The door sequence needs the measured position within 0.08 m; wait for localization to settle, then retry.')
        if abs(math.atan2(math.sin(theta),math.cos(theta)))>math.radians(4):raise ValueError('Stop at LM6 facing east along the node route before starting.')
        if abs(start_y-lm6_y)>.01:raise ValueError('The AMR must be on the LM6 straight-lane centerline.')
        # Approach from the centered LM6 dock. The DD base backs straight
        # along the LM6-LM7 lane while the left leaf opens, without strafe.
        left_base=(lm6_x-.20,lm6_y,0.);left_open_end_x=lm6_x-.60
        right_base=(lm6_x+.30,lm6_y,0.)
        stow_q=[0.,-math.pi/2,math.pi/2,-math.pi/2,-math.pi/2,0.]
        left_paths=plan_door(left,simulator.q,lambda degrees:(left_base[0]+(left_open_end_x-left_base[0])*max(0.,(degrees-20.)/100.),left_base[1],0.),orientation_ratio=1.,mirror_hook=True,avoid_door=right)
        left_end_base=left_paths[-1][2]
        stow_base=(left_end_base[0]-.20,left_end_base[1],0.)
        stow_candidates=(stow_q,[0.,-1.75,1.35,-1.35,-1.45,0.],[0.,-1.30,1.25,-1.50,-1.55,0.],
                         [.25,-1.55,1.15,-1.30,-1.35,.15],[-.25,-1.55,1.15,-1.30,-1.35,-.15],
                         [0.,-1.95,1.40,-1.25,-1.45,0.],[0.,-1.25,1.45,-1.75,-1.55,0.])
        safe_stow=None;stow_failures=[]
        for candidate in stow_candidates:
            try:
                collision_free_approach(candidate,candidate,stow_base,None,fixed_angles={left['name']:120.,right['name']:0.})
                safe_stow=list(candidate);break
            except ValueError as error:stow_failures.append(str(error))
        if safe_stow is None:raise ValueError('No FR5 stow pose clears the open left door. Adjust the AMR retreat distance or arm mount before running the right-door approach. '+(stow_failures[-1] if stow_failures else ''))
        stow_q=safe_stow
        def frame_push_point(door):
            # Reach the free-side front corner from the left-door opening.
            # The roller overlaps the rail by 7 mm and pushes along +X; its
            # front-side offset from the hinge produces the rightward-opening
            # moment without threading the hook through the chrome handle.
            points=[p for face in door['asset'].faces for p in face]
            if not points:raise ValueError('No solid door-frame geometry is available for '+door['name'])
            lo=[min(p[i] for p in points) for i in range(3)];hi=[max(p[i] for p in points) for i in range(3)]
            return (lo[0]-.018,lo[1]-.018,lo[2]+.30*(hi[2]-lo[2]))
        def endeffector_roller_geometry():
            components=getattr(view.tool_asset,'component_faces',{})
            visuals=getattr(view.tool_asset,'links',{}).get('base_link',())
            for (name,_count),(faces,_local,_color) in zip(components.items(),visuals):
                if 'Polymer_ROECRHA50-10-30-T5' not in name:continue
                points=[p for face in faces for p in face]
                if not points:break
                lo=[min(p[i] for p in points) for i in range(3)];hi=[max(p[i] for p in points) for i in range(3)]
                center=tuple((a+b)/2 for a,b in zip(lo,hi));size=tuple(max(.002,b-a) for a,b in zip(lo,hi))
                return center,size
            raise ValueError('Could not find the physical polymer roller in the loaded end-effector GLB.')
        roller_center,roller_size=endeffector_roller_geometry()
        roller_tool_point=(roller_center[0],roller_center[1],roller_center[2]+.1)
        right_contact=frame_push_point(right);right_strategy='roller push'
        def roller_push_base(degrees):
            if degrees<=92.:x=lm6_x+.30
            elif degrees<=104.:x=lm6_x+.30+.25*(degrees-92.)/12.
            else:x=lm6_x+.55+.17*(degrees-104.)/16.
            return (x,lm6_y,0.)
        try:
            right_paths=plan_door(right,stow_q,roller_push_base,retract=False,orientation_ratio=.5,avoid_door=left,avoid_angle=120.,contact_local=right_contact,contact_tool_point=roller_tool_point)
        except ValueError as push_error:
            raise ValueError(f'Could not plan the right-door inner-frame roller push: {push_error}') from push_error
        right_contact_spec=dict(local=right_contact,tool_point=roller_tool_point,tool_kind='roller',roller_center=roller_center,roller_size=roller_size)
        arm_samples=[];motion_segments=[]
        def require_joint_limits(q):
            bounded=simulator.kin.clamp(list(q))
            if any(abs(actual-safe)>1e-8 for actual,safe in zip(q,bounded)):
                limits=', '.join(f"{joint['name']} [{math.degrees(joint['lower']):.1f}, {math.degrees(joint['upper']):.1f}] deg" for joint in simulator.kin.joints)
                raise ValueError('Planned FR5 pose exceeds the loaded URDF joint limits: '+limits)
        def append_sweep(q0,base0,angles0,q1,base1,angles1,active):
            require_joint_limits(q0);require_joint_limits(q1)
            dx,dy=base1[0]-base0[0],base1[1]-base0[1]
            delta_theta=math.atan2(math.sin(base1[2]-base0[2]),math.cos(base1[2]-base0[2]))
            if math.hypot(dx,dy)>1e-6:
                lateral=-dx*math.sin(base0[2])+dy*math.cos(base0[2])
                if abs(delta_theta)>1e-6 or abs(lateral)>.002:
                    raise ValueError('DD motion rejected: rotate in place first, then drive forward or backward along the AMR heading.')
            # Validate at a resolution fine enough that a long tool component
            # cannot visibly tunnel through a door between rendered frames.
            joint_steps=max((abs(b-a) for a,b in zip(q0,q1)),default=0.)/.01
            base_steps=math.hypot(base1[0]-base0[0],base1[1]-base0[1])/.005
            turn_steps=abs(math.atan2(math.sin(base1[2]-base0[2]),math.cos(base1[2]-base0[2])))/math.radians(1.)
            # Door angles are stored in degrees throughout the planner.
            # Convert each 2-degree sample directly; treating degrees as
            # radians inflated every opening segment by about 57x.
            angle_steps=max(abs(angles1[name]-angles0[name]) for name in angles0)/.5
            count=max(2,math.ceil(max(joint_steps,base_steps,turn_steps,angle_steps)))
            distance=math.hypot(base1[0]-base0[0],base1[1]-base0[1])
            turn=abs(math.atan2(math.sin(base1[2]-base0[2]),math.cos(base1[2]-base0[2])))
            joint_delta=max((abs(b-a) for a,b in zip(q0,q1)),default=0.)
            duration=max(.22,distance/.20,turn/.55,joint_delta/.8,angle_steps*.22)
            motion_segments.append((list(q0),tuple(base0),dict(angles0),list(q1),tuple(base1),dict(angles1),active,duration))
            for i in range(1,count+1):
                t=i/count;q=[a+(b-a)*t for a,b in zip(q0,q1)]
                delta_theta=math.atan2(math.sin(base1[2]-base0[2]),math.cos(base1[2]-base0[2]))
                base=tuple(a+(b-a)*t for a,b in zip(base0,base1));base=(base[0],base[1],base0[2]+delta_theta*t)
                angles={name:angles0[name]+(angles1[name]-angles0[name])*t for name in angles0}
                arm_samples.append((q,base,angles,active))
        start_base=(start_x,start_y,theta);aligned_base=(start_x,start_y,0.);initial_angles={left['name']:0.,right['name']:0.}
        append_sweep(list(simulator.q),start_base,initial_angles,list(simulator.q),aligned_base,initial_angles,None)
        append_sweep(list(simulator.q),aligned_base,initial_angles,list(simulator.q),left_base,initial_angles,None)
        approach=collision_free_approach(list(simulator.q),left_paths[0][0],left_base,left['name'])
        for q0,q1 in zip(approach,approach[1:]):
            append_sweep(q0,left_base,initial_angles,q1,left_base,initial_angles,left['name'])
        previous_q=left_paths[0][0];previous_base=left_base;previous_angle=0.
        for target_q,degrees,base,active in left_paths[1:]:
            append_sweep(previous_q,previous_base,{left['name']:previous_angle,right['name']:0.},target_q,base,{left['name']:float(degrees),right['name']:0.},active)
            previous_q,previous_base,previous_angle=target_q,base,float(degrees)
        # Back the DD base away from the open leaf before folding the arm. This
        # prevents the elbow/tool sweep from crossing the 120-degree door arc.
        stow_base=(previous_base[0]-.20,previous_base[1],0.)
        append_sweep(previous_q,previous_base,{left['name']:120.,right['name']:0.},previous_q,stow_base,{left['name']:120.,right['name']:0.},left['name'])
        fold_route=collision_free_approach(previous_q,stow_q,stow_base,None,fixed_angles={left['name']:120.,right['name']:0.})
        for q0,q1 in zip(fold_route,fold_route[1:]):
            append_sweep(q0,stow_base,{left['name']:120.,right['name']:0.},q1,stow_base,{left['name']:120.,right['name']:0.},None)
        previous_base=stow_base
        # Keep the chassis on the LM6 centerline. After folding the arm, drive
        # forward along +X directly to the right-door station; no side-step.
        transfer=[(stow_base[0],stow_base[1],0.),right_base]
        for base in transfer:
            append_sweep(stow_q,previous_base,{left['name']:120.,right['name']:0.},stow_q,base,{left['name']:120.,right['name']:0.},None)
            previous_base=base
        right_base=right_paths[0][2]
        right_approach=collision_free_approach(stow_q,right_paths[0][0],right_base,right['name'],avoid_angle=120.,contact_local=right_contact,contact_tool_point=roller_tool_point)
        for q0,q1 in zip(right_approach,right_approach[1:]):
            append_sweep(q0,right_base,{left['name']:120.,right['name']:0.},q1,right_base,{left['name']:120.,right['name']:0.},right['name'])
        previous_q=right_paths[0][0];previous_angle=0.;previous_base=right_base
        for target_q,degrees,base,active in right_paths[1:]:
            append_sweep(previous_q,previous_base,{left['name']:120.,right['name']:previous_angle},target_q,base,{left['name']:120.,right['name']:float(degrees)},active)
            previous_q,previous_angle,previous_base=target_q,float(degrees),base
        self._spatial_validate_door_demo(arm_samples,left,right,cad_world,contacts={right['name']:right_contact_spec})
        view.cad_doors_auto.set(False);view.cad_door_hold_angle=None
        view.cad_door_hold_angles={left['name']:0.,right['name']:0.}
        view._hook_demo=dict(segments=motion_segments,index=0,progress=0.,last_tick=time.monotonic(),duration=motion_segments[0][7],left_name=left['name'],right_name=right['name'],chain_panel=True)
        self.log('SIM',f'Start coordinates confirmed; LM7-to-LM6 route history is not required. DD-safe sequence: left door 120 degrees, then right door 120 degrees using right-door {right_strategy}.')
        self._spatial_hook_demo_tick()

    def _spatial_run_panel_demo(self):
        """SIM-only vacuum pickup from the default SIMPLE_WHOLE shelf position."""
        view=self.world3d;simulator=getattr(self,'arm_dev_sim',None)
        if self.real:raise ValueError('This panel motion is a SIM preview only; it never sends commands to hardware.')
        if simulator is None or view.cad_asset is None:raise ValueError('Load SIMPLE_WHOLE and the SIM arm before running panel extraction.')
        if view.tool_asset is None or not getattr(view.tool_asset,'name','').startswith('EndEffector V3_EndOnly_Change1-org'):
            raise ValueError('Load the EndEffector V3 GLB before running the vacuum pickup.')
        if simulator.state in ('RUNNING','PAUSED') or (getattr(self,'studio_runner',None) and self.studio_runner.active):raise ValueError('Stop the current arm or mission task first.')
        if getattr(self.sim,'route',[]):raise ValueError('Stop AMR navigation before extracting the panel.')
        if view._hook_demo:raise ValueError('Another spatial SIM motion is already running.')
        doors=list(getattr(view.cad_asset,'door_parts',()))
        left=next((d for d in doors if d['name'].endswith(':1')),None);right=next((d for d in doors if d['name'].endswith(':2')),None)
        if left is None or right is None:raise ValueError('Could not find both SIMPLE_WHOLE front doors.')
        door_mesh_trees={d['name']:_build_triangle_bvh(list(d['asset'].faces)+list(getattr(d['asset'],'transparent_faces',()))) for d in (left,right)}
        tool_names=list(getattr(view.tool_asset,'component_faces',{}));tool_components=[]
        for index,(faces,local,_color) in enumerate(getattr(view.tool_asset,'links',{}).get('base_link',())):
            if not faces:continue
            pts=[p for face in faces for p in face];lo=[min(p[i] for p in pts) for i in range(3)];hi=[max(p[i] for p in pts) for i in range(3)]
            tool_components.append((tool_names[index] if index<len(tool_names) else f'tool component {index+1}',local,
                                    tuple((a+b)/2 for a,b in zip(lo,hi)),tuple(max(.002,b-a) for a,b in zip(lo,hi))))
        if any(float(view.cad_door_hold_angles.get(d['name'],0.))<math.radians(119.) for d in (left,right)):
            raise ValueError('Open both front doors to 120 degrees first.')
        physics=simulator.physics
        if physics.held:raise ValueError('Release the currently held SIM object before starting the panel pickup.')
        # This default phone-sized workpiece is centered over the broad internal
        # shelf plane measured at CAD z=0.05 m. It is only a SIM proxy.
        cad_world=transform(view.cad_origin[:3],view.cad_origin[3:]);cad_scale=float(view.scales.get('cad',1.))
        panel_size=tuple(v*cad_scale for v in SIM_PANEL_SIZE);panel_local=tuple(v*cad_scale for v in getattr(view,'phone_panel_center',SIM_PANEL_CENTER))
        panel_world=multiply(cad_world,transform(panel_local,(0,0,getattr(view,'phone_panel_yaw',math.pi/2))));panel_name='SIM phone panel'
        state=self.current_state();base=(float(state.get('x',0.)),float(state.get('y',0.)),float(state.get('theta',0.)))
        mount_offset=point(transform((0.,0.,0.),(0.,0.,base[2])),view.mount[:3])
        pick_base=(panel_world[0][3]-mount_offset[0],base[1],base[2])
        arm_world=multiply(transform((pick_base[0],pick_base[1],0.),(0.,0.,pick_base[2])),transform(view.mount[:3],view.mount[3:]))
        from .arm_physics import inverse,obb,separation
        panel_arm=multiply(inverse(arm_world),panel_world)
        def euler_from_matrix(matrix):
            pitch=math.asin(max(-1.,min(1.,-matrix[2][0])))
            if abs(math.cos(pitch))>1e-7:roll=math.atan2(matrix[2][1],matrix[2][2]);yaw=math.atan2(matrix[1][0],matrix[0][0])
            else:roll=0.;yaw=math.atan2(-matrix[0][1],matrix[1][1])
            return roll,pitch,yaw
        panel_pose=[panel_arm[i][3] for i in range(3)]+list(euler_from_matrix(panel_arm))
        physics.configure(dict(kind='vacuum',mount_offset_m=.17,cup_diameter_m=.04,stop_on_collision=True))
        physics.objects=[o for o in physics.objects if o.get('name')!=panel_name]
        panel_obj=physics.add_object(panel_name,'panel',panel_size,.16,panel_pose,static=True,friction=.5,sealable=True)
        panel_obj['vacuum_pickable']=True
        view.panel_pick=dict(name='phone panel on shelf · SIM',object_name=panel_name,matrix=panel_world,size=panel_size)
        # The loaded assembly contains a four-pad vacuum head; the physics model
        # uses one equivalent 40 mm seal centered on that measured pad array.
        desired=transform((0.,0.,0.),(math.pi,0.,0.))
        mount_rotation=multiply(transform((0.,0.,0.),(0.,0.,pick_base[2])),transform((0.,0.,0.),view.mount[3:]))
        relative_rotation=multiply(inverse(mount_rotation),desired)
        relative_euler=euler_from_matrix(relative_rotation)
        flange_offset=physics.config['mount_offset_m']+.115
        arm_inverse=inverse(arm_world)
        def solve_cup(world_point,seed):
            local=point(arm_inverse,world_point);offset=point(relative_rotation,(0.,0.,flange_offset))
            pose=[(local[i]-offset[i])*1000. for i in range(3)]+[math.degrees(v) for v in relative_euler]
            seeds=[list(seed),[0.,-math.pi/2,math.pi/2,-math.pi/2,-math.pi/2,0.],[0.,-1.,1.,-1.,-1.5,0.],[0.,-2.,1.2,-.8,-1.5,0.]]
            failures=[]
            for candidate in seeds:
                try:
                    q=simulator.kin.ik(pose,candidate,iterations=260,position_tolerance=.0015,rotation_tolerance=.012)
                    bounded=simulator.kin.clamp(q)
                    if any(abs(a-b)>1e-7 for a,b in zip(q,bounded)):continue
                    return q
                except ValueError as error:failures.append(error)
            raise ValueError('Vacuum target is outside the loaded FR5 model range: '+(str(failures[0]) if failures else 'joint-limit check failed'))
        panel_top=point(panel_world,(0.,0.,panel_size[2]/2))
        approach=tuple(panel_top[i]+(0.,0.,.08)[i] for i in range(3))
        # The tray rim is 11 mm above its floor. Lift 50 mm to clear it while
        # keeping the panel inside the FR5 working envelope at the LM6 station.
        lift=tuple(panel_top[i]+(0.,0.,.05)[i] for i in range(3))
        outside=point(cad_world,tuple(panel_local[i]+(0.,-.30,.056)[i]*cad_scale for i in range(3)))
        q=[0.,-math.pi/2,math.pi/2,-math.pi/2,-math.pi/2,0.];waypoints=[]
        for label,target in (('approach',approach),('panel-contact',panel_top),('vacuum-hold',panel_top),('lift',lift)):
            q=solve_cup(target,q);waypoints.append((label,list(q)))
        # Pull the secured panel straight out through the open front. The AMR
        # first backs along the LM6 lane to center the arm; no lateral base move is used.
        for step in range(1,11):
            t=step/10.;target=tuple(lift[i]+(outside[i]-lift[i])*t for i in range(3))
            q=solve_cup(target,q);waypoints.append(('extract',list(q)))
        base_angles={left['name']:120.,right['name']:120.};segments=[];samples=[];q0=list(simulator.q);current_base=base
        def add_segment(target_q,active,duration=None,target_base=None):
            nonlocal q0,current_base
            if any(abs(a-b)>1e-7 for a,b in zip(target_q,simulator.kin.clamp(target_q))):raise ValueError('Panel path exceeds the loaded FR5 URDF joint limits.')
            end_base=current_base if target_base is None else tuple(target_base)
            dx,dy=end_base[0]-current_base[0],end_base[1]-current_base[1]
            turn=math.atan2(math.sin(end_base[2]-current_base[2]),math.cos(end_base[2]-current_base[2]))
            lateral=-dx*math.sin(current_base[2])+dy*math.cos(current_base[2])
            if math.hypot(dx,dy)>1e-7 and (abs(turn)>1e-7 or abs(lateral)>.002):raise ValueError('DD panel approach permits straight forward/backward travel only; no diagonal base motion.')
            delta=max((abs(a-b) for a,b in zip(q0,target_q)),default=0.)
            distance=math.hypot(dx,dy);steps=max(1,math.ceil(max(delta/.045,distance/.015,abs(turn)/math.radians(3.))))
            span=max(.22,duration or delta/.8,distance/.45,abs(turn)/.55)
            segments.append((list(q0),current_base,dict(base_angles),list(target_q),end_base,dict(base_angles),active,span))
            for index in range(1,steps+1):
                ratio=index/steps;sweep=[a+(b-a)*ratio for a,b in zip(q0,target_q)];sample_base=tuple(a+(b-a)*ratio for a,b in zip(current_base,end_base))
                samples.append((sweep,sample_base,dict(base_angles),active))
            q0=list(target_q);current_base=end_base
        # Plan the first transfer around both open leaves. The door-opening
        # sequence ends with the roller at the right frame, so a straight joint
        # interpolation can sweep the vacuum body through that leaf.
        original_objects=physics.objects
        def obstacles_at(path_base,include_panel):
            path_arm=multiply(transform((path_base[0],path_base[1],0.),(0.,0.,path_base[2])),transform(view.mount[:3],view.mount[3:]));path_inverse=inverse(path_arm);obstacles=[]
            for door in (left,right):
                faces=list(door['asset'].faces)+list(getattr(door['asset'],'transparent_faces',()))
                vertices=[p for face in faces for p in face]
                lo=[min(p[i] for p in vertices) for i in range(3)];hi=[max(p[i] for p in vertices) for i in range(3)]
                center=tuple((a+b)/2 for a,b in zip(lo,hi));size=tuple(max(.002,b-a) for a,b in zip(lo,hi))
                angle=math.radians(120.)*door['sign'];pivot=door['pivot']
                swing=multiply(multiply(transform(pivot),axis_rotation((0.,0.,1.),angle)),transform(tuple(-v for v in pivot)))
                frame_world=multiply(cad_world,swing);world=multiply(frame_world,transform(tuple(v*cad_scale for v in center)))
                mesh_inverse=inverse(multiply(path_inverse,frame_world))
                obstacles.append(dict(name='CAD door '+door['name'],matrix=multiply(path_inverse,world),size=tuple(v*cad_scale for v in size),static=True,mesh_inverse=mesh_inverse,mesh_tree=door_mesh_trees[door['name']]))
            objects=[obj for obj in original_objects if obj.get('name')!=panel_name]
            if include_panel:
                obj=dict(panel_obj);obj['matrix']=multiply(path_inverse,panel_world);objects.append(obj)
            return path_arm,obstacles,objects
        def clear_arm(q,path_base,include_panel):
            path_arm,door_obstacles,objects=obstacles_at(path_base,include_panel)
            physics.objects=objects+door_obstacles
            risks,_near=physics.contacts(q)
            for link,name in risks:
                if link=='tool' and (name.startswith('CAD ') or name==panel_name):continue
                return False
            poses=simulator.kin.asset.link_transforms(simulator.kin.positions(q))
            tool_link=simulator.kin.tip
            if tool_link not in poses:return False
            tool_root=multiply(poses[tool_link],transform((0.,0.,.1)))
            for _name,local,center,size in tool_components:
                component=obb(multiply(multiply(tool_root,local),transform(center)),size)
                for ob in door_obstacles:
                    if _obb_intersects_mesh(component,ob['mesh_inverse'],ob['mesh_tree']):return False
            return True
        def edge_clear(a,b,path_base,include_panel):
            count=max(1,math.ceil(max(abs(x-y) for x,y in zip(a,b))/.02))
            return all(clear_arm([x+(y-x)*i/count for x,y in zip(a,b)],path_base,include_panel) for i in range(1,count+1))
        def route_to_approach(start,goal,path_base,include_panel):
            if clear_arm(goal,path_base,include_panel) and edge_clear(start,goal,path_base,include_panel):return [list(start),list(goal)]
            if not clear_arm(goal,path_base,include_panel):raise ValueError('The FR5 target pose intersects the open-door geometry or SIM panel.')
            import random
            rng=random.Random(29)
            limits=[(j['lower'],j['upper']) if j['type']!='continuous' else (-math.pi,math.pi) for j in simulator.kin.joints]
            def distance(a,b):return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
            def extend(tree,target):
                near=min(range(len(tree[0])),key=lambda i:distance(tree[0][i],target));source=tree[0][near];gap=distance(source,target)
                candidate=list(target) if gap<=.32 else [x+(y-x)*.32/gap for x,y in zip(source,target)]
                if not edge_clear(source,candidate,path_base,include_panel):return None,gap<=.32
                tree[0].append(candidate);tree[1].append(near)
                return len(tree[0])-1,gap<=.32
            def path(tree,index):
                result=[]
                while index is not None:result.append(tree[0][index]);index=tree[1][index]
                return result[::-1]
            starts=([[list(start)],[None]]);goals=([[list(goal)],[None]])
            for _ in range(1200):
                if _%4:
                    fraction=rng.random();sample=[max(lo,min(hi,x+(y-x)*fraction+rng.uniform(-.55,.55))) for x,y,(lo,hi) in zip(start,goal,limits)]
                else:sample=[rng.uniform(*lim) for lim in limits]
                si,_=extend(starts,sample)
                if si is None:continue
                while True:
                    gi,connected=extend(goals,starts[0][si])
                    if gi is None:break
                    if connected:
                        raw=path(starts,si)+path(goals,gi)[-2::-1]
                        smooth=[raw[0]];i=0
                        while i<len(raw)-1:
                            j=len(raw)-1
                            while j>i+1 and not edge_clear(raw[i],raw[j],path_base,include_panel):j-=1
                            smooth.append(raw[j]);i=j
                        return smooth
            raise ValueError('No collision-free FR5 path to the internal panel was found with both doors open.')
        stow_q=[0.,-math.pi/2,math.pi/2,-math.pi/2,-math.pi/2,0.]
        try:retract_route=route_to_approach(list(simulator.q),stow_q,base,False)
        finally:physics.objects=original_objects
        for clear_q in retract_route[1:]:add_segment(clear_q,None)
        add_segment(stow_q,None,target_base=pick_base)
        try:approach_route=route_to_approach(stow_q,waypoints[0][1],pick_base,True)
        finally:physics.objects=original_objects
        for clear_q in approach_route[1:]:add_segment(clear_q,None)
        vacuum_on_from=None
        for label,target_q in waypoints[1:]:
            active='panel-contact' if label in ('panel-contact','vacuum-hold') else 'panel-attached' if label in ('lift','extract') else None
            if label=='vacuum-hold':vacuum_on_from=len(segments)
            add_segment(target_q,active,.45*self._spatial_motion_speed() if label=='vacuum-hold' else None)
        # The output turns on at the 450 ms seal dwell and stays on while the
        # SIM panel is carried outside. The simulator checks lift/shear capacity.
        contact_q=next(q for label,q in waypoints if label=='panel-contact')
        view.panel_pick['relative']=multiply(inverse(physics.tcp(contact_q)),panel_obj['matrix'])
        self._spatial_validate_door_demo(samples,left,right,cad_world)
        io=physics.config
        simulator.io[io['do_bank']][io['do_channel']]=0
        view.cad_doors_auto.set(False);view.cad_door_hold_angles={left['name']:math.radians(120.),right['name']:math.radians(120.)}
        view._hook_demo=dict(segments=segments,index=0,progress=0.,last_tick=time.monotonic(),duration=segments[0][7],left_name=left['name'],right_name=right['name'],vacuum_on_from=vacuum_on_from,panel_name=panel_name)
        self.log('SIM','Both doors are open. DD backs along the LM6 centerline to align with the shelf; the vacuum head seals, lifts, and extracts the panel straight through the front.')
        self._spatial_hook_demo_tick()

    def _spatial_start_panel_pick(self):
        """Open both doors first when needed, then run the SIM vacuum extraction."""
        view=self.world3d
        doors=list(getattr(getattr(view,'cad_asset',None),'door_parts',()))
        left=next((d for d in doors if d.get('name','').endswith(':1')),None)
        right=next((d for d in doors if d.get('name','').endswith(':2')),None)
        if left is None or right is None:raise ValueError('SIMPLE_WHOLE 전면 좌우 문 모델을 불러오세요.')
        angles=getattr(view,'cad_door_hold_angles',{})
        if any(float(angles.get(d['name'],0.))<math.radians(119.) for d in (left,right)):
            self._spatial_run_hook_demo()
            return
        self._spatial_run_panel_demo()

    def _spatial_validate_door_demo(self,samples,left,right,cad_world,contacts=None):
        """Reject any planned AMR/body or arm-link sweep that overlaps the CAD enclosure."""
        view=self.world3d;simulator=self.arm_dev_sim;contacts=contacts or {}
        self._spatial_sync_cad_lidar()
        walls=list(getattr(self.map,'cad_walls',[]))
        body=view.body_asset;vertices=[]
        if body is not None and hasattr(body,'links'):
            vertices=[point(local,p) for visuals in body.links.values() for faces,local,_color in visuals for face in faces for p in face]
        if vertices:
            minx,maxx=min(p[0] for p in vertices),max(p[0] for p in vertices)
            miny,maxy=min(p[1] for p in vertices),max(p[1] for p in vertices)
            minz,maxz=min(p[2] for p in vertices),max(p[2] for p in vertices)
        else:
            model=getattr(self.map,'robot_model',{});minx,maxx=-float(model.get('length',.95))/2,float(model.get('length',.95))/2
            miny,maxy=-float(model.get('width',.63))/2,float(model.get('width',.63))/2
            minz,maxz=0.,.75
        body_center=tuple((a+b)/2 for a,b in ((minx,maxx),(miny,maxy),(minz,maxz)))
        body_size=tuple(max(.002,b-a) for a,b in ((minx,maxx),(miny,maxy),(minz,maxz)))
        def orient(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
        def intersects(a,b,c,d):
            o1,o2,o3,o4=orient(a,b,c),orient(a,b,d),orient(c,d,a),orient(c,d,b)
            if o1*o2<0. and o3*o4<0.:return True
            def on_segment(p,x,y):
                return min(x[0],y[0])-1e-9<=p[0]<=max(x[0],y[0])+1e-9 and min(x[1],y[1])-1e-9<=p[1]<=max(x[1],y[1])+1e-9
            return (abs(o1)<1e-9 and on_segment(c,a,b)) or (abs(o2)<1e-9 and on_segment(d,a,b)) or (abs(o3)<1e-9 and on_segment(a,c,d)) or (abs(o4)<1e-9 and on_segment(b,c,d))
        def point_segment(p,a,b):
            dx=b[0]-a[0];dy=b[1]-a[1];den=dx*dx+dy*dy
            t=0. if den<=1e-12 else max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den))
            return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
        polygon=[(wall[0],wall[1]) for wall in walls]
        if walls:polygon.append((walls[-1][2],walls[-1][3]))
        def inside(p):
            result=False
            for a,b in zip(polygon,polygon[1:]+polygon[:1]):
                if (a[1]>p[1])!=(b[1]>p[1]) and p[0]<(b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0]:result=not result
            return result
        def check_amr(base):
            x,y,theta=base;c=math.cos(theta);s=math.sin(theta)
            corners=[]
            for lx,ly in ((minx,miny),(maxx,miny),(maxx,maxy),(minx,maxy)):
                corners.append((x+c*lx-s*ly,y+s*lx+c*ly))
            if walls and inside((x,y)):raise ValueError(f'Unsafe AMR path: center ({x:.3f}, {y:.3f}) enters the CAD enclosure.')
            edges=list(zip(corners,corners[1:]+corners[:1]))
            for wall in walls:
                a,b=(wall[0],wall[1]),(wall[2],wall[3])
                if any(intersects(c,d,a,b) or min(point_segment(a,c,d),point_segment(b,c,d),point_segment(c,a,b),point_segment(d,a,b))<.010 for c,d in edges):
                    raise ValueError(f'Unsafe AMR path: footprint at ({x:.3f}, {y:.3f}) intersects the CAD enclosure.')
        # Check the complete base sweep at 15 mm spacing, including transfers between doors.
        previous=None
        for _q,base,_angles,_active in samples:
            if previous is not None:
                count=max(1,math.ceil(math.hypot(base[0]-previous[0],base[1]-previous[1])/.015))
                for i in range(1,count):
                    t=i/count;check_amr(tuple(a+(b-a)*t for a,b in zip(previous,base)))
            check_amr(base);previous=base
        panels=[]
        for door in (left,right):
            faces=list(door['asset'].faces)+list(getattr(door['asset'],'transparent_faces',()))
            points=[p for face in faces for p in face]
            lo=[min(p[i] for p in points) for i in range(3)];hi=[max(p[i] for p in points) for i in range(3)]
            center=tuple((a+b)/2 for a,b in zip(lo,hi));size=tuple(max(.002,b-a) for a,b in zip(lo,hi))
            panels.append((door,center,size))
        door_mesh_trees={door['name']:_build_triangle_bvh(list(door['asset'].faces)+list(getattr(door['asset'],'transparent_faces',()))) for door in (left,right)}
        def contact_local(door):
            selected=contacts.get(door['name'])
            if isinstance(selected,dict):return selected.get('local')
            if selected is not None:return selected
            handle_faces=[p for face,material in zip(door['asset'].faces,door['asset'].face_materials) if material==36 for p in face]
            return tuple((min(p[i] for p in handle_faces)+max(p[i] for p in handle_faces))/2 for i in range(3)) if handle_faces else None
        def contact_tool_point(door):
            selected=contacts.get(door['name'])
            return selected.get('tool_point',(0.,-.14714,.148)) if isinstance(selected,dict) else (0.,-.14714,.148)
        def contact_tool_kind(door):
            selected=contacts.get(door['name'])
            return selected.get('tool_kind','hook') if isinstance(selected,dict) else 'hook'
        def contact_world(door,degrees):
            local=contact_local(door)
            if local is None:return None
            angle=math.radians(degrees)*door['sign'];pivot=door['pivot']
            swing=multiply(multiply(transform(pivot),axis_rotation((0.,0.,1.),angle)),transform(tuple(-v for v in pivot)))
            return point(cad_world,point(swing,local))
        from .arm_physics import inverse,obb,separation
        # The URDF's terminal link is only a TCP proxy. Build collision boxes
        # from every named mesh group in the loaded end-effector assembly so
        # its camera, vacuum head, hook, roller, and housings are all checked.
        tool_components=[]
        tool_asset=getattr(view,'tool_asset',None)
        visuals=getattr(tool_asset,'links',{}).get('base_link',()) if tool_asset is not None else ()
        component_names=list(getattr(tool_asset,'component_faces',{})) if tool_asset is not None else []
        for index,(faces,local,_color) in enumerate(visuals):
            if not faces:continue
            points=[p for face in faces for p in face]
            lo=[min(p[i] for p in points) for i in range(3)];hi=[max(p[i] for p in points) for i in range(3)]
            center=tuple((a+b)/2 for a,b in zip(lo,hi));size=tuple(max(.002,b-a) for a,b in zip(lo,hi))
            name=component_names[index] if index<len(component_names) else f'end-effector component {index+1}'
            tool_components.append((name,local,center,size))
        if not tool_components:raise ValueError('The loaded end-effector has no component geometry for collision checking.')
        amr_scale=float(view.scales.get('amr',1.));cad_scale=float(view.scales.get('cad',1.))
        scaled_body_center=tuple(v*amr_scale for v in body_center);scaled_body_size=tuple(v*amr_scale for v in body_size)
        for _q,base,angles,_active in samples:
            base_world=transform((base[0],base[1],0.),(0.,0.,base[2]))
            body_box=obb(multiply(base_world,transform(scaled_body_center)),scaled_body_size)
            for door,center,size in panels:
                angle=math.radians(angles.get(door['name'],0.))*door['sign'];pivot=door['pivot']
                swing=multiply(multiply(transform(pivot),axis_rotation((0.,0.,1.),angle)),transform(tuple(-v for v in pivot)))
                scaled_center=tuple(v*cad_scale for v in center);scaled_size=tuple(v*cad_scale for v in size)
                panel_world=multiply(multiply(cad_world,swing),transform(scaled_center))
                if separation(body_box,obb(panel_world,scaled_size))<=.015:
                    raise ValueError(f"Unsafe AMR path: footprint at ({base[0]:.3f}, {base[1]:.3f}, {math.degrees(base[2]):.1f} degrees) intersects moving door {door['name']} at {math.degrees(angle):.1f} degrees.")
        cad_faces=list(view.cad_asset.faces)+list(getattr(view.cad_asset,'transparent_faces',()))
        cad_points=[point(cad_world,tuple(p[i]*view.scales.get('cad',1.) for i in range(3))) for face in cad_faces for p in face]
        floor_z=min(p[2] for p in cad_points);ceiling_z=max(p[2] for p in cad_points)
        door_ranges=[];cad_scale=view.scales.get('cad',1.)
        for door,center,size in panels:
            scaled_center=tuple(v*cad_scale for v in center);scaled_size=tuple(v*cad_scale for v in size)
            corners=[point(cad_world,tuple(scaled_center[i]+sign*scaled_size[i]/2 for i,sign in enumerate((dx,dy,dz)))) for dx in (-1,1) for dy in (-1,1) for dz in (-1,1)]
            door_ranges.append([min(p[0] for p in corners),max(p[0] for p in corners),min(p[1] for p in corners),max(p[1] for p in corners)])
        static_walls=[]
        for index,wall in enumerate(walls):
            x1,y1,x2,y2=wall;length=math.hypot(x2-x1,y2-y1)
            if length<.02:continue
            mx,my=(x1+x2)/2,(y1+y2)/2
            if any(r[0]-.06<=mx<=r[1]+.06 and r[2]-.06<=my<=r[3]+.06 for r in door_ranges):continue
            matrix=transform((mx,my,(floor_z+ceiling_z)/2),(0.,0.,math.atan2(y2-y1,x2-x1)))
            static_walls.append((f'CAD enclosure wall {index+1}',matrix,(length,.025,max(.05,ceiling_z-floor_z))))
        physics=simulator.physics;original=physics.objects
        try:
            for sample_index,(q,base,angles,active) in enumerate(samples):
                arm_world=multiply(transform((base[0],base[1],0.),(0.,0.,base[2])),transform(view.mount[:3],view.mount[3:]))
                arm_inverse=inverse(arm_world);obstacles=[]
                for name,matrix,size in static_walls:
                    obstacles.append(dict(name=name,matrix=multiply(arm_inverse,matrix),size=size,static=True))
                for door,center,size in panels:
                    angle=math.radians(angles.get(door['name'],0.))*door['sign'];pivot=door['pivot']
                    swing=multiply(multiply(transform(pivot),axis_rotation((0.,0.,1.),angle)),transform(tuple(-v for v in pivot)))
                    frame_world=multiply(cad_world,swing);panel_world=multiply(frame_world,transform(center));mesh_inverse=inverse(multiply(arm_inverse,frame_world))
                    obstacles.append(dict(name='CAD door '+door['name'],matrix=multiply(arm_inverse,panel_world),size=size,static=True,mesh_inverse=mesh_inverse,mesh_tree=door_mesh_trees[door['name']]))
                carry_panel=active=='panel-attached'
                panel_obj=next((obj for obj in original if obj.get('name')=='SIM phone panel'),None)
                panel_sample=None
                if panel_obj is not None and not carry_panel:
                    panel_sample=dict(panel_obj);panel_sample['matrix']=multiply(arm_inverse,view.panel_pick['matrix'])
                scene_objects=[obj for obj in original if obj.get('name')!='SIM phone panel']
                if panel_sample is not None:scene_objects.append(panel_sample)
                carried_matrix=None
                if carry_panel and panel_obj is not None:
                    carried_matrix=multiply(physics.tcp(q),view.panel_pick['relative'])
                physical_obstacles=list(obstacles)
                if panel_sample is not None:physical_obstacles.append(panel_sample)
                physical_obstacle_boxes=[]
                for obstacle in physical_obstacles:
                    bounds=obb(obstacle['matrix'],obstacle['size'])
                    radius=math.sqrt(sum(v*v for v in bounds[2]))
                    physical_obstacle_boxes.append((obstacle,bounds,radius))
                physics.objects=scene_objects+obstacles
                ground_check=physics.config.get('ground_collision',True)
                physics.config['ground_collision']=False
                try:risks,_near=physics.contacts(q)
                finally:physics.config['ground_collision']=ground_check
                # ArmPhysics uses generic terminal-link boxes. Ignore a box
                # at the intended contact only when the selected physical
                # hook or roller center is at the modeled contact point.
                unsafe=[]
                for link,obstacle_name in risks:
                    # The URDF's terminal "tool" collision is a generic
                    # placeholder; the loaded hook OBB below is the physical
                    # end-effector envelope used for every CAD obstacle.
                    proxy_contact=link=='tool' and (obstacle_name.startswith('CAD ') or obstacle_name=='SIM phone panel')
                    if active and obstacle_name=='CAD door '+active and link in ('tool','vacuum_body','cup','gripper_body','finger'):
                        door=next((d for d in (left,right) if d['name']==active),None)
                        # The URDF TCP link is only a frame. Its broad box is
                        # superseded by the actual hook OBB checked below.
                        if link=='tool':proxy_contact=True
                        if door is not None and link!='tool':
                            poses=simulator.kin.asset.link_transforms(simulator.kin.positions(q),arm_world)
                            tool_link=simulator.kin.tip
                            if tool_link in poses:
                                contact_center=point(poses[tool_link],contact_tool_point(door))
                                intended=contact_world(door,angles.get(active,0.))
                                if intended is not None:proxy_contact=math.dist(contact_center,intended)<=.03
                    if active=='panel-contact' and obstacle_name=='SIM phone panel' and link=='cup':
                        panel=next((item for item in physics.objects if item.get('name')==obstacle_name),None)
                        tool_link=simulator.kin.tip
                        if panel is not None and tool_link in simulator.kin.asset.links:
                            poses=simulator.kin.asset.link_transforms(simulator.kin.positions(q),arm_world)
                            cup_center=point(poses[tool_link],(0.,0.,physics.config['mount_offset_m']+.115))
                            panel_world=multiply(arm_world,panel['matrix'])
                            seal_surface=point(panel_world,(0.,0.,panel['size'][2]/2))
                            proxy_contact=math.dist(cup_center,seal_surface)<=.025
                    if not proxy_contact:unsafe.append((link,obstacle_name))
                if unsafe:
                    link,obstacle=unsafe[0]
                    raise ValueError(f"Unsafe arm path sample {sample_index}: {link} intersects {obstacle} at AMR ({base[0]:.3f}, {base[1]:.3f}, {math.degrees(base[2]):.1f} degrees), active contact {active}, door angles {angles}, joints {[round(v, 3) for v in q]}. Move the AMR/arm or adjust the door geometry before running.")
                if carried_matrix is not None:
                    carried_box=obb(carried_matrix,panel_obj['size'])
                    for obstacle in obstacles:
                        if separation(carried_box,obb(obstacle['matrix'],obstacle['size']))<=0.:
                            raise ValueError(f"Unsafe extraction path sample {sample_index}: carried phone panel intersects {obstacle['name']}.")
                    low=carried_box[0][2]-sum(abs(axis[2])*half for axis,half in zip(carried_box[1],carried_box[2]))
                    if low<-.005:raise ValueError(f'Unsafe extraction path sample {sample_index}: carried phone panel intersects the floor.')
                # Check the actual hook envelope for hook motions. A roller
                # push checks both the hook and the measured polymer roller.
                poses=simulator.kin.asset.link_transforms(simulator.kin.positions(q),arm_world)
                tool_link=simulator.kin.tip
                if tool_link not in poses:raise ValueError('The loaded arm has no TCP frame for the hook collision model.')
                if ground_check:
                    for link,center,size in physics.bounds:
                        if link in getattr(simulator.kin.asset,'roots',[]):continue
                        bounds=obb(multiply(poses[link],transform(center)),size)
                        low=bounds[0][2]-sum(abs(axis[2])*half for axis,half in zip(bounds[1],bounds[2]))
                        if low<-.005:raise ValueError(f'Unsafe arm path: {link} intersects the map floor.')
                local_poses=simulator.kin.asset.link_transforms(simulator.kin.positions(q))
                hook_frame=multiply(multiply(local_poses[tool_link],transform((0.,0.,.1))),transform((0.,-.1243,.0485)))
                hook_box=obb(hook_frame,(.033,.046,.026));hook_center=point(hook_frame,(0.,0.,0.))
                tool_checks=[('hook',hook_box,hook_center)]
                active_door=next((door for door in (left,right) if door['name']==active),None)
                active_kind=contact_tool_kind(active_door) if active_door is not None else None
                if active_door is not None and contact_tool_kind(active_door)=='roller':
                    spec=contacts[active_door['name']];roller_frame=multiply(local_poses[tool_link],transform((0.,0.,.1)))
                    roller_pose=multiply(roller_frame,transform(spec['roller_center']))
                    roller_box=obb(roller_pose,spec['roller_size']);roller_world=point(roller_pose,(0.,0.,0.))
                    tool_checks.append(('roller',roller_box,roller_world))
                for obstacle,obstacle_box,obstacle_radius in physical_obstacle_boxes:
                    for tool_name,tool_box,tool_center in tool_checks:
                        tool_radius=math.sqrt(sum(v*v for v in tool_box[2]))
                        delta=[tool_center[i]-obstacle_box[0][i] for i in range(3)]
                        if sum(v*v for v in delta)>(tool_radius+obstacle_radius)**2:continue
                        if obstacle.get('mesh_tree') is not None:
                            if not _obb_intersects_mesh(tool_box,obstacle['mesh_inverse'],obstacle['mesh_tree']):continue
                        elif separation(tool_box,obstacle_box)>0.:continue
                        allowed=False;contact_gap=None
                        if tool_name==active_kind and active_door is not None and obstacle['name']=='CAD door '+active_door['name']:
                            intended=contact_world(active_door,angles.get(active_door['name'],0.))
                            if intended is not None:
                                if active_kind=='roller':contact_point=point(local_poses[tool_link],contacts[active_door['name']]['tool_point'])
                                else:contact_point=point(local_poses[tool_link],contact_tool_point(active_door))
                                contact_gap=math.dist(contact_point,point(arm_inverse,intended));allowed=contact_gap<=(.035 if active_kind=='roller' else .045)
                        if not allowed:
                            raise ValueError(f"Unsafe arm path sample {sample_index}: {tool_name} intersects {obstacle['name']} outside the intended contact zone (active={active}, door angles={angles}, contact gap={contact_gap}, joints={[round(v, 3) for v in q]}).")
                tool_root=multiply(local_poses[tool_link],transform((0.,0.,.1)))
                arm_inverse=inverse(arm_world)
                for component_name,component_local,component_center,component_size in tool_components:
                    component_pose=multiply(multiply(tool_root,component_local),transform(component_center))
                    component_box=obb(component_pose,component_size)
                    component_center_world=component_box[0]
                    component_radius=math.sqrt(sum(v*v for v in component_box[2]))
                    lower_name=component_name.lower()
                    for obstacle,obstacle_box,obstacle_radius in physical_obstacle_boxes:
                        obstacle_name=obstacle['name']
                        delta=[component_center_world[i]-obstacle_box[0][i] for i in range(3)]
                        if sum(v*v for v in delta)>(component_radius+obstacle_radius)**2:continue
                        if obstacle.get('mesh_tree') is not None:
                            if not _obb_intersects_mesh(component_box,obstacle['mesh_inverse'],obstacle['mesh_tree']):continue
                        elif separation(component_box,obstacle_box)>0.:continue
                        allowed=False
                        active_door=next((door for door in (left,right) if door['name']==active),None)
                        if active_door is not None and obstacle_name=='CAD door '+active_door['name']:
                            intended=contact_world(active_door,angles.get(active_door['name'],0.))
                            if intended is not None:
                                if contact_tool_kind(active_door)=='roller':
                                    spec=contacts[active_door['name']]
                                    contact_point=point(local_poses[tool_link],spec['tool_point'])
                                    roller_part=any(token in lower_name for token in ('polymer_roecrha50-10-30-t5','core_roecrha50-10-30-t5','bearing-1_roecrha50-10-30-t5'))
                                    allowed=(roller_part and math.dist(contact_point,point(arm_inverse,intended))<=.035)
                                else:
                                    contact_point=point(local_poses[tool_link],contact_tool_point(active_door))
                                    is_hook=('hook_v2_hook' in lower_name or 'hook_uretan_c' in lower_name)
                                    allowed=is_hook and math.dist(contact_point,point(arm_inverse,intended))<=.045
                        if active=='panel-contact' and obstacle_name=='SIM phone panel':
                            pad_part=any(token in lower_name for token in ('vacuum','suction','cup','pad','zpt','zp10'))
                            panel=panel_obj
                            if pad_part and panel is not None:
                                panel_world=multiply(arm_world,panel_sample['matrix']) if panel_sample is not None else None
                                if panel_world is not None:
                                    seal_surface=point(panel_world,(0.,0.,panel['size'][2]/2))
                                    cup_center=point(poses[tool_link],(0.,0.,physics.config['mount_offset_m']+.115))
                                    allowed=math.dist(cup_center,seal_surface)<=.025
                        if not allowed:
                            raise ValueError(f"Unsafe arm path sample {sample_index}: end-effector component '{component_name}' intersects {obstacle_name} outside its intended contact zone (active={active}, AMR=({base[0]:.3f}, {base[1]:.3f}), joints={[round(v, 3) for v in q]}).")
        finally:physics.objects=original

    def _spatial_hook_demo_tick(self):
        view=self.world3d;task=view._hook_demo
        if not task or not self.winfo_exists():return
        now=time.monotonic();segment=task['segments'][task['index']]
        q0,base0,angles0,q1,base1,angles1,_active,duration=segment
        elapsed=max(0.,now-task.get('last_tick',now));task['last_tick']=now
        task['progress']=min(1.,task.get('progress',0.)+elapsed*self._spatial_motion_speed()/max(.001,duration))
        fraction=task['progress'];smooth=fraction*fraction*(3.-2.*fraction)
        delta_theta=math.atan2(math.sin(base1[2]-base0[2]),math.cos(base1[2]-base0[2]))
        self.sim.state.x=base0[0]+(base1[0]-base0[0])*smooth
        self.sim.state.y=base0[1]+(base1[1]-base0[1])*smooth
        self.sim.state.theta=base0[2]+delta_theta*smooth
        self.arm_dev_sim.q=[a+(b-a)*smooth for a,b in zip(q0,q1)]
        for name in angles0:view.cad_door_hold_angles[name]=math.radians(angles0[name]+(angles1[name]-angles0[name])*smooth)
        if 'vacuum_on_from' in task:
            simulator=self.arm_dev_sim;config=simulator.physics.config
            if simulator.physics.held is None:
                panel=next((obj for obj in simulator.physics.objects if obj.get('name')==task.get('panel_name')),None)
                if panel is not None and view.panel_pick:
                    from .arm_physics import inverse
                    current_arm=multiply(transform((self.sim.state.x,self.sim.state.y,0.),(0.,0.,self.sim.state.theta)),transform(view.mount[:3],view.mount[3:]))
                    panel['matrix']=multiply(inverse(current_arm),view.panel_pick['matrix'])
            simulator.io[config['do_bank']][config['do_channel']]=int(task['index']>=task['vacuum_on_from'])
            simulator.physics.advance(simulator.q,simulator.io,elapsed)
        if fraction>=1.:
            task['index']+=1
            if task['index']>=len(task['segments']):
                view.cad_door_hold_angles[task['left_name']]=math.radians(120.)
                view.cad_door_hold_angles[task['right_name']]=math.radians(120.)
                view._hook_demo=None;self._aw_sync_main(render=False)
                if task.get('panel_name'):
                    if self.arm_dev_sim.physics.held and self.arm_dev_sim.physics.held.get('name')==task['panel_name']:
                        self.log('SIM','Vacuum seal held. Panel lifted from the shelf and extracted through the 120-degree open front; the AMR stayed on the LM6 centerline.')
                    else:
                        self.arm_dev_sim.io[self.arm_dev_sim.physics.config['do_bank']][self.arm_dev_sim.physics.config['do_channel']]=0
                        panel=next((o for o in self.arm_dev_sim.physics.objects if o.get('name')==task['panel_name']),None)
                        if panel:panel['static']=True
                        self.log('SIM','Panel extraction stopped: the vacuum seal did not engage; panel remains on the shelf.')
                else:self.log('SIM','Complete: both front doors opened 120 degrees; DD route and arm/door collision checks passed.')
                if self.view_mode.get()=='3D':self.draw_map()
                if task.get('chain_panel'):
                    try:self._spatial_run_panel_demo()
                    except Exception as error:self.log('SIM',f'Doors are open, but panel extraction did not start: {error}')
                return
            task['progress']=0.;task['last_tick']=now;task['duration']=task['segments'][task['index']][7]
        self._aw_sync_main(render=False)
        if self.view_mode.get()=='3D':self.draw_map()
        self.after(40,self._spatial_hook_demo_tick)

    def _spatial_demo_path(self):
        from .app import ROOT,USER_DIR
        from .fairino_model import installed
        return installed(USER_DIR) or ROOT/'examples/fairino_fr5.urdf'

    def _spatial_switch(self):
        self.release_drive()
        if self.view_mode.get()=='3D':
            self.operation_canvas.pack_forget();self.world3d.pack(fill='both',expand=True)
            self.world3d.refresh()
        else:
            self.world3d.pack_forget();self.operation_canvas.pack(fill='both',expand=True)
        self.after_idle(self.draw_map)

    def _spatial_camera(self):
        self.view_mode.set('3D');self._spatial_switch()
        if self.camera_view.get() in ('로봇 추적','뒤따라 보기'):
            self.world3d.chase=self.camera_view.get()=='뒤따라 보기';self.world3d.focus_robot()
        elif self.camera_view.get()=='사진형':
            self.world3d.chase=False;self.world3d.focus_robot();self.world3d.follow.set(False);self.world3d.preset('사진형')
        else:self.world3d.chase=False;self.world3d.follow.set(False);self.world3d.preset(self.camera_view.get())

    def _spatial_projection(self):
        self.world3d.camera.perspective=self.projection_view.get()=='원근'
        self.view_mode.set('3D');self._spatial_switch()

    def _spatial_render(self):
        self.world3d.runtime_visual_joints={}
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
            feedback=getattr(self,'fr5_feedback',{})
            if self.studio_config['arm'].get('driver')=='fairino' and time.monotonic()-getattr(self,'fr5_rx',0)<=2 and self.fr5_client.connected:
                arm=feedback
                names=[j['name'] for j in self.world3d.arm_asset.movable()]
                values=feedback.get('joints_rad',[])
                if len(names)==len(values)==6:joints=dict(zip(names,values))
            # Runtime uses J1..J6 in degrees; the adapter maps these to the
            # loaded arm's six movable joints in radians. SDK feedback wins.
            asset=self.world3d.arm_asset
            names=[j['name'] for j in asset.movable()] if hasattr(asset,'movable') else []
            if not names or not all(name in joints for name in names):
                runtime=getattr(self,'runtime_arm_feedback',None)
                if runtime is not None:
                    joints=runtime.joints(names)
                    if hasattr(runtime,'visual_joints'):self.world3d.runtime_visual_joints=runtime.visual_joints(names)
                    if joints:arm=dict(status='REAL',pose='MEASURED',feedback_source='MoMa Runtime · 실시간 관절값')
        if self.real:
            lidar=self.real_laser_points if 'laser_beams(WORLD)' in self.lidar_source else [self._lidar_map_xy(x,y) for x,y in self.real_laser_points]
            if not self.connected or time.monotonic()-self.last_laser_rx>2:lidar=[]
        else:lidar=self.scan_points if self.connected and self.sim_powered else []
        self.world3d._isaac_lidar=list(lidar)
        self.world3d.draw_scene(self.map,state,arm,route,joints,lidar if self.layers['LiDAR'].get() else [])

    def _spatial_save(self):
        if not hasattr(self,'studio_config'):return
        view=self.world3d
        self.studio_config['viewer3d']=dict(view=self.view_mode.get(),camera=self.camera_view.get(),projection=self.projection_view.get(),
            runtime_url=self.studio_config.get('viewer3d',{}).get('runtime_url','http://192.168.30.13:8000/runtime.html'),assembly_version='lm6-wall-parallel-dd-mirrored-hook-20261102',mount=view.mount,cad_origin=view.cad_origin,phone_panel_center=list(getattr(view,'phone_panel_center',SIM_PANEL_CENTER)),equipment_gap_target_m=getattr(view,'equipment_gap_target_m',.21),phone_panel_yaw=getattr(view,'phone_panel_yaw',math.pi/2),phone_panel_pose_version=2,vacuum_display=getattr(view,'vacuum_display_config',self.studio_config.get('viewer3d',{}).get('vacuum_display',{})),wall_height=view.wall_height,cad_doors_auto=view.cad_doors_auto.get(),real_doors_follow=view.real_doors_follow.get(),cad_placement_confirmation=view.cad_placement_confirmation,webcam_doors_enabled=view.webcam_doors_enabled.get(),door_initial_angles=getattr(view,'door_initial_angles',{}),layers={k:v.get() for k,v in view.layers.items()},assets=self.spatial_assets)
        self._studio_save_settings()
        self._spatial_sync_cad_lidar()

    def _spatial_place_cell_at_lm6(self,config):
        """Migrate the cell pose so LM6 is the left-door hook approach station."""
        view=self.world3d
        if config.get('assembly_version')=='lm6-wall-parallel-dd-mirrored-hook-20261102':return
        node=getattr(self,'map',None)
        node=getattr(node,'nodes',{}).get('LM6') if node is not None else None
        if not isinstance(node,dict) or view.cad_asset is None:return
        door=next((part for part in getattr(view.cad_asset,'door_parts',()) if part.get('name','').endswith(':1')),None)
        if door is None:return
        handle_vertices=[p for face,material in zip(door['asset'].faces,door['asset'].face_materials) if material==36 for p in face]
        if not handle_vertices:return
        handle=tuple((min(p[i] for p in handle_vertices)+max(p[i] for p in handle_vertices))/2 for i in range(3))
        state=self.current_state()
        station_theta=float(state.get('theta',node.get('r',node.get('angle',0.))))
        station_x=float(node.get('x',0.));station_y=float(node.get('y',0.))
        # Keep the enclosure front/door plane parallel to the horizontal upper
        # wall; the model stays north of the LM7 -> LM6 straight travel lane.
        cad_theta=0.
        desired_rotation=multiply(transform((0.,0.,0.),(0.,0.,cad_theta)),transform((0.,0.,0.),(math.pi,0.,0.)))
        # This reachable J6 target was checked against the six-axis FR5 model.
        # The fixed 1.40 m north offset leaves extra clearance for the FR5
        # wrist sweep while keeping the cabinet parallel to the upper wall.
        fixture_y=station_y+1.40
        j6_target=(station_x+.60,fixture_y+handle[1]-.14714,view.cad_origin[2]+handle[2]+.148)
        hook_offset=point(desired_rotation,(0.,-.14714,.148))
        handle_world=tuple(j6_target[i]+hook_offset[i] for i in range(3))
        rotated_handle=point(transform((0.,0.,0.),(0.,0.,cad_theta)),handle)
        origin_x=handle_world[0]-rotated_handle[0]
        origin_y=handle_world[1]-rotated_handle[1]
        view.cad_origin=[origin_x,origin_y,view.cad_origin[2],0.,0.,cad_theta]
        view.cad_doors_auto.set(False)
        self.log('3D',f'SIMPLE_WHOLE placed parallel to the upper wall for LM6: origin=({origin_x:.3f}, {origin_y:.3f}), yaw={math.degrees(cad_theta):.1f} deg; LM7-LM6 lane remains straight and clear.')
        self._spatial_save()

    def _spatial_sync_cad_lidar(self):
        """Project the placed CAD cross-section into SIM scan/collision walls."""
        view=getattr(self,'world3d',None);asset=getattr(view,'cad_asset',None)
        origin=tuple(getattr(view,'cad_origin',(0.,)*6));scale=float(getattr(view,'scales',{}).get('cad',1.))
        signature=(id(getattr(self,'map',None)),id(getattr(self,'sim',None)),id(asset),origin,scale)
        if signature==getattr(self,'_cad_lidar_signature',None):return
        segments=[]
        if asset is not None and scale>0 and math.isfinite(scale):
            from .geometry3d import transform,point
            world=transform(origin[:3],origin[3:]);plane=.1965;intersections=set()
            faces=list(asset.faces)+list(getattr(asset,'transparent_faces',()))
            for face in faces:
                vertices=[point(world,tuple(p[i]*scale for i in range(3))) for p in face]
                for a,b in zip(vertices,vertices[1:]+vertices[:1]):
                    da,db=a[2]-plane,b[2]-plane
                    if abs(da)<1e-8 and abs(db)<1e-8:
                        intersections.add((round(a[0],3),round(a[1],3)));intersections.add((round(b[0],3),round(b[1],3)))
                    elif da*db<=0 and abs(da-db)>1e-12:
                        t=da/(da-db)
                        if 0.<=t<=1.:
                            intersections.add((round(a[0]+t*(b[0]-a[0]),3),round(a[1]+t*(b[1]-a[1]),3)))
            points=sorted(intersections)
            def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
            if len(points)>=3:
                lower=[]
                for p in points:
                    while len(lower)>=2 and cross(lower[-2],lower[-1],p)<=0:lower.pop()
                    lower.append(p)
                upper=[]
                for p in reversed(points):
                    while len(upper)>=2 and cross(upper[-2],upper[-1],p)<=0:upper.pop()
                    upper.append(p)
                hull=lower[:-1]+upper[:-1]
                segments=[[a[0],a[1],b[0],b[1]] for a,b in zip(hull,hull[1:]+hull[:1])]
        model=getattr(self,'map',None);sim=getattr(self,'sim',None)
        if model is not None:model.cad_walls=segments
        if sim is not None and getattr(sim,'map',None) is not model:sim.map.cad_walls=segments
        self._cad_lidar_signature=signature

    def _spatial_restore(self):
        config=self.studio_config.get('viewer3d',{})
        if not isinstance(config,dict):config={}
        view=self.world3d
        view.cad_placement_confirmation=config.get('cad_placement_confirmation')
        gap=config.get('equipment_gap_target_m',.21)
        view.equipment_gap_target_m=gap if type(gap) in (int,float) and math.isfinite(gap) and gap>0 else .21
        view.phone_panel_yaw=float(config.get('phone_panel_yaw',math.pi/2))
        if config.get('phone_panel_pose_version')!=2:
            config=dict(config,phone_panel_center=list(SIM_PANEL_CENTER))
        view.vacuum_display_config=dict(config.get('vacuum_display',{}))
        panel_center=config.get('phone_panel_center')
        if isinstance(panel_center,list) and len(panel_center)==3 and all(type(v) in (int,float) and math.isfinite(v) for v in panel_center):
            view.phone_panel_center=list(panel_center)

        if type(config.get('cad_doors_auto')) is bool:view.cad_doors_auto.set(config['cad_doors_auto'])
        view.real_doors_follow.set(bool(config.get('real_doors_follow',True)))
        view.webcam_doors_enabled.set(bool(config.get('webcam_doors_enabled',False)))
        initial=config.get('door_initial_angles',{})
        view.door_initial_angles={side:float(initial[side]) for side in ('left','right') if isinstance(initial,dict) and type(initial.get(side)) in (int,float) and math.isfinite(initial[side]) and 0<=initial[side]<=120}
        if view.door_initial_angles:view.cad_doors_auto.set(False)
        for key in ('mount','cad_origin'):
            values=config.get(key)
            if isinstance(values,list) and len(values)==6 and all(type(v) in (int,float) and math.isfinite(v) for v in values):setattr(view,key,values)
        height=config.get('wall_height')
        if config.get('assembly_version') not in ('front-sync-20261005','lm6-wall-parallel-dd-20261102','lm6-wall-parallel-dd-mirrored-hook-20261102'):
            central=abs(view.mount[0])<.001 and abs(view.mount[1])<.001 and view.mount[2]<=.65
            prior_default=view.mount==[.31,0.,.608,0.,0.,0.]
            if central or prior_default:view.mount=[.34,0.,.626,0.,0.,0.]
        if type(height) in (int,float) and .02<=height<=10:view.wall_height=height
        for key,value in (config.get('layers') if isinstance(config.get('layers'),dict) else {}).items():
            if key in view.layers and type(value) is bool:view.layers[key].set(value)
        for role,spec in (config.get('assets') if isinstance(config.get('assets'),dict) else {}).items():
            if role not in ('arm','amr','cad','tool') or not isinstance(spec,dict):continue
            try:
                scale=float(spec.get('scale',1))
                if not math.isfinite(scale) or scale<=0:raise ValueError('3D 모델 배율이 비정상입니다.')
                rotation=spec.get('rotation',(0.,0.,0.))
                if not isinstance(rotation,(list,tuple)) or len(rotation)!=3 or not all(type(v) in (int,float) and math.isfinite(v) for v in rotation):rotation=(0.,0.,0.)
                from .app import ROOT
                asset_path=Path(spec['path'])
                if not asset_path.is_absolute():asset_path=ROOT/asset_path
                view.load_asset(role,asset_path,spec.get('root'),scale,rotation);self.spatial_assets[role]=spec
            except (OSError,ValueError,TypeError,KeyError) as error:self.log('3D',f'{role} 모델 복원 실패: {error}')
        # First run and pre-extctrl settings get the bundled MoMa AMR model automatically.
        # Keep a user-selected AMR asset if one was already saved.
        configured_assets=config.get('assets') if isinstance(config.get('assets'),dict) else {}
        if 'amr' not in configured_assets:
            try:
                from .app import ROOT
                amr_path=ROOT/'models/extctrl_linux/SDC-400EU-light.glb'
                view.load_asset('amr',amr_path,scale=1.,rotation=(0.,90.,90.))
                view.layers['AMR'].set(True)
                self.spatial_assets['amr']=dict(path='models/extctrl_linux/SDC-400EU-light.glb',root='',scale=1.,rotation=[0.,90.,90.])
                view.mount=[.35,0.,.5085,0.,0.,math.radians(135.)]
            except (OSError,ValueError,TypeError,KeyError) as error:self.log('3D',f'기본 MoMa AMR 모델 복원 실패: {error}')
        if 'arm' not in configured_assets:
            try:
                from .app import ROOT
                arm_path=ROOT/'models/extctrl_linux/FR5_Assembly.glb'
                view.load_asset('arm',arm_path,scale=1.)
                view.layers[chr(0xB85C)+chr(0xBD07)+chr(0xD314)].set(True)
                self.spatial_assets['arm']=dict(path='models/extctrl_linux/FR5_Assembly.glb',root='',scale=1.,rotation=[0.,0.,0.])
                if getattr(self,'arm_dev_sim',None) is not None:self._aw_sync_main(render=False)
            except (OSError,ValueError,TypeError,KeyError) as error:self.log('3D',f'기본 extctrl FR5 모델 복원 실패: {error}')
        # Always restore the matching full end-effector assembly. Older saved
        # viewer settings can point at a reduced/no-hook mesh; the door-contact
        # model is calibrated to this GLB and its authored pose.
        try:
            from .app import ROOT
            tool_path=ROOT/'models/extctrl_linux/gripper/EndEffector V3_EndOnly_Change1-org.glb'
            view.load_asset('tool',tool_path,scale=1.,rotation=(-90.,0.,0.))
            view.layers[chr(0xB85C)+chr(0xBD07)+chr(0xD314)].set(True)
            self.spatial_assets['tool']=dict(path='models/extctrl_linux/gripper/EndEffector V3_EndOnly_Change1-org.glb',root='',scale=1.,rotation=[-90.,0.,0.])
        except (OSError,ValueError,TypeError,KeyError) as error:self.log('3D',f'extctrl hook end-effector restore failed: {error}')
        # Add the bundled whole-cell CAD as the default 3D map environment.
        # A previously configured CAD asset always takes precedence.
        if view.cad_asset is None:
            try:
                from .app import ROOT
                cad_path=ROOT/'SIMPLE_WHOLE.glb'
                cad=view.load_asset('cad',cad_path,scale=1.)
                # The exported assembly is centered around Z=0; lift its lowest
                # point to the map floor while keeping its authored XY placement.
                minimum_z=min(p[2] for face in cad.faces for p in face)
                view.cad_origin=[0.,0.,max(0.,-minimum_z),0.,0.,0.]
                view.layers['CAD'].set(True)
                self.spatial_assets['cad']=dict(path='SIMPLE_WHOLE.glb',root='',scale=1.,rotation=[0.,0.,0.])
            except (OSError,ValueError,TypeError,KeyError) as error:self.log('3D',f'湲곕낯 SIMPLE_WHOLE CAD 紐⑤뜽 蹂듭썝 ?ㅽ뙣: {error}')
        if getattr(view.body_asset,'name','').startswith('SEER SDC-400EU-light'):
            view.layers['AMR'].set(True)
        if getattr(view.arm_asset,'name','')=='FAIRINO FR5 extctrl assembly':
            view.layers[chr(0xB85C)+chr(0xBD07)+chr(0xD314)].set(True)
        if getattr(view.tool_asset,'name','').startswith('EndEffector'):
            view.layers[chr(0xB85C)+chr(0xBD07)+chr(0xD314)].set(True)
        self.camera_view.set(config.get('camera','사선'));view.preset(self.camera_view.get())
        self.projection_view.set('직교' if config.get('projection')=='직교' else '원근');view.camera.perspective=self.projection_view.get()=='원근'
        self.view_mode.set('3D' if config.get('view','3D')=='3D' else '2D');self._spatial_switch()
        if self.camera_view.get() in ('로봇 추적','뒤따라 보기'):
            view.chase=self.camera_view.get()=='뒤따라 보기';view.focus_robot()

        if self.view_mode.get()=='3D' and not view.follow.get():
            state=self.current_state();theta=state.get('theta',0.)
            view.camera.target=[state.get('x',0.)+.35*math.cos(theta),state.get('y',0.)+.35*math.sin(theta),.85]
            view.camera.distance=3.5;view.follow.set(False);view.fitted=id(self.map)
        self._spatial_place_cell_at_lm6(config)
        self._spatial_sync_cad_lidar()

    def _spatial_dialog(self):
        self.tabs.select(self.operation_page);self.view_mode.set('3D');self._spatial_switch()
        win=tk.Toplevel(self);win.title('3D 모델 / 관절 / 표시 설정');self._fit_dialog(win,840,680,680,540);win.configure(bg=BG);win.transient(self)
        view=self.world3d
        _,body,_canvas=self._scrollable_frame(win,bg=PANEL)
        container=body.master.master;container.pack(fill='both',expand=True,padx=8,pady=8)
        self.label(body,'3D 공간 · AMR + 로봇팔',14,INK,True).pack(anchor='w',padx=12,pady=(10,5))
        self.label(body,'관절 슬라이더는 화면 미리보기용입니다. 기본 팔은 SIM 미션에 연동됩니다.\nURDF: link/visual/joint 변환 · CAD: STL/OBJ/GLB 정적 메시 · STEP/IGES는 STL/OBJ로 내보내세요.',9,MUTED,justify='left').pack(fill='x',padx=12,pady=5)
        row=tk.Frame(body,bg=PANEL);row.pack(fill='x',padx=12,pady=5)
        for key,var in view.layers.items():tk.Checkbutton(row,text=key,variable=var,bg=PANEL,command=view.refresh).pack(side='left')
        row=tk.Frame(body,bg=PANEL);row.pack(fill='x',padx=12,pady=5)
        tk.Checkbutton(row,text='로봇 추적',variable=view.follow,bg=PANEL,command=lambda:view.focus_robot() if view.follow.get() else view.refresh()).pack(side='left')
        tk.Checkbutton(row,text='실기 수신 관절 연동 · SIM은 항상 동기화',variable=view.arm_follow,bg=PANEL,command=view.refresh).pack(side='left',padx=10)
        self.button(row,'SIM 팔 동작 보기',lambda:self.guarded(self._studio_test_arm),BLUE).pack(side='left')
        self.button(row,'지도 전체 보기',view.fit).pack(side='left',padx=6)
        row=tk.Frame(body,bg=PANEL);row.pack(fill='x',padx=12,pady=5)
        role=tk.StringVar(value='로봇팔');units=tk.StringVar(value='m');root=tk.StringVar(value='')
        ttk.Combobox(row,textvariable=role,values=['AMR','로봇팔','장면 CAD'],state='readonly',width=12).pack(side='left')
        ttk.Combobox(row,textvariable=units,values=['m','mm'],state='readonly',width=5).pack(side='left',padx=5)
        summary=self.label(body,'',9,MUTED,justify='left',wraplength=760);summary.pack(fill='x',padx=12,pady=5)
        joint_frame=tk.Frame(body,bg=PANEL)
        def refresh_summary():
            summary.configure(text=f"AMR: {getattr(view.body_asset,'name','AMR')} | FR5: {view.arm_asset.name} | Tool: {getattr(view.tool_asset,'name','none')} | CAD: {getattr(view.cad_asset,'name','none')}\n"+(' | '.join(getattr(view.arm_asset,'warnings',[])) or 'URDF units m/rad. Set the package root to resolve package:// meshes.'))
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
                    if part=='arm' and not self.real and hasattr(self,'arm_dev_sim'):
                        q=list(self.arm_dev_sim.q);names=[j['name'] for j in self.arm_dev_sim.kin.joints]
                        if key in names:q[names.index(key)]=float(value)/scale
                        self.guarded(lambda:self._aw_set_synced_joints(q));return
                    view.arm_follow.set(False);view.positions[part][key]=float(value)/scale;view.refresh()
                tk.Scale(row,from_=lower*factor,to=upper*factor,variable=variable,orient='horizontal',resolution=.001 if prismatic else 1,
                         showvalue=True,bg=PANEL,highlightthickness=0,length=420,command=changed).pack(side='left',fill='x',expand=True)
        def load(kind):
            path=filedialog.askopenfilename(parent=win,filetypes=[('URDF','*.urdf')] if kind=='urdf' else [('3D mesh','*.stl *.obj *.glb')])
            if not path:return
            def apply():
                key={'AMR':'amr','로봇팔':'arm','장면 CAD':'cad'}[role.get()];scale=1. if kind=='urdf' or units.get()=='m' else .001
                if key=='arm' and self.arm_dev_sim.state in ('RUNNING','PAUSED'):raise ValueError('팔 재생을 정지한 뒤 모델을 변경하세요.')
                rotation=(0.,90.,90.) if str(path).lower().endswith('.glb') and key=='amr' else (0.,0.,0.)
                asset=view.load_asset(key,path,root.get() or None,scale,rotation)
                if key=='arm' and isinstance(asset,RobotDescription):self._aw_adopt_loaded_asset(asset,path)
                self.spatial_assets[key]=dict(path=path,root=root.get(),scale=scale,rotation=list(rotation))
                refresh_summary();joint_controls();self._spatial_save()
                warnings=getattr(asset,'warnings',[])
                if warnings:messagebox.showwarning('URDF 가져오기: 일부 visual 제외','\n'.join(warnings[:15]),parent=win)
            self.guarded(apply)
        self.button(row,'URDF 가져오기',lambda:load('urdf'),BLUE).pack(side='left',padx=5)
        self.button(row,'STL / OBJ / GLB 가져오기',lambda:load('mesh')).pack(side='left',padx=5)
        def restore_demo():
            if self.arm_dev_sim.state in ('RUNNING','PAUSED'):raise ValueError('팔 재생을 정지한 뒤 모델을 변경하세요.')
            self._aw_adopt_loaded_asset(RobotDescription.load(self._spatial_demo_path()),self._spatial_demo_path())
            view.arm_asset=RobotDescription.load(self._spatial_demo_path());view.arm_asset.synthetic=True;view.positions['arm']={};view.arm_follow.set(True)
            view.scales['arm']=1.;view.last_arm_operation=None
            self.spatial_assets.pop('arm',None);refresh_summary();joint_controls();view.refresh();self._spatial_save()
        self.button(row,'기본 FR5 복원',restore_demo).pack(side='left',padx=5)
        row=tk.Frame(body,bg=PANEL);row.pack(fill='x',padx=12,pady=3)
        self.label(row,'사진형 전장 케이스 + 앞쪽 FR5 · AMR: SBA-400EU · FR5: 설치된 공식 ROS2 모델 / 경량 대체 모델',9,MUTED).pack(side='left')
        def restore_amr():
            view.body_asset=RobotDescription.load(Path(__file__).resolve().parents[1]/'models/seer_sba400eu_description/urdf/mobile_manipulator.urdf')
            view.positions['amr']={};view.scales['amr']=1.;view.mount=[.34,0.,.626,0.,0.,0.];self.spatial_assets.pop('amr',None)
            for i,var in enumerate(mount_values):var.set(f'{view.mount[i] if i<3 else math.degrees(view.mount[i]):.3f}')
            refresh_summary();joint_controls();view.refresh();self._spatial_save()
        self.button(row,'사진형 MoMa 복원',restore_amr).pack(side='left',padx=5)
        def restore_linux_moma():
            if self.arm_dev_sim.state in ('RUNNING','PAUSED'):raise ValueError('팔 재생을 정지한 뒤 모델을 변경하세요.')
            from .app import ROOT
            amr_path=ROOT/'models/extctrl_linux/SDC-400EU-light.glb'
            arm_path=ROOT/'models/extctrl_linux/FR5_Assembly.glb'
            tool_path=ROOT/'models/extctrl_linux/gripper/EndEffector V3_EndOnly_Change1-org.glb'
            view.load_asset('amr',amr_path,scale=1.,rotation=(0.,90.,90.))
            view.layers['AMR'].set(True)
            asset=view.load_asset('arm',arm_path,scale=1.)
            view.layers[chr(0xB85C)+chr(0xBD07)+chr(0xD314)].set(True)
            view.load_asset('tool',tool_path,scale=1.,rotation=(-90.,0.,0.))
            view.layers[chr(0xB85C)+chr(0xBD07)+chr(0xD314)].set(True)
            view.arm_asset.synthetic=True;view.positions['arm']={};view.arm_follow.set(True)
            self._aw_sync_main(render=False)
            view.mount=[.35,0.,.5085,0.,0.,math.radians(135.)]
            view.scales['amr']=1.;view.positions['amr']={}
            self.spatial_assets['amr']=dict(path='models/extctrl_linux/SDC-400EU-light.glb',root='',scale=1.,rotation=[0.,90.,90.])
            self.spatial_assets['arm']=dict(path='models/extctrl_linux/FR5_Assembly.glb',root='',scale=1.,rotation=[0.,0.,0.])
            self.spatial_assets['tool']=dict(path='models/extctrl_linux/gripper/EndEffector V3_EndOnly_Change1-org.glb',root='',scale=1.,rotation=[-90.,0.,0.])
            for i,var in enumerate(mount_values):var.set(f'{view.mount[i] if i<3 else math.degrees(view.mount[i]):.3f}')
            refresh_summary();joint_controls();view.refresh();self._spatial_save()
        self.button(row,'extctrl Linux MoMa 적용',lambda:self.guarded(restore_linux_moma),BLUE).pack(side='left',padx=8)
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
        self.button(body,'실제 LiDAR로 설비 배치 / 확정',lambda:self.guarded(self._spatial_equipment_placement),BLUE).pack(anchor='w',padx=12,pady=4)
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
        def nudge_cad():
            values=[float(var.get()) for var in cad_values]
            if not all(math.isfinite(v) for v in values):raise ValueError('CAD 배치 좌표를 확인하세요.')
            view.cad_placement_confirmation=None
            view.reset_real_doors()
            apply_cad()
        from .placement_nudge import add_nudge_controls
        add_nudge_controls(body,[cad_values[0],cad_values[1],cad_values[5]],nudge_cad,self.guarded)
        self.label(body,'설비 내부 휴대폰 패널 위치 · 설비 기준 X/Y (m)',10,INK,True).pack(anchor='w',padx=12,pady=(10,4))
        panel_position=getattr(view,'phone_panel_center',SIM_PANEL_CENTER)
        panel_vars=[tk.StringVar(value=f'{panel_position[i]:.4f}') for i in range(2)]
        panel_row=tk.Frame(body,bg=PANEL);panel_row.pack(fill='x',padx=12,pady=4)
        step=tk.StringVar(value='0.001')
        panel_yaw=tk.StringVar(value=f'{math.degrees(getattr(view,"phone_panel_yaw",math.pi/2)):.1f}')
        def apply_panel():
            xy=[float(var.get()) for var in panel_vars];yaw=math.radians(float(panel_yaw.get()))
            if not math.isfinite(yaw) or not all(math.isfinite(v) for v in xy):raise ValueError('패널 X/Y에 유한한 숫자를 입력하세요.')
            simulator=getattr(self,'arm_dev_sim',None)
            if (getattr(view,'_hook_demo',None) or (simulator and getattr(simulator,'state',None) in ('RUNNING','PAUSED')) or (simulator and getattr(simulator,'physics',None) and simulator.physics.held)):
                raise ValueError('패널 작업을 종료하고 물체를 놓은 뒤 위치를 변경하세요.')
            old=getattr(view,'phone_panel_center',SIM_PANEL_CENTER)
            view.phone_panel_center=[xy[0],xy[1],old[2]]
            view.phone_panel_yaw=yaw
            if simulator and getattr(simulator,'physics',None):
                simulator.physics.objects=[obj for obj in simulator.physics.objects if obj.get('name')!='SIM phone panel']
            view.panel_pick=None
            if hasattr(view,'_vacuum_panel'):delattr(view,'_vacuum_panel')
            view.refresh();self._spatial_save()
        def nudge_panel(index,direction):
            amount=float(step.get())
            if not math.isfinite(amount) or amount<=0:raise ValueError('증분은 0보다 큰 유한한 숫자여야 합니다.')
            previous=panel_vars[index].get()
            panel_vars[index].set(f'{float(previous)+amount*direction:.6f}')
            try:apply_panel()
            except Exception:
                panel_vars[index].set(previous);raise
        for index,name in enumerate(('X','Y')):
            self.label(panel_row,name,9,INK).pack(side='left')
            ttk.Entry(panel_row,textvariable=panel_vars[index],width=10).pack(side='left',padx=4)
            for direction,label in ((-1,'−'),(1,'＋')):
                ttk.Button(panel_row,text=label,width=3,command=lambda i=index,d=direction:self.guarded(lambda:nudge_panel(i,d))).pack(side='left',padx=2)
        self.label(panel_row,'방향 °',9,INK).pack(side='left',padx=4)
        ttk.Entry(panel_row,textvariable=panel_yaw,width=6).pack(side='left')
        self.button(panel_row,'적용 / 저장',lambda:self.guarded(apply_panel),BLUE).pack(side='left',padx=6)
        step_row=tk.Frame(body,bg=PANEL);step_row.pack(fill='x',padx=12,pady=3)
        self.label(step_row,'이동 증분 (m)',9,MUTED).pack(side='left')
        ttk.Combobox(step_row,textvariable=step,values=('0.001','0.005','0.01'),width=9).pack(side='left',padx=6)
        def reset_panel():
            for i,var in enumerate(panel_vars):var.set(str(SIM_PANEL_CENTER[i]))
            panel_yaw.set('90')
            self.guarded(apply_panel)
        self.button(step_row,'기본 위치 복원',reset_panel).pack(side='left',padx=4)
        self.label(body,'설비와 함께 이동하는 패널 위치입니다. Z 높이는 유지됩니다. ± 버튼은 즉시 적용·저장합니다.',9,MUTED,wraplength=650).pack(anchor='w',padx=12,pady=3)
        io_cfg=getattr(view,'vacuum_display_config',{})
        io_box=ttk.LabelFrame(body,text='패널 베큠 IO 표시 · 실제 출력 제어 없음');io_box.pack(fill='x',padx=12,pady=8)
        io_source=tk.StringVar(value=io_cfg.get('source','Runtime'))
        io_bank=tk.StringVar(value=io_cfg.get('bank','DO'));io_channel=tk.StringVar(value=str(io_cfg.get('channel','')))
        io_path=tk.StringVar(value=io_cfg.get('runtime_io_path',''));io_high=tk.BooleanVar(value=io_cfg.get('active_high',True))
        io_row=ttk.Frame(io_box);io_row.pack(fill='x',padx=6,pady=4)
        ttk.Combobox(io_row,textvariable=io_source,values=('FR5 SDK','Runtime'),state='readonly',width=10).pack(side='left',padx=3)
        ttk.Combobox(io_row,textvariable=io_bank,values=('DO','ToolDO'),state='readonly',width=8).pack(side='left',padx=3)
        ttk.Label(io_row,text='채널').pack(side='left');ttk.Entry(io_row,textvariable=io_channel,width=4).pack(side='left',padx=4)
        ttk.Checkbutton(io_row,text='ON=1',variable=io_high).pack(side='left',padx=4)
        io_mode=tk.StringVar(value=io_cfg.get('mode','paired'))
        pair_row=ttk.Frame(io_box);pair_row.pack(fill='x',padx=6,pady=4)
        ttk.Combobox(pair_row,textvariable=io_mode,values=('paired','single'),state='readonly',width=8).pack(side='left')
        on_channels=tk.StringVar(value=','.join(map(str,io_cfg.get('on_channels',[2,4]))))
        off_channels=tk.StringVar(value=','.join(map(str,io_cfg.get('off_channels',[3,5]))))
        ttk.Label(pair_row,text='ON 채널').pack(side='left');ttk.Entry(pair_row,textvariable=on_channels,width=8).pack(side='left')
        ttk.Label(pair_row,text='OFF 채널').pack(side='left');ttk.Entry(pair_row,textvariable=off_channels,width=8).pack(side='left')
        ttk.Label(pair_row,text='paired: 2·4 흡착 / 3·5 블로우 · 레벨 판정').pack(side='left',padx=5)
        io_row2=ttk.Frame(io_box);io_row2.pack(fill='x',padx=6,pady=4)
        ttk.Label(io_row2,text='Runtime IO 경로 (제공될 때)').pack(side='left')
        ttk.Entry(io_row2,textvariable=io_path,width=30).pack(side='left',padx=4)
        def save_vacuum_display():
            channel=None;ons=[];offs=[]
            if io_source.get()=='FR5 SDK':
                limit=2 if io_bank.get()=='ToolDO' else 16
                if io_mode.get()=='paired':
                    ons=[int(v.strip()) for v in on_channels.get().split(',')];offs=[int(v.strip()) for v in off_channels.get().split(',')]
                    if set(ons)&set(offs):raise ValueError('ON/OFF 채널은 서로 달라야 합니다.')
                    if any(not 0<=v<limit for v in ons+offs):raise ValueError('DO는 0~15, ToolDO는 0~1입니다.')
                else:
                    channel=int(io_channel.get())
                    if not 0<=channel<limit:raise ValueError('DO는 0~15, ToolDO는 0~1입니다.')
            # Empty Runtime path uses the verified viewer vacuum_io feedback.
            view.vacuum_display_config=dict(source=io_source.get(),bank=io_bank.get(),channel=channel,
                active_high=io_high.get(),runtime_io_path=io_path.get().strip(),mode=io_mode.get(),on_channels=ons,off_channels=offs)
            self._spatial_save()
        ttk.Button(io_row2,text='IO 설정 저장',command=lambda:self.guarded(save_vacuum_display)).pack(side='left',padx=4)
        ttk.Label(io_box,text='Runtime 기본: Viewer의 실제 베큠 출력 조회. 미수신/수신 지연은 OFF로 처리하지 않습니다.').pack(anchor='w',padx=6,pady=4)
        place_status=tk.StringVar(value='')
        def placement_changed(active):
            place_status.set('Click the 3D map to place CAD.' if active else 'CAD position saved.')
        view.cad_placement_callback=placement_changed
        def begin_cad_placement():
            view.cad_placement_mode=True
            placement_changed(True)
        self.button(body,'Place CAD by clicking map',begin_cad_placement,BLUE).pack(anchor='w',padx=12,pady=(4,2))
        def cad_doors_changed():
            if view.cad_doors_auto.get():
                view.cad_door_hold_angle=None;view.cad_door_hold_angles={};view._cad_door_epoch=time.monotonic()
            self._spatial_save();view.refresh()
        ttk.Checkbutton(body,text='Automatic front door open / close',variable=view.cad_doors_auto,command=cad_doors_changed).pack(anchor='w',padx=12,pady=4)
        ttk.Checkbutton(body,text='실기 관절값으로 문 열림 추정 (표시 전용)',variable=view.real_doors_follow,command=view.refresh).pack(anchor='w',padx=12,pady=4)
        self.button(body,'문 추정: 현재 문을 닫힘 기준으로 초기화',view.reset_real_doors,BLUE).pack(anchor='w',padx=12,pady=4)
        self.label(body,'Click a floor location in the 3D map. X/Y are updated; Z and rotation stay as set.',9,MUTED,justify='left',wraplength=690).pack(anchor='w',padx=12,pady=(0,3))
        self.label(body,'',9,BLUE,textvariable=place_status).pack(anchor='w',padx=12,pady=(0,4))
        self.button(body,'CAD 배치 적용 / 저장',lambda:self.guarded(apply_cad),BLUE).pack(anchor='w',padx=12,pady=4)
        self.label(body,'관절 미리보기 · 수동 조절 시 미션 연동이 해제됩니다.',10,INK,True).pack(anchor='w',padx=12,pady=(10,4))
        joint_frame.pack(fill='x',padx=12,pady=(0,12))
        role.trace_add('write',lambda *args:joint_controls());refresh_summary();joint_controls()
        self.button(win,'닫기',win.destroy).pack(side='bottom',pady=5)
        win.joint_frame=joint_frame
        return win
