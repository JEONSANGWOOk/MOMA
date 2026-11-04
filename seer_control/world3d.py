"""Orbitable 3D world viewport using Tk's software polygon renderer."""
import math
import time
from pathlib import Path
import tkinter as tk
from .geometry3d import RobotDescription, MeshAsset, load_mesh, load_glb_robot, load_glb_static_robot, box, cylinder, transform, multiply, point, identity, axis_rotation
from .smap import path_record_geometry
from .smooth_renderer import renderer_for
from .fleet_ui import get_fleet,positioned,color
from .door_feedback import DoorFollower, ContactDoorFollower, SurfaceDoorFollower


PRESETS={'사선':(-135,38),'위':(-90,89.9),'정면':(-90,8),'측면':(0,8),'뒤':(90,8),'사진형':(135,18)}
SIM_PANEL_CENTER=(.05,-.695,.059)  # 10 mm inside the SIMPLE_WHOLE tray front edge; bottom rests at z=.053 m.
SIM_PANEL_SIZE=(.16,.09,.012)


def dot(a,b):return sum(x*y for x,y in zip(a,b))
def cross(a,b):return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def subtract(a,b):return tuple(x-y for x,y in zip(a,b))
def normalize(v):
    length=max(1e-12,math.sqrt(dot(v,v)))
    return tuple(value/length for value in v)


class Camera:
    def __init__(self):self.target=[0.,0.,.4];self.distance=15.;self.yaw=-135.;self.pitch=38.;self.perspective=True
    def basis(self):
        key=(tuple(self.target),self.distance,self.yaw,self.pitch)
        if key==getattr(self,'_basis_key',None):return self._basis_value
        yaw,pitch=math.radians(self.yaw),math.radians(self.pitch)
        eye=tuple(self.target[i]+self.distance*v for i,v in enumerate((math.cos(pitch)*math.cos(yaw),math.cos(pitch)*math.sin(yaw),math.sin(pitch))))
        forward=normalize(subtract(self.target,eye));right=normalize(cross(forward,(0,0,1)));up=cross(right,forward)
        self._basis_key=key;self._basis_value=(eye,right,up,forward)
        return self._basis_value
    def project(self,p,width,height):
        eye,right,up,forward=self.basis();relative=subtract(p,eye)
        depth=dot(relative,forward)
        if depth<=.02:return None
        scale=min(width,height)*.95/depth if self.perspective else min(width,height)*1.6/self.distance
        return width/2+dot(relative,right)*scale,height/2-dot(relative,up)*scale,depth
    def fit(self,model):
        pts=[(n['x'],n['y']) for n in model.nodes.values()]+[(wall[i],wall[i+1]) for wall in model.walls for i in (0,2)]
        cloud=getattr(model,'cloud',[]);pts+=cloud[::max(1,len(cloud)//500)]
        xs,ys=zip(*(pts or [(0,0),(2,2)]))
        self.target=[(min(xs)+max(xs))/2,(min(ys)+max(ys))/2,.4]
        self.distance=max(4.,max(max(xs)-min(xs),max(ys)-min(ys))*1.05)

    def ground(self,x,y,width,height):
        """Inverse projection onto the map plane, for picking and relocation."""
        eye,right,up,forward=self.basis()
        scale=min(width,height)*(.95 if self.perspective else 1.6/self.distance)
        dx,dy=(x-width/2)/scale,(height/2-y)/scale
        if self.perspective:
            origin=eye;direction=tuple(forward[i]+right[i]*dx+up[i]*dy for i in range(3))
        else:
            origin=tuple(eye[i]+right[i]*dx+up[i]*dy for i in range(3));direction=forward
        if abs(direction[2])<1e-9:return None
        distance=-origin[2]/direction[2]
        if distance<0:return None
        return tuple(origin[i]+distance*direction[i] for i in (0,1))


class WorldView3D(tk.Canvas):
    def __init__(self,parent,app,demo_path):
        super().__init__(parent,bg='#dce5ef',highlightthickness=0)
        self.app=app;self.camera=Camera();self.fitted=None;self.drag=None
        self.arm_asset=RobotDescription.load(demo_path);self.arm_asset.synthetic=True
        self.body_asset=RobotDescription.load(Path(__file__).resolve().parents[1]/'models/seer_sba400eu_description/urdf/mobile_manipulator.urdf');self.cad_asset=None
        self.scales={'arm':1.,'amr':1.,'cad':1.,'tool':1.}
        self.tool_asset=None
        self.positions={'arm':{},'amr':{},'cad':{},'tool':{}}
        self.mount=[.34,0.,.626,0.,0.,0.];self.cad_origin=[0.,0.,0.,0.,0.,0.]
        self.cad_placement_mode=False;self.cad_placement_callback=None
        self.cad_doors_auto=tk.BooleanVar(value=True);self._cad_door_epoch=time.monotonic();self._door_animation_job=None
        self.cad_door_hold_angle=None;self.cad_door_hold_angles={};self._hook_demo=None;self.panel_pick=None
        self.real_doors_follow=tk.BooleanVar(value=True)
        self._real_door_follower=None;self._real_door_key=None
        self.real_door_status='문 추정 · 접촉 대기'
        self.cad_placement_confirmation=None
        self.webcam_doors_enabled=tk.BooleanVar(value=False)
        self.layers={key:tk.BooleanVar(value=True) for key in ('지도','점군','경로','좌표축','AMR','로봇팔','CAD')}
        self.follow=tk.BooleanVar(value=False);self.arm_follow=tk.BooleanVar(value=True)
        self.chase=False
        self.wall_height=.45;self.last_arm_operation=None;self.arm_start={};self.arm_source='SIM 합성 관절'
        self.scene_pose={};self.scene_joint_values={};self.render_count=0;self.total_faces=0
        self.bind('<Configure>',lambda event:self.refresh())
        self.bind('<ButtonPress-1>',lambda event:self.begin_drag(event,'orbit'))
        self.bind('<B1-Motion>',self.drag_camera)
        self.bind('<ButtonRelease-1>',self.end_drag)
        self.bind('<Double-Button-1>',self.double_click)
        self.bind('<ButtonPress-3>',lambda event:self.begin_drag(event,'pan'))
        self.bind('<B3-Motion>',self.drag_camera)
        self.bind('<MouseWheel>',lambda event:self.zoom(.88 if event.delta>0 else 1/.88))
        self.bind('<Button-4>',lambda event:self.zoom(.88));self.bind('<Button-5>',lambda event:self.zoom(1/.88))
    def refresh(self):
        if getattr(self.app,'view_mode',None) and self.app.view_mode.get()=='3D':
            if getattr(self,'_refresh_job',None) is None:self._refresh_job=self.after_idle(self._refresh_frame)
            if not self.app.real and self.cad_doors_auto.get() and getattr(self.cad_asset,'door_parts',()) and self._door_animation_job is None:
                self._door_animation_job=self.after(80,self._tick_door_animation)
    def _tick_door_animation(self):
        self._door_animation_job=None
        if not self.winfo_exists() or self.app.real or not self.cad_doors_auto.get() or self.app.view_mode.get()!='3D':return
        self.app.draw_map();self._door_animation_job=self.after(80,self._tick_door_animation)
    def cad_door_open_fraction(self):
        phase=(time.monotonic()-self._cad_door_epoch)%7.2
        ramp=1.2
        if phase<1.0:return 0.
        if phase<1.0+ramp:
            t=(phase-1.0)/ramp;return t*t*(3-2*t)
        if phase<4.2:return 1.
        if phase<4.2+ramp:
            t=(phase-4.2)/ramp;return 1.-t*t*(3-2*t)
        return 0.
    def cad_door_angle(self,part):
        name=part.get('name','')
        if self.app.real:
            manual=getattr(self,'manual_real_door_display',None)
            if manual and manual[0] is self.cad_asset:
                return manual[1].get(name,0.)
            follower=self._real_door_follower
            side='left' if name.endswith(':1') else 'right' if name.endswith(':2') else None
            initial=math.radians(getattr(self,'door_initial_angles',{}).get(side,0.))
            return follower.angles.get(name,initial) if follower else initial
        if name in self.cad_door_hold_angles:return self.cad_door_hold_angles[name]
        if self.cad_door_hold_angle is not None:
            return self.cad_door_hold_angle if name.endswith(':1') else 0.
        if not self.cad_doors_auto.get():
            side='left' if name.endswith(':1') else 'right' if name.endswith(':2') else None
            return math.radians(getattr(self,'door_initial_angles',{}).get(side,0.))
        return math.radians(100.)*self.cad_door_open_fraction()
    def reset_real_doors(self):
        self._real_door_follower=None;self._real_door_key=None
        self.real_door_status='문 추정 · 닫힘 기준 초기화'
        self.refresh()
    def placement_confirmed(self):
        from .equipment_placement import confirmed
        return confirmed(self.app,self)
    def _update_real_doors(self,world,values,measured):
        if not self.app.real:
            if self._real_door_follower:self._real_door_follower.detach()
            return
        if not self.placement_confirmed():
            if self._real_door_follower:self._real_door_follower.detach()
            self.real_door_status='문 추정 대기 · LiDAR 설비 배치를 먼저 확정하세요'
            return
        key=(id(self.cad_asset),tuple(self.cad_origin),self.scales['cad'],id(self.arm_asset),id(self.tool_asset),tuple(self.mount))
        if self._real_door_key!=key:
            scale=self.scales['cad'];doors=[]
            for part in getattr(self.cad_asset,'door_parts',()):
                vertices=[p for face,material in zip(part['asset'].faces,part['asset'].face_materials) if material==36 for p in face]
                if not vertices:continue
                handle=tuple((min(p[i] for p in vertices)+max(p[i] for p in vertices))/2*scale for i in range(3))
                points=[p for face in part['asset'].faces for p in face]
                lo=[min(p[i] for p in points) for i in range(3)];hi=[max(p[i] for p in points) for i in range(3)]
                push=((lo[0]-.018)*scale,(lo[1]-.018)*scale,(lo[2]+.30*(hi[2]-lo[2]))*scale)
                doors.append(dict(name=part['name'],pivot=tuple(v*scale for v in part['pivot']),handle=handle,push=push,sign=part['sign'],surface_faces=[tuple(tuple(v*scale for v in p) for p in face) for face in part['asset'].faces],hook_open_only=True,hook_inside_exit=part['name'].endswith(':2'),hook_dwell=0.,hook_faces=[tuple(tuple(v*scale for v in p) for p in face) for face,mat in zip(part['asset'].faces,part['asset'].face_materials) if mat in (33,36,37)]))
            self._real_door_follower=SurfaceDoorFollower(doors);self._real_door_key=key
            for name in self._real_door_follower.angles:
                side='left' if name.endswith(':1') else 'right' if name.endswith(':2') else None
                self._real_door_follower.angles[name]=math.radians(getattr(self,'door_initial_angles',{}).get(side,0.))
        follower=self._real_door_follower
        fresh=(self.real_doors_follow.get() and self.arm_follow.get() and getattr(self.app,'connected',False)
               and time.monotonic()-getattr(self.app,'last_state',0)<=3.)
        names=[j['name'] for j in self.arm_asset.movable()] if isinstance(self.arm_asset,RobotDescription) else []
        fresh=fresh and len(names)==6 and all(type(measured.get(n)) in (int,float) and math.isfinite(measured[n]) for n in names)
        tip=None;roller=None
        if fresh and getattr(self.tool_asset,'name','').startswith('EndEffector V3_EndOnly_Change1-org'):
            poses=self.arm_asset.link_transforms(measured,world)
            if 'J6' in poses:
                from .arm_physics import inverse
                tip_world=point(poses['J6'],(0.,-.14714,.148))
                cad_inverse=inverse(transform(self.cad_origin[:3],self.cad_origin[3:]))
                tip=point(cad_inverse,tip_world)
                components=getattr(self.tool_asset,'component_faces',{})
                visuals=getattr(self.tool_asset,'links',{}).get('base_link',())
                for (name,_count),(faces,local,_color) in zip(components.items(),visuals):
                    if 'Polymer_ROECRHA50-10-30-T5' not in name:continue
                    points=[point(local,p) for face in faces for p in face]
                    if points:
                        center=tuple((min(p[i] for p in points)+max(p[i] for p in points))/2 for i in range(3))
                        follower.roller_radius=max(max(p[i] for p in points)-min(p[i] for p in points) for i in range(3))/2
                        roller=point(cad_inverse,point(multiply(poses['J6'],transform((0.,0.,.1))),center))
                    break
        follower.update_contacts(tip,roller,time.monotonic())
        if not self.real_doors_follow.get():self.real_door_status='문 추정 · 추종 꺼짐'
        elif tip is None:self.real_door_status='문 추정 · 실시간 자세/후크 모델 대기'
        else:
            angles=' / '.join(f'{math.degrees(a):.1f}°' for a in follower.angles.values())
            self.real_door_status='문 추정 · '+(('후크 문틀/접촉 추종' if follower.contact_mode=='hook' else '문틀 롤러 밀기 추종') if follower.active else '접촉 대기')+' · '+(angles or '문 모델 없음')+' · 기하 추정/힘 미측정'
    def _refresh_frame(self):
        self._refresh_job=None
        if self.winfo_exists() and self.app.view_mode.get()=='3D':self.app.draw_map()
    def begin_drag(self,event,mode):
        self.focus_set();self.drag=(event.x,event.y,self.camera.yaw,self.camera.pitch,list(self.camera.target),mode)
        self.drag_moved=False
        self.drag_reloc=self.app.reloc_mode if mode=='orbit' and not self.cad_placement_mode else None
        if mode=='orbit' and self.app.reloc_mode and not self.cad_placement_mode:
            self.app.guarded(lambda:self.app.map_click(event))
    def drag_camera(self,event):
        if not self.drag:return
        x,y,yaw,pitch,target,mode=self.drag;dx,dy=event.x-x,event.y-y
        if mode=='orbit' and self.cad_placement_mode:
            self.drag_moved=math.hypot(dx,dy)>=5
            return
        if mode=='orbit' and self.drag_reloc:
            if self.app.reloc_mode=='manual':self.app.guarded(lambda:self.app.map_drag(event))
            return
        if math.hypot(dx,dy)<5 and not self.drag_moved:return
        self.drag_moved=True
        if mode=='orbit' and self.chase:self.chase=False;self.follow.set(False)
        if mode=='orbit':self.camera.yaw=yaw-dx*.45;self.camera.pitch=max(3.,min(89.9,pitch+dy*.35))
        else:
            _,right,up,_=self.camera.basis();scale=self.camera.distance/max(100,self.winfo_height())
            self.camera.target=[target[i]-right[i]*dx*scale+up[i]*dy*scale for i in range(3)]
            self.follow.set(False)
        self.refresh()
    def end_drag(self,event):
        if not self.drag:return
        mode=self.drag[-1]
        if mode=='orbit':
            if self.drag_reloc:
                if self.app.reloc_mode=='manual':self.app.map_release(event)
            elif self.cad_placement_mode:
                if not self.drag_moved:
                    location=self.camera.ground(event.x,event.y,self.winfo_width(),self.winfo_height())
                    if location is not None:
                        self.cad_origin[0],self.cad_origin[1]=location
                        self.cad_placement_mode=False
                        self.refresh()
                        if callable(self.cad_placement_callback):self.cad_placement_callback(False)
                        if hasattr(self.app,'_spatial_save'):self.app._spatial_save()
            elif not self.drag_moved:self.app.guarded(lambda:self.app.map_click(event))
        self.drag=None
    def double_click(self,event):
        self.drag=None
        self.app.guarded(lambda:self.app.map_double_click(event))
    def map_xy(self,x,y):
        projected=self.camera.project((x,y,0),self.winfo_width(),self.winfo_height())
        return projected[:2] if projected else (float('inf'),float('inf'))
    def map_world(self,x,y):
        result=self.camera.ground(x,y,self.winfo_width(),self.winfo_height())
        if result is None:raise ValueError('지도 바닥이 보이는 위치를 선택하세요.')
        return result
    def zoom(self,factor):self.camera.distance=max(.5,min(10000.,self.camera.distance*factor));self.refresh()
    def preset(self,name):
        self.camera.yaw,self.camera.pitch=PRESETS.get(name,PRESETS['사선']);self.refresh()
    def fit(self):self.camera.fit(self.app.map);self.follow.set(False);self.fitted=id(self.app.map);self.refresh()
    def focus_robot(self):
        state=self.app.current_state();self.camera.target=[state.get('x',0),state.get('y',0),.7];self.camera.distance=4. if self.chase else 3.
        self.fitted=id(self.app.map);self.follow.set(True);self.refresh()
    def load_asset(self,role,path,asset_root=None,scale=1.,rotation=(0.,0.,0.)):
        source=Path(path)
        if str(path).lower().endswith('.urdf'):asset=RobotDescription.load(path,asset_root)
        elif role=='arm' and source.name.lower()=='fr5_assembly.glb':asset=load_glb_robot(path)
        elif role=='amr' and source.name.lower().startswith('sdc-400eu'):asset=load_glb_static_robot(path,max_faces=120000,rotation=rotation,color='#afbac4')
        elif role=='tool' and source.suffix.lower()=='.glb':asset=load_glb_static_robot(path,max_faces=120000,rotation=rotation,color='#8fa1b2')
        elif role=='cad' and source.suffix.lower()=='.glb':asset=load_mesh(path,max_faces=300000,rotation=rotation)
        else:asset=load_mesh(path,rotation=rotation)
        if role=='arm':self.arm_asset=asset;self.last_arm_operation=None
        elif role=='amr':asset.name='SEER '+asset.name;self.body_asset=asset
        elif role=='tool':self.tool_asset=asset
        else:self.cad_asset=asset
        self.scales[role]=scale;self.positions[role]={};self.refresh()
        return asset
    def _arm_positions(self,arm,measured):
        asset=self.arm_asset
        if not isinstance(asset,RobotDescription):return {}
        values=self.positions['arm'];joints=asset.movable()
        dev=getattr(self.app,'arm_dev_sim',None)
        if not self.app.real and dev is not None:
            values.clear();sim_values=dev.kin.positions(dev.q)
            if getattr(asset,'name','')=='FAIRINO FR5 extctrl assembly':
                for index,joint in enumerate(joints):
                    if index<len(dev.q):values[joint['name']]=dev.q[index]
            else:values.update(sim_values)
            self.arm_source='워크스페이스 SIM 동기화';return values
        if self.arm_follow.get() and isinstance(measured,dict) and measured:
            for joint in joints:
                value=measured.get(joint['name'])
                if type(value) in (int,float) and math.isfinite(value):values[joint['name']]=value
            self.arm_source=arm.get('feedback_source','수신 관절값')
        elif self.arm_follow.get() and asset.synthetic and not self.app.real:
            if (asset.name.startswith('FAIRINO FR5') or asset.name=='fairino5_v6_robot'):
                safe=[0.,-1.57,1.57,-1.57,-1.57,0.];work=[.7,-1.,.9,-1.4,-1.2,.4]
            else:safe=[0.,-.35,.7,0.,.5,0.];work=[.9,-1.15,1.3,.8,.8,.4]
            operation=arm.get('operation','');key=(operation,arm.get('status'))
            target=safe if operation=='safe_pose' or arm.get('pose')=='SAFE' and arm.get('status')!='RUNNING' else work
            if arm.get('status')=='RUNNING':
                if self.last_arm_operation!=key:self.arm_start=dict(values);self.last_arm_operation=key
                progress=max(0.,min(1.,float(arm.get('progress',0))))
                progress=progress*progress*(3-2*progress)
                for i,joint in enumerate(joints):
                    start=self.arm_start.get(joint['name'],safe[i%6]);values[joint['name']]=start+(target[i%6]-start)*progress
            else:
                values.update({joint['name']:target[i%6] for i,joint in enumerate(joints)});self.last_arm_operation=key
            self.arm_source='SIM 합성 관절'
        else:self.arm_source='수동 관절 / 수신 대기'
        return values
    def draw_scene(self,model,state,arm,route_points=(),measured=None,lidar=()):
        if self.fitted!=id(model):self.camera.fit(model);self.fitted=id(model)
        self.delete('all');width,height=max(1,self.winfo_width()),max(1,self.winfo_height())
        if width<10 or height<10:return
        pose={key:state.get(key,0.) for key in ('x','y','theta')}
        if not all(type(v) in (int,float) and math.isfinite(v) for v in pose.values()):pose=dict(x=0.,y=0.,theta=0.)
        raw_pose=dict(pose)
        from .motion_display import MotionDisplay
        if not hasattr(self,'_pose_display'):
            self._pose_display=MotionDisplay();self._joint_display=MotionDisplay()
        display_now=time.monotonic()
        display_context=(id(model),self.app.real,getattr(self.app,'connected',False))
        pose=self._pose_display.update(pose,display_now,display_context,angular=('theta',))
        self.scene_pose=pose
        if self.follow.get():
            self.camera.target=[pose['x'],pose['y'],.7]
            if self.chase:self.camera.yaw=math.degrees(pose['theta'])+180.;self.camera.pitch=28.
        faces=[];lines=[];texts=[];gpu_assets=[];gpu_meshes=[];renderer=renderer_for(self)
        def add_faces(geometry,world,color,tag):faces.extend((tuple(point(world,p) for p in face),color,tag) for face in geometry)
        def add_asset(asset,role,world,color,tag):
            if isinstance(asset,RobotDescription):
                if renderer:
                    scaled=[list(row) for row in world]
                    for i in range(3):
                        for j in range(3):scaled[i][j]*=self.scales[role]
                    gpu_assets.append((asset,dict(self.positions[role]),scaled,None,1));return
                for vertices,shade,_ in asset.draw_faces(self.positions[role]):faces.append((tuple(point(world,tuple(v*self.scales[role] for v in p)) for p in vertices),shade,tag))
            elif asset:
                scale=self.scales[role]
                if renderer and isinstance(asset,MeshAsset):
                    scaled=tuple(tuple(world[i][j]*scale if i<3 and j<3 else world[i][j] for j in range(4)) for i in range(4))
                    gpu_meshes.append((asset,scaled,color))
                    parts=getattr(asset,'door_parts',())
                    for part in parts:
                        angle=self.cad_door_angle(part)
                        pivot=part['pivot'];delta=multiply(multiply(transform(pivot),axis_rotation((0,0,1),angle*part['sign'])),transform(tuple(-v for v in pivot)))
                        gpu_meshes.append((part['asset'],multiply(scaled,delta),color))
                else:
                    selected_faces=getattr(asset,'lod_faces',asset.faces)
                    selected_materials=getattr(asset,'lod_face_materials',asset.face_materials)
                    faces.extend((vertices,shade,tag) for vertices,shade,_ in asset.draw_faces(world,(scale,)*3,color,selected_faces,selected_materials))
                    glass=getattr(asset,'lod_transparent_faces',getattr(asset,'transparent_faces',()))
                    for face in glass:
                        vertices=tuple(point(world,tuple(p[i]*scale for i in range(3))) for p in face)
                        faces.append((vertices,'#8bd9eb',tag+'_glass',.30))
                    parts=getattr(asset,'lod_door_parts',getattr(asset,'door_parts',()))
                    for part in parts:
                        angle=self.cad_door_angle(part)
                        pivot=part['pivot'];delta=multiply(multiply(transform(pivot),axis_rotation((0,0,1),angle*part['sign'])),transform(tuple(-v for v in pivot)))
                        part_world=multiply(world,delta);door=part['asset']
                        faces.extend((vertices,shade,tag+'_door') for vertices,shade,_ in door.draw_faces(part_world,(scale,)*3,color))
                        for face in getattr(door,'transparent_faces',()):
                            vertices=tuple(point(part_world,tuple(p[i]*scale for i in range(3))) for p in face)
                            faces.append((vertices,'#8bd9eb',tag+'_door_glass',.30))
        if self.layers['지도'].get():
            pts=[(n['x'],n['y']) for n in model.nodes.values()]+[(wall[i],wall[i+1]) for wall in model.walls for i in (0,2)]
            xs,ys=zip(*(pts or [(0,0),(2,2)]));x0,x1=min(xs)-1,max(xs)+1;y0,y1=min(ys)-1,max(ys)+1
            floor=[self.camera.project(p,width,height) for p in ((x0,y0,0),(x1,y0,0),(x1,y1,0),(x0,y1,0))]
            if all(floor):self.create_polygon(*[v for p in floor for v in p[:2]],fill='#edf2f7',outline='#c2cfdd',tags='world_floor')
            step=max(1.,10**math.floor(math.log10(max(1.,x1-x0,y1-y0)/12)))
            for i in (range(math.floor(x0/step),math.ceil(x1/step)+1) if self.app.layers['그리드'].get() else []):lines.append(((i*step,y0,.002),(i*step,y1,.002),'#d0dbe6',1,'world_grid'))
            for i in (range(math.floor(y0/step),math.ceil(y1/step)+1) if self.app.layers['그리드'].get() else []):lines.append(((x0,i*step,.002),(x1,i*step,.002),'#d0dbe6',1,'world_grid'))
            for x1w,y1w,x2w,y2w in (model.walls if self.app.layers['벽'].get() else []):
                length=math.hypot(x2w-x1w,y2w-y1w)
                add_faces(box((length,.055,self.wall_height)),transform(((x1w+x2w)/2,(y1w+y2w)/2,self.wall_height/2),(0,0,math.atan2(y2w-y1w,x2w-x1w))),'#8d9caa','world_map')
            for key,n in (model.nodes.items() if self.app.layers['노드'].get() else []):
                add_faces(cylinder(.065,.025,8),transform((n['x'],n['y'],.015)),'#42a39a' if n.get('kind')=='dock' else '#6089c6','world_node')
                texts.append(((n['x'],n['y'],.12),key,'#314d6d'))
                if key==self.app.selected:
                    circle=[(n['x']+.16*math.cos(i*math.pi/12),n['y']+.16*math.sin(i*math.pi/12),.04) for i in range(25)]
                    lines.extend((a,b,'#136de2',3,'world_selected_node') for a,b in zip(circle,circle[1:]))
            for obstacle in (getattr(model,'obstacles',[]) if self.app.layers['장애물'].get() else []):
                if not obstacle.get('dynamic'):
                    add_faces(cylinder(obstacle['radius'],.3),transform((obstacle['x'],obstacle['y'],.15)),'#d96573','world_obstacle')
                    continue
                base=transform((obstacle['x'],obstacle['y'],0),(0,0,obstacle.get('_heading',0)))
                color='#df983d' if obstacle['kind']=='person' else '#8270c1'
                if obstacle['kind']=='person':
                    add_faces(box((.30,.22,.50)),multiply(base,transform((0,0,1.05))),color,'world_actor')
                    add_faces(cylinder(.11,.22,8),multiply(base,transform((0,0,1.45))),'#f5d6af','world_actor')
                    for side in (-1,1):
                        add_faces(box((.10,.10,.75)),multiply(base,transform((0,side*.09,.425))),'#4c617a','world_actor')
                        add_faces(box((.08,.08,.48)),multiply(base,transform((0,side*.22,1.0))),color,'world_actor')
                    label_height=1.7
                else:
                    scale=obstacle['radius']/.61
                    add_faces(box((.9582*scale,.6314*scale,.182)),multiply(base,transform((0,0,.17))),color,'world_actor')
                    add_faces(cylinder(.05,.04,8),multiply(base,transform((.34*scale,0,.28))),'#5eaadb','world_actor')
                    label_height=.4
                texts.append(((obstacle['x'],obstacle['y'],label_height),obstacle.get('id','')+' · '+obstacle.get('_motion','준비'),color))
                from .dynamic_obstacles import compile_actor_path
                try:actor_points,_=compile_actor_path(self.app.map,obstacle)
                except (ValueError,KeyError):actor_points=[]
                for a,b in zip(actor_points,actor_points[1:]):lines.append(((*a,.02),(*b,.02),'#a396bd',1,'world_actor_route'))
            for a,b,c,d in (getattr(model,'virtual_walls',[]) if self.app.layers['벽'].get() else []):lines.append(((a,b,.08),(c,d,.08),'#d1446a',3,'world_virtual_wall'))
            if self.app.layers['영역'].get():
                for area in getattr(model,'area_records',[]):
                    vertices=tuple((p[0],p[1],.012) for p in area.get('points',[]))
                    if len(vertices)>=3:faces.append((vertices,'#dfb1ba' if (area.get('properties') or {}).get('forbidden') else '#c2dbce','world_area'))
        if not self.app.real and self.app.layers['장애물'].get() and self.app.show_tracks.get():
            from .obstacle_tracking import CLASSES
            for record in self.app.sim.obstacle_tracker.records():
                color='#84919c' if record['state']=='미관측' else {'dynamic':'#2196c7','static':'#d14b54','unknown':'#e9a237'}[record['kind']]
                x,y=record['x'],record['y'];r=record['radius']+.1
                corners=[(x-r,y-r,.10),(x+r,y-r,.10),(x+r,y+r,.10),(x-r,y+r,.10),(x-r,y-r,.10)]
                lines.extend((a,b,color,2,'world_tracked_obstacle') for a,b in zip(corners,corners[1:]))
                trail=list(record['trail']);lines.extend(((*a,.06),(*b,.06),color,2,'world_tracked_trail') for a,b in zip(trail,trail[1:]))
                texts.append(((x,y,.8),record['id']+' · '+CLASSES[record['kind']]+' · '+record['state'],color))
        if self.layers['점군'].get():
            cloud=getattr(model,'cloud',[])
            # Map points belong to the scene depth buffer, not the Tk label overlay.
            for x,y in cloud[::max(1,len(cloud)//1200)]:
                lines.append(((x-.008,y,.008),(x+.008,y,.008),'#687a8b',2,'world_cloud'))
        if self.layers['경로'].get() and self.app.layers['경로'].get():
            for goal in (self.app.sim.skipped_goals if not self.app.real else []):
                if goal not in model.nodes:continue
                n=model.nodes[goal];x,y=n['x'],n['y']
                lines.append(((x-.15,y-.15,.08),(x+.15,y+.15,.08),'#c93043',4,'world_skipped_goal'))
                lines.append(((x-.15,y+.15,.08),(x+.15,y-.15,.08),'#c93043',4,'world_skipped_goal'))
                texts.append(((x,y,.3),goal+' · 도달 불가 / 패스','#c93043'))
            records=getattr(model,'path_records',[]);represented=set()
            for record in records:
                pair=frozenset((record.get('a'),record.get('b')))
                if pair in represented:continue
                represented.add(pair);geometry=path_record_geometry(record.get('raw',{}))
                sampled=geometry[::max(1,len(geometry)//30)]
                if geometry and sampled[-1]!=geometry[-1]:sampled.append(geometry[-1])
                lines.extend(((a[0],a[1],.035),(b[0],b[1],.035),'#6e8ba9',1,'world_path') for a,b in zip(sampled,sampled[1:]))
            for a,b in model.edges:
                if frozenset((a,b)) not in represented:lines.append(((model.nodes[a]['x'],model.nodes[a]['y'],.035),(model.nodes[b]['x'],model.nodes[b]['y'],.035),'#6e8ba9',1,'world_path'))
            lines.extend(((a[0],a[1],.07),(b[0],b[1],.07),'#10966e',3,'world_active_route') for a,b in zip(route_points,route_points[1:]))
        overrides=getattr(self.app,'lidar_mount_overrides',[])
        if self.body_asset and self.body_asset.name.startswith('SEER SBA-400EU'):
            for index,values in enumerate(overrides[:2]):
                if len(values)==3 and all(math.isfinite(float(v)) for v in values):
                    joint=next((j for j in self.body_asset.joints if j['name']==f'lidar_{index+1}_mount'),None)
                    if joint:joint['origin']=transform((float(values[0]),float(values[1]),.1965),(0,0,float(values[2])))
        world=transform((pose['x'],pose['y'],0.),(0.,0.,pose['theta']))
        texts.append((point(world,(0,0,.20)),'SEER' if self.body_asset and self.body_asset.name.startswith('SEER') else 'AMR','#275176'))
        candidate=self.app.reloc_candidate
        if candidate:
            x,y,angle=candidate
            for a,b in (((x-.2,y,.09),(x+.2,y,.09)),((x,y-.2,.09),(x,y+.2,.09))):lines.append((a,b,'#eb782c',3,'world_reloc'))
            if angle is not None:lines.append(((x,y,.09),(x+.6*math.cos(angle),y+.6*math.sin(angle),.09),'#eb782c',4,'world_reloc'))
        if self.layers['AMR'].get():
            for vehicle in positioned(get_fleet(self.app)):
                texts.append(((vehicle['x'],vehicle['y'],.75),vehicle['id'],color(vehicle)))
                if not vehicle.get('selected'):
                    add_faces(box((1.,1.,.4)),transform((vehicle['x'],vehicle['y'],.25)),color(vehicle),'world_acs_fleet')
            if self.body_asset:add_asset(self.body_asset,'amr',world,'#5286bd','world_amr')
            else:
                cfg=getattr(model,'robot_model',{});length=cfg.get('length',.7);breadth=cfg.get('width',.5)
                add_faces(box((length,breadth,.30)),multiply(world,transform((0,0,.25))),'#4382c8','world_amr')
                add_faces(box((length*.65,breadth*.8,.04)),multiply(world,transform((0,0,.42))),'#cedbe7','world_amr')
                for x in (-length*.32,length*.32):
                    for y in (-breadth*.5,breadth*.5):
                        add_faces(cylinder(.12,.065),multiply(world,transform((x,y,.12),(math.pi/2,0,0))),'#344354','world_amr')
                lines.append((point(world,(0,0,.46)),point(world,(length*.65,0,.46)),'#e76158',3,'world_heading'))
        values=self._arm_positions(arm,measured or {})
        values=self._joint_display.update(values,display_now,display_context,jump=math.pi)
        self.scene_joint_values=dict(values)
        arm_world=multiply(world,transform(self.mount[:3],self.mount[3:]))
        self._update_real_doors(multiply(transform((raw_pose['x'],raw_pose['y'],0),(0,0,raw_pose['theta'])),transform(self.mount[:3],self.mount[3:])),values,measured or {})
        if self.layers[chr(0xB85C)+chr(0xBD07)+chr(0xD314)].get():add_asset(self.arm_asset,'arm',arm_world,'#e9ac42','world_arm')
        if self.layers[chr(0xB85C)+chr(0xBD07)+chr(0xD314)].get() and self.tool_asset and isinstance(self.arm_asset,RobotDescription):
            poses=self.arm_asset.link_transforms(values,arm_world)
            if 'J6' in poses:add_asset(self.tool_asset,'tool',multiply(poses['J6'],transform((0.,0.,.1))),'#4c657a','world_tool')
        overlay=getattr(self.app,'sim_pose_overlay',None)
        if self.app.real and overlay and overlay.get():
            sim=self.app.sim.state
            simworld=transform((sim.x,sim.y,0),(0,0,sim.theta))
            if renderer and isinstance(self.body_asset,RobotDescription) and self.layers['AMR'].get():gpu_assets.append((self.body_asset,{},simworld,'#319fea',.42))
            dev=getattr(self.app,'arm_dev_sim',None)
            if renderer and dev and self.layers['로봇팔'].get():gpu_assets.append((dev.kin.asset,dev.kin.positions(dev.q),multiply(simworld,transform(self.mount[:3],self.mount[3:])),'#319fea',.42))
            texts.append((point(simworld,(0,0,.6)),'SIM 개발 · 파란색','#1687d0'))
        dev=getattr(self.app,'arm_dev_sim',None)
        if not self.app.real and dev and self.layers[chr(0xB85C)+chr(0xBD07)+chr(0xD314)].get() and not self.tool_asset:
            toolworld=multiply(world,transform(self.mount[:3],self.mount[3:]))
            for name,matrix,size in dev.physics.tool_boxes(dev.q):
                color='#35b8cf' if name in ('finger','cup') else '#354153'
                add_faces(box(size),multiply(toolworld,matrix),color,'world_tool')
        if self.layers['CAD'].get() and self.cad_asset:add_asset(self.cad_asset,'cad',transform(self.cad_origin[:3],self.cad_origin[3:]),'#c4cbd0','world_cad')
        panel=getattr(self,'panel_pick',None)
        if panel is None and self.cad_asset and self.cad_asset.name.upper().startswith('SIMPLE_WHOLE'):
            cad_scale=float(self.scales.get('cad',1.))
            center=tuple(value*cad_scale for value in SIM_PANEL_CENTER)
            size=tuple(value*cad_scale for value in SIM_PANEL_SIZE)
            panel=dict(name='phone panel · SIM default',matrix=multiply(transform(self.cad_origin[:3],self.cad_origin[3:]),transform(center)),size=size)
        if panel:
            panel_matrix=panel['matrix']
            if dev and dev.physics.held and dev.physics.held.get('name')==panel.get('object_name'):
                panel_matrix=multiply(arm_world,dev.physics.held['matrix'])
            panel_width,panel_depth,thickness=panel['size']
            add_faces(box(panel['size']),panel_matrix,'#aab8c5','world_sim_panel_frame')
            screen=transform((0.,0.,thickness/2+.001),(0.,0.,0.))
            add_faces(box((panel_width*.91,panel_depth*.86,.0015)),multiply(panel_matrix,screen),'#172632','world_sim_panel_screen')
            flex=transform((0.,-panel_depth*.47,0.),(0.,0.,0.))
            add_faces(box((panel_width*.28,panel_depth*.12,.002)),multiply(panel_matrix,flex),'#d8a747','world_sim_panel_flex')
            texts.append((point(panel_matrix,(0,0,thickness/2+.025)),'PHONE DISPLAY PANEL · SIM','#126b91'))
        if self.layers['좌표축'].get():
            for matrix in (identity(),world,multiply(world,transform(self.mount[:3],self.mount[3:]))):
                origin=point(matrix,(0,0,.01))
                for p,color,label in [((.5,0,.01),'#d44b4b','X'),((0,.5,.01),'#329467','Y'),((0,0,.51),'#397fca','Z')]:
                    end=point(matrix,p);lines.append((origin,end,color,2,'world_axes'));texts.append((end,label,color))
        for x,y in list(lidar)[::max(1,len(lidar)//1600)]:
            lines.append(((x-.008,y,.1965),(x+.008,y,.1965),'#12ac64',3,'world_lidar'))
        self.total_faces=len(faces)+sum(sum(len(v[0]) for rows in a.links.values() for v in rows) for a,*_ in gpu_assets)+sum(len(a.faces)+len(getattr(a,'transparent_faces',())) for a,_,_ in gpu_meshes)
        if len(faces)>6000:faces=faces[::math.ceil(len(faces)/6000)]
        items=[]
        for face_item in ([] if renderer else faces):
            vertices,color,tag=face_item[:3];alpha=face_item[3] if len(face_item)>3 else 1.
            projected=[self.camera.project(p,width,height) for p in vertices]
            if not all(projected):continue
            if max(p[0] for p in projected)<-100 or min(p[0] for p in projected)>width+100 or max(p[1] for p in projected)<-100 or min(p[1] for p in projected)>height+100:continue
            normal=normalize(cross(subtract(vertices[1],vertices[0]),subtract(vertices[2],vertices[0])))
            lighting=.68+.32*abs(dot(normal,normalize((.3,-.5,1))))
            rgb=[int(color[i:i+2],16) for i in (1,3,5)];shade='#'+''.join(f'{min(255,round(v*lighting)):02x}' for v in rgb)
            items.append((sum(p[2] for p in projected)/len(projected),'face',projected,(shade,alpha),tag))
        for a,b,color,line_width,tag in ([] if renderer else lines):
            projected=[self.camera.project(p,width,height) for p in (a,b)]
            if all(projected):items.append((sum(p[2] for p in projected)/2,'line',projected,(color,line_width),tag))
        for _,kind,projected,color,tag in sorted(items,key=lambda item:item[0],reverse=True):
            coords=[v for p in projected for v in p[:2]]
            if kind=='face':self.create_polygon(*coords,fill=color[0],outline=color[0],stipple='gray25' if color[1]<1 else '',tags=('world_mesh',tag))
            else:self.create_line(*coords,fill=color[0],width=color[1],tags=tag)
        if renderer:
            from PIL import ImageTk
            frame=renderer.render(self.camera,width,height,assets=gpu_assets,meshes=gpu_meshes,faces=faces,lines=[row[:4] for row in lines])
            current=getattr(self,'render_image',None)
            if current and (current.width(),current.height())==frame.size:current.paste(frame)
            else:self.render_image=ImageTk.PhotoImage(frame)
            self.create_image(0,0,anchor='nw',image=self.render_image,tags=('world_mesh','world_arm','world_amr'))
        for p,label,color in texts:
            projected=self.camera.project(p,width,height)
            if projected and 0<projected[0]<width and 0<projected[1]<height:self.create_text(*projected[:2],text=label,fill=color,font=(self.app.font,8,'bold'),tags='world_label')
        interaction='재배치: 클릭 위치 · 드래그 방향' if self.app.reloc_mode else '클릭 노드 선택 · 더블클릭 이동 · 좌드래그 회전'
        text=f"3D WORLD · {'REAL' if self.app.real else 'SIM'} · {self.arm_source}\n{interaction} · 우드래그 이동 · 휠 확대"
        if self.app.real:
            manual=getattr(self,'manual_real_door_display',None)
            text+='\n'+('문 표시 · 수동 설정 (실측 아님)' if manual and manual[0] is self.cad_asset else self.real_door_status)
        item=self.create_text(12,12,anchor='nw',text=text,fill='#28405d',font=(self.app.font,9),tags='world_status')
        rect=self.bbox(item)
        if rect:
            bg=self.create_rectangle(rect[0]-5,rect[1]-4,rect[2]+5,rect[3]+4,fill='#f5f8fc',outline='');self.tag_lower(bg,item)
        self.render_count+=1
