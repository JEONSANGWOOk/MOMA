"""Orbitable 3D world viewport using Tk's software polygon renderer."""
import math
import tkinter as tk
from .geometry3d import RobotDescription, MeshAsset, load_mesh, box, cylinder, transform, multiply, point, identity
from .smap import path_record_geometry


PRESETS={'사선':(-135,38),'위':(-90,89.9),'정면':(-90,8),'측면':(0,8),'뒤':(90,8)}


def dot(a,b):return sum(x*y for x,y in zip(a,b))
def cross(a,b):return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def subtract(a,b):return tuple(x-y for x,y in zip(a,b))
def normalize(v):
    length=max(1e-12,math.sqrt(dot(v,v)))
    return tuple(value/length for value in v)


class Camera:
    def __init__(self):self.target=[0.,0.,.4];self.distance=15.;self.yaw=-135.;self.pitch=38.;self.perspective=True
    def basis(self):
        yaw,pitch=math.radians(self.yaw),math.radians(self.pitch)
        eye=tuple(self.target[i]+self.distance*v for i,v in enumerate((math.cos(pitch)*math.cos(yaw),math.cos(pitch)*math.sin(yaw),math.sin(pitch))))
        forward=normalize(subtract(self.target,eye));right=normalize(cross(forward,(0,0,1)));up=cross(right,forward)
        return eye,right,up,forward
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
        self.body_asset=RobotDescription.load(demo_path.parent/'seer_amb_csw04_ce.urdf');self.cad_asset=None
        self.scales={'arm':1.,'amr':1.,'cad':1.}
        self.positions={'arm':{},'amr':{},'cad':{}}
        self.mount=[0.,0.,.308,0.,0.,0.];self.cad_origin=[0.,0.,0.,0.,0.,0.]
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
        if getattr(self.app,'view_mode',None) and self.app.view_mode.get()=='3D':self.app.draw_map()
    def begin_drag(self,event,mode):
        self.focus_set();self.drag=(event.x,event.y,self.camera.yaw,self.camera.pitch,list(self.camera.target),mode)
        self.drag_moved=False
        self.drag_reloc=self.app.reloc_mode if mode=='orbit' else None
        if mode=='orbit' and self.app.reloc_mode:
            self.app.guarded(lambda:self.app.map_click(event))
    def drag_camera(self,event):
        if not self.drag:return
        x,y,yaw,pitch,target,mode=self.drag;dx,dy=event.x-x,event.y-y
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
    def load_asset(self,role,path,asset_root=None,scale=1.):
        asset=RobotDescription.load(path,asset_root) if str(path).lower().endswith('.urdf') else load_mesh(path)
        if role=='arm':self.arm_asset=asset;self.last_arm_operation=None
        elif role=='amr':self.body_asset=asset
        else:self.cad_asset=asset
        self.scales[role]=scale;self.positions[role]={};self.refresh()
        return asset
    def _arm_positions(self,arm,measured):
        asset=self.arm_asset
        if not isinstance(asset,RobotDescription):return {}
        values=self.positions['arm'];joints=asset.movable()
        if self.arm_follow.get() and isinstance(measured,dict) and measured:
            for joint in joints:
                value=measured.get(joint['name'])
                if type(value) in (int,float) and math.isfinite(value):values[joint['name']]=value
            self.arm_source='수신 관절값'
        elif self.arm_follow.get() and asset.synthetic and not self.app.real:
            if asset.name.startswith('FAIRINO FR5'):
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
        self.scene_pose=pose
        if self.follow.get():
            self.camera.target=[pose['x'],pose['y'],.7]
            if self.chase:self.camera.yaw=math.degrees(pose['theta'])+180.;self.camera.pitch=28.
        faces=[];lines=[];texts=[]
        def add_faces(geometry,world,color,tag):faces.extend((tuple(point(world,p) for p in face),color,tag) for face in geometry)
        def add_asset(asset,role,world,color,tag):
            if isinstance(asset,RobotDescription):
                for vertices,shade,_ in asset.draw_faces(self.positions[role]):faces.append((tuple(point(world,tuple(v*self.scales[role] for v in p)) for p in vertices),shade,tag))
            elif asset:faces.extend((vertices,shade,tag) for vertices,shade,_ in asset.draw_faces(world,(self.scales[role],)*3,color))
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
                a,b=obstacle['motion_path'];lines.append(((*a,.02),(*b,.02),'#a396bd',1,'world_actor_route'))
            for a,b,c,d in (getattr(model,'virtual_walls',[]) if self.app.layers['벽'].get() else []):lines.append(((a,b,.08),(c,d,.08),'#d1446a',3,'world_virtual_wall'))
            if self.app.layers['영역'].get():
                for area in getattr(model,'area_records',[]):
                    vertices=tuple((p[0],p[1],.012) for p in area.get('points',[]))
                    if len(vertices)>=3:faces.append((vertices,'#dfb1ba' if (area.get('properties') or {}).get('forbidden') else '#c2dbce','world_area'))
        if self.layers['점군'].get():
            cloud=getattr(model,'cloud',[])
            for x,y in cloud[::max(1,len(cloud)//1200)]:texts.append(((x,y,.008),'·','#687a8b'))
        if self.layers['경로'].get() and self.app.layers['경로'].get():
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
        world=transform((pose['x'],pose['y'],0.),(0.,0.,pose['theta']))
        texts.append((point(world,(0,0,.20)),'SEER' if self.body_asset and self.body_asset.name.startswith('SEER') else 'AMR','#275176'))
        candidate=self.app.reloc_candidate
        if candidate:
            x,y,angle=candidate
            for a,b in (((x-.2,y,.09),(x+.2,y,.09)),((x,y-.2,.09),(x,y+.2,.09))):lines.append((a,b,'#eb782c',3,'world_reloc'))
            if angle is not None:lines.append(((x,y,.09),(x+.6*math.cos(angle),y+.6*math.sin(angle),.09),'#eb782c',4,'world_reloc'))
        if self.layers['AMR'].get():
            if self.body_asset:add_asset(self.body_asset,'amr',world,'#5286bd','world_amr')
            else:
                cfg=getattr(model,'robot_model',{});length=cfg.get('length',.7);breadth=cfg.get('width',.5)
                add_faces(box((length,breadth,.30)),multiply(world,transform((0,0,.25))),'#4382c8','world_amr')
                add_faces(box((length*.65,breadth*.8,.04)),multiply(world,transform((0,0,.42))),'#cedbe7','world_amr')
                for x in (-length*.32,length*.32):
                    for y in (-breadth*.5,breadth*.5):
                        add_faces(cylinder(.12,.065),multiply(world,transform((x,y,.12),(math.pi/2,0,0))),'#344354','world_amr')
                lines.append((point(world,(0,0,.46)),point(world,(length*.65,0,.46)),'#e76158',3,'world_heading'))
        values=self._arm_positions(arm,measured or {});self.scene_joint_values=dict(values)
        if self.layers['로봇팔'].get():add_asset(self.arm_asset,'arm',multiply(world,transform(self.mount[:3],self.mount[3:])),'#e9ac42','world_arm')
        if self.layers['CAD'].get() and self.cad_asset:add_asset(self.cad_asset,'cad',transform(self.cad_origin[:3],self.cad_origin[3:]),'#a5b4c4','world_cad')
        if self.layers['좌표축'].get():
            for matrix in (identity(),world,multiply(world,transform(self.mount[:3],self.mount[3:]))):
                origin=point(matrix,(0,0,.01))
                for p,color,label in [((.5,0,.01),'#d44b4b','X'),((0,.5,.01),'#329467','Y'),((0,0,.51),'#397fca','Z')]:
                    end=point(matrix,p);lines.append((origin,end,color,2,'world_axes'));texts.append((end,label,color))
        self.total_faces=len(faces)
        if len(faces)>6000:faces=faces[::math.ceil(len(faces)/6000)]
        items=[]
        for vertices,color,tag in faces:
            projected=[self.camera.project(p,width,height) for p in vertices]
            if not all(projected):continue
            if max(p[0] for p in projected)<-100 or min(p[0] for p in projected)>width+100 or max(p[1] for p in projected)<-100 or min(p[1] for p in projected)>height+100:continue
            normal=normalize(cross(subtract(vertices[1],vertices[0]),subtract(vertices[2],vertices[0])))
            lighting=.68+.32*abs(dot(normal,normalize((.3,-.5,1))))
            rgb=[int(color[i:i+2],16) for i in (1,3,5)];shade='#'+''.join(f'{min(255,round(v*lighting)):02x}' for v in rgb)
            items.append((sum(p[2] for p in projected)/len(projected),'face',projected,shade,tag))
        for a,b,color,line_width,tag in lines:
            projected=[self.camera.project(p,width,height) for p in (a,b)]
            if all(projected):items.append((sum(p[2] for p in projected)/2,'line',projected,(color,line_width),tag))
        for _,kind,projected,color,tag in sorted(items,key=lambda item:item[0],reverse=True):
            coords=[v for p in projected for v in p[:2]]
            if kind=='face':self.create_polygon(*coords,fill=color,outline=color,tags=('world_mesh',tag))
            else:self.create_line(*coords,fill=color[0],width=color[1],tags=tag)
        for p,label,color in texts:
            projected=self.camera.project(p,width,height)
            if projected and 0<projected[0]<width and 0<projected[1]<height:self.create_text(*projected[:2],text=label,fill=color,font=(self.app.font,8,'bold'),tags='world_label')
        for x,y in list(lidar)[::max(1,len(lidar)//1600)]:
            p=self.camera.project((x,y,.1965),width,height)
            if p and 0<p[0]<width and 0<p[1]<height:self.create_oval(p[0]-1.6,p[1]-1.6,p[0]+1.6,p[1]+1.6,fill='#12ac64',outline='',tags='world_lidar')
        interaction='재배치: 클릭 위치 · 드래그 방향' if self.app.reloc_mode else '클릭 노드 선택 · 더블클릭 이동 · 좌드래그 회전'
        text=f"3D WORLD · {'REAL' if self.app.real else 'SIM'} · {self.arm_source}\n{interaction} · 우드래그 이동 · 휠 확대"
        item=self.create_text(12,12,anchor='nw',text=text,fill='#28405d',font=(self.app.font,9),tags='world_status')
        rect=self.bbox(item)
        if rect:
            bg=self.create_rectangle(rect[0]-5,rect[1]-4,rect[2]+5,rect[3]+4,fill='#f5f8fc',outline='');self.tag_lower(bg,item)
        self.render_count+=1
