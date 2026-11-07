"""Run INSIDE Isaac Sim Script Editor. Read-only; requires no ROS installation."""
import json
import gzip
import os
import math
import queue
import threading
import time
import urllib.request
import omni.kit.app
import omni.usd
import omni.timeline
from pxr import Gf,UsdGeom,UsdShade,Sdf


class IsaacMirror:
    ROOT='/World/MOMA_Mirror'
    def __init__(self,url):
        self.url=url.rstrip('/');self.stop_event=threading.Event();self.pending=queue.Queue(maxsize=1)
        self.stage=None;self.session=None;self.transforms={};self.targets={};self.last_sequence=None
        self.last_frame_at=0.;self.last_tick=time.monotonic();self.last_warning=0.;self.todo=[];self.geometry={}
        self.map_revision=None;self.framed=False;self.lidar=[]
        self.ready=threading.Event();self.heartbeat_at=0.
        self.physics_enabled=os.environ.get('MOMA_DOOR_PHYSICS')=='1';self.physics=None;self.physics_fault=False;self.instances={}
        # Default to the desktop's contact-estimated door pose. Dynamic doors
        # are opt-in because they cannot guarantee exact live-mirror motion.
        from pathlib import Path
        config_path=os.environ.get('MOMA_DOOR_PHYSICS_CONFIG','')
        config=json.loads(Path(config_path).read_text(encoding='utf-8')) if config_path and Path(config_path).is_file() else {}
        self.door_mode=config.get('door_motion_mode','mirror')
        if self.door_mode=='mirror':
            self.physics_enabled=False
            print('[MOMA mirror] Doors follow desktop hook/roller estimates directly; no simulated door forces.')
        self.thread=threading.Thread(target=self.receive,daemon=True,name='moma-mirror-reader');self.thread.start()
        self.subscription=omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self.update,name='MOMA read-only mirror')
    def close(self):
        self.stop_event.set();self.subscription=None
    def fetch(self,opener,path,timeout=4):
        with opener.open(self.url+path,timeout=timeout) as response:
            body=response.read()
            if response.headers.get('Content-Encoding')=='gzip':body=gzip.decompress(body)
            return json.loads(body)
    def receive(self):
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}));manifest=None
        while not self.stop_event.is_set():
            try:
                if self.ready.is_set() and time.monotonic()-self.heartbeat_at>1.:
                    self.fetch(opener,'/client-ready');self.heartbeat_at=time.monotonic()
                frame=self.fetch(opener,'/frame')
                if frame.get('version')!=1:raise ValueError('Unsupported frame version')
                if manifest is None or manifest['session']!=frame['session'] or manifest['revision']<frame['revision']:
                    manifest=self.fetch(opener,'/manifest',30)
                if manifest['session']!=frame['session'] or manifest['revision']<frame['revision']:
                    raise ValueError('Scene update in progress')
                packet=(manifest,frame)
                try:self.pending.put_nowait(packet)
                except queue.Full:
                    try:self.pending.get_nowait()
                    except queue.Empty:pass
                    try:self.pending.put_nowait(packet)
                    except queue.Full:pass
            except Exception as error:
                if time.monotonic()-self.last_warning>10:
                    print('[MOMA mirror] Waiting for desktop:',error);self.last_warning=time.monotonic()
                self.stop_event.wait(.5)
            self.stop_event.wait(.05)
    @staticmethod
    def matrix(rows):
        # Desktop uses column vectors; USD/Gf uses row vectors.
        if len(rows)!=4 or any(len(row)!=4 for row in rows):raise ValueError('Invalid matrix')
        if not all(math.isfinite(float(v)) for row in rows for v in row):raise ValueError('Nonfinite matrix')
        return Gf.Matrix4d(*[float(rows[j][i]) for i in range(4) for j in range(4)])
    def create_part(self,name,data):
        path=self.ROOT+'/'+name
        root=UsdGeom.Xform.Define(self.stage,path)
        op=root.AddTransformOp();op.Set(self.targets.get(name,Gf.Matrix4d(1)))
        mesh=UsdGeom.Mesh.Define(self.stage,path+'/Geometry')
        points=data['points'];counts=data['counts']
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
        mesh.CreateFaceVertexCountsAttr(counts);mesh.CreateFaceVertexIndicesAttr(data.get('indices',list(range(len(points)))))
        mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
        rgb=[int(data['color'][i:i+2],16)/255 for i in (1,3,5)]
        rgb=[v/12.92 if v<=.04045 else ((v+.055)/1.055)**2.4 for v in rgb]
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
        material=UsdShade.Material.Define(self.stage,path+'/Material')
        shader=UsdShade.Shader.Define(self.stage,path+'/Material/Shader');shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.55)
        shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(float(data.get('opacity',1)))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
        self.transforms[name]=op
    def reset(self,stage,session):
        # Only the mirror subtree is owned by this script; leave other user assets intact.
        if stage.GetPrimAtPath(self.ROOT):stage.RemovePrim(self.ROOT)
        self.stage=stage;self.session=session;self.transforms={};self.targets={};self.todo=[];self.geometry={}
        self.last_sequence=None;self.map_revision=None;self.framed=False;self.lidar=[]
        self.physics=None;self.physics_fault=False;self.instances={}
        UsdGeom.Xform.Define(stage,self.ROOT)
    def draw_points(self,path,points,color,width,z):
        shape=UsdGeom.Points.Define(self.stage,self.ROOT+path)
        shape.CreatePointsAttr([Gf.Vec3f(float(p[0]),float(p[1]),z) for p in points])
        shape.CreateWidthsAttr([width]);shape.SetWidthsInterpolation(UsdGeom.Tokens.constant)
        shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if points:
            xs=[p[0] for p in points];ys=[p[1] for p in points]
            shape.CreateExtentAttr([Gf.Vec3f(min(xs)-width,min(ys)-width,z-width),Gf.Vec3f(max(xs)+width,max(ys)+width,z+width)])
        return shape
    def draw_lines(self,path,lines,color,width,z):
        shape=UsdGeom.BasisCurves.Define(self.stage,self.ROOT+path)
        valid=[line for line in lines if len(line)>=2]
        shape.CreateTypeAttr('linear');shape.CreateWrapAttr('nonperiodic')
        shape.CreatePointsAttr([Gf.Vec3f(float(p[0]),float(p[1]),z) for line in valid for p in line])
        shape.CreateCurveVertexCountsAttr([len(line) for line in valid])
        shape.CreateWidthsAttr([width]);shape.SetWidthsInterpolation(UsdGeom.Tokens.constant)
        shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    def draw_map(self,data):
        revision=data.get('revision')
        if revision==self.map_revision:return
        self.map_revision=revision
        root=self.ROOT+'/Map'
        if self.stage.GetPrimAtPath(root):self.stage.RemovePrim(root)
        UsdGeom.Xform.Define(self.stage,root)
        self.draw_points('/Map/Cloud',data.get('cloud',[]),(.25,.35,.42),.018,.005)
        walls=[[(w[0],w[1]),(w[2],w[3])] for w in data.get('walls',[]) if len(w)>=4]
        self.draw_lines('/Map/Walls',walls,(.4,.55,.65),.025,.01)
        virtual=[[(w[0],w[1]),(w[2],w[3])] for w in data.get('virtual_walls',[]) if len(w)>=4]
        self.draw_lines('/Map/VirtualWalls',virtual,(.9,.15,.15),.025,.015)
        nodes={node['id']:node for node in data.get('nodes',[])}
        paths=[]
        for edge in data.get('edges',[]):
            if len(edge)>=2 and edge[0] in nodes and edge[1] in nodes:
                a,b=nodes[edge[0]],nodes[edge[1]];paths.append([(a['x'],a['y']),(b['x'],b['y'])])
        self.draw_lines('/Map/Routes',data.get('curves') or paths,(.15,.65,.4),.018,.025)
        self.draw_points('/Map/Nodes',[(n['x'],n['y']) for n in nodes.values()],(.2,.7,1.),.10,.04)
        for index,node in enumerate(nodes.values()):
            marker=UsdGeom.Xform.Define(self.stage,root+f'/Node_{index}')
            marker.GetPrim().SetDisplayName(node['id'])
            marker.AddTranslateOp().Set(Gf.Vec3d(node['x'],node['y'],.04))
            marker.GetPrim().SetCustomDataByKey('station',node['id'])
    def focus_paths(self):
        # Global map/point clouds can span a whole factory; never frame them
        # automatically. Frame the real robot, arm, and end effector instead.
        return [self.ROOT+'/'+name for name,row in self.instances.items()
                if row.get('physics_role') in ('robot','tool') and name in self.transforms and name in self.targets]

    def set_workcell_camera(self,viewport):
        robots=[name for name,row in self.instances.items() if row.get('physics_role')=='robot' and name in self.targets]
        equipment=[name for name,row in self.instances.items() if row.get('physics_role')=='equipment' and name in self.targets]
        if not robots or not equipment:return False
        robot=self.targets[robots[0]].ExtractTranslation()
        cad=self.targets[equipment[0]]
        center=cad.ExtractTranslation()
        # Equipment front is local -Y. Use a level frontal view, like the
        # operator's reference, with both the mobile base and cabinet visible.
        front=cad.TransformDir(Gf.Vec3d(0,-1,0))
        front=Gf.Vec3d(front[0],front[1],0)
        if front.GetLength()<1e-6:return False
        front=front.GetNormalized()
        if (center-robot).GetLength()>6.:return False
        target=Gf.Vec3d((robot[0]+center[0])/2,(robot[1]+center[1])/2,.95)
        eye=target+front*4.2+Gf.Vec3d(0,0,.65)
        camera=UsdGeom.Camera.Define(self.stage,self.ROOT+'/WorkcellCamera')
        camera.CreateFocalLengthAttr(35.)
        camera.CreateHorizontalApertureAttr(36.)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.05,10000.))
        xf=UsdGeom.Xformable(camera.GetPrim());xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(eye,target,Gf.Vec3d(0,0,1)).GetInverse())
        viewport.camera_path=camera.GetPath().pathString
        return True

    def frame_scene(self):
        if self.framed or self.todo:return
        try:
            from omni.kit.viewport.utility import get_active_viewport,frame_viewport_prims
            viewport=get_active_viewport()
            if viewport:
                if self.set_workcell_camera(viewport):
                    self.framed=True;return
                paths=self.focus_paths()
                if not paths:return
                frame_viewport_prims(viewport,paths);self.framed=True
        except Exception as error:
            if time.monotonic()-self.last_warning>10:
                print('[MOMA mirror] Could not frame robot; select a robot part and press F:',error);self.last_warning=time.monotonic()

    def update(self,event):
        stage=omni.usd.get_context().get_stage()
        if stage is None:return
        if UsdGeom.GetStageMetersPerUnit(stage)!=1. or UsdGeom.GetStageUpAxis(stage)!=UsdGeom.Tokens.z:
            if time.monotonic()-self.last_warning>10:
                print('[MOMA mirror] Use a Z-up stage with metersPerUnit=1 (File > New).');self.last_warning=time.monotonic()
            return
        now=time.monotonic();dt=min(.1,now-self.last_tick);self.last_tick=now
        try:manifest,frame=self.pending.get_nowait()
        except queue.Empty:manifest=frame=None
        if frame:
            if self.stage!=stage or self.session!=frame['session']:self.reset(stage,frame['session'])
            meta=frame.get('metadata',{})
            key=(frame['session'],frame['sequence'])
            fresh=(meta.get('connected') is True and (meta.get('mode')!='REAL' or meta.get('telemetry_age_s',999)<=2))
            if not fresh:self.last_frame_at=0.
            if key!=self.last_sequence and fresh:
                self.last_sequence=key;self.last_frame_at=now
                self.instances={entry['id']:entry for entry in frame['instances']}
                self.targets={entry['id']:self.matrix(entry['matrix']) for entry in frame['instances']}
                self.geometry=manifest['geometry']
                self.draw_map(manifest.get('map',{}))
                self.lidar=frame.get('lidar',[])
                self.draw_points('/LiDAR',self.lidar,(.12,1.,.5),.025,.1965)
                waiting=set(self.todo)
                self.todo.extend(name for name in self.targets if name not in self.transforms and name not in waiting)
        if self.stage!=stage:return
        root=stage.GetPrimAtPath(self.ROOT)
        if root:root.SetCustomDataByKey('connection','LIVE' if now-self.last_frame_at<=2 else 'STALE - frozen')
        # Build a few material groups per frame to avoid one giant initial UI stall.
        for _ in range(min(3,len(self.todo))):
            name=self.todo.pop(0)
            if name in self.geometry:self.create_part(name,self.geometry[name])
        if now-self.last_frame_at>2:
            self.draw_points('/LiDAR',[],(.12,1.,.5),.025,.1965)
            if self.physics is not None:
                omni.timeline.get_timeline_interface().pause()
            self.ready.clear();return
        if self.physics_enabled and self.transforms and not self.todo and not self.physics_fault:
            try:
                if self.physics is None:
                    import importlib.util
                    from pathlib import Path
                    client=Path(os.environ['MOMA_MIRROR_CLIENT'])
                    spec=importlib.util.spec_from_file_location('moma_door_physics',client.with_name('isaac_door_physics.py'))
                    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
                    params=json.loads(Path(os.environ['MOMA_DOOR_PHYSICS_CONFIG']).read_text(encoding='utf-8'))
                    self.physics=module.DoorPhysics(stage,self.ROOT,params,self.matrix)
                    omni.timeline.get_timeline_interface().pause()
                if not self.physics.initialized:
                    if self.physics.setup(self.instances,self.transforms):
                        if self.physics.p.get('auto_play',True):
                            omni.timeline.get_timeline_interface().play()
                            print('[MOMA physics] Mirror physics started automatically. Hook/roller hinge assist active; no real robot commands.')
                else:
                    self.physics.check_layout(self.instances)
                    self.physics.update_contact_assist(self.instances)
            except Exception as error:
                self.physics_fault=True
                omni.timeline.get_timeline_interface().pause()
                print('[MOMA physics] Setup/update failed, simulation PAUSED:',error)
        if self.transforms and not self.todo and not self.physics_fault:self.ready.set()
        alpha=1-math.exp(-dt/.05)
        for name,op in self.transforms.items():
            if self.physics is not None and name in self.physics.door_ids:
                UsdGeom.Imageable(op.GetAttr().GetPrim()).MakeInvisible();continue
            UsdGeom.Imageable(op.GetAttr().GetPrim()).MakeVisible() if name in self.targets else UsdGeom.Imageable(op.GetAttr().GetPrim()).MakeInvisible()
            if name not in self.targets:continue
            target=self.targets[name]
            if self.instances.get(name,{}).get('physics_role')=='door' and self.physics is None:
                op.Set(target);continue
            current=op.Get()
            a,b=Gf.Transform(current),Gf.Transform(target)
            if (a.GetTranslation()-b.GetTranslation()).GetLength()>1.:
                op.Set(target);continue
            transform=Gf.Transform()
            transform.SetTranslation(a.GetTranslation()*(1-alpha)+b.GetTranslation()*alpha)
            transform.SetScale(a.GetScale()*(1-alpha)+b.GetScale()*alpha)
            transform.SetRotation(Gf.Rotation(Gf.Slerp(alpha,a.GetRotation().GetQuat(),b.GetRotation().GetQuat())))
            op.Set(transform.GetMatrix())
        self.frame_scene()


if '_MOMA_MIRROR' in globals():_MOMA_MIRROR.close()
_MOMA_MIRROR=IsaacMirror(globals().get('MOMA_MIRROR_URL','http://127.0.0.1:8770'))
print('[MOMA mirror] Started. Keep the desktop 3D view open. Select /World/MOMA_Mirror and press F. No robot commands are sent.')
