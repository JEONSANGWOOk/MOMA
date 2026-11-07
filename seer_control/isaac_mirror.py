"""Opt-in, one-way desktop-render mirror. No robot command interfaces."""
import json
import gzip
import math
import os
from pathlib import Path
import queue
import threading
import time
import uuid
from .geometry3d import multiply,load_glb_static_robot,transform,axis_rotation


def directory():
    return Path(os.environ.get('MOMA_ISAAC_DIR',str(Path.home()/'.seer_amr_console'/'isaac_mirror')))


def atomic_json(path,data):
    pending=path.with_suffix('.tmp')
    pending.write_text(json.dumps(data,allow_nan=False,separators=(',',':')),encoding='utf-8')
    os.replace(pending,path)


class MirrorPublisher:
    def __init__(self,path=None):
        self.path=Path(path) if path else directory();self.path.mkdir(parents=True,exist_ok=True)
        self.session=uuid.uuid4().hex;self.geometry={};self.ids={};self.revision=0;self.sequence=0
        self.full_assets={}
        self.map_data={};self.map_revision=None
        self.error='';self.queue=queue.Queue(maxsize=1);self.stop_event=threading.Event()
        self.thread=threading.Thread(target=self.run,daemon=True,name='isaac-display-export');self.thread.start()
    def submit(self,assets,meshes,primitives,metadata):
        packet=(tuple(assets),tuple(meshes),tuple(primitives),dict(metadata))
        try:self.queue.put_nowait(packet)
        except queue.Full:
            try:self.queue.get_nowait()
            except queue.Empty:pass
            try:self.queue.put_nowait(packet)
            except queue.Full:pass
    def close(self):self.stop_event.set()
    def register(self,key,faces,color,opacity=1.):
        if key in self.ids:return self.ids[key]
        name=f'part_{len(self.ids)}';points=[];counts=[];indices=[];vertices={}
        for face in faces:
            if len(face)<3:continue
            for i in range(1,len(face)-1):
                for p in (face[0],face[i],face[i+1]):
                    keypoint=tuple(p)
                    index=vertices.get(keypoint)
                    if index is None:
                        index=len(points);vertices[keypoint]=index;points.append(keypoint)
                    indices.append(index)
                counts.append(3)
        self.ids[key]=name
        self.geometry[name]=dict(points=points,counts=counts,indices=indices,color=color,opacity=opacity,triangles=len(counts))
        self.revision+=1
        return name
    def export(self,assets,meshes,primitives,metadata):
        before=self.revision;instances=[]
        metadata=dict(metadata);map_data=metadata.pop('_map',None);lidar=metadata.pop('_lidar',[]);door_info=metadata.pop('_door_info',{});cad_id=metadata.pop('_cad_id',None)
        if map_data is not None and map_data.get('revision')!=self.map_revision:
            self.map_data=map_data;self.map_revision=map_data.get('revision');self.revision+=1
        for instance,(asset,positions,world,tint,alpha) in enumerate(assets):
            source=getattr(asset,'source_static_glb',None)
            if source:
                original=asset
                cached=self.full_assets.get(id(original))
                if cached is None:
                    # Full original geometry, including orphan root components.
                    full=load_glb_static_robot(source['path'],max_faces=None,rotation=source['rotation'],offset=source['offset'],color=source['color'])
                    if sum(len(faces) for faces,_,_ in full.links['base_link'])!=full.source_triangle_count:
                        raise ValueError('원본 GLB 전체 면 수 검증 실패')
                    cached=(original,full);self.full_assets[id(original)]=cached
                asset=cached[1]
            role='tool' if source and ('EndEffector' in source['path'] or 'gripper' in source['path']) else 'robot'
            poses=asset.link_transforms(positions,world)
            for link,visuals in asset.links.items():
                for index,(faces,local,color) in enumerate(visuals):
                    # Use the same full visual geometry as the GPU renderer.
                    faces=getattr(asset,'full_visuals',{}).get((link,index),faces)
                    name=self.register(('robot',instance,id(asset),link,index,tint,alpha),faces,tint or color,alpha)
                    instances.append(dict(id=name,matrix=multiply(poses[link],local),physics_role=role))
        for instance,(asset,world,tint) in enumerate(meshes):
            base=('mesh',instance,id(asset),tint)
            grouped={}
            # Group only once; model geometry and materials are immutable during a scene.
            if base not in self.ids:
                for face,material in zip(asset.faces,asset.face_materials):
                    color=getattr(asset,'material_colors',{}).get(material,getattr(asset,'material_overrides',{}).get(material,tint))
                    grouped.setdefault((color,1.),[]).append(face)
                glass=getattr(asset,'transparent_faces',())
                if glass:grouped[('#8bd9eb',.3)]=glass
                self.ids[base]=[self.register(base+(color,opacity),faces,color,opacity) for (color,opacity),faces in grouped.items()]
            for name in self.ids[base]:
                row=dict(id=name,matrix=world,physics_role='door' if id(asset) in door_info else 'equipment' if id(asset)==cad_id else 'visual')
                if id(asset) in door_info:row['door']=door_info[id(asset)]
                instances.append(row)
        for index,(tag,faces,world,color) in enumerate(primitives):
            name=self.register(('primitive',index,tag,color,tuple(tuple(tuple(p) for p in face) for face in faces)),faces,color)
            instances.append(dict(id=name,matrix=world))
        if before!=self.revision or not (self.path/'manifest.json').exists():
            atomic_json(self.path/'manifest.json',dict(version=1,session=self.session,revision=self.revision,units='metres',up_axis='Z',geometry=self.geometry,map=self.map_data))
            packed=self.path/'manifest.json.gz.tmp'
            with (self.path/'manifest.json').open('rb') as source, gzip.open(packed,'wb',compresslevel=1) as target:
                import shutil
                shutil.copyfileobj(source,target)
            os.replace(packed,self.path/'manifest.json.gz')
        self.sequence+=1
        atomic_json(self.path/'frame.json',dict(version=1,session=self.session,revision=self.revision,sequence=self.sequence,timestamp=time.time(),metadata=metadata,instances=instances,lidar=lidar))
    def run(self):
        while not self.stop_event.is_set():
            try:packet=self.queue.get(timeout=.2)
            except queue.Empty:continue
            try:self.export(*packet);self.error=''
            except Exception as error:self.error=str(error)


def map_snapshot(model):
    cloud=list(getattr(model,'cloud',()) or ())
    nodes=[dict(id=str(name),x=float(node['x']),y=float(node['y'])) for name,node in model.nodes.items()]
    return dict(revision=uuid.uuid4().hex,name=model.name,nodes=nodes,edges=list(model.edges),
        walls=list(model.walls),virtual_walls=list(getattr(model,'virtual_walls',()) or ()),
        curves=list(getattr(model,'curves',()) or ()),cloud=sample_points(cloud,40000))


def sample_points(points,limit=4000):
    rows=list(points);step=max(1,math.ceil(len(rows)/limit));result=[]
    for p in rows[::step]:
        if len(p)>=2 and all(type(v) in (int,float) and math.isfinite(v) for v in p[:2]):result.append([p[0],p[1]])
    return result


def publish_view(view,assets,meshes,primitives):
    if os.environ.get('MOMA_ISAAC_MIRROR')!='1':return
    now=time.monotonic()
    if now-getattr(view,'_isaac_at',0)<.05:return
    if not hasattr(view,'_isaac_publisher'):
        view._isaac_publisher=MirrorPublisher()
        view.bind('<Destroy>',lambda event:view._isaac_publisher.close() if event.widget is view else None,add='+')
    app=view.app
    model=app.map
    map_key=(id(model),id(getattr(model,'cloud',None)),len(getattr(model,'cloud',()) or ()),
        tuple((str(k),v.get('x'),v.get('y')) for k,v in model.nodes.items()),
        repr(model.edges),repr(model.walls),repr(getattr(model,'virtual_walls',())),repr(getattr(model,'curves',())))
    if map_key!=getattr(view,'_isaac_map_key',None):
        view._isaac_map=map_snapshot(model);view._isaac_map_key=map_key
    lidar=sample_points(getattr(view,'_isaac_lidar',()),4000)
    if app.real and (not app.connected or now-getattr(app,'last_laser_rx',0)>2):lidar=[]
    cad=view.cad_asset;door_info={}
    if cad is not None:
        cad_world=transform(view.cad_origin[:3],view.cad_origin[3:]);scale=view.scales.get('cad',1.)
        scaled=tuple(tuple(cad_world[i][j]*scale if i<3 and j<3 else cad_world[i][j] for j in range(4)) for i in range(4))
        for part in getattr(cad,'door_parts',()):
            side='left' if part['name'].endswith(':1') else 'right'
            hinge=multiply(scaled,transform(part['pivot']))
            angle=view.cad_door_angle(part)*part['sign']
            door_info[id(part['asset'])]=dict(side=side,sign=part['sign'],hinge_world=hinge,
                body_world=multiply(hinge,axis_rotation((0,0,1),angle)),
                target_angle_deg=math.degrees(angle),
                contact_assist=bool(app.real and view.real_doors_follow.get() and getattr(view,'_real_door_follower',None)
                    and not getattr(view,'manual_real_door_display',None)))
    view._isaac_publisher.submit(assets,meshes,primitives,dict(mode='REAL' if app.real else 'SIM',
        connected=bool(app.connected),telemetry_age_s=max(0.,now-app.last_state) if app.real else 0.,arm_source=getattr(view,'arm_source','unknown'),_map=view._isaac_map,_lidar=lidar,_door_info=door_info,_cad_id=id(cad)))
    view._isaac_at=now
