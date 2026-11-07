"""Small dependency-free geometry/FK helpers. Units are metres and radians."""
import math
import json
import struct
from pathlib import Path
import xml.etree.ElementTree as ET


def vector(text,size=3):
    values=tuple(float(v) for v in text.split())
    if len(values)!=size or not all(math.isfinite(v) for v in values):raise ValueError('3D 좌표/각도 값이 비정상입니다.')
    return values


def identity():return ((1.,0.,0.,0.),(0.,1.,0.,0.),(0.,0.,1.,0.),(0.,0.,0.,1.))


def multiply(a,b):
    # Fixed 4x4 transforms are a rendering/FK hot path. Avoid creating a
    # generator and calling sum for each of the sixteen output components.
    columns=tuple(zip(*b))
    return tuple(tuple(row[0]*c[0]+row[1]*c[1]+row[2]*c[2]+row[3]*c[3] for c in columns) for row in a)


def point(matrix,p):
    x,y,z=p
    return tuple(row[0]*x+row[1]*y+row[2]*z+row[3] for row in matrix[:3])


def axis_rotation(axis,angle):
    length=math.sqrt(sum(v*v for v in axis))
    if length<1e-12:raise ValueError('관절 회전축은 0 벡터일 수 없습니다.')
    x,y,z=(v/length for v in axis);c=math.cos(angle);s=math.sin(angle);t=1-c
    return ((t*x*x+c,t*x*y-s*z,t*x*z+s*y,0.),(t*x*y+s*z,t*y*y+c,t*y*z-s*x,0.),
            (t*x*z-s*y,t*y*z+s*x,t*z*z+c,0.),(0.,0.,0.,1.))


def transform(xyz=(0.,0.,0.),rpy=(0.,0.,0.)):
    rotation=multiply(multiply(axis_rotation((0,0,1),rpy[2]),axis_rotation((0,1,0),rpy[1])),axis_rotation((1,0,0),rpy[0]))
    return tuple(tuple(xyz[i] if j==3 and i<3 else rotation[i][j] for j in range(4)) for i in range(4))


def euler_xyz_transform(rpy=(0.,0.,0.)):
    """Three.js Euler XYZ rotation matrix for GLB scene transforms."""
    return multiply(multiply(axis_rotation((1,0,0),rpy[0]),axis_rotation((0,1,0),rpy[1])),axis_rotation((0,0,1),rpy[2]))


def box(size):
    x,y,z=(v/2 for v in size)
    vertices=[(-x,-y,-z),(x,-y,-z),(x,y,-z),(-x,y,-z),(-x,-y,z),(x,-y,z),(x,y,z),(-x,y,z)]
    return [tuple(vertices[i] for i in face) for face in [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]]


def cylinder(radius,length,steps=64):
    rings=[[(radius*math.cos(i*2*math.pi/steps),radius*math.sin(i*2*math.pi/steps),z) for i in range(steps)] for z in (-length/2,length/2)]
    return [tuple(reversed(rings[0])),tuple(rings[1])]+[(rings[0][i],rings[0][(i+1)%steps],rings[1][(i+1)%steps],rings[1][i]) for i in range(steps)]


def sphere(radius):
    faces=[]
    for j in range(6):
        a=-math.pi/2+j*math.pi/6;b=a+math.pi/6
        for i in range(12):
            c=i*math.pi/6;d=c+math.pi/6
            faces.append(tuple((radius*math.cos(lat)*math.cos(lon),radius*math.cos(lat)*math.sin(lon),radius*math.sin(lat)) for lat,lon in ((a,c),(a,d),(b,d),(b,c))))
    return faces


def simplify_faces(faces,budget):
    """Merge nearby vertices rather than dropping independent surface triangles."""
    if len(faces)<=budget:return faces
    vertices=set(p for face in faces for p in face)
    lo=[min(p[i] for p in vertices) for i in range(3)];hi=[max(p[i] for p in vertices) for i in range(3)]
    extent=max(b-a for a,b in zip(lo,hi))
    if extent<=1e-12:return faces[:budget]
    best=faces
    for resolution in (80,60,45,32,24,18,12,8,5,3):
        size=extent/resolution;cells={};keys={}
        for p in vertices:
            key=tuple(round((p[i]-lo[i])/size) for i in range(3));keys[p]=key
            sums,count=cells.get(key,([0.,0.,0.],0));cells[key]=([a+b for a,b in zip(sums,p)],count+1)
        centers={key:tuple(v/count for v in sums) for key,(sums,count) in cells.items()}
        result=[];seen=set()
        for face in faces:
            ids=tuple(keys[p] for p in face)
            if len(set(ids))<3:continue
            canonical=tuple(sorted(ids))
            if canonical in seen:continue
            seen.add(canonical);result.append(tuple(centers[key] for key in ids))
        if result:best=result
        if result and len(result)<=budget:return result
    return best[:budget]


def _split_simple_whole_glass(asset):
    """Separate the upper outward-facing enclosure shell into tinted glass."""
    points=[p for face in asset.faces for p in face]
    lo=[min(p[i] for p in points) for i in range(3)];hi=[max(p[i] for p in points) for i in range(3)]
    ground=-lo[2];glass=[];solid=[];glass_materials=[];solid_materials=[]
    for face,material in zip(asset.faces,asset.face_materials):
        a,b,c=face;u=tuple(b[i]-a[i] for i in range(3));v=tuple(c[i]-a[i] for i in range(3))
        normal=(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])
        length=max(1e-12,math.sqrt(sum(x*x for x in normal)));normal=tuple(x/length for x in normal)
        center=tuple(sum(p[i] for p in face)/3 for i in range(3));upper=center[2]+ground>=.82
        on_x=(abs(center[0]-lo[0])<=.025 or abs(center[0]-hi[0])<=.025) and abs(normal[0])>=.78
        on_y=(abs(center[1]-lo[1])<=.025 or abs(center[1]-hi[1])<=.025) and abs(normal[1])>=.78
        if material in (32,34,35) or (upper and (on_x or on_y)):glass.append(face);glass_materials.append(material)
        else:solid.append(face);solid_materials.append(material)
    asset.faces=solid;asset.face_materials=solid_materials
    asset.transparent_faces=glass;asset.transparent_face_materials=glass_materials
    asset.material_overrides={36:'#e7edf1'}
    return asset


def load_mesh(path,max_faces=3000,rotation=(0.,0.,0.)):
    path=Path(path)
    if path.stat().st_size>64*1024*1024:raise ValueError('3D 미리보기 파일은 64 MB 이하로 가져오세요.')
    faces=[]
    if path.suffix.lower()=='.obj':
        vertices=[]
        for line in path.read_text(encoding='utf-8-sig').splitlines():
            parts=line.split()
            if not parts:continue
            if parts[0]=='v':vertices.append(vector(' '.join(parts[1:4])))
            elif parts[0]=='f':
                indices=[int(v.split('/')[0]) for v in parts[1:]]
                if len(indices)<3:continue
                if any(v==0 or v>len(vertices) or v < -len(vertices) for v in indices):raise ValueError('OBJ 면의 정점 번호가 비정상입니다.')
                polygon=[vertices[v-1 if v>0 else v] for v in indices]
                faces.extend((polygon[0],polygon[i],polygon[i+1]) for i in range(1,len(polygon)-1))
    elif path.suffix.lower()=='.stl':
        data=path.read_bytes();count=struct.unpack_from('<I',data,80)[0] if len(data)>=84 else 0
        if count and 84+count*50==len(data):
            for i in range(count):
                values=struct.unpack_from('<12f',data,84+i*50)
                faces.append(tuple(tuple(values[3+j*3:6+j*3]) for j in range(3)))
        else:
            vertices=[]
            for line in data.decode('utf-8-sig').splitlines():
                parts=line.split()
                if parts and parts[0].lower()=='vertex':vertices.append(vector(' '.join(parts[1:])))
            if len(vertices)%3:raise ValueError('ASCII STL 정점 수가 비정상입니다.')
            faces=[tuple(vertices[i:i+3]) for i in range(0,len(vertices),3)]
    elif path.suffix.lower()=='.glb':
        asset=load_glb(path,max_faces,rotation)
        if path.stem.lower()=='simple_whole':
            asset=_split_simple_whole_glass(asset)
            if max_faces>=100000:
                lod=_split_simple_whole_glass(load_glb(path,30000,rotation))
                asset.lod_faces=lod.faces;asset.lod_face_materials=lod.face_materials
                asset.lod_transparent_faces=lod.transparent_faces
                asset.lod_door_parts=lod.door_parts
        return asset
    else:raise ValueError('미리보기 메시 형식: STL / OBJ / GLB. STEP/IGES는 STL/OBJ로 내보내세요.')
    if not faces or not all(math.isfinite(v) for face in faces for p in face for v in p):raise ValueError('유효한 3D 면이 없습니다.')
    count=len(faces)
    return MeshAsset(path.name,simplify_faces(faces,max_faces),count,faces)


def _glb_face_lod(vertices,ids,mode,face_count,budget,world,extra):
    """Spatially simplify one GLB primitive while keeping its visible shell."""
    if face_count<=budget:
        stride=1
    else:stride=max(1,math.ceil(face_count/max(1,budget)))
    if stride==1:
        selected=range(face_count)
        result=[]
        for fi in selected:
            if mode==4:a,b,c=ids[fi*3:fi*3+3]
            elif mode==5:
                a,b,c=ids[fi:fi+3]
                if fi%2:a,b=b,a
            else:a,b,c=ids[0],ids[fi+1],ids[fi+2]
            result.append(tuple(point(extra,point(world,vertices[i][:3])) for i in (a,b,c)))
        return result
    lo=tuple(min(p[i] for p in vertices) for i in range(3));hi=tuple(max(p[i] for p in vertices) for i in range(3))
    extent=max(hi[i]-lo[i] for i in range(3));resolution=max(3,int(math.sqrt(max(1,budget)/2)))
    if extent<1e-12:return []
    cell_sums={};cell_counts={};triangles=[];seen=set()
    def key_for(index):
        p=vertices[index][:3]
        key=tuple(max(0,min(resolution-1,int((p[i]-lo[i])/extent*resolution))) for i in range(3))
        if key not in cell_sums:
            cell_sums[key]=[float(p[0]),float(p[1]),float(p[2])];cell_counts[key]=1
        else:
            acc=cell_sums[key];acc[0]+=p[0];acc[1]+=p[1];acc[2]+=p[2];cell_counts[key]+=1
        return key
    for fi in range(face_count):
        if mode==4:a,b,c=ids[fi*3:fi*3+3]
        elif mode==5:
            a,b,c=ids[fi:fi+3]
            if fi%2:a,b=b,a
        else:a,b,c=ids[0],ids[fi+1],ids[fi+2]
        keys=(key_for(a),key_for(b),key_for(c))
        if len(set(keys))<3:continue
        canonical=tuple(sorted(keys))
        if canonical in seen:continue
        seen.add(canonical);triangles.append(keys)
    centers={key:tuple(value/cell_counts[key] for value in sums) for key,sums in cell_sums.items()}
    result=[tuple(point(extra,point(world,centers[k])) for k in triangle) for triangle in triangles]
    if len(result)>budget*1.5:result=result[::math.ceil(len(result)/budget)]
    return result


def load_glb(path,max_faces=3000,rotation=(0.,0.,0.)):
    """Read triangle meshes from a binary glTF scene without external packages."""
    path=Path(path)
    if path.stat().st_size>64*1024*1024:raise ValueError('3D 미리보기 파일은 64 MB 이하로 가져오세요.')
    data=path.read_bytes()
    if len(data)<20 or data[:4]!=b'glTF':raise ValueError('Binary glTF(GLB) 헤더가 유효하지 않습니다.')
    version,total=struct.unpack_from('<II',data,4)
    if version!=2 or total>len(data):raise ValueError('GLB 2.0 파일만 지원합니다.')
    offset=12;document=None;binary=None
    while offset+8<=total:
        length,kind=struct.unpack_from('<II',data,offset);offset+=8
        chunk=data[offset:offset+length];offset+=length
        if kind==0x4E4F534A:document=json.loads(chunk.decode('utf-8').rstrip('\x00 '))
        elif kind==0x004E4942:binary=chunk
    if not isinstance(document,dict) or binary is None:raise ValueError('GLB JSON/BIN chunk가 없습니다.')
    buffers=document.get('buffers',[])
    if not buffers or buffers[0].get('uri'):raise ValueError('External buffer를 사용하는 GLB는 지원하지 않습니다.')
    views=document.get('bufferViews',[]);accessors=document.get('accessors',[]);meshes=document.get('meshes',[]);nodes=document.get('nodes',[])
    component={5121:('B',1),5123:('H',2),5125:('I',4),5126:('f',4)}
    type_size={'SCALAR':1,'VEC2':2,'VEC3':3,'VEC4':4,'MAT4':16}
    def accessor_values(index):
        item=accessors[index];view=views[item['bufferView']];fmt,size=component[item['componentType']];width=type_size[item['type']]
        stride=view.get('byteStride',size*width);start=view.get('byteOffset',0)+item.get('byteOffset',0)
        return [struct.unpack_from('<'+fmt*width,binary,start+i*stride) for i in range(item['count'])]
    def trs(node):
        if 'matrix' in node:
            values=node['matrix'];return tuple(tuple(float(values[c*4+r]) for c in range(4)) for r in range(4))
        t=node.get('translation',(0.,0.,0.));s=node.get('scale',(1.,1.,1.));q=node.get('rotation',(0.,0.,0.,1.));x,y,z,w=q
        rot=((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),0.),(2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),0.),(2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y),0.),(0.,0.,0.,1.))
        scaled=tuple(tuple(rot[r][c]*s[c] if c<3 else rot[r][c] for c in range(4)) for r in range(4))
        return multiply(transform(tuple(t)),scaled)
    extra=euler_xyz_transform(tuple(math.radians(float(v)) for v in rotation))
    selected=document.get('scenes',[{}])[document.get('scene',0)].get('nodes',[]) if document.get('scenes') else list(range(len(nodes)))
    plans=[];total_faces=0;visited=set()
    def visit(node_index,parent,door_name=None):
        nonlocal total_faces
        if node_index in visited:return
        visited.add(node_index)
        node=nodes[node_index];world=multiply(parent,trs(node));name=str(node.get('name',''))
        if name.upper().startswith('SIMPLE_SASSY-DOOR'):door_name=name
        if 'mesh' in node:
            for primitive in meshes[node['mesh']].get('primitives',[]):
                if 'POSITION' not in primitive.get('attributes',{}):continue
                mode=primitive.get('mode',4)
                if mode not in (4,5,6):continue
                vertex_count=accessors[primitive['attributes']['POSITION']]['count']
                index_count=accessors[primitive['indices']]['count'] if 'indices' in primitive else vertex_count
                face_count=index_count//3 if mode==4 else max(0,index_count-2)
                total_faces+=face_count;plans.append((primitive,world,mode,face_count,primitive.get('material'),door_name,name))
        for child in node.get('children',[]):visit(child,world,door_name)
    identity_matrix=identity()
    for root in selected:visit(root,identity_matrix)
    # Some CAD exports leave tool subassemblies (notably the hook bracket)
    # as orphan scene roots. Include those roots so the viewer matches the
    # complete authored assembly instead of silently dropping the gripper.
    parents={child for node in nodes for child in node.get('children',[])}
    for root in range(len(nodes)):
        if root not in parents and root not in visited:visit(root,identity_matrix)
    for node_index in range(len(nodes)):
        if node_index not in visited:visit(node_index,identity_matrix)
    if total_faces==0:raise ValueError('GLB 안에 유효한 triangle mesh가 없습니다.')
    budget=total_faces if max_faces is None else max(1,int(max_faces));reserve=min(120,max(1,budget//max(1,len(plans))))
    allocations=[min(count,reserve) for _,_,_,count,_,_,_ in plans]
    remaining=max(0,budget-sum(allocations))
    while remaining:
        capacity=[max(0,count-allocated) for allocated,(_,_,_,count,_,_,_) in zip(allocations,plans)]
        total_capacity=sum(capacity)
        if not total_capacity:break
        before=remaining
        for i,room in enumerate(capacity):
            if not room or not remaining:continue
            share=max(1,int(before*room/total_capacity))
            amount=min(room,share,remaining);allocations[i]+=amount;remaining-=amount
    faces=[];face_materials=[];face_nodes=[];door_geometry={}
    for (primitive,world,mode,face_count,material,door_name,node_name),allocation in zip(plans,allocations):
        if allocation<=0:continue
        vertices=accessor_values(primitive['attributes']['POSITION'])
        indices=accessor_values(primitive['indices']) if 'indices' in primitive else [(i,) for i in range(len(vertices))]
        ids=[value[0] for value in indices]
        generated=_glb_face_lod(vertices,ids,mode,face_count,allocation,world,extra)
        if door_name:
            part=door_geometry.setdefault(door_name,([],[]));part[0].extend(generated);part[1].extend([material]*len(generated))
        else:
            faces.extend(generated);face_materials.extend([material]*len(generated));face_nodes.extend([node_name]*len(generated))
    asset=MeshAsset(path.name,faces,total_faces,faces);asset.face_materials=face_materials;asset.face_nodes=face_nodes
    def source_color(material):
        rgb=material.get('pbrMetallicRoughness',{}).get('baseColorFactor',[1,1,1,1])[:3]
        def channel(v):
            v=max(0.,min(1.,float(v)))
            return round(255*(12.92*v if v<=.0031308 else 1.055*v**(1/2.4)-.055))
        return '#'+''.join(f'{channel(v):02x}' for v in rgb)
    asset.material_colors={i:source_color(m) for i,m in enumerate(document.get('materials',[]))}
    asset.door_parts=[]
    for name,(part_faces,part_materials) in door_geometry.items():
        if not part_faces:continue
        xs=[p[0] for face in part_faces for p in face];ys=[p[1] for face in part_faces for p in face]
        hinge_x=min(xs) if name.endswith(':1') else max(xs);hinge_y=(min(ys)+max(ys))/2
        solid=[];solid_materials=[];glass=[]
        for face,material in zip(part_faces,part_materials):
            if material in (32,34,35):glass.append(face)
            else:solid.append(face);solid_materials.append(material)
        part_asset=MeshAsset(name,solid,len(part_faces),solid);part_asset.face_materials=solid_materials
        part_asset.transparent_faces=glass;part_asset.material_overrides={}
        part_asset.material_colors=dict(asset.material_colors)
        asset.door_parts.append(dict(name=name,asset=part_asset,pivot=(hinge_x,hinge_y,0.),sign=-1. if name.endswith(':1') else 1.))
    return asset


def load_glb_static_robot(path,max_faces=12000,rotation=(0.,0.,0.),color='#8292a1',offset=(0.,0.,0.)):
    """Load a static GLB assembly as a renderer-friendly single-link robot."""
    mesh=load_glb(path,max_faces=None,rotation=rotation)
    if not mesh.faces:raise ValueError('GLB 메시가 비어 있습니다.')
    if any(offset):mesh.faces=[tuple(tuple(p[i]+offset[i] for i in range(3)) for p in face) for face in mesh.faces]
    groups={}
    for face,name in zip(mesh.faces,getattr(mesh,'face_nodes',())):groups.setdefault(name,[]).append(face)
    by_color={}
    for face,material in zip(mesh.faces,mesh.face_materials):
        shade=mesh.material_colors.get(material,color)
        by_color.setdefault(shade,[]).append(face)
    # Keep every source triangle. The display uses full geometry, so computing
    # an unused simplified copy here only delays the initial viewport.
    links={'base_link':[(faces,identity(),shade) for shade,faces in by_color.items()]}
    asset=RobotDescription(Path(path).stem,links,[],['base_link'])
    asset.full_visuals={('base_link',i):faces for i,faces in enumerate(by_color.values())}
    asset.component_faces={name:len(faces) for name,faces in groups.items()}
    asset.component_bounds={name:(tuple(min(p[i] for face in faces for p in face) for i in range(3)),
                                 tuple(max(p[i] for face in faces for p in face) for i in range(3)))
                            for name,faces in groups.items() if faces and 'Polymer_ROECRHA50-10-30-T5' in name}
    asset.source_static_glb=dict(path=str(Path(path).resolve()),rotation=tuple(rotation),offset=tuple(offset),color=color)
    asset.source_triangle_count=mesh.total
    pads=[p for name,faces in groups.items() if name.startswith('ZP10UN_T') for face in faces for p in face]
    if pads:
        lo=[min(p[i] for p in pads) for i in range(3)];hi=[max(p[i] for p in pads) for i in range(3)]
        normal=min(range(3),key=lambda i:hi[i]-lo[i]);longest=max(range(3),key=lambda i:hi[i]-lo[i])
        center=[(a+b)/2 for a,b in zip(lo,hi)];center[normal]=hi[normal]
        z=[float(i==normal) for i in range(3)];x=[float(i==longest) for i in range(3)]
        y=[z[1]*x[2]-z[2]*x[1],z[2]*x[0]-z[0]*x[2],z[0]*x[1]-z[1]*x[0]]
        asset.vacuum_mount=[[x[i],y[i],z[i],center[i]] for i in range(3)]+[[0.,0.,0.,1.]]

    return asset


def load_glb_robot(path,max_faces=3000):
    """Load the extctrl FR5 assembly GLB as an articulated 6-axis robot."""
    path=Path(path);data=path.read_bytes()
    if len(data)>64*1024*1024 or len(data)<20 or data[:4]!=b'glTF':raise ValueError('유효한 FR5 GLB 파일이 아닙니다.')
    version,total=struct.unpack_from('<II',data,4)
    if version!=2 or total>len(data):raise ValueError('GLB 2.0 파일만 지원합니다.')
    offset=12;document=None;binary=None
    while offset+8<=total:
        length,kind=struct.unpack_from('<II',data,offset);offset+=8;chunk=data[offset:offset+length];offset+=length
        if kind==0x4E4F534A:document=json.loads(chunk.decode('utf-8').rstrip('\x00 '))
        elif kind==0x004E4942:binary=chunk
    if not isinstance(document,dict) or binary is None:raise ValueError('GLB JSON/BIN 데이터가 없습니다.')
    nodes=document.get('nodes',[]);meshes=document.get('meshes',[]);views=document.get('bufferViews',[]);accessors=document.get('accessors',[])
    component={5121:('B',1),5123:('H',2),5125:('I',4),5126:('f',4)};widths={'SCALAR':1,'VEC2':2,'VEC3':3,'VEC4':4,'MAT4':16}
    def values(index):
        item=accessors[index];view=views[item['bufferView']];fmt,size=component[item['componentType']];width=widths[item['type']]
        start=view.get('byteOffset',0)+item.get('byteOffset',0);stride=view.get('byteStride',size*width)
        return [struct.unpack_from('<'+fmt*width,binary,start+i*stride) for i in range(item['count'])]
    def node_transform(node):
        if 'matrix' in node:
            m=node['matrix'];return tuple(tuple(float(m[c*4+r]) for c in range(4)) for r in range(4))
        t=node.get('translation',(0.,0.,0.));s=node.get('scale',(1.,1.,1.));x,y,z,w=node.get('rotation',(0.,0.,0.,1.))
        rot=((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),0.),(2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),0.),(2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y),0.),(0.,0.,0.,1.))
        scaled=tuple(tuple(rot[r][c]*s[c] if c<3 else rot[r][c] for c in range(4)) for r in range(4))
        return multiply(transform(tuple(t)),scaled)
    def inverse(m):
        rows=[list(m[r])+[1. if r==c else 0. for c in range(4)] for r in range(4)]
        for col in range(4):
            pivot=max(range(col,4),key=lambda r:abs(rows[r][col]))
            if abs(rows[pivot][col])<1e-12:raise ValueError('FR5 GLB 노드 변환을 역산할 수 없습니다.')
            rows[col],rows[pivot]=rows[pivot],rows[col];scale=rows[col][col];rows[col]=[v/scale for v in rows[col]]
            for r in range(4):
                if r==col:continue
                factor=rows[r][col];rows[r]=[a-factor*b for a,b in zip(rows[r],rows[col])]
        return tuple(tuple(rows[r][c+4] for c in range(4)) for r in range(4))
    roots=document.get('scenes',[{}])[document.get('scene',0)].get('nodes',[]) if document.get('scenes') else range(len(nodes))
    world={}
    def visit(index,parent):
        node=nodes[index];pose=multiply(parent,node_transform(node));name=node.get('name')
        if name:world[name]=(index,pose)
        for child in node.get('children',[]):visit(child,pose)
    for root in roots:visit(root,identity())
    parts=('BASE','J1','J2','J3','J4','J5','J6')
    if any(name not in world for name in parts):raise ValueError('FR5 GLB에 BASE/J1~J6 파트가 모두 있어야 합니다.')
    links={};full_visuals={};budget=max(7,int(max_faces));lod_budget=max(1,budget//7)
    for name in parts:
        index,_=world[name];node=nodes[index];source_faces=[]
        if 'mesh' in node:
            for primitive in meshes[node['mesh']].get('primitives',[]):
                attrs=primitive.get('attributes',{});mode=primitive.get('mode',4)
                if 'POSITION' not in attrs or mode not in (4,5,6):continue
                vertices=values(attrs['POSITION']);ids=[v[0] for v in values(primitive['indices'])] if 'indices' in primitive else list(range(len(vertices)))
                face_count=len(ids)//3 if mode==4 else max(0,len(ids)-2)
                for fi in range(face_count):
                    if mode==4:a,b,c=ids[fi*3:fi*3+3]
                    elif mode==5:
                        a,b,c=ids[fi:fi+3]
                        if fi%2:a,b=b,a
                    else:a,b,c=ids[0],ids[fi+1],ids[fi+2]
                    source_faces.append(tuple(tuple(float(v) for v in vertices[i][:3]) for i in (a,b,c)))
        if not source_faces:raise ValueError(f'FR5 GLB mesh missing for {name}.')
        full_visuals[(name,0)]=source_faces
        faces=simplify_faces(source_faces,lod_budget) if len(source_faces)>lod_budget else source_faces
        links[name]=[(faces,identity(),'#9ba8b5')]
    limits={'J1':(-175,175),'J2':(-265,85),'J3':(-150,150),'J4':(-265,85),'J5':(-175,175),'J6':(-175,175)}
    joints=[]
    for i,name in enumerate(parts[1:]):
        parent=parts[i];origin=multiply(inverse(world[parent][1]),world[name][1]);lo,hi=limits[name]
        joints.append(dict(name=name,type='revolute',parent=parent,child=name,origin=origin,axis=(0.,0.,1.),lower=math.radians(lo),upper=math.radians(hi),mimic=None,multiplier=1.,offset=0.))
    asset=RobotDescription('FAIRINO FR5 extctrl assembly',links,joints,['BASE']);asset.full_visuals=full_visuals
    return asset


class MeshAsset:
    def __init__(self,name,faces,total=None,full_faces=None):self.name=name;self.faces=faces;self.total=total or len(faces);self.full_faces=full_faces or faces;self.face_materials=[None]*len(faces)
    def draw_faces(self,world=identity(),scale=(1,1,1),color='#b7c4d3',faces=None,materials=None):
        overrides=getattr(self,'material_overrides',{});faces=self.faces if faces is None else faces
        materials=getattr(self,'face_materials',()) if materials is None else materials
        return [(tuple(point(world,tuple(p[i]*scale[i] for i in range(3))) for p in face),getattr(self,'material_colors',{}).get(material,overrides.get(material,color)),self.name) for face,material in zip(faces,materials)]


class RobotDescription:
    def __init__(self,name,links,joints,roots):
        self.name,self.links,self.joints,self.roots=name,links,joints,roots
        self.warnings=[];self.synthetic=False;self.full_visuals={}

    @classmethod
    def load(cls,path,asset_root=None):
        path=Path(path)
        if path.stat().st_size>8*1024*1024:raise ValueError('URDF 파일이 너무 큽니다.')
        try:root=ET.fromstring(path.read_text(encoding='utf-8-sig'))
        except ET.ParseError as error:raise ValueError('URDF XML 형식 오류: '+str(error)) from error
        if root.tag!='robot':raise ValueError('robot 요소를 포함한 URDF가 필요합니다. Xacro는 URDF로 변환하세요.')
        links={};joints=[];warnings=[];full_visuals={}
        materials={e.get('name'):e.find('color').get('rgba') for e in root.findall('material') if e.find('color') is not None}
        def origin(element):
            e=element.find('origin')
            return transform(vector(e.get('xyz','0 0 0')),vector(e.get('rpy','0 0 0'))) if e is not None else identity()
        for link in root.findall('link'):
            name=link.get('name')
            if not name or name in links:raise ValueError('URDF link 이름이 비었거나 중복되었습니다.')
            visuals=[]
            for visual in link.findall('visual'):
                geometry=visual.find('geometry')
                if geometry is None:continue
                material=visual.find('material');rgba=None
                if material is not None:
                    rgba=material.find('color').get('rgba') if material.find('color') is not None else materials.get(material.get('name'))
                color='#e9ac42'
                if rgba:
                    values=vector(rgba,4);color='#'+''.join(f'{round(max(0,min(1,v))*255):02x}' for v in values[:3])
                full=None
                try:
                    e=geometry.find('box')
                    if e is not None:
                        size=vector(e.get('size',''))
                        if min(size)<=0:raise ValueError('box 크기는 양수여야 합니다.')
                        faces=box(size)
                    elif geometry.find('cylinder') is not None:
                        e=geometry.find('cylinder');radius=float(e.get('radius'));length=float(e.get('length'))
                        if not math.isfinite(radius+length) or min(radius,length)<=0:raise ValueError('cylinder 크기가 비정상입니다.')
                        faces=cylinder(radius,length)
                    elif geometry.find('sphere') is not None:
                        radius=float(geometry.find('sphere').get('radius'))
                        if not math.isfinite(radius) or radius<=0:raise ValueError('sphere 크기가 비정상입니다.')
                        faces=sphere(radius)
                    elif geometry.find('mesh') is not None:
                        e=geometry.find('mesh');filename=e.get('filename','');scale=vector(e.get('scale','1 1 1'))
                        if filename.startswith('package://'):
                            relative=filename[len('package://'):]
                            bases=[Path(asset_root)] if asset_root else [path.parent,path.parent.parent]
                            candidates=[base/relative for base in bases]+[base/relative.split('/',1)[-1] for base in bases]
                        elif filename.startswith('file://'):candidates=[Path(filename[7:])]
                        elif '://' in filename:raise ValueError('원격 메시 주소는 지원하지 않습니다.')
                        else:candidates=[path.parent/filename]
                        resolved=next((candidate for candidate in candidates if candidate.is_file()),None)
                        if resolved is None:raise ValueError('메시 파일을 찾지 못했습니다: '+filename)
                        mesh=load_mesh(resolved,max_faces=320)
                        faces=[tuple(tuple(p[i]*scale[i] for i in range(3)) for p in face) for face in mesh.faces]
                        full=[tuple(tuple(p[i]*scale[i] for i in range(3)) for p in face) for face in mesh.full_faces]
                        if mesh.total>len(mesh.faces):warnings.append(f'{name}: 메시 단순화 {mesh.total} → {len(mesh.faces)}면')
                    else:raise ValueError('지원하지 않는 geometry입니다.')
                except (ValueError,OSError,UnicodeError,TypeError) as error:
                    warnings.append(f'{name}: {error}');continue
                if full is not None:full_visuals[(name,len(visuals))]=full
                visuals.append((faces,origin(visual),color))
            links[name]=visuals
        if not links:raise ValueError('URDF link가 없습니다.')
        child_links=set();joint_names=set()
        for joint in root.findall('joint'):
            name=joint.get('name');typ=joint.get('type')
            if not name or name in joint_names:raise ValueError('URDF joint 이름이 비었거나 중복되었습니다.')
            joint_names.add(name)
            if typ not in ('fixed','revolute','continuous','prismatic'):raise ValueError('지원하지 않는 joint type: '+str(typ))
            parent=joint.find('parent');child=joint.find('child')
            if parent is None or child is None:raise ValueError('joint parent/child가 필요합니다.')
            parent=parent.get('link');child=child.get('link')
            if parent not in links or child not in links or child in child_links:raise ValueError('URDF link 연결이 비정상입니다.')
            child_links.add(child);axis=joint.find('axis');limit=joint.find('limit');mimic=joint.find('mimic')
            joints.append(dict(name=name,type=typ,parent=parent,child=child,origin=origin(joint),
                               axis=vector(axis.get('xyz','1 0 0')) if axis is not None else (1.,0.,0.),
                               lower=float(limit.get('lower',-math.pi)) if limit is not None else -math.pi,
                               upper=float(limit.get('upper',math.pi)) if limit is not None else math.pi,
                               mimic=mimic.get('joint') if mimic is not None else None,
                               multiplier=float(mimic.get('multiplier',1)) if mimic is not None else 1.,offset=float(mimic.get('offset',0)) if mimic is not None else 0.))
        for joint in joints:
            length=math.sqrt(sum(v*v for v in joint["axis"]))
            if length<1e-12:raise ValueError('URDF 관절 축은 0일 수 없습니다.')
            joint["axis"]=tuple(v/length for v in joint["axis"])
        roots=[name for name in links if name not in child_links]
        if len(roots)!=1:raise ValueError('URDF는 하나의 root link를 가진 트리여야 합니다.')
        model=cls(root.get('name',path.stem),links,joints,roots);model.warnings=warnings;model.full_visuals=full_visuals
        if model.name=='fairino5_v6_robot':
            from .visual_materials import style_fr5
            style_fr5(model)
        if len(model.link_transforms({}))!=len(links):raise ValueError('URDF link에 순환 또는 끊어진 연결이 있습니다.')
        if any(not math.isfinite(j['lower']+j['upper']+j['multiplier']+j['offset']) or j['lower']>j['upper'] for j in joints):raise ValueError('URDF 관절 범위가 비정상입니다.')
        return model

    def movable(self):return [j for j in self.joints if j['type']!='fixed' and not j.get('mimic')]

    def link_transforms(self,positions,world=identity()):
        values={}
        def value(joint,seen=()):
            name=joint['name']
            if name in seen:raise ValueError('mimic 관절 순환입니다.')
            if name in values:return values[name]
            if joint.get('mimic'):
                source=next((j for j in self.joints if j['name']==joint['mimic']),None)
                if source is None:raise ValueError('mimic 대상 관절이 없습니다.')
                v=value(source,seen+(name,))*joint['multiplier']+joint['offset']
            else:v=float(positions.get(name,0))
            if not math.isfinite(v):raise ValueError('관절값이 비정상입니다.')
            if joint['type'] in ('revolute','prismatic'):v=max(joint['lower'],min(joint['upper'],v))
            values[name]=v;return v
        poses={name:world for name in self.roots}
        for _ in range(len(self.joints)+1):
            changed=False
            for joint in self.joints:
                if joint['child'] in poses or joint['parent'] not in poses:continue
                v=value(joint)
                motion=axis_rotation(joint['axis'],v) if joint['type'] in ('revolute','continuous') else transform(tuple(v*a for a in joint['axis'])) if joint['type']=='prismatic' else identity()
                poses[joint['child']]=multiply(multiply(poses[joint['parent']],joint['origin']),motion);changed=True
            if not changed:break
        return poses

    def draw_faces(self,positions,world=identity(),full_resolution=False):
        result=[]
        for name,pose in self.link_transforms(positions,world).items():
            for index,(faces,local,color) in enumerate(self.links[name]):
                if full_resolution:faces=self.full_visuals.get((name,index),faces)
                matrix=multiply(pose,local)
                result.extend((tuple(point(matrix,p) for p in face),color,name) for face in faces)
        return result
