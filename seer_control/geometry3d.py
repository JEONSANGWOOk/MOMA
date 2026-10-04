"""Small dependency-free geometry/FK helpers. Units are metres and radians."""
import math
import struct
from pathlib import Path
import xml.etree.ElementTree as ET


def vector(text,size=3):
    values=tuple(float(v) for v in text.split())
    if len(values)!=size or not all(math.isfinite(v) for v in values):raise ValueError('3D 좌표/각도 값이 비정상입니다.')
    return values


def identity():return ((1.,0.,0.,0.),(0.,1.,0.,0.),(0.,0.,1.,0.),(0.,0.,0.,1.))


def multiply(a,b):return tuple(tuple(sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)) for i in range(4))


def point(matrix,p):return tuple(sum(matrix[i][j]*p[j] for j in range(3))+matrix[i][3] for i in range(3))


def axis_rotation(axis,angle):
    length=math.sqrt(sum(v*v for v in axis))
    if length<1e-12:raise ValueError('관절 회전축은 0 벡터일 수 없습니다.')
    x,y,z=(v/length for v in axis);c=math.cos(angle);s=math.sin(angle);t=1-c
    return ((t*x*x+c,t*x*y-s*z,t*x*z+s*y,0.),(t*x*y+s*z,t*y*y+c,t*y*z-s*x,0.),
            (t*x*z-s*y,t*y*z+s*x,t*z*z+c,0.),(0.,0.,0.,1.))


def transform(xyz=(0.,0.,0.),rpy=(0.,0.,0.)):
    rotation=multiply(multiply(axis_rotation((0,0,1),rpy[2]),axis_rotation((0,1,0),rpy[1])),axis_rotation((1,0,0),rpy[0]))
    return tuple(tuple(xyz[i] if j==3 and i<3 else rotation[i][j] for j in range(4)) for i in range(4))


def box(size):
    x,y,z=(v/2 for v in size)
    vertices=[(-x,-y,-z),(x,-y,-z),(x,y,-z),(-x,y,-z),(-x,-y,z),(x,-y,z),(x,y,z),(-x,y,z)]
    return [tuple(vertices[i] for i in face) for face in [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]]


def cylinder(radius,length,steps=12):
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


def load_mesh(path,max_faces=3000):
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
    else:raise ValueError('미리보기 메시 형식: STL / OBJ. STEP/IGES는 STL/OBJ로 내보내세요.')
    if not faces or not all(math.isfinite(v) for face in faces for p in face for v in p):raise ValueError('유효한 3D 면이 없습니다.')
    count=len(faces)
    return MeshAsset(path.name,simplify_faces(faces,max_faces),count)


class MeshAsset:
    def __init__(self,name,faces,total=None):self.name=name;self.faces=faces;self.total=total or len(faces)
    def draw_faces(self,world=identity(),scale=(1,1,1),color='#b7c4d3'):
        return [(tuple(point(world,tuple(p[i]*scale[i] for i in range(3))) for p in face),color,self.name) for face in self.faces]


class RobotDescription:
    def __init__(self,name,links,joints,roots):
        self.name,self.links,self.joints,self.roots=name,links,joints,roots
        self.warnings=[];self.synthetic=False

    @classmethod
    def load(cls,path,asset_root=None):
        path=Path(path)
        if path.stat().st_size>8*1024*1024:raise ValueError('URDF 파일이 너무 큽니다.')
        try:root=ET.fromstring(path.read_text(encoding='utf-8-sig'))
        except ET.ParseError as error:raise ValueError('URDF XML 형식 오류: '+str(error)) from error
        if root.tag!='robot':raise ValueError('robot 요소를 포함한 URDF가 필요합니다. Xacro는 URDF로 변환하세요.')
        links={};joints=[];warnings=[]
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
                        if mesh.total>len(mesh.faces):warnings.append(f'{name}: 메시 단순화 {mesh.total} → {len(mesh.faces)}면')
                    else:raise ValueError('지원하지 않는 geometry입니다.')
                except (ValueError,OSError,UnicodeError,TypeError) as error:
                    warnings.append(f'{name}: {error}');continue
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
        model=cls(root.get('name',path.stem),links,joints,roots);model.warnings=warnings
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

    def draw_faces(self,positions,world=identity()):
        result=[]
        for name,pose in self.link_transforms(positions,world).items():
            for faces,local,color in self.links[name]:
                matrix=multiply(pose,local)
                result.extend((tuple(point(matrix,p) for p in face),color,name) for face in faces)
        return result
