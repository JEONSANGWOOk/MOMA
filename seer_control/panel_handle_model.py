"""Photo-based KAHL-1057-B(R) appearance; dimensions are explicit SIM estimates."""
import json,math
from pathlib import Path
from .geometry3d import box,cylinder,transform,point

DEFAULT=dict(product='KAHL-1057-B(R)',dimension_source='photo_estimate',handle_length_mm=185.,
    head_radius_mm=18.,base_radius_mm=32.,socket_radius_mm=4.5,pin_radius_mm=2.4,
    barrel_radius_mm=11.,barrel_depth_mm=26.,key_outer_radius_mm=4.,key_inner_radius_mm=3.,
    key_shaft_mm=30.,insertion_mm=22.,key_bow_width_mm=28.,key_bow_length_mm=30.)


def settings(path=None):
    data=dict(DEFAULT)
    path=Path(path) if path else Path(__file__).resolve().parents[1]/'config/panel_handle.json'
    if path.exists():data.update(json.loads(path.read_text(encoding='utf-8')))
    for name,default in DEFAULT.items():
        if isinstance(default,float) and (type(data[name]) not in (int,float) or not math.isfinite(data[name]) or not .1<=data[name]<=500):
            raise ValueError('손잡이 SIM 치수 오류: '+name)
    if not data['pin_radius_mm']<data['key_inner_radius_mm']<data['key_outer_radius_mm']<data['socket_radius_mm']<data['barrel_radius_mm']:
        raise ValueError('원통형 키/구멍의 내외경 순서 오류')
    if not data['insertion_mm']<min(data['barrel_depth_mm']-1,data['key_shaft_mm']):raise ValueError('삽입 깊이보다 키/구멍이 길어야 합니다.')
    return data


def tube(outer,inner,length,steps=32):
    faces=[]
    for i in range(steps):
        a=i*math.tau/steps;b=(i+1)*math.tau/steps
        def p(r,angle,z):return (r*math.cos(angle),r*math.sin(angle),z)
        for z in (0.,-length):faces.append((p(outer,a,z),p(outer,b,z),p(inner,b,z),p(inner,a,z)))
        faces.append((p(outer,a,0),p(outer,a,-length),p(outer,b,-length),p(outer,b,0)))
        faces.append((p(inner,a,0),p(inner,b,0),p(inner,b,-length),p(inner,a,-length)))
    return faces


def moved(faces,t):return [tuple(point(t,p) for p in face) for face in faces]


def handle_meshes(c):
    """Socket frame: Z towards camera, Y up. Hollow bore remains unobstructed."""
    out=[];mm=.001
    out.append((moved(tube(c['base_radius_mm']*mm,.016,.007),transform((0,-.008,-.0195))), '#bac6ce','chrome_mount_plate'))
    # Tapered, rounded long handle below the keyhole, separate from the rotating plug.
    levels=[(-.025,.017,.003),(-.05,.016,.005),(-.10,.013,.006),(-(c['handle_length_mm']-22)*mm,.0065,.003)]
    rings=[[(rx*math.cos(i*math.tau/16),y,z+.006*math.sin(i*math.tau/16)) for i in range(16)] for y,rx,z in levels]
    shaft=[]
    for a,b in zip(rings,rings[1:]):
        shaft.extend((a[i],a[(i+1)%16],b[(i+1)%16],b[i]) for i in range(16))
    shaft.extend([tuple(reversed(rings[0])),tuple(rings[-1])]);out.append((shaft,'#cbd6de','chrome_tapered_handle'))
    # Ring-shaped upper shell, rounded cap, split seam and two side latch ears.
    out.append((tube(c['head_radius_mm']*mm,c['barrel_radius_mm']*mm,.019), '#dce4e9','chrome_handle_head'))
    for x in (-.026,.026):out.append((moved(box((.018,.009,.009)),transform((x,.004,-.014),rpy=(0,0,.25 if x>0 else -.25))), '#b2c1cb','side_latch_ear'))
    return out


def socket_meshes(c,turn=0.):
    out=[(tube(c['barrel_radius_mm']*.001,c['socket_radius_mm']*.001,c['barrel_depth_mm']*.001),'#e5ecf0','tubular_lock_bezel'),
         (moved(cylinder(c['pin_radius_mm']*.001,(c['barrel_depth_mm']-1)*.001,24),transform((0,0,-(c['barrel_depth_mm']-1)*.0005))), '#8798a6','lock_center_pin')]
    # Stylized circular recesses only; the actual key code/pin layout is unknown.
    for i in range(8):
        a=i*math.tau/8;r=(c['socket_radius_mm']+1.3)*.001
        out.append((moved(cylinder(.00065,.0004,8),transform((r*math.cos(a),r*math.sin(a),.00015))), '#35424b','reference_pin_recess'))
    return [(moved(faces,transform(rpy=(0,0,-turn))),color,name) for faces,color,name in out]


def key_meshes(c):
    out=[(tube(c['key_outer_radius_mm']*.001,c['key_inner_radius_mm']*.001,c['key_shaft_mm']*.001),'#cfdae2','hollow_tubular_key')]
    # Flat silver bow with a ring hole, behind the cylindrical shaft.
    center=-(c['key_shaft_mm']+20)*.001;rx=c['key_bow_width_mm']*.0005;rz=c['key_bow_length_mm']*.0005
    faces=[]
    for i in range(24):
        a=i*math.tau/24;b=(i+1)*math.tau/24
        def p(angle,inside,y):return ((.004 if inside else rx)*math.cos(angle),y,center+(.005 if inside else rz)*math.sin(angle))
        for y in (-.0015,.0015):faces.append((p(a,False,y),p(b,False,y),p(b,True,y),p(a,True,y)))
        faces.append((p(a,False,-.0015),p(a,False,.0015),p(b,False,.0015),p(b,False,-.0015)))
        faces.append((p(a,True,-.0015),p(b,True,-.0015),p(b,True,.0015),p(a,True,.0015)))
    out.append((faces,'#becbd4','silver_key_bow'))
    out.append((moved(box((.008,.003,.012)),transform((0,0,-(c['key_shaft_mm']+5)*.001))), '#c5d1d8','key_neck'))
    return out
