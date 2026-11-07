"""LiDAR-assisted XY/yaw placement of equipment; no robot commands."""
import math
from .geometry3d import transform, point


def context(app):
    return dict(host=app.host.get().strip(),map=str(app.current_robot_map or app.map.name),
                alignment=list(app.lidar_map_alignment),mounts=[list(m) if m is not None else None for m in getattr(app,'lidar_mount_overrides',[])])


def signature(app, view):
    asset=view.cad_asset
    return dict(context=context(app),origin=list(view.cad_origin),scale=view.scales['cad'],
                asset=getattr(asset,'name',''),faces=len(getattr(asset,'faces',())),
                path=getattr(app,'spatial_assets',{}).get('cad',{}).get('path',''))


def confirmed(app, view):
    record=getattr(view,'cad_placement_confirmation',None)
    return (isinstance(record,dict) and record.get('source')=='real_lidar_operator_confirmed'
            and record.get('signature')==signature(app,view))


def section(asset, origin, scale, height=.1965):
    # A horizontal scan cannot measure equipment Z, roll or pitch. Keep these
    # authored values fixed and fit only the section's XY translation and yaw.
    pose=transform((0,0,origin[2]),(origin[3],origin[4],0))
    hits=set()
    for face in asset.faces:
        vertices=[point(pose,tuple(v*scale for v in p)) for p in face]
        for a,b in zip(vertices,vertices[1:]+vertices[:1]):
            da,db=a[2]-height,b[2]-height
            if abs(da)<1e-8 and abs(db)<1e-8:
                hits.update(((round(a[0],4),round(a[1],4)),(round(b[0],4),round(b[1],4))))
            elif da*db<=0 and abs(da-db)>1e-12:
                t=da/(da-db)
                hits.add((round(a[0]+t*(b[0]-a[0]),4),round(a[1]+t*(b[1]-a[1]),4)))
    points=sorted(hits)
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    if len(points)<3:raise ValueError('LiDAR 높이에서 설비 외곽을 찾지 못했습니다. CAD 높이를 확인하세요.')
    lower=[];upper=[]
    for p in points:
        while len(lower)>=2 and cross(lower[-2],lower[-1],p)<=0:lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper)>=2 and cross(upper[-2],upper[-1],p)<=0:upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]


def placed(outline, pose):
    x,y,a=pose;c,s=math.cos(a),math.sin(a)
    return [(x+c*u-s*v,y+s*u+c*v) for u,v in outline]


def distance_segment(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/max(1e-12,dx*dx+dy*dy)))
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


def quality(outline, cloud, pose):
    polygon=placed(outline,pose);edges=list(zip(polygon,polygon[1:]+polygon[:1]))
    xs,ys=zip(*polygon)
    nearby=[p for p in cloud if min(xs)-.3<=p[0]<=max(xs)+.3 and min(ys)-.3<=p[1]<=max(ys)+.3]
    nearby=nearby[::max(1,len(nearby)//300)]
    distances=[min(distance_segment(p,a,b) for a,b in edges) for p in nearby]
    matched=[d for d in distances if d<=.12]
    coverage=len(matched)/max(1,len(distances))
    error=math.sqrt(sum(d*d for d in matched)/len(matched)) if matched else 1.
    ok=len(matched)>=12 and coverage>=.35 and error<=.07
    return dict(error=error,coverage=coverage,matches=len(matched),ok=ok,
                score=error+.2*(1-coverage)+.1*max(0,12-len(matched)))


def refine(outline, cloud, initial):
    # Bounded local search, seeded by the operator. A partial LiDAR outline
    # cannot uniquely identify a whole machine among unrelated nearby walls.
    pose=tuple(initial)
    for step,angle in ((.1,math.radians(5)),(.03,math.radians(1)),(.01,math.radians(.3))):
        seed=pose;candidates=[]
        for dx in (-step,0,step):
            for dy in (-step,0,step):
                for da in (-angle,0,angle):
                    p=(seed[0]+dx,seed[1]+dy,seed[2]+da)
                    candidates.append((quality(outline,cloud,p)['score'],p))
        pose=min(candidates)[1]
    return pose


def polygon_gap(first,second):
    """Shortest XY distance between convex outlines, zero for overlap."""
    if len(first)<3 or len(second)<3:raise ValueError('거리 계산 외곽이 부족합니다.')
    separated=False
    for polygon in (first,second):
        for a,b in zip(polygon,polygon[1:]+polygon[:1]):
            nx,ny=-(b[1]-a[1]),b[0]-a[0]
            p=[x*nx+y*ny for x,y in first];q=[x*nx+y*ny for x,y in second]
            if max(p)<min(q) or max(q)<min(p):separated=True
    if not separated:return 0.
    return min(distance_segment(p,a,b) for source,target in ((first,second),(second,first))
               for p in source for a,b in zip(target,target[1:]+target[:1]))


def body_outline(app,view):
    body=getattr(view,'body_asset',None);vertices=[]
    if body is not None and hasattr(body,'links'):
        vertices=[point(local,p) for visuals in body.links.values() for faces,local,_ in visuals for face in faces for p in face]
    if vertices:
        scale=view.scales.get('amr',1.)
        lo=[min(p[i] for p in vertices)*scale for i in range(2)]
        hi=[max(p[i] for p in vertices)*scale for i in range(2)]
    else:
        model=getattr(app.map,'robot_model',{})
        length=float(model.get('length',.95));width=float(model.get('width',.63))
        lo=[-length/2,-width/2];hi=[length/2,width/2]
    return [(lo[0],lo[1]),(hi[0],lo[1]),(hi[0],hi[1]),(lo[0],hi[1])]
