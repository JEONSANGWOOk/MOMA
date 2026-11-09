"""Conservative mesh/OBB visual clearance checks. Never controls the robot."""
import math
import numpy as np
from .geometry3d import transform,multiply,point,axis_rotation
from .arm_physics import inverse


def triangles_hit_box(triangles,half):
    """Triangle vs axis-aligned box SAT, in box-local coordinates."""
    t=np.asarray(triangles,dtype=float)
    if not len(t):return False
    h=np.asarray(half);live=np.all(t.min(axis=1)<=h,axis=1)&np.all(t.max(axis=1)>=-h,axis=1)
    t=t[live]
    if not len(t):return False
    # Fully vectorized all axes; degenerate axes impose no separation.
    edges=np.roll(t,-1,axis=1)-t
    all_axes=np.concatenate([np.cross(edges[:,0],edges[:,1])[:,None,:]]+
        [np.cross(edges[:,i,None,:],np.eye(3)[None,:,:]) for i in range(3)],axis=1)
    projections=np.einsum('nvc,nac->nav',t,all_axes)
    radii=np.einsum('nac,c->na',np.abs(all_axes),h)
    overlap=(projections.min(axis=2)<=radii+1e-10)&(projections.max(axis=2)>=-radii-1e-10)
    return bool(np.any(np.all(overlap,axis=1)))


def bounds_for_arm(asset):
    cached=getattr(asset,'door_collision_bounds',None)
    if cached is not None:return cached
    result=[]
    for name,visuals in asset.links.items():
        for index,(faces,local,color) in enumerate(visuals):
            faces=getattr(asset,'full_visuals',{}).get((name,index),faces)
            if not faces:continue
            vertices=np.asarray(faces,dtype=float).reshape(-1,3)
            # GLB link visuals use identity local transforms; other assets may not.
            lo=vertices.min(axis=0);hi=vertices.max(axis=0)
            center=(lo+hi)/2;size=np.maximum(.001,hi-lo)
            result.append((name,multiply(local,transform(tuple(center))),tuple(size)))
    asset.door_collision_bounds=result
    return result


class DoorClearance:
    def __init__(self,part,scale=1.):
        faces=list(part['asset'].faces)+list(getattr(part['asset'],'transparent_faces',()))
        rows=[]
        for face in faces:
            for i in range(1,len(face)-1):rows.append((face[0],face[i],face[i+1]))
        self.triangles=np.asarray(rows,dtype=float)*scale
        self.lo=self.triangles.min(axis=1);self.hi=self.triangles.max(axis=1)
        self.bounds_lo=self.lo.min(axis=0);self.bounds_hi=self.hi.max(axis=0)
        self.pivot=tuple(v*scale for v in part['pivot']);self.sign=part['sign']
    def collisions(self,angle,boxes,margin=.002,first_only=False):
        p=self.pivot
        delta=multiply(multiply(transform(p),axis_rotation((0,0,1),angle*self.sign)),transform(tuple(-v for v in p)))
        inverse_door=inverse(delta);hits=[]
        for name,matrix,size in boxes:
            # Robot box in the unrotated CAD door frame.
            m=multiply(inverse_door,matrix);half=np.asarray(size)/2+margin
            corners=np.asarray([point(m,(x*half[0],y*half[1],z*half[2])) for x in (-1,1) for y in (-1,1) for z in (-1,1)])
            lo=corners.min(axis=0);hi=corners.max(axis=0)
            if np.any(self.bounds_lo>hi) or np.any(self.bounds_hi<lo):continue
            select=np.all(self.lo<=hi,axis=1)&np.all(self.hi>=lo,axis=1)
            triangles=self.triangles[select]
            if not len(triangles):continue
            inv=np.asarray(inverse(m));local=triangles@inv[:3,:3].T+inv[:3,3]
            if triangles_hit_box(local,half):
                hits.append(name)
                if first_only:return hits
        return hits
    def resolve(self,angle,boxes,maximum=math.radians(160)):
        hits=self.collisions(angle,boxes,first_only=True)
        if not hits:return angle,[]
        candidate=angle
        while candidate<maximum-1e-9:
            candidate=min(maximum,candidate+math.radians(2))
            if not self.collisions(candidate,boxes,first_only=True):return candidate,[]
        return angle,hits


def robot_boxes(view,poses,cad_inverse):
    boxes=[]
    for name,local,size in bounds_for_arm(view.arm_asset):
        if name in poses:boxes.append(('arm:'+name,multiply(cad_inverse,multiply(poses[name],local)),size))
    if 'J6' in poses:
        world=multiply(cad_inverse,multiply(poses['J6'],transform((0,0,.1))))
        scale=view.scales.get('tool',1.)
        for name,(lo,hi) in getattr(view.tool_asset,'collision_component_bounds',{}).items():
            center=tuple((lo[i]+hi[i])/2*scale for i in range(3));size=tuple(max(.001,(hi[i]-lo[i])*scale) for i in range(3))
            boxes.append(('tool:'+name,multiply(world,transform(center)),size))
    return boxes
