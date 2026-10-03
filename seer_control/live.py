"""Documented API operations. No automatic retries of robot commands."""
import math
import re
from .transport import ReadOnlyClient, MAX_FILE_PAYLOAD

COMMANDS = {'navigate': (19206,3051), 'pause': (19206,3001),
            'resume': (19206,3002), 'cancel': (19206,3003),
            'relocate': (19205,2002), 'confirm_loc': (19205,2003), 'cancel_reloc': (19205,2004),
            'motion': (19205,2010), 'stop_motion': (19205,2000),
            'load_map': (19205,2022), 'mode': (19207,4000),
            'lock_control': (19207,4005), 'unlock_control': (19207,4006),
            'slam_start': (19210,6100), 'slam_stop': (19210,6101)}
TASKS = ['NONE','WAITING','RUNNING','SUSPENDED','COMPLETED','FAILED','CANCELED']

class SeerClient(ReadOnlyClient):
    def query(self, api, payload=None):
        if not (1000 <= int(api) <= 1999):
            raise ValueError('상태 조회 API 범위를 벗어났습니다.')
        return self.request(dict(port=19204,request_type=api,response_type=api+10000,payload=payload))

    def command(self, name, payload=None):
        port, api = COMMANDS[name]
        if name=='load_map' and (not isinstance(payload,dict) or not isinstance(payload.get('map_name'),str) or not re.fullmatch(r'[A-Za-z0-9_-]+',payload['map_name'])):
            raise ValueError('문서 1.2.1의 지도 이름은 영문/숫자/_/-만 지원합니다.')
        if name=='navigate' and (not isinstance(payload,dict) or not isinstance(payload.get('id'),str) or not payload['id']):
            raise ValueError('로봇의 유효한 목적지 ID가 필요합니다.')
        if name in ('lock_control','unlock_control'):
            if not isinstance(payload,dict) or not isinstance(payload.get('nick_name'),str) or not payload['nick_name'].strip():
                raise ValueError(f'{4005 if name=="lock_control" else 4006} 제어권 API에는 nick_name이 필요합니다.')
        if name=='motion':
            if not isinstance(payload,dict):
                raise ValueError('수동 주행 속도 명령이 필요합니다.')
            vals=[]
            for key in ('vx','vy','w'):
                value=payload.get(key,0.0)
                if type(value) not in (int,float) or not math.isfinite(value):
                    raise ValueError('수동 주행 속도 값이 비정상입니다.')
                vals.append(float(value))
            if abs(vals[0])>.3 or abs(vals[1])>.3 or abs(vals[2])>.6:
                raise ValueError('수동 주행 제한: |vx/vy| ≤ 0.30 m/s, |w| ≤ 0.60 rad/s')
        # Control/navigation firmware variants may return a response API number that differs
        # from request+10000. Keep frame/sequence validation, but accept the controller's
        # actual response type and expose it in _response_type for diagnostics.
        return self.request(dict(port=port,request_type=api,response_type=api+10000,payload=payload,
                                 accept_any_response=True))

    def download_map(self, map_name):
        if not isinstance(map_name,str) or not map_name.strip():
            raise ValueError('다운로드할 지도 이름이 필요합니다.')
        # SEER config API 4011: request {\"map_name\": ...}, response body is the map JSON.
        return self.request(dict(port=19207,request_type=4011,response_type=14011,
                                 payload={'map_name':map_name},max_payload=MAX_FILE_PAYLOAD))

    def upload_map(self, map_data):
        """SEER config API 4010. Request body is the complete JSON SMAP."""
        if not isinstance(map_data,dict) or not isinstance(map_data.get('header'),dict):
            raise ValueError('4010 업로드에는 header가 포함된 전체 SMAP JSON이 필요합니다.')
        return self.request(dict(port=19207,request_type=4010,response_type=14010,
                                 payload=map_data,accept_any_response=True,max_payload=MAX_FILE_PAYLOAD))

    def snapshot(self):
        state, raw = super().snapshot()
        for k in ('x','y','theta'):
            if type(state.get(k)) not in (float,int) or not math.isfinite(state[k]):
                raise ValueError('위치 필드 누락/비정상: '+k)
        status = state.get('task_status')
        state['task'] = TASKS[status] if type(status) is int and 0<=status<len(TASKS) else 'UNKNOWN'
        state['mode'] = {0:'MANUAL',1:'AUTO'}.get(state.get('mode'),'UNKNOWN')
        state['safety'] = ('E-STOP' if state.get('emergency') is True else
                           'BLOCKED' if state.get('blocked') is True else
                           'CLEAR' if state.get('blocked') is False and state.get('emergency') is False else 'UNKNOWN')
        return state, raw


def points(value):
    if not isinstance(value,list): return []
    return [(float(p[0]),float(p[1])) for p in value if isinstance(p,(list,tuple)) and len(p)>=2
            and all(type(v) in (int,float) and math.isfinite(v) for v in p[:2])]



def extract_laser_points(payload, pose=None, prefer_world=False):
    """Parse SEER laser data into map/world points.

    Important for firmware that exposes ``lasers`` as a list of multiple laser
    devices/scans: collect *all* valid scans instead of returning the first one.
    This keeps 360-degree coverage stable when one side is occluded.
    """
    def finite(v):
        return type(v) in (int, float) and math.isfinite(v)

    def xy_array(arr):
        out=[]
        if not isinstance(arr,list): return out
        for item in arr:
            if isinstance(item,(list,tuple)) and len(item)>=2 and finite(item[0]) and finite(item[1]):
                out.append((float(item[0]),float(item[1])))
            elif isinstance(item,dict):
                x=item.get('x'); y=item.get('y')
                if finite(x) and finite(y): out.append((float(x),float(y)))
        return out

    def polar_array(arr):
        out=[]
        if not isinstance(arr,list): return out
        for item in arr:
            if not isinstance(item,dict): continue
            a=item.get('angle',item.get('theta'))
            r=item.get('range',item.get('distance',item.get('dist')))
            if finite(a) and finite(r) and float(r)>0:
                out.append((float(r)*math.cos(float(a)),float(r)*math.sin(float(a))))
        return out

    def ranges_obj(obj):
        ranges=obj.get('ranges') if isinstance(obj,dict) else None
        amin=obj.get('angle_min',obj.get('min_angle')) if isinstance(obj,dict) else None
        inc=obj.get('angle_increment',obj.get('angle_step',obj.get('resolution'))) if isinstance(obj,dict) else None
        if not (isinstance(ranges,list) and finite(amin) and finite(inc)): return []
        rmin=obj.get('range_min',0.02); rmax=obj.get('range_max',100.0)
        rmin=float(rmin) if finite(rmin) else 0.02; rmax=float(rmax) if finite(rmax) else 100.0
        out=[]
        for i,r in enumerate(ranges):
            if finite(r) and rmin <= float(r) <= rmax:
                a=float(amin)+i*float(inc)
                out.append((float(r)*math.cos(a),float(r)*math.sin(a)))
        return out

    def transform_local(pts, local_pose=None):
        use_pose = local_pose if local_pose is not None else pose
        if use_pose is None: return pts
        try:
            rx,ry,theta=map(float,use_pose); ct,st=math.cos(theta),math.sin(theta)
            return [(rx+ct*x-st*y, ry+st*x+ct*y) for x,y in pts]
        except Exception:
            return pts

    def median(values):
        if not values: return float('inf')
        v=sorted(values); n=len(v)
        return v[n//2] if n%2 else (v[n//2-1]+v[n//2])/2.0

    def normalize_cartesian(pts, field_name, path):
        if not pts:
            return pts, ''
        src=(path+'.'+field_name).strip('.') if field_name else (path or 'array')
        if field_name == 'laser_beams':
            return pts, src+'@WORLD'
        if pose is None:
            return pts, src+'@RAW'
        try:
            rx,ry,theta=map(float,pose)
        except Exception:
            return pts, src+'@RAW'
        sample=pts[::max(1,len(pts)//256)]
        raw_d=median([math.hypot(x-rx,y-ry) for x,y in sample])
        local_d=median([math.hypot(x,y) for x,y in sample])
        if raw_d <= local_d*1.25 + 0.35:
            return pts, src+f'@WORLD(auto:{raw_d:.2f}/{local_d:.2f})'
        return transform_local(pts), src+f'@LOCAL->WORLD(auto:{raw_d:.2f}/{local_d:.2f})'

    direct_names=('laser_beams','laser_points','points','point_list','scan_points','beams','beam_points','cloud','pointcloud','scan_data')

    def extract_one(root, root_path=''):
        queue=[(root_path,root)]
        seen=set()
        while queue:
            path,obj=queue.pop(0)
            oid=id(obj)
            if oid in seen: continue
            seen.add(oid)
            if isinstance(obj,dict):
                for key in direct_names:
                    arr=obj.get(key)
                    pts=xy_array(arr)
                    if pts:
                        return normalize_cartesian(pts,key,path)
                    pol=polar_array(arr)
                    if pol:
                        return transform_local(pol),(path+'.'+key).strip('.')+'@POLAR_LOCAL->WORLD'
                pts=ranges_obj(obj)
                if pts:
                    return transform_local(pts),(path+'.ranges').strip('.')+'@RANGES_LOCAL->WORLD'
                for key,child in obj.items():
                    if isinstance(child,(dict,list)) and len(path.split('.'))<7:
                        # Do not descend into sibling lasers here; caller aggregates them.
                        if key == 'lasers' and isinstance(child,list):
                            continue
                        queue.append(((path+'.'+str(key)).strip('.'),child))
            elif isinstance(obj,list):
                pts=xy_array(obj)
                if pts:
                    leaf=path.rsplit('.',1)[-1] if path else 'array'
                    return normalize_cartesian(pts,leaf,path.rsplit('.',1)[0] if '.' in path else '')
                pol=polar_array(obj)
                if pol:
                    return transform_local(pol),(path or 'polar_array')+'@POLAR_LOCAL->WORLD'
                for i,child in enumerate(obj):
                    if isinstance(child,(dict,list)) and len(path.split('.'))<8:
                        queue.append((f'{path}[{i}]' if path else f'[{i}]',child))
        return [],''

    # Firmware variant used by the user's robot: multiple scans/devices under lasers[].
    # Aggregate every valid member instead of returning only member 0.
    if isinstance(payload,dict) and isinstance(payload.get('lasers'),list):
        lasers=payload.get('lasers')
        # Sometimes ``lasers`` itself is already an XY array. Handle that first.
        direct=xy_array(lasers)
        if direct:
            return normalize_cartesian(direct,'lasers','')
        merged=[]; sources=[]
        for i,item in enumerate(lasers):
            pts,src=extract_one(item,f'lasers[{i}]')
            if pts:
                merged.extend(pts)
                sources.append(src or f'lasers[{i}]')
        if merged:
            # Keep source compact for the GUI footer.
            return merged, 'MERGED['+','.join(sources[:4])+('…' if len(sources)>4 else '')+']'

    return extract_one(payload,'')


def extract_laser_scans(payload, pose=None, mount_overrides=None):
    """Return per-laser world-frame scans with the correct sensor origin.

    Newer RBK firmware may expose ``lasers[]`` where each device contains polar
    ``beams`` plus its mounting pose.  Treating every beam as if it originated at
    the chassis origin shifts/rotates the cloud and does not match RoboShop.

    Result: list of {source, origin:(x,y), points:[(x,y)], mount:(x,y,yaw)|None}.
    """
    def finite(v): return type(v) in (int,float) and math.isfinite(v)
    def pose3(obj):
        if not isinstance(obj,dict): return None
        wrappers=('install_info','installInfo','pose','position','offset','transform','laser_pose','laserPose','sensor_pose','sensorPose','mount_pose','mountPose','install_pose','installPose')
        candidates=[obj]
        for k in wrappers:
            if isinstance(obj.get(k),dict): candidates.insert(0,obj[k])
        for c in candidates:
            x=c.get('x',c.get('dx')); y=c.get('y',c.get('dy'))
            a=c.get('yaw',c.get('theta',c.get('r',c.get('dir',c.get('angle')))))
            if finite(x) and finite(y):
                av=float(a) if finite(a) else 0.0
                if abs(av) > (2*math.pi + 0.25):
                    av=math.radians(av)
                return (float(x),float(y),av)
        return None
    def beam_local(arr, owner=None):
        """Convert polar beams to *obstacle hit* points only.

        Mapping must not treat no-return/max-range beams as obstacles. Some RBK
        firmware emits a full 360-degree array where missing returns are encoded
        as range_max (or a repeated maximum sentinel). RoboShop does not draw
        those as wall points, so filter them here before world conversion.
        """
        vals=[]
        if not isinstance(arr,list): return [], {'raw':0,'hits':0,'dropped':0,'min':None,'max':None,'cutoff':None}
        for b in arr:
            if not isinstance(b,dict):
                continue
            a=b.get('angle',b.get('theta'))
            r=b.get('range',b.get('distance',b.get('dist')))
            valid=b.get('valid', True)
            if valid is False:
                continue
            if finite(a) and finite(r) and float(r)>0:
                av=float(a)
                # RBK lasers[].beams on this controller reports angles in degrees
                # (e.g. -120, -119.5 ...), while some variants use radians.
                if abs(av) > (2*math.pi + 0.25):
                    av=math.radians(av)
                vals.append((av,float(r)))
        if not vals:
            return [], {'raw':0,'hits':0,'dropped':0,'min':None,'max':None,'cutoff':None}
        ranges=[r for _,r in vals]
        explicit_max=None
        if isinstance(owner,dict):
            for k in ('range_max','max_range','rangeMax','maxRange','laser_max_range','max_distance','maxDistance'):
                v=owner.get(k)
                if finite(v) and float(v)>0:
                    explicit_max=float(v); break
        rmax=max(ranges); rmin=min(ranges)
        cutoff=None
        if explicit_max is not None:
            # Keep a small margin below the configured maximum. Equality/near-
            # equality normally means "no return" rather than a physical wall.
            cutoff=max(0.02, explicit_max-max(0.03,explicit_max*0.005))
        else:
            # Infer the common no-return sentinel only when the upper plateau is
            # repeated often enough. This avoids deleting a genuine isolated
            # far obstacle.
            tol=max(0.03,rmax*0.003)
            plateau=sum(1 for r in ranges if r>=rmax-tol)
            if plateau>=max(8,int(len(ranges)*0.03)):
                cutoff=rmax-tol
        out=[]; dropped=0
        for a,r in vals:
            if r<0.02:
                dropped+=1; continue
            if cutoff is not None and r>=cutoff:
                dropped+=1; continue
            out.append((r*math.cos(a),r*math.sin(a)))
        return out, {'raw':len(vals),'hits':len(out),'dropped':dropped,
                     'min':rmin,'max':rmax,'cutoff':cutoff,'explicit_max':explicit_max}
    def world_from_robot(local_xy):
        if pose is None: return local_xy
        rx,ry,ra=map(float,pose); c=math.cos(ra); d=math.sin(ra)
        x,y=local_xy
        return (rx+c*x-d*y, ry+d*x+c*y)
    scans=[]
    # Official NetProtocol 1101/1009-compatible world-frame laser points.
    # SEER documents `laser_beams` as [x,y] points already expressed in the
    # map/world coordinate system. Prefer it over firmware-specific lasers[].
    if isinstance(payload,dict):
        official=payload.get('laser_beams')
        if isinstance(official,list):
            pts=[]
            for item in official:
                if isinstance(item,(list,tuple)) and len(item)>=2 and finite(item[0]) and finite(item[1]):
                    pts.append((float(item[0]),float(item[1])))
                elif isinstance(item,dict) and finite(item.get('x')) and finite(item.get('y')):
                    pts.append((float(item['x']),float(item['y'])))
            if pts:
                origin=tuple(map(float,pose[:2])) if pose is not None else (0.0,0.0)
                return [{'source':'laser_beams@WORLD_OFFICIAL','origin':origin,'points':pts,'mount':None,
                         'official_world':True,'stats':{'raw':len(pts),'hits':len(pts),'dropped':0}}]
    if isinstance(payload,dict) and isinstance(payload.get('lasers'),list):
        for i,item in enumerate(payload['lasers']):
            if not isinstance(item,dict): continue
            mount=pose3(item)
            if mount is None and isinstance(mount_overrides,(list,tuple)) and i < len(mount_overrides):
                ov=mount_overrides[i]
                if isinstance(ov,(list,tuple)) and len(ov)>=3 and all(finite(v) for v in ov[:3]):
                    mount=(float(ov[0]),float(ov[1]),float(ov[2]))
            pol=[]; field=''; stats=None
            for key in ('beams','beam_points','scan_data'):
                pol,stats=beam_local(item.get(key),item)
                # A valid laser may contain only max-range/no-return beams. Keep
                # looking only if the field itself was absent.
                if isinstance(item.get(key),list):
                    field=key; break
            if pol:
                sx,sy,sa=mount if mount is not None else (0.0,0.0,0.0)
                ca,sa_s=math.cos(sa),math.sin(sa)
                base_pts=[(sx+ca*x-sa_s*y, sy+sa_s*x+ca*y) for x,y in pol]
                world_pts=[world_from_robot(q) for q in base_pts]
                origin=world_from_robot((sx,sy))
                scans.append({'source':f'lasers[{i}].{field}','origin':origin,'points':world_pts,'mount':mount,'local_points':pol,'stats':stats or {}})
                continue
            elif field:
                # Field exists but every beam was filtered as no-return/max-range.
                origin=world_from_robot((mount[0],mount[1])) if mount is not None else (tuple(map(float,pose[:2])) if pose is not None else (0.0,0.0))
                scans.append({'source':f'lasers[{i}].{field}','origin':origin,'points':[],'mount':mount,'local_points':[],'stats':stats or {}})
                continue
            pts,src=extract_laser_points(item,pose=pose,prefer_world=False)
            if pts:
                origin=world_from_robot((mount[0],mount[1])) if mount is not None else (tuple(map(float,pose[:2])) if pose is not None else (0.0,0.0))
                scans.append({'source':src or f'lasers[{i}]','origin':origin,'points':pts,'mount':mount})
        if scans: return scans
    pts,src=extract_laser_points(payload,pose=pose,prefer_world=True)
    if pts:
        origin=tuple(map(float,pose[:2])) if pose is not None else (0.0,0.0)
        return [{'source':src or 'laser','origin':origin,'points':pts,'mount':None}]
    return []

def laser_points(value, pose=None):
    pts,_ = extract_laser_points(value, pose=pose, prefer_world=False)
    return pts
