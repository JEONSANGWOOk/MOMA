"""UI-independent map and deterministic simulated robot. Units: m, rad, s."""
import heapq
import json
import math
from dataclasses import dataclass, asdict
from pathlib import Path
from .studio_core import DEFAULT_MODEL, validate_model, collision_reason, zone_limits, path_clearance, enabled


def number(v):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ValueError('좌표는 유한한 숫자여야 합니다.')
    return float(v)


class MapModel:
    def __init__(self, data):
        if data.get('format') != 'amr-console-map-v1':
            raise ValueError('지원 형식: amr-console-map-v1 JSON. SEER 원본 SMAP 변환은 별도 어댑터가 필요합니다.')
        self.name = str(data.get('name', 'Untitled'))
        self.nodes = {}
        for n in data['nodes']:
            key = str(n['id']).strip()
            if not key or key in self.nodes:
                raise ValueError('노드 ID가 비어 있거나 중복됩니다.')
            # Preserve every station option returned by RoboShop/1301/SMAP.
            # Older versions reduced a station to id/x/y/kind, which discarded
            # heading (r), spin and destination-specific navigation settings.
            node=dict(n)
            node.update(id=key, x=number(n['x']), y=number(n['y']),
                        kind=str(n.get('kind', 'station')))
            self.nodes[key] = node
        if not self.nodes:
            raise ValueError('노드가 한 개 이상 필요합니다.')
        self.edges = []
        for edge in data.get('edges', []):
            a, b = edge
            if a not in self.nodes or b not in self.nodes or a == b:
                raise ValueError('경로에 없는 노드 또는 자기 연결이 있습니다.')
            if [a, b] not in self.edges and [b, a] not in self.edges:
                self.edges.append([a, b])
        self.walls = []
        for w in data.get('walls', []):
            if len(w) != 4:
                raise ValueError('벽은 [x1,y1,x2,y2]입니다.')
            self.walls.append([number(v) for v in w])
        if 'path_records' in data:
            self.path_records=data['path_records']
            for rec in self.path_records:
                if rec.get('a') not in self.nodes or rec.get('b') not in self.nodes:
                    raise ValueError('Path에 없는 노드가 있습니다.')
        self.curves=[]
        self.area_records=data.get('area_records',[])
        self.virtual_walls=data.get('virtual_walls',[])
        self.obstacles=data.get('obstacles',[])
        self.cloud=data.get('cloud',[])
        self.robot_model=validate_model(data.get('robot_model',{}))
        for wall in self.virtual_walls:
            if not isinstance(wall,(list,tuple)) or len(wall)!=4:raise ValueError('가상벽은 [x1,y1,x2,y2]입니다.')
            for v in wall:number(v)
        for obs in self.obstacles:
            for key in ('x','y','radius'):number(obs[key])
            if obs['radius']<=0:raise ValueError('장애물 반경은 0보다 커야 합니다.')
        for area in self.area_records:
            if len(area.get('points',[]))<3:raise ValueError('영역 꼭짓점은 3개 이상 필요합니다.')
            for pt in area['points']:
                if len(pt)!=2:raise ValueError('영역 좌표는 [x,y]입니다.')
                for v in pt:number(v)
        for pt in self.cloud:
            if len(pt)!=2:raise ValueError('점군 좌표는 [x,y]입니다.')
            for v in pt:number(v)

    @classmethod
    def load(cls, path):
        with open(path, encoding='utf-8-sig') as f:
            return cls(json.load(f))

    def save(self, path):
        Path(path).write_text(json.dumps(self.data(), ensure_ascii=False, indent=2), encoding='utf-8')

    def data(self):
        data=dict(format='amr-console-map-v1', name=self.name, nodes=list(self.nodes.values()),
                  edges=self.edges, walls=self.walls)
        if hasattr(self,'path_records'):data['path_records']=self.path_records
        for key in ('area_records','virtual_walls','obstacles','cloud','robot_model'):
            if hasattr(self,key):data[key]=getattr(self,key)
        return data

    def distance(self, a, b):
        rec=next((r for r in getattr(self,'path_records',[]) if r.get('a')==a and r.get('b')==b),None)
        if rec:
            from .smap import path_record_geometry
            points=path_record_geometry(rec.get('raw',{}))
            if len(points)>1:return sum(math.dist(p,q) for p,q in zip(points,points[1:]))
        a, b = self.nodes[a], self.nodes[b]
        return math.hypot(a['x']-b['x'], a['y']-b['y'])

    def nearest(self, x, y):
        return min(self.nodes, key=lambda k: math.hypot(self.nodes[k]['x']-x, self.nodes[k]['y']-y))

    def route(self, start, goal):
        if start not in self.nodes or goal not in self.nodes:
            raise ValueError('노드가 존재하지 않습니다.')
        queue, seen = [(0, start, [start])], set()
        while queue:
            cost, key, path = heapq.heappop(queue)
            if key in seen:
                continue
            if key == goal:
                return path
            seen.add(key)
            records=getattr(self,'path_records',None)
            if records:
                outgoing=[(r.get('a'),r.get('b')) for r in records if r.get('a') and r.get('b')]
                for a,b in outgoing:
                    nxt=b if a==key else None
                    if nxt and nxt not in seen:
                        heapq.heappush(queue,(cost+self.distance(key,nxt),nxt,path+[nxt]))
            else:
                for a, b in self.edges:
                    nxt = b if a == key else a if b == key else None
                    if nxt and nxt not in seen:
                        heapq.heappush(queue, (cost+self.distance(key, nxt), nxt, path+[nxt]))
        raise ValueError(f'{start} → {goal}: 연결된 경로가 없습니다.')


@dataclass
class State:
    x: float = 2.0
    y: float = 2.0
    theta: float = 0.0
    battery: float = 82.0
    speed: float = 0.0
    mode: str = 'IDLE'
    last_node: str = 'LM1'
    target: str = ''
    charging: bool = False
    blocked: bool = False
    localization: float = 0.99
    motor: bool = True
    stopped: bool = False
    task: str = '대기'


class Simulator:
    def __init__(self, map_model):
        self.map = map_model
        first = next(iter(map_model.nodes.values()))
        self.state = State(x=first['x'], y=first['y'], last_node=first['id'])
        self.route = []
        self._waypoints = []
        self._segment_start = None
        self._segment_reverse = False
        self._arrival_align = False
        self._collision_blocked = False
        self.block_reason = ''
        self.detected_obstacle=''
        self.obstacle_policy='wait'
        self.collision_radius=0.
        self.avoidance_status=''
        self._avoid_time=0.;self._avoid_next=0.
        self.applied_limits = dict(DEFAULT_MODEL)
        self.di = {i:False for i in range(64)}
        self.do = {i:False for i in range(64)}
        self.arm = dict(status='IDLE',operation='',progress=0.,pose='SAFE')
        self.auto_charge = dict(enabled=False,low=20.,high=80.,rate=2.,phase='IDLE')
        self._auto_charge_goal = None
        self._velocity = 0.
        self.v = self.w = self.lease = 0.0
        self.mapping = False
        self.cloud = []

    def status(self):
        return asdict(self.state)

    def navigate(self, goal, route_nodes=None):
        self._ready()
        s = self.state
        if self.mapping:
            raise ValueError('맵 생성 중에는 수동 탐색만 가능합니다.')
        start = self.map.nearest(s.x, s.y)
        if route_nodes is None:
            planned=self.map.route(start,goal)
        else:
            from .route_planner import adjacency
            graph=adjacency(self.map);planned=list(route_nodes)
            if not planned or planned[0]!=start or planned[-1]!=goal:
                raise ValueError('계획 경로의 시작/목적지가 현재 로봇 위치와 다릅니다.')
            if any(a not in graph or not any(nxt==b for nxt,_,_ in graph[a]) for a,b in zip(planned,planned[1:])):
                raise ValueError('계획 경로에 연결되지 않은 구간이 있습니다.')
        at_start=len(planned)>1 and math.hypot(s.x-self.map.nodes[start]['x'],s.y-self.map.nodes[start]['y'])<1e-6
        self.route = planned[1:] if at_start else planned
        self._waypoints = []
        self._segment_start = start if at_start else None
        self._arrival_align = False
        self._velocity = 0.
        self._avoid_next=0.;self.avoidance_status=''
        self.v = self.w = self.lease = 0
        s.target, s.task, s.mode, s.charging = goal, f'이동 → {goal}', 'RUNNING', False

    def _ready(self):
        if self.arm['status']=='RUNNING' or self.arm.get('pose')!='SAFE':
            raise ValueError('로봇팔 작업 완료 후 safe_pose가 필요합니다.')
        if self.state.stopped or not self.state.motor:
            raise ValueError('정지 해제 및 모터 ON이 필요합니다.')
        if self.state.blocked:
            raise ValueError('장애물을 해제하세요.')

    def drive(self, v, w):
        self._ready()
        if self.route:
            raise ValueError('Task를 취소한 뒤 수동 조작하세요.')
        self.v, self.w, self.lease = max(-.3, min(.3, v)), max(-.6, min(.6, w)), .3
        self.state.charging = False
        self.state.mode = 'MANUAL'

    def stop(self, latch=False):
        self.route.clear()
        self._waypoints.clear()
        self._segment_start = None
        self._arrival_align = False
        self._velocity = 0.
        self.v = self.w = self.lease = 0
        s = self.state
        s.speed, s.target, s.mode, s.task = 0, '', 'STOPPED' if latch else 'IDLE', '취소됨'
        s.stopped = s.stopped or latch

    def command(self, name):
        s = self.state
        if name == 'stop':
            self.stop(True)
        elif name == 'cancel':
            self.stop()
        elif name == 'reset':
            s.stopped = False
            s.mode = 'IDLE'
        elif name == 'pause':
            self.v = self.w = 0
            if self.route:
                s.mode = 'PAUSED'
            s.speed = 0
        elif name == 'resume':
            self._ready()
            if self.route:
                s.mode = 'RUNNING'
        elif name == 'motor_off':
            self.stop()
            s.motor = False
        elif name == 'motor_on':
            s.motor = True
        elif name == 'charge':
            docks = [k for k, n in self.map.nodes.items() if n['kind'] == 'dock']
            if not docks:
                raise ValueError('충전 노드가 없습니다.')
            self.navigate(docks[0])
        elif name == 'mapping_start':
            self.stop()
            self._ready()
            self.mapping, self.cloud = True, []
        elif name == 'mapping_stop':
            self.stop()
            self.mapping = False
        elif name == 'obstacle':
            s.blocked = not s.blocked
            self.v = self.w = 0
        else:
            raise ValueError('알 수 없는 명령: '+name)

    def segment_geometry(self, start, goal):
        from .smap import path_record_geometry
        rec=next((r for r in getattr(self.map,'path_records',[])
                  if r.get('a')==start and r.get('b')==goal),None)
        if rec:
            return path_record_geometry(rec.get('raw',{})), rec
        n=self.map.nodes[goal]
        return [(n['x'],n['y'])], None

    def navigation_points(self):
        pts=[(self.state.x,self.state.y)]
        if not self.route:return pts
        if self._waypoints:pts.extend(self._waypoints)
        else:pts.extend(self.segment_geometry(self._segment_start,self.route[0])[0])
        for a,b in zip(self.route,self.route[1:]):pts.extend(self.segment_geometry(a,b)[0])
        return pts

    def _arrive(self,n,dt):
        s=self.state
        heading=n.get('r',n.get('angle'))
        if heading is not None and len(self.route)==1:
            heading=number(heading)
            delta=math.atan2(math.sin(heading-s.theta),math.cos(heading-s.theta))
            if enabled(n.get('spin')) and abs(delta)>1e-4:
                step=self.applied_limits['maxrot']*dt
                s.theta+=max(-step,min(step,delta));self._arrival_align=True
                s.speed=0;self._velocity=0
                return
            s.theta=heading
        self._arrival_align=False
        s.x,s.y,s.last_node=n['x'],n['y'],n['id']
        self._segment_start=n['id'];self.route.pop(0)
        if not self.route:
            s.mode,s.task,s.target='IDLE','완료','';s.speed=0;self._velocity=0
            s.charging=n['kind']=='dock'

    def _charge_tick(self):
        policy=self.auto_charge;s=self.state
        if not policy['enabled']:return
        if policy['phase']=='IDLE' and s.battery<=policy['low'] and not self.mapping and not s.stopped and s.motor and self.arm.get('pose')=='SAFE' and self.arm['status']!='RUNNING':
            docks=[k for k,n in self.map.nodes.items() if n['kind']=='dock' or n.get('className')=='ChargePoint']
            reachable=[]
            for k in docks:
                try:
                    route=self.map.route(self.map.nearest(s.x,s.y),k)
                    reachable.append((sum(self.map.distance(a,b) for a,b in zip(route,route[1:])),k))
                except ValueError:pass
            if reachable:
                self._auto_charge_goal=s.target
                self.navigate(min(reachable)[1]);policy['phase']='TO_DOCK'
        elif policy['phase']=='TO_DOCK' and s.charging:policy['phase']='CHARGING'
        elif policy['phase']=='CHARGING' and s.battery>=policy['high']:
            policy['phase']='IDLE';s.charging=False
            goal=self._auto_charge_goal;self._auto_charge_goal=None
            if goal and goal in self.map.nodes:self.navigate(goal)

    def tick(self, dt):
        dt = max(0, min(dt, .1))
        self._avoid_time+=dt
        s = self.state
        s.speed = 0
        self.detected_obstacle=''
        if self._collision_blocked:
            s.blocked=False;self._collision_blocked=False;self.block_reason=''
        self._charge_tick()
        if s.stopped or not s.motor or s.blocked:
            self.v = self.w = self.lease = 0
            self._velocity=0
            return
        if self.route and s.mode == 'RUNNING':
            n = self.map.nodes[self.route[0]]
            if self._arrival_align:
                self._arrive(n,dt)
                return
            if not self._waypoints:
                geometry,rec=self.segment_geometry(self._segment_start,self.route[0])
                self._waypoints=list(geometry)
                self._segment_reverse=bool(rec and str((rec.get('properties') or {}).get('direction',0))=='1')
            rec=next((r for r in getattr(self.map,'path_records',[]) if r.get('a')==self._segment_start and r.get('b')==self.route[0]),None)
            limits=dict(getattr(self.map,'robot_model',DEFAULT_MODEL))
            limits['radius']=max(limits['radius'],self.collision_radius)
            for key in ('maxspeed','maxacc','maxdec','maxrot'):
                value=(rec.get('properties') or {}).get(key) if rec else None
                if value is not None:
                    value=max(0,number(value))
                    if key=='maxrot':value=math.radians(value)
                    limits[key]=min(limits[key],value)
            limits=zone_limits(self.map,s.x,s.y,limits);self.applied_limits=limits
            distance_left=0.;prev=(s.x,s.y)
            for point in self._waypoints:distance_left+=math.dist(prev,point);prev=point
            distance_to_goal=distance_left+sum(self.map.distance(a,b) for a,b in zip(self.route,self.route[1:]))
            desired=min(limits['maxspeed'],math.sqrt(max(0,2*limits['maxdec']*distance_to_goal)))
            preview=[(s.x,s.y)]+[p for p in self._waypoints if math.dist((s.x,s.y),p)>1e-6]
            if len(preview)<3 and len(self.route)>1:
                outgoing,_=self.segment_geometry(self.route[0],self.route[1])
                preview.extend(p for p in outgoing if math.dist(preview[-1],p)>1e-6)
            if len(preview)>=3:
                a,b,c=preview[:3];ab=math.dist(a,b);bc=math.dist(b,c)
                angle1=math.atan2(b[1]-a[1],b[0]-a[0]);angle2=math.atan2(c[1]-b[1],c[0]-b[0])
                curvature=abs(math.atan2(math.sin(angle2-angle1),math.cos(angle2-angle1)))/max(.001,(ab+bc)/2)
                if curvature>1e-6:desired=min(desired,limits['maxrot']/curvature)
            if len(preview)>1:
                p=preview[1];heading=math.atan2(p[1]-s.y,p[0]-s.x)+(math.pi if self._segment_reverse else 0)
                delta_heading=math.atan2(math.sin(heading-s.theta),math.cos(heading-s.theta))
                if self._velocity<=1e-6 and abs(delta_heading)>limits['maxrot']*dt+.08:
                    s.theta+=max(-limits['maxrot']*dt,min(limits['maxrot']*dt,delta_heading))
                    self._velocity=0
                    return
            props=(rec.get('properties') or {}) if rec else {}
            stop_dist=max(.025,float(props.get('obsStopDist',.05) or .05))
            dec_dist=max(stop_dist,float(props.get('obsDecDist',.5) or .5))
            clearance,reason=path_clearance(self.map,(s.x,s.y),self._waypoints,limits['radius'],max(dec_dist,self._velocity**2/(2*max(.01,limits['maxdec']))+stop_dist))
            self.detected_obstacle=reason
            if reason:
                if self.obstacle_policy=='avoid' and self._avoid_time>=self._avoid_next:
                    from .avoidance import detour
                    self._avoid_next=self._avoid_time+1.
                    points=detour(self.map,(s.x,s.y),(n['x'],n['y']),limits['radius'])
                    if not points and len(self.route)>1:
                        final=self.map.nodes[self.route[-1]]
                        points=detour(self.map,(s.x,s.y),(final['x'],final['y']),limits['radius'])
                        if points:
                            self.route=[self.route[-1]];self._segment_start=None
                    if points:
                        self._waypoints=points[1:];self._segment_reverse=False
                        self.avoidance_status='우회 주행';self._velocity=0.;return
                    self.avoidance_status='우회 경로 없음 · 대기'
                elif self.obstacle_policy=='wait':self.avoidance_status='장애물 해제 대기'
                desired=min(desired,math.sqrt(2*limits['maxdec']*max(0,clearance-stop_dist)))
                if clearance<=stop_dist+.025:
                    self._collision_blocked=True;s.blocked=True;self.block_reason=reason;self._velocity=0
                    return
            old_velocity=self._velocity
            delta=desired-old_velocity
            self._velocity=old_velocity+max(-limits['maxdec']*dt,min(limits['maxacc']*dt,delta))
            remaining=min(distance_left,self._velocity*dt*limits.get('wheel_scale',1.))
            travel=remaining
            old_pose=(s.x,s.y,s.theta);old_points=list(self._waypoints)
            swept=[]
            while self._waypoints:
                x,y=self._waypoints[0]
                dx,dy=x-s.x,y-s.y;distance=math.hypot(dx,dy)
                if distance>1e-9:
                    angle=math.atan2(dy,dx)+(math.pi if self._segment_reverse else 0)
                    s.theta=math.atan2(math.sin(angle),math.cos(angle))
                if distance<=remaining+1e-9:
                    s.x,s.y=x,y;remaining=max(0,remaining-distance)
                    swept.append((s.x,s.y))
                    self._waypoints.pop(0)
                else:
                    s.x+=remaining*dx/distance;s.y+=remaining*dy/distance
                    swept.append((s.x,s.y))
                    remaining=0.
                    break
                reason=collision_reason(self.map,s.x,s.y,limits['radius'])
                if reason:
                    s.x,s.y,s.theta=old_pose;self._waypoints=old_points
                    self._collision_blocked=True;s.blocked=True;self.block_reason=reason
                    self._velocity=0;s.speed=0
                    return
            _,reason=path_clearance(self.map,old_pose[:2],swept,limits['radius'],sum(math.dist(a,b) for a,b in zip([old_pose[:2]]+swept,swept)))
            if reason:
                s.x,s.y,s.theta=old_pose;self._waypoints=old_points
                self._collision_blocked=True;s.blocked=True;self.block_reason=reason
                self._velocity=0;s.speed=0;return
            s.speed=(-1 if self._segment_reverse else 1)*self._velocity if remaining<travel else 0
            if not self._waypoints:
                self._arrive(n,dt)
                if not self.route:self.avoidance_status='목적지 도착'
        elif self.lease > 0:
            self.lease -= dt
            if self.lease <= 0:
                self.v = self.w = 0
                s.mode = 'IDLE'
            s.theta = math.atan2(math.sin(s.theta+self.w*dt), math.cos(s.theta+self.w*dt))
            old_xy=(s.x,s.y)
            scale=getattr(self.map,'robot_model',DEFAULT_MODEL)['wheel_scale']
            s.x += self.v*dt*scale*math.cos(s.theta)
            s.y += self.v*dt*scale*math.sin(s.theta)
            s.speed = self.v
            _,reason=path_clearance(self.map,old_xy,[(s.x,s.y)],max(getattr(self.map,'robot_model',DEFAULT_MODEL)['radius'],self.collision_radius),math.dist(old_xy,(s.x,s.y)))
            if reason:
                s.x,s.y=old_xy
                self._collision_blocked=True;s.blocked=True;self.block_reason=reason;s.speed=0
        else:
            self.v = self.w = 0
            if s.mode == 'MANUAL':
                s.mode = 'IDLE'
        s.battery = max(0, min(100, s.battery + (self.auto_charge['rate'] if s.charging else -.0005)*dt))
        if self.mapping:
            self.cloud.extend(self.scan()[::5])
            self.cloud = self.cloud[-15000:]

    def scan(self):
        """Ray cast against demo walls; this is synthetic data, not a sensor."""
        result = []
        cfg=getattr(self.map,'robot_model',DEFAULT_MODEL);pose=self.state
        c,s=math.cos(pose.theta),math.sin(pose.theta)
        x=pose.x+cfg['laser_x']*c-cfg['laser_y']*s
        y=pose.y+cfg['laser_x']*s+cfg['laser_y']*c
        walls=list(self.map.walls)+list(getattr(self.map,'virtual_walls',[]))
        for area in getattr(self.map,'area_records',[]):
            if enabled((area.get('properties') or {}).get('forbidden')):
                pts=area.get('points',[])
                walls.extend([*a,*b] for a,b in zip(pts,pts[1:]+pts[:1]))
        for i in range(120):
            a = self.state.theta+cfg['laser_yaw'] + i*math.tau/120
            dx, dy = math.cos(a), math.sin(a)
            best = 10.0
            for x1, y1, x2, y2 in walls:
                sx, sy = x2-x1, y2-y1
                den = dx*sy-dy*sx
                if abs(den) < 1e-9:
                    continue
                qx, qy = x1-x, y1-y
                t = (qx*sy-qy*sx)/den
                u = (qx*dy-qy*dx)/den
                if 0 < t < best and 0 <= u <= 1:
                    best = t
            for obs in getattr(self.map,'obstacles',[]):
                qx,qy=obs['x']-x,obs['y']-y
                projection=qx*dx+qy*dy
                disc=obs.get('radius',.25)**2-(qx*qx+qy*qy-projection*projection)
                if disc>=0:
                    hit=projection-math.sqrt(disc)
                    if 0<hit<best:best=hit
            if best < 10:
                result.append([x+best*dx, y+best*dy])
        return result
