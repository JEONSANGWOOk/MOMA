"""UI-independent map and deterministic simulated robot. Units: m, rad, s."""
import heapq
import json
from .decision_log import decide, audited, snapshot
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
        ids=set()
        for index,obs in enumerate(self.obstacles,1):
            for key in ('x','y','radius'):obs[key]=number(obs[key])
            if obs['radius']<=0:raise ValueError('장애물 반경은 0보다 커야 합니다.')
            from .dynamic_obstacles import validate_actor
            validate_actor(obs)
            if obs.get('dynamic'):
                obs.setdefault('id',('PERSON' if obs['kind']=='person' else 'AMR')+str(index))
                if not isinstance(obs['id'],str) or not obs['id'] or obs['id'] in ids:raise ValueError('동적 장애물 ID는 중복되지 않는 문자열이어야 합니다.')
                ids.add(obs['id'])
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
        from .dynamic_obstacles import actor_data
        data['obstacles']=[actor_data(o) for o in self.obstacles]
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
    def __init__(self, map_model, decision_journal=None):
        self.decision_journal=decision_journal
        self.map = map_model
        self.map.decision_journal=decision_journal
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
        from .obstacle_tracking import ObstacleTracker,DEFAULTS
        self.obstacle_tracker=ObstacleTracker();self.obstacle_tracker.decision_journal=decision_journal;self.obstacle_tracker.register_map(self.map.obstacles);self.auto_scenarios=dict(DEFAULTS)
        self.auto_wait_s=5.;self.auto_moving_speed=.08;self.auto_static_s=2.
        self.auto_obstacle_status='관측 대기';self._auto_block=None;self._auto_block_at=0.
        self.prefer_graph_routes=True;self._route_scan_next=0.
        self.reroute_wait_s=5.;self.reroute_attempt_limit=3
        self._reroute_block_at=None;self._reroute_failures=0;self._reroute_forced=False
        self.skipped_goals={};self.skip_result=None;self._skipped_context=None
        self._obstacle_latched=False;self._manual_stop_reason=''
        self.collision_radius=0.
        self.avoidance_status=''
        self._avoid_time=0.;self._avoid_next=0.
        self._progress_pose=None;self._progress_at=0.;self._recovery_next=0.
        self._local_avoidance_active=False;self._rejoin_check_at=0.
        from .blocked_recovery import BlockedRecovery
        self.blocked_recovery=BlockedRecovery();self._recovery_narrow=False;self.prefer_line_rejoin=False
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

    @audited('SIM.주행 요청', '계획 경로로 주행 시작')
    def navigate(self, goal, route_nodes=None):
        self.blocked_recovery.new_goal();self._recovery_narrow=False
        self._ready()
        s = self.state
        next_plan=None
        if self.obstacle_policy in ('reroute','auto') and self._skipped_context:
            from .alternate_routes import alternate_route
            a,b,ref=self._skipped_context
            next_plan=alternate_route(self.map,(s.x,s.y),goal,a,b,ref,max(self.collision_radius,self.map.robot_model['radius']),include_dynamic=True,optimize=True)
        if self.mapping:
            raise ValueError('맵 생성 중에는 수동 탐색만 가능합니다.')
        start = self.map.nearest(s.x, s.y)
        no_nominal=False
        if route_nodes is None:
            try:planned=self.map.route(start,goal)
            except ValueError:
                if self.obstacle_policy not in ('reroute','auto'):raise
                planned=[start,goal];no_nominal=True
        else:
            from .route_planner import adjacency
            graph=adjacency(self.map);planned=list(route_nodes)
            if not planned or planned[0]!=start or planned[-1]!=goal:
                raise ValueError('계획 경로의 시작/목적지가 현재 로봇 위치와 다릅니다.')
            if any(a not in graph or not any(nxt==b for nxt,_,_ in graph[a]) for a,b in zip(planned,planned[1:])):
                raise ValueError('계획 경로에 연결되지 않은 구간이 있습니다.')
        at_start=len(planned)>1 and math.hypot(s.x-self.map.nodes[start]['x'],s.y-self.map.nodes[start]['y'])<1e-6
        decide(self,'SIM.경로 선택',self.status(),dict(start=start,goal=goal,nodes=planned,provided=route_nodes is not None,nominal_connected=not no_nominal,policy=self.obstacle_policy),'연결 경로 선택' if not no_nominal else '연결 경로 없음','경로 적용' if not no_nominal else '대체 경로 탐색 대기',force=True)
        self.route = planned[1:] if at_start else planned
        self._waypoints = []
        self._segment_start = start if at_start else None
        self._arrival_align = False
        self._velocity = 0.
        self._avoid_next=0.;self.avoidance_status='';self._auto_block=None
        self._reroute_block_at=None;self._reroute_failures=0;self._reroute_forced=False;self.skip_result=None
        self.v = self.w = self.lease = 0
        self._progress_pose=(s.x,s.y,s.theta);self._progress_at=self._avoid_time;self._recovery_next=0.
        self._local_avoidance_active=False;self._rejoin_check_at=0.
        s.target, s.task, s.mode, s.charging = goal, f'이동 → {goal}', 'RUNNING', False
        if self.obstacle_policy in ('reroute','auto') and self._skipped_context:
            if next_plan:self._apply_alternate(next_plan,reason=self.block_reason or self.detected_obstacle or '이전 목적지 패스 후 연결 경로 재탐색')
            else:
                self.route=[goal];self._waypoints=[(s.x,s.y)];self._reroute_forced=True
        elif no_nominal:
            self.route=[goal];self._waypoints=[(s.x,s.y)];self._reroute_forced=True
            self._skipped_context=(None,start,[(s.x,s.y),(self.map.nodes[start]['x'],self.map.nodes[start]['y'])])

    def _apply_alternate(self,plan,reason=None):
        reason=reason or self.block_reason or self.detected_obstacle or '연결 경로 재계획'
        self._local_avoidance_active=False
        self.route=list(plan['nodes']);self._waypoints=list(plan['prefix'])
        self._reference_waypoints=list(plan['prefix']);self._segment_start=None
        self._segment_reverse=False;self._velocity=0.;self._arrival_align=False
        self._reroute_forced=False;self._skipped_context=None
        self._reroute_block_at=None;self._reroute_failures=0
        self.avoidance_status='다른 연결 경로 탐색 완료 · '+ ' → '.join(plan['nodes'])
        decide(self,'SIM.대체 경로',self.status(),dict(plan=plan,reason=reason,radius=self.applied_limits.get('radius')),'연결 가능한 대체 경로 발견','대체 경로 적용 · 주행 재개',force=True)

    def _auto_wait_active(self):
        return bool(self.obstacle_policy=='auto' and self._auto_block
                    and self._auto_block[-1].startswith('wait_')
                    and self._avoid_time-self._auto_block_at<self.auto_wait_s)

    def _try_line_bypass(self,node,radius,reason=None):
        """Keep the nominal line; replace only its currently obstructed interval."""
        if self._auto_wait_active():return False
        reason=reason or self.block_reason or self.detected_obstacle
        from .avoidance import rejoin_detour
        reference=getattr(self,'_reference_waypoints',[])
        if len(reference)<2 or math.dist(reference[-1],(node['x'],node['y']))>1e-5:return False
        points=rejoin_detour(self.map,(self.state.x,self.state.y),reference,radius,
                            safety_margin=.01 if self._recovery_narrow else .04,risk_aware=True)
        if not points:
            decide(self,'SIM.국소 우회',self.status(),dict(goal=node['id'],radius=radius,reason=reason),'충돌 없는 국소 경로 없음','대기 또는 다음 복구 전략 검토',key='search')
            return False
        self._waypoints=points[1:];self._local_avoidance_active=True;self._segment_reverse=False
        self._velocity=0.;self._arrival_align=False
        self.state.blocked=False;self._collision_blocked=False;self.block_reason=''
        self._reroute_block_at=None;self._reroute_failures=0
        self.avoidance_status='장애물 구간만 우회 → 원래 경로 복귀'
        decide(self,'SIM.국소 우회',self.status(),dict(points=points,radius=radius,reason=reason),'원래 경로 재합류 가능한 우회 발견','장애물 구간 우회 후 경로 복귀',force=True)
        return True

    def _try_local_detour(self,node,radius):
        if self._auto_wait_active():return False
        reason=self.block_reason or self.detected_obstacle
        from .avoidance import detour,rejoin_detour
        s=self.state
        reference=getattr(self,'_reference_waypoints',[])
        if reference and math.dist(reference[-1],(node['x'],node['y']))>1e-5:reference=[]
        points=rejoin_detour(self.map,(s.x,s.y),reference,radius,safety_margin=.01 if self._recovery_narrow else .04,risk_aware=True)
        rejoined=bool(points)
        if not points:points=detour(self.map,(s.x,s.y),(node['x'],node['y']),radius,safety_margin=.01 if self._recovery_narrow else .04,risk_aware=True)
        if not points and len(self.route)>1:
            final=self.map.nodes[self.route[-1]]
            points=detour(self.map,(s.x,s.y),(final['x'],final['y']),radius,risk_aware=True)
            if points:self.route=[self.route[-1]];self._segment_start=None
        if not points and not collision_reason(self.map,s.x,s.y,radius) and collision_reason(self.map,s.x,s.y,radius+.04):
            # At a close standstill, relax only the extra planning buffer;
            # preserve the full physical collision radius and swept checks.
            points=rejoin_detour(self.map,(s.x,s.y),reference,radius,safety_margin=0,risk_aware=True)
            rejoined=bool(points)
            if not points:points=detour(self.map,(s.x,s.y),(node['x'],node['y']),radius,safety_margin=0,risk_aware=True)
        if not points:
            decide(self,'SIM.국소 우회',self.status(),dict(goal=node['id'],radius=radius,reason=self.block_reason or self.detected_obstacle),'충돌 없는 국소 경로 없음','대기 또는 다음 복구 전략 검토',key='search')
            return False
        self._local_avoidance_active=True
        self._waypoints=points[1:];self._segment_reverse=False;self._velocity=0.
        self.avoidance_status='우회 후 기존 경로 복귀' if rejoined else '우회 주행'
        decide(self,'SIM.국소 우회',self.status(),dict(points=points,radius=radius,rejoined=rejoined,reason=reason),'충돌 없는 우회 발견',self.avoidance_status,force=True)
        s.blocked=False;self._collision_blocked=False;self._reroute_block_at=None;self._reroute_failures=0
        return True

    @audited('SIM.최종 복구 탐색')
    def try_recovery_detour(self):
        """Last SIM recovery: search free space before abandoning a graph goal."""
        if self._auto_wait_active():return False
        from .avoidance import detour,rejoin_detour
        s=self.state
        if not self.route or not s.target or s.stopped or not s.motor or self._obstacle_latched:return False
        if self.obstacle_policy=='auto' and self._auto_block and self._auto_block[-1] in ('wait','stop'):return False
        # A fallback is obstacle recovery, not a connection between unrelated maps.
        if self._reroute_forced:
            try:self.map.route(self.map.nearest(s.x,s.y),s.target)
            except ValueError:return False
        radius=max(self.collision_radius,self.map.robot_model['radius'])
        radius=max(radius,getattr(self,'applied_limits',{}).get('radius',radius))
        if collision_reason(self.map,s.x,s.y,radius):return False
        next_id=self.route[0];node=self.map.nodes[next_id]
        next_pos=(node['x'],node['y']);start=(s.x,s.y)
        reference=getattr(self,'_reference_waypoints',[])
        margins=[.04,0.]
        for margin in margins:
            if collision_reason(self.map,*start,radius+margin):continue
            # Restore the closest reachable point of the original segment first.
            points=None
            if reference and math.dist(reference[-1],next_pos)<1e-5:
                points=rejoin_detour(self.map,start,reference,radius,safety_margin=margin,risk_aware=True)
            if points:
                destination=next_id;rejoined=True
            else:
                destination=next_id;rejoined=False
                points=detour(self.map,start,next_pos,radius,max_cells=80000,safety_margin=margin,grid_step=.08,risk_aware=True)
            if not points and next_id!=s.target:
                goal=self.map.nodes[s.target];destination=s.target
                points=detour(self.map,start,(goal['x'],goal['y']),radius,max_cells=80000,safety_margin=margin,grid_step=.08,risk_aware=True)
            if not points:continue
            if destination!=next_id:self.route=[destination];self._segment_start=None
            self._local_avoidance_active=True
            self._waypoints=points[1:]
            # Keep the original segment separate from temporary avoidance points.
            if destination!=next_id:self._reference_waypoints=list(points)
            self._segment_reverse=False;self._velocity=0.;self._arrival_align=False
            self._reroute_forced=False;self._skipped_context=None
            self._reroute_block_at=None;self._reroute_failures=0
            s.blocked=False;self._collision_blocked=False;self.block_reason=''
            self.avoidance_status='패스 전 자율 우회 · '+('기존 경로 복귀' if rejoined else '자유 공간 탐색 → '+destination)
            self._avoid_next=self._avoid_time+1.
            decide(self,'SIM.최종 복구 탐색',self.status(),dict(radius=radius,margin=margin,destination=destination,points=points),'자유 공간 경로 발견',self.avoidance_status,force=True)
            return True
        self.avoidance_status='패스 전 자율 우회 탐색 실패 · 충돌 없는 통로 없음'
        decide(self,'SIM.최종 복구 탐색',self.status(),dict(radius=radius,margins=margins),'연결/국소/자유 공간 우회 실패','안전 정지 · 목적지 패스 검토',force=True)
        return False

    def skip_destination(self,reason):
        goal=self.state.target
        if not goal:return
        decide(self,'SIM.목적지 패스',self.status(),dict(reason=reason,attempts=self._reroute_failures,limit=self.reroute_attempt_limit),'목적지 도달 불가','현재 목표 패스 · 후속 작업 생략 요청',force=True)
        self.skip_result=dict(goal=goal,reason=reason,time_s=round(self._avoid_time,1))
        self.skipped_goals[goal]=dict(self.skip_result)
        self._skipped_context=self._skipped_context or (self._segment_start,self.route[0] if self.route else goal,
            list(getattr(self,'_reference_waypoints',[])))
        self.route=[];self._waypoints=[];self._velocity=0.;self.v=self.w=self.lease=0.
        self.state.speed=0.;self.state.blocked=False;self._collision_blocked=False
        self.state.mode='IDLE';self.state.task='도달 불가 · 패스 '+goal
        self.avoidance_status='목적지 패스 · '+goal+' · '+reason

    @audited('SIM.이동 인터록')
    def _ready(self):
        if self.arm['status']=='RUNNING' or self.arm.get('pose')!='SAFE':
            raise ValueError('로봇팔 작업 완료 후 safe_pose가 필요합니다.')
        if self.state.stopped or not self.state.motor:
            raise ValueError('정지 해제 및 모터 ON이 필요합니다.')
        if self.state.blocked:
            raise ValueError('장애물을 해제하세요.')
        decide(self,'SIM.이동 인터록',self.status(),dict(arm=self.arm,stopped=self.state.stopped,motor=self.state.motor,blocked=self.state.blocked),'팔 안전 자세·모터·정지·장애물 조건 충족','이동 요청의 경로/속도 검증 진행')

    @audited('SIM.수동 조종')
    def drive(self, v, w):
        from .manual_safety import manual_motion_reason
        # A rejected forward jog must not prevent a checked retreat.
        if self._collision_blocked and not self._obstacle_latched:self.state.blocked=False
        self._ready()
        if self.route:
            raise ValueError('Task를 취소한 뒤 수동 조작하세요.')
        from .studio_core import zone_limits
        limits=zone_limits(self.map,self.state.x,self.state.y,dict(self.map.robot_model))
        self.applied_limits=limits
        maxv=min(.3,limits['maxspeed']);maxw=min(.6,limits['maxrot'])
        v,w=max(-maxv,min(maxv,v)),max(-maxw,min(maxw,w))
        radius=max(self.map.robot_model['radius'],self.collision_radius)
        reason=manual_motion_reason(self.map,(self.state.x,self.state.y,self.state.theta),v,w,radius)
        if reason:
            self.v=self.w=self.lease=0.;self.state.speed=0.
            self.state.blocked=True;self._collision_blocked=True;self.block_reason=reason;self._manual_stop_reason=reason
            self.avoidance_status='수동 조작 차단 · '+reason
            raise ValueError('수동 조작 차단 · '+reason)
        self.state.blocked=False;self._collision_blocked=False;self.block_reason='';self._manual_stop_reason=''
        decide(self,'SIM.수동 조종',self.status(),dict(v=v,w=w,max_v=maxv,max_w=maxw,radius=radius,lease_s=.3),'예상 이동 구간 충돌 없음','검증된 수동 속도 적용',identity=(v,w,maxv,maxw))
        self.v, self.w, self.lease = v,w,.3
        self.state.charging = False
        self.state.mode = 'MANUAL'

    def stop(self, latch=False):
        if self._manual_stop_reason:
            self.state.blocked=False;self._collision_blocked=False;self._manual_stop_reason='';self.block_reason=''
        self._skipped_context=None
        if self._obstacle_latched:self.state.blocked=False
        self._obstacle_latched=False
        self.route.clear()
        self._waypoints.clear()
        self._segment_start = None
        self._arrival_align = False
        self._velocity = 0.
        self.v = self.w = self.lease = 0
        s = self.state
        s.speed, s.target, s.mode, s.task = 0, '', 'STOPPED' if latch else 'IDLE', '취소됨'
        s.stopped = s.stopped or latch

    @audited('SIM.명령', 'SIM 명령 적용')
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
        recovery=self.blocked_recovery
        if recovery.active:
            if recovery.phase=='BACK':pts.append((self.state.x-math.cos(recovery.heading)*recovery.remaining,self.state.y-math.sin(recovery.heading)*recovery.remaining))
            return pts
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
                decide(self,'SIM.도착 자세',self.status(),dict(target_heading=heading,delta=delta,maxrot=self.applied_limits['maxrot']),'목표 자세와 차이 있음','제자리 회전으로 정렬')
                s.theta+=max(-step,min(step,delta));self._arrival_align=True
                s.speed=0;self._velocity=0
                return
            s.theta=heading
        decide(self,'SIM.도착',self.status(),dict(node=n['id'],remaining_nodes=len(self.route),heading=heading),'경유점 도달','다음 구간 진행' if len(self.route)>1 else ('충전 시작' if n['kind']=='dock' else '주행 완료'),force=True)
        self._arrival_align=False
        s.x,s.y,s.last_node=n['x'],n['y'],n['id']
        self.skipped_goals.pop(n['id'],None)
        self._local_avoidance_active=False;self._recovery_narrow=False
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
                decide(self,'SIM.자동 충전',self.status(),dict(battery=s.battery,low=policy['low'],reachable=reachable),'저전압 · 도달 가능한 최단 충전소 선택','충전소 이동 · 기존 목표 보관',force=True)
                self.navigate(min(reachable)[1]);policy['phase']='TO_DOCK'
            else:decide(self,'SIM.자동 충전',self.status(),dict(battery=s.battery,low=policy['low'],docks=docks),'도달 가능한 충전소 없음','충전 이동 보류')
        elif policy['phase']=='TO_DOCK' and s.charging:
            decide(self,'SIM.자동 충전',self.status(),dict(phase=policy['phase']),'충전소 도착 확인','충전 상태 전환',force=True)
            policy['phase']='CHARGING'
        elif policy['phase']=='CHARGING' and s.battery>=policy['high']:
            decide(self,'SIM.자동 충전',self.status(),dict(battery=s.battery,high=policy['high']),'충전 상한 도달','충전 종료 · 보관 목표 재개 검토',force=True)
            policy['phase']='IDLE';s.charging=False
            goal=self._auto_charge_goal;self._auto_charge_goal=None
            if goal and goal in self.map.nodes:self.navigate(goal)

    def tick(self, dt):
        origin=(self.state.x,self.state.y)
        try:
            self._tick(dt)
        finally:
            # Keep the last actual speed available while deciding. All no-move
            # returns (waiting, replanning, heading alignment) still publish 0.
            if math.hypot(self.state.x-origin[0],self.state.y-origin[1])<=1e-12:
                self.state.speed=0.

    def _tick(self, dt):
        dt = max(0, min(dt, .1))
        from .dynamic_obstacles import update_actors
        update_actors(self.map,dt,self.state,max(self.collision_radius,self.map.robot_model['radius']))
        self.obstacle_tracker.observe(self.map,self.state,dt)
        self.obstacle_tracker.update(self.map.obstacles,dt,self.auto_moving_speed,self.auto_static_s)
        self._avoid_time+=dt
        s = self.state
        if self._reroute_forced and self.obstacle_policy not in ('reroute','auto') and self.route:
            self.stop()
            self._reroute_forced=False
            self.avoidance_status='정책 변경 · 연결 경로 없는 이동 취소'
        self.detected_obstacle=''
        if self._obstacle_latched:
            s.blocked=True;self._velocity=0.;self.v=self.w=self.lease=0.
            self.avoidance_status='장애물 정지 유지 · 수동 재개 필요'
            decide(self,'SIM.정지 유지',self.status(),dict(reason=self.block_reason,policy=self.obstacle_policy),'장애물 정지 래치 활성','정지 유지 · 수동 재개 필요')
            return
        was_blocked=self._collision_blocked and s.blocked
        if was_blocked:
            recovery=self.blocked_recovery
            allowed=recovery.allowed(self)
            decide(self,'SIM.BLOCKED 복구 허용',self.status(),dict(enabled=recovery.enabled,policy=self.obstacle_policy,latched=self._obstacle_latched,manual_stop=self._manual_stop_reason,route_available=bool(self.route),auto_scenario=self._auto_block[-1] if self._auto_block else None,auto_elapsed_s=self._avoid_time-self._auto_block_at,auto_wait_s=self.auto_wait_s),'BLOCKED 복구 허용' if allowed else 'BLOCKED 복구 조건 미충족','국소 우회/제한 후진 검토' if allowed else '현재 장애물 정책 유지',identity=(allowed,recovery.enabled,self.obstacle_policy,self._auto_block))
        if self.blocked_recovery.active and self.blocked_recovery.step(self,dt):return
        if was_blocked and self.blocked_recovery.allowed(self):
            if not self._reroute_forced and self._try_local_detour(self.map.nodes[self.route[0]],self.blocked_recovery.radius(self)):return
            if self.blocked_recovery.begin(self,self.block_reason) and self.blocked_recovery.step(self,dt):return
        if self._collision_blocked and not self._manual_stop_reason:
            s.blocked=False;self._collision_blocked=False;self.block_reason=''
        self._charge_tick()
        if s.stopped or not s.motor or s.blocked:
            decide(self,'SIM.주행 안전',self.status(),dict(stopped=s.stopped,motor=s.motor,blocked=s.blocked,reason=self.block_reason),'이동 허용 조건 불충족','속도/조종 lease 해제 · 안전 정지',identity=(s.stopped,s.motor,s.blocked,self.block_reason))
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
                self._reference_waypoints=[(s.x,s.y)]+list(geometry)
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
            decide(self,'SIM.주행 제한',self.status(),dict(segment=[self._segment_start,self.route[0]],limits=limits,path_properties=(rec.get('properties') or {}) if rec else {}),'모델·경로·영역 제한 중 안전한 값 선택','속도/가감속/회전/충돌 반경 적용',identity=limits)
            if self._local_avoidance_active and not self._auto_wait_active() and self._avoid_time>=self._rejoin_check_at:
                self._rejoin_check_at=self._avoid_time+.5
                reference=getattr(self,'_reference_waypoints',[])
                if reference and math.dist(reference[-1],(n['x'],n['y']))<1e-5:
                    from .avoidance import nearest_clear_rejoin
                    merged=nearest_clear_rejoin(self.map,(s.x,s.y),reference,limits['radius'])
                    if merged:
                        self._waypoints=merged[1:];self._local_avoidance_active=False;self._recovery_narrow=False
                        self._velocity=0.;self.avoidance_status='장애물 통과 · 최단 연결로 기존 경로 복귀'
                        decide(self,'SIM.경로 재합류',self.status(),dict(radius=limits['radius'],points=merged),'기존 경로까지 충돌 없는 연결 발견','최단 연결로 기존 경로 복귀',force=True)
            # An independent progress watchdog also covers repeated heading /
            # collision returns that never reach the normal reroute branch.
            pose=(s.x,s.y,s.theta)
            previous=self._progress_pose
            turned=abs(math.atan2(math.sin(s.theta-previous[2]),math.cos(s.theta-previous[2]))) if previous else 0.
            if previous is None or math.dist(pose[:2],previous[:2])>=.08 or turned>=.25:
                self._progress_pose=pose;self._progress_at=self._avoid_time
            stall_limit=max(6.,self.auto_wait_s+2.)
            if self.obstacle_policy in ('auto','reroute') and not self._auto_wait_active() and self._avoid_time-self._progress_at>=stall_limit and self._avoid_time>=self._recovery_next:
                decide(self,'SIM.무진행 감시',self.status(),dict(stalled_s=self._avoid_time-self._progress_at,limit_s=stall_limit,distance_threshold_m=.08,rotation_threshold_rad=.25),'시간 내 이동/회전 진전 부족','자유 공간 복구 경로 재탐색',force=True)
                self._recovery_next=self._avoid_time+3.
                if self.try_recovery_detour():
                    self._progress_at=self._avoid_time
                    self.avoidance_status='정체 탈출 · '+self.avoidance_status
                    return
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
            if self.prefer_graph_routes and not (self.prefer_line_rejoin and self._local_avoidance_active) and self.obstacle_policy in ('auto','reroute') and self._avoid_time>=self._route_scan_next:
                self._route_scan_next=self._avoid_time+1.
                remaining=self.navigation_points()[1:];previous=(s.x,s.y);horizon=0.
                for point in remaining:horizon+=math.dist(previous,point);previous=point
                _,ahead_reason=path_clearance(self.map,(s.x,s.y),remaining,limits['radius'],horizon)
                permit=True
                key=kind=scenario=None;elapsed=None
                if ahead_reason and self.obstacle_policy=='auto':
                    from .obstacle_tracking import SCENARIOS
                    key,kind,_,obs=self.obstacle_tracker.blocker(self.map,(s.x,s.y),remaining,limits['radius'],horizon,ahead_reason)
                    scenario=(obs or {}).get('auto_scenarios',{}).get(kind,self.auto_scenarios[kind])
                    if scenario not in SCENARIOS:scenario='wait'
                    permit=scenario not in ('wait','stop')
                    if scenario.startswith('wait_'):
                        elapsed=self._avoid_time-self._auto_block_at if self._auto_block==(key,kind,scenario) else 0.
                        permit=self._auto_block==(key,kind,scenario) and elapsed>=self.auto_wait_s
                decide(self,'SIM.전방 전체 경로 검사',self.status(),dict(reason=ahead_reason,horizon=horizon,radius=limits['radius'],policy=self.obstacle_policy,permit=permit,obstacle=key,classification=kind,scenario=scenario,elapsed_s=elapsed,wait_s=self.auto_wait_s if scenario and scenario.startswith('wait_') else None),'장애물 감지' if ahead_reason else '전방 경로 통과 가능','대체 경로 탐색' if ahead_reason and permit else ('사전 우회 보류 · 장애물 정책 판단 계속' if ahead_reason else '기존 경로 유지'),identity=(ahead_reason,permit,key,kind,scenario))
                if ahead_reason and permit:
                    if self.prefer_line_rejoin and not self._local_avoidance_active and not self._reroute_forced and self._try_line_bypass(n,limits['radius'],reason=ahead_reason):return
                    from .alternate_routes import alternate_route
                    plan=alternate_route(self.map,(s.x,s.y),s.target,self._segment_start,self.route[0],getattr(self,'_reference_waypoints',[]),limits['radius'],include_dynamic=True,optimize=True)
                    if plan:
                        self._apply_alternate(plan,reason=ahead_reason)
                        self.avoidance_status='다른 연결 경로 · 전체 구간 검사 · '+ ' → '.join(plan['nodes'])
                        return
            if len(preview)>1:
                p=preview[1];heading=math.atan2(p[1]-s.y,p[0]-s.x)+(math.pi if self._segment_reverse else 0)
                delta_heading=math.atan2(math.sin(heading-s.theta),math.cos(heading-s.theta))
                if self._velocity<=1e-6 and abs(delta_heading)>limits['maxrot']*dt+.08:
                    decide(self,'SIM.진행 방향',self.status(),dict(delta_heading=delta_heading,maxrot=limits['maxrot'],reverse=self._segment_reverse),'출발 방향 정렬 필요','정지 상태에서 진행 방향으로 회전')
                    s.theta+=max(-limits['maxrot']*dt,min(limits['maxrot']*dt,delta_heading))
                    self._velocity=0
                    return
            from .studio_core import zone_obstacle_properties
            props=zone_obstacle_properties(self.map,s.x,s.y,(rec.get('properties') or {}) if rec else {})
            stop_dist=max(.025,float(props.get('obsStopDist',.05) or .05))
            dec_dist=max(stop_dist,float(props.get('obsDecDist',.5) or .5))
            clearance,reason=path_clearance(self.map,(s.x,s.y),self._waypoints,limits['radius']+max(0,float(props.get('obsExpansion',0) or 0)),max(dec_dist,self._velocity**2/(2*max(.01,limits['maxdec']))+stop_dist))
            from .navigation_quality import projected_conflict
            predicted=None
            if not reason:
                predicted=projected_conflict(self.map,(s.x,s.y),self._waypoints,limits['radius'],max(self._velocity,desired))
                if predicted:
                    decide(self,'SIM.동적 충돌 예측',self.status(),dict(prediction=predicted,radius=limits['radius'],requested_speed=desired),'관측 속도로 예상한 경로 충돌','예상 충돌까지 거리/시간을 반영해 감속',identity=predicted['obstacle'].get('id'))
                    clearance=predicted['distance'];reason='동적 장애물 예상 충돌 · '+str(predicted['obstacle'].get('id','이동체'))
                    desired=min(desired,max(0.,(clearance-stop_dist)/max(.2,predicted['time'])))
            self.detected_obstacle=reason
            if self._reroute_forced:reason=self.detected_obstacle='기존 연결 경로에 복귀 불가'
            if not reason:
                decide(self,'SIM.장애물 대응',self.status(),dict(clearance=clearance,stop_distance=stop_dist,deceleration_distance=dec_dist),'경로 장애물 없음','기존 경로 주행 · 장애물 대기/재시도 초기화')
                self._reroute_block_at=None;self._reroute_failures=0;self._auto_block=None;self.auto_obstacle_status='경로 장애물 없음'
            if reason:
                dynamic=reason.startswith('동적 장애물')
                policy=self.obstacle_policy
                if policy in ('adaptive','reroute'):
                    _,classification,_,_=self.obstacle_tracker.blocker(self.map,(s.x,s.y),self._waypoints,limits['radius'],max(dec_dist,clearance+.05),reason)
                    dynamic=predicted is not None or classification!='static'
                if policy=='auto':
                    from .obstacle_tracking import CLASSES,SCENARIOS
                    key,classification,speed,obstacle=self.obstacle_tracker.blocker(self.map,(s.x,s.y),self._waypoints,limits['radius'],max(dec_dist,clearance+.05),reason)
                    overrides=(obstacle or {}).get('auto_scenarios',{})
                    scenario=overrides.get(classification,self.auto_scenarios[classification])
                    if scenario not in SCENARIOS:scenario='wait'
                    token=(key,classification,scenario)
                    if predicted:
                        obstacle=predicted['obstacle'];classification='dynamic';speed=obstacle.get('_observed_speed',0.)
                        key=obstacle.get('_track_id',id(obstacle));scenario=obstacle.get('auto_scenarios',{}).get(classification,self.auto_scenarios[classification]);token=(key,classification,scenario)
                    if token!=self._auto_block:self._auto_block=token;self._auto_block_at=self._avoid_time;self._reroute_block_at=None;self._reroute_failures=0
                    elapsed=self._avoid_time-self._auto_block_at
                    self.auto_obstacle_status=f'{CLASSES[classification]} · 관측 {speed:.2f} m/s · '+SCENARIOS.get(scenario,'대기')
                    dynamic=classification=='dynamic'
                    policy=scenario
                    decide(self,'SIM.장애물 분류',self.status(),dict(obstacle=key,classification=classification,observed_speed=speed,moving_speed_threshold=self.auto_moving_speed,static_time_threshold=self.auto_static_s,scenario=scenario,elapsed_s=elapsed,wait_s=self.auto_wait_s,clearance=clearance,reason=reason),'관측 분류에 맞는 시나리오 선택',SCENARIOS.get(scenario,'대기'),identity=token)
                    if scenario.startswith('wait_'):
                        if elapsed<self.auto_wait_s:
                            decide(self,'SIM.장애물 대응',self.status(),dict(reason=reason,classification=classification,elapsed_s=elapsed,threshold_s=self.auto_wait_s,scenario=scenario),'설정 대기 시간 미도달','정지 상태로 장애물 관측 유지',identity=token)
                            self._velocity=0.;s.blocked=True;self._collision_blocked=True;self.block_reason=reason
                            self.avoidance_status=f'자동 {CLASSES[classification]} 대기 {elapsed:.1f}/{self.auto_wait_s:.1f}s';return
                        policy=scenario[5:]
                        decide(self,'SIM.장애물 대응',self.status(),dict(reason=reason,elapsed_s=elapsed,threshold_s=self.auto_wait_s,policy=policy),'설정 대기 시간 경과','대기 종료 · '+policy+' 전략 적용',identity=token)
                    # User-selected graph rerouting also applies to a moving blocker.
                    if policy=='reroute':dynamic=False

                if policy=='recover':
                    decide(self,'SIM.장애물 대응',self.status(),dict(reason=reason,clearance=clearance,policy=policy),'단계적 복구 정책','안전 정지 · 국소 우회 우선 탐색')
                    self._velocity=0.;s.blocked=True;self._collision_blocked=True;self.block_reason=reason
                    if self._avoid_time<self._avoid_next:return
                    self._avoid_next=self._avoid_time+1.
                    if not self._reroute_forced and self._try_local_detour(n,limits['radius']):return
                    policy='reroute';dynamic=False
                if policy=='reroute' and not dynamic:
                    if self._reroute_block_at is None:self._reroute_block_at=self._avoid_time
                    elapsed=self._avoid_time-self._reroute_block_at
                    self._velocity=0.;s.blocked=True;self._collision_blocked=True;self.block_reason=reason
                    self.avoidance_status=f'정적 장애물 대기 {elapsed:.1f}/{self.reroute_wait_s:.1f}s · 다른 경로 탐색 {self._reroute_failures}/{self.reroute_attempt_limit}'
                    retry_wait=0 if self.obstacle_policy=='auto' and self._reroute_failures==0 else self.reroute_wait_s
                    if self.obstacle_policy=='auto':self.avoidance_status=f'자동 다른 경로 탐색 · 재시도 대기 {elapsed:.1f}/{retry_wait:.1f}s · 실패 {self._reroute_failures}/{self.reroute_attempt_limit}'
                    decide(self,'SIM.재탐색 대기',self.status(),dict(reason=reason,elapsed_s=elapsed,wait_s=retry_wait,failures=self._reroute_failures,limit=self.reroute_attempt_limit),'재시도 시간 도달' if elapsed>=retry_wait else '재시도 대기 중','연결 경로 탐색' if elapsed>=retry_wait else '정지 유지',identity=self._reroute_failures)
                    if elapsed>=retry_wait:
                        if self.prefer_line_rejoin and not self._reroute_forced and self._try_line_bypass(n,limits['radius'],reason=reason):return
                        from .alternate_routes import alternate_route
                        context=self._skipped_context or (self._segment_start,self.route[0],getattr(self,'_reference_waypoints',[]))
                        plan=alternate_route(self.map,(s.x,s.y),s.target,*context,limits['radius'],include_dynamic=True,optimize=True)
                        if plan:
                            s.blocked=False;self._collision_blocked=False;self._apply_alternate(plan,reason=reason)
                        else:
                            self._reroute_failures+=1;self._reroute_block_at=self._avoid_time
                            decide(self,'SIM.재탐색 결과',self.status(),dict(reason=reason,failures=self._reroute_failures,limit=self.reroute_attempt_limit),'충돌 없는 연결 경로 없음','최종 복구 탐색' if self._reroute_failures>=self.reroute_attempt_limit else '다음 재시도까지 대기',force=True)
                            if self._reroute_failures>=self.reroute_attempt_limit:
                                if not self.try_recovery_detour():self.skip_destination(reason+' · 연결 경로 및 자율 우회 통로 없음')
                    return
                if policy=='stop':
                    decide(self,'SIM.장애물 대응',self.status(),dict(reason=reason,policy=policy,clearance=clearance),'정지 정책 선택','정지 래치 · 수동 재개 필요',force=True)
                    self._obstacle_latched=True;s.blocked=True;self.block_reason=reason
                    self._velocity=0.;self.avoidance_status='장애물 정지 유지 · 수동 재개 필요';return
                avoid=policy=='avoid' or policy=='adaptive' and not dynamic
                decide(self,'SIM.장애물 대응',self.status(),dict(reason=reason,policy=policy,dynamic=dynamic,clearance=clearance,stop_distance=stop_dist),'국소 우회 선택' if avoid else '장애물 해제 대기 선택','우회 탐색' if avoid else '거리 기반 감속/정지',identity=(reason,policy,dynamic))
                if avoid and self._avoid_time>=self._avoid_next:
                    self._avoid_next=self._avoid_time+1.
                    if self._try_local_detour(n,limits['radius']):return
                    self.avoidance_status='우회 경로 없음 · 대기'
                elif not avoid:self.avoidance_status='동적 장애물 통과 대기' if dynamic else '장애물 해제 대기'
                desired=min(desired,math.sqrt(2*limits['maxdec']*max(0,clearance-stop_dist)))
                if clearance<=stop_dist+.025:
                    decide(self,'SIM.충돌 정지',self.status(),dict(clearance=clearance,stop_distance=stop_dist,tolerance=.025,reason=reason),'장애물이 정지 거리 이내','이동 차단 · 속도 0',identity=reason)
                    self._collision_blocked=True;s.blocked=True;self.block_reason=reason;self._velocity=0
                    return
            decide(self,'SIM.속도 결정',self.status(),dict(desired_speed=desired,remaining_distance=distance_to_goal,limits=limits,obstacle=reason),'주행 제한·곡률·제동 거리·장애물 조건 반영','가감속 제한으로 목표 속도 적용',identity=(bool(reason),round(desired,1),limits['maxspeed']))
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
                    decide(self,'SIM.이동 구간 충돌',dict(pose=old_pose,proposed_pose=[s.x,s.y,s.theta]),dict(reason=reason,radius=limits['radius']),'이동 후 충돌 검출','직전 위치 복원 · 안전 정지',identity=reason)
                    s.x,s.y,s.theta=old_pose;self._waypoints=old_points
                    self._collision_blocked=True;s.blocked=True;self.block_reason=reason
                    self._velocity=0;s.speed=0
                    return
            _,reason=path_clearance(self.map,old_pose[:2],swept,limits['radius'],sum(math.dist(a,b) for a,b in zip([old_pose[:2]]+swept,swept)))
            if reason:
                decide(self,'SIM.이동 구간 충돌',dict(pose=old_pose,swept=swept),dict(reason=reason,radius=limits['radius']),'연속 이동 구간 충돌 검출','직전 위치 복원 · 안전 정지',identity=reason)
                s.x,s.y,s.theta=old_pose;self._waypoints=old_points
                self._collision_blocked=True;s.blocked=True;self.block_reason=reason
                self._velocity=0;s.speed=0;return
            s.speed=(-1 if self._segment_reverse else 1)*self._velocity if remaining<travel else 0
            if not self._waypoints:
                self._arrive(n,dt)
                if not self.route:self.avoidance_status='목적지 도착'
        elif self.lease > 0:
            from .manual_safety import manual_motion_reason,arc_pose
            radius=max(self.map.robot_model['radius'],self.collision_radius)
            reason=manual_motion_reason(self.map,(s.x,s.y,s.theta),self.v,self.w,radius)
            if reason:
                decide(self,'SIM.수동 조종 감시',self.status(),dict(reason=reason,v=self.v,w=self.w,radius=radius),'예상 조종 구간 위험','조종 lease 해제 · 안전 정지',identity=reason)
                self.v=self.w=self.lease=0.;s.speed=0.;s.blocked=True
                self._collision_blocked=True;self.block_reason=reason;self._manual_stop_reason=reason
                self.avoidance_status='수동 조작 차단 · '+reason
                return
            self.lease -= dt
            if self.lease <= 0:
                decide(self,'SIM.수동 조종 감시',self.status(),dict(lease_s=self.lease),'조종 갱신 기한 만료','수동 속도 해제 · IDLE',force=True)
                self.v = self.w = 0
                s.mode = 'IDLE'
            old_xy=(s.x,s.y)
            old_pose=(s.x,s.y,s.theta)
            scale=getattr(self.map,'robot_model',DEFAULT_MODEL)['wheel_scale']
            s.x,s.y,s.theta=arc_pose(old_pose,self.v,self.w,dt,scale)
            s.theta=math.atan2(math.sin(s.theta),math.cos(s.theta))
            s.speed = self.v
            _,reason=path_clearance(self.map,old_xy,[(s.x,s.y)],max(getattr(self.map,'robot_model',DEFAULT_MODEL)['radius'],self.collision_radius),math.dist(old_xy,(s.x,s.y)))
            if reason:
                decide(self,'SIM.수동 이동 구간',dict(start=old_xy,end=[s.x,s.y]),dict(reason=reason),'연속 수동 이동 충돌','직전 위치 복원 · 안전 정지',identity=reason)
                s.x,s.y=old_xy
                self.v=self.w=self.lease=0.
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
