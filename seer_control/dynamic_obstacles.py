"""SIM actors: repeat a taught segment, with swept collision checks."""
import math
from types import SimpleNamespace
from .studio_core import path_clearance

POLICIES={'auto':'자동 판단 / 장애물별 시나리오','wait':'대기 후 기존 경로로 진행','avoid':'장애물 피해서 가기',
          'stop':'정지하기 (수동 재개)','adaptive':'동적 대기 / 정적 우회',
          'reroute':'정적 대기 → 다른 경로 / 불가 목적지 패스'}


def validate_actor(obs):
    if 'dynamic' in obs and type(obs['dynamic']) is not bool:raise ValueError('dynamic은 true/false입니다.')
    if not obs.get('dynamic'):return
    if 'paused' in obs and type(obs['paused']) is not bool:raise ValueError('paused는 true/false입니다.')
    if obs.get('kind') not in ('person','amr'):raise ValueError('동적 장애물 종류: person / amr')
    pts=obs.get('motion_path')
    if not isinstance(pts,list) or len(pts)<2:raise ValueError('동적 장애물 경로는 두 개 이상의 포인트입니다.')
    for p in pts:
        if not isinstance(p,(list,tuple)) or len(p)!=2 or not all(math.isfinite(float(v)) for v in p):raise ValueError('동적 장애물 경로 좌표 오류')
    if any(math.dist(a,b)<.05 for a,b in zip(pts,pts[1:])):raise ValueError('동적 장애물 이동 거리는 0.05m 이상입니다.')
    if obs.get('encounter_policy','reverse') not in ('reverse','wait'):raise ValueError('다른 장애물 대응은 reverse / wait입니다.')
    encounter_wait=float(obs.get('encounter_wait_s',2))
    if not math.isfinite(encounter_wait) or not .1<=encounter_wait<=60:raise ValueError('방향 전환 대기: 0.1~60초')
    obs['encounter_wait_s']=encounter_wait
    speed=float(obs.get('speed_mps',.5));dwell=float(obs.get('dwell_s',.5))
    if not math.isfinite(speed) or not .01<=speed<=2:raise ValueError('장애물 속도: 0.01~2m/s')
    if not math.isfinite(dwell) or not 0<=dwell<=60:raise ValueError('끝점 대기: 0~60초')
    obs['speed_mps']=speed;obs['dwell_s']=dwell
    obs['motion_path']=[[float(v) for v in p] for p in pts]


def compile_actor_path(model,obs):
    """Expand taught AMR nodes through directed map lanes, including curves."""
    from .alternate_routes import lane
    nodes=obs.get('motion_nodes')
    if obs.get('motion_mode','pingpong') not in ('pingpong','loop'):raise ValueError('이동 방식 오류')
    if nodes:
        if obs['kind']!='amr' or len(nodes)<2 or any(n not in model.nodes for n in nodes):raise ValueError('기존 AMR 노드를 두 개 이상 선택하세요.')
        points=[(model.nodes[nodes[0]]['x'],model.nodes[nodes[0]]['y'])];stops=set()
        sequence=list(nodes)+ (list(reversed(nodes[:-1])) if obs.get('motion_mode','pingpong')=='pingpong' else [nodes[0]])
        for a,b in zip(sequence,sequence[1:]):
            if a==b:raise ValueError('연속된 동일 노드는 사용할 수 없습니다.')
            route=model.route(a,b)
            for x,y in zip(route,route[1:]):
                for point in lane(model,x,y):
                    if math.dist(points[-1],point)>1e-8:points.append(tuple(point))
            stops.add(len(points)-1)
        return points,stops
    points=[tuple(p) for p in obs['motion_path']]
    points+=list(reversed(points[:-1])) if obs.get('motion_mode','pingpong')=='pingpong' else [points[0]]
    return points,set(range(1,len(points)))


def actor_data(obs):
    result={k:v for k,v in obs.items() if not k.startswith('_')}
    if obs.get('dynamic'):result['x'],result['y']=obs['motion_path'][0]
    return result


def actor_can_reverse(model,obs,points,index,direction):
    if obs['kind']=='person' or not obs.get('motion_nodes'):return True
    from .route_planner import adjacency
    from .alternate_routes import lane
    from .studio_core import point_segment_distance
    previous=points[index-direction];current=(obs['x'],obs['y'])
    def on_path(point,geometry):
        return len(geometry)>1 and min(point_segment_distance(*point,a,b) for a,b in zip(geometry,geometry[1:]))<=.04
    graph=adjacency(model)
    for a,edges in graph.items():
        for b,_,_ in edges:
            if not any(nxt==a for nxt,_,_ in graph.get(b,[])):continue
            geometry=lane(model,a,b);reverse=lane(model,b,a)
            if all(on_path(p,geometry) and on_path(p,reverse) for p in (previous,current)):return True
    return False


def next_actor_index(points,index,direction):
    if direction>0:return 1 if index==len(points)-1 else index+1
    return len(points)-2 if index==0 else index-1


def update_actors(model,dt,robot,radius):
    for obs in model.obstacles:
        if not obs.get('dynamic'):continue
        if getattr(model,'dynamic_paused',False) or obs.get('paused',False):
            obs['_motion']='일시정지';continue
        if obs.get('_dwell',0)>0:
            obs['_dwell']=max(0,obs['_dwell']-dt);obs['_motion']='끝점 대기';continue
        try:
            if '_compiled' not in obs:obs['_compiled']=compile_actor_path(model,obs)
            points,stops=obs['_compiled']
        except (ValueError,KeyError):obs['_motion']='이동 방식 오류';continue
        direction=obs.get('_direction',1)
        index=obs.get('_goal',1);goal=points[index]
        dx,dy=goal[0]-obs['x'],goal[1]-obs['y'];distance=math.hypot(dx,dy)
        if distance<1e-8:
            obs['_goal']=next_actor_index(points,index,direction)
            obs['_dwell']=obs.get('dwell_s',.5) if index in stops or index==0 and len(points)-1 in stops else 0;continue
        travel=min(distance,dt*obs.get('speed_mps',.5));start=(obs['x'],obs['y'])
        end=(obs['x']+dx*travel/distance,obs['y']+dy*travel/distance)
        obstacles=[o for o in model.obstacles if o is not obs]+[dict(x=robot.x,y=robot.y,radius=radius)]
        scene=SimpleNamespace(walls=model.walls,virtual_walls=model.virtual_walls,
            area_records=model.area_records,obstacles=obstacles)
        _,reason=path_clearance(scene,start,[end],obs['radius'],travel)
        if reason:
            obs['_motion']='통행 대기'
            blockers=[]
            for other in model.obstacles:
                if other is obs or not other.get('dynamic'):continue
                test=SimpleNamespace(walls=[],virtual_walls=[],area_records=[],obstacles=[other])
                if path_clearance(test,start,[end],obs['radius'],travel)[1]:blockers.append(other)
            if blockers and obs.get('encounter_policy','reverse')=='reverse':
                key=id(blockers[0])
                if obs.get('_blocked_by')!=key:obs['_blocked_by']=key;obs['_blocked_elapsed']=0.
                obs['_blocked_elapsed']=obs.get('_blocked_elapsed',0.)+dt
                if obs['_blocked_elapsed']>=obs.get('encounter_wait_s',2.):
                    if actor_can_reverse(model,obs,points,index,direction):
                        obs['_goal']=index-direction;obs['_direction']=-direction;obs['_dwell']=.25
                        obs['_blocked_elapsed']=0.;obs['_motion']='다른 장애물 회피 · 방향 전환'
                    else:obs['_motion']='반대 방향 경로 없음 · 통행 대기'
            else:obs['_blocked_elapsed']=0.;obs.pop('_blocked_by',None)
            continue
        obs['_blocked_elapsed']=0.;obs.pop('_blocked_by',None)
        obs['x'],obs['y']=end;obs['_heading']=math.atan2(dy,dx)
        obs['_motion']='걷는 중' if obs['kind']=='person' else '주행 중'
        if travel>=distance-1e-8:
            obs['_goal']=next_actor_index(points,index,direction)
            obs['_dwell']=obs.get('dwell_s',.5) if index in stops or index==0 and len(points)-1 in stops else 0
