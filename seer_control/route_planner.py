"""Plan through connected directed map paths; never invent diagonal links."""
from .decision_log import decide
import heapq
import math


def adjacency(model):
    graph={key:[] for key in model.nodes}
    records=getattr(model,'path_records',[]) or []
    pairs=[(r.get('a'),r.get('b'),r) for r in records] if records else [(a,b,None) for a,b in model.edges]+[(b,a,None) for a,b in model.edges]
    for a,b,record in pairs:
        if a not in graph or b not in graph or a==b:continue
        length=model.distance(a,b)
        speed=getattr(model,'robot_model',{}).get('maxspeed',.45)
        configured=(record.get('properties') or {}).get('maxspeed') if record else None
        try:
            if configured is not None and math.isfinite(float(configured)) and float(configured)>0:speed=min(float(speed),float(configured))
            speed=max(.01,float(speed))
        except (TypeError,ValueError):speed=.45
        graph[a].append((b,length,length/speed))
    return graph


def shortest_path(graph,start,goal,policy='distance',excluded=()):
    excluded=set(excluded)
    if start not in graph or goal not in graph:raise ValueError('지도에 없는 노드입니다.')
    if start in excluded or goal in excluded:raise ValueError('방문 노드는 우회 제외 대상으로 설정할 수 없습니다.')
    queue=[(0.,start,[start],0.,0.)];seen=set()
    while queue:
        cost,key,path,distance,seconds=heapq.heappop(queue)
        if key in seen:continue
        seen.add(key)
        if key==goal:return dict(nodes=path,distance=distance,seconds=seconds)
        for nxt,length,duration in graph[key]:
            if nxt not in seen and nxt not in excluded:
                heapq.heappush(queue,(cost+(duration if policy=='time' else length),nxt,path+[nxt],distance+length,seconds+duration))
    raise ValueError('연결된 이동 경로가 없습니다.')


def plan_stops(model,stops,policy='distance',excluded=()):
    graph=adjacency(model);expanded=stops[:1];legs=[];errors=[];distance=seconds=0.
    unknown=set(excluded)-set(model.nodes)
    if unknown:errors.append('없는 제외 노드: '+', '.join(sorted(unknown)))
    for a,b in zip(stops,stops[1:]):
        try:
            leg=shortest_path(graph,a,b,policy,excluded)
            legs.append(dict(start=a,goal=b,**leg));expanded.extend(leg['nodes'][1:])
            distance+=leg['distance'];seconds+=leg['seconds']
        except ValueError as error:
            errors.append(f'{a} → {b}: {error}')
            legs.append(dict(start=a,goal=b,nodes=[],distance=0.,seconds=0.))
    result=dict(stops=list(stops),nodes=expanded,legs=legs,distance=distance,seconds=seconds,errors=errors,policy=policy,excluded=list(excluded))
    decide(model,'경로.방문 순서 계획',dict(stops=stops),dict(policy=policy,excluded=list(excluded),distance_m=distance,seconds=seconds,legs=legs,errors=errors),'연결 경로 계획 성공' if not errors else '일부 방문 구간 연결 불가','계획 경로 제시' if not errors else '이동 작업 생성 차단',force=True)
    return result


def plan_actions(plan,dwell_ms=0):
    if plan['errors']:raise ValueError('\n'.join(plan['errors']))
    actions=[]
    for leg in plan['legs']:
        path=leg['nodes']
        if len(path)>1:
            actions.append(dict(type='Path Nav',goal=path[-1],route_nodes=list(path),delay_ms=dwell_ms,
                                timeout_s=max(120.,leg['seconds']*3+30)))
    return actions


def upgrade_loop_chain(model,chain):
    """Migrate only our older, unedited per-edge generated loop actions."""
    if chain.get('routing_format')==2 or not chain.get('loop_stops') or not chain.get('routing_policy'):return False
    tasks=chain.get('tasks',[])
    if len(tasks)!=1 or len(tasks[0].get('groups',[]))!=1:return False
    group=tasks[0]['groups'][0];actions=group.get('actions',[])
    if not actions or any(a.get('type')!='Path Nav' or len(a.get('route_nodes',[]))!=2 for a in actions):return False
    if [a.get('goal') for a in actions]!=chain.get('loop_route',[])[1:]:return False
    plan=plan_stops(model,chain['loop_stops'],'time' if chain['routing_policy']=='예상 시간 우선' else 'distance',chain.get('excluded_nodes',[]))
    expected=[]
    for leg in plan['legs']:
        path=leg['nodes']
        expected.extend((b,chain.get('dwell_ms',0) if index==len(path)-2 else 0) for index,(a,b) in enumerate(zip(path,path[1:])))
    if [(a['goal'],a.get('delay_ms',0)) for a in actions]!=expected:return False
    group['actions']=plan_actions(plan,chain.get('dwell_ms',0))
    chain['loop_route']=plan['nodes'];chain['routing_format']=2
    return True
