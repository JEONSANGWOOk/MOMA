"""SIM graph rerouting: follow directed lanes, never fabricate diagonal links."""
import math
from types import SimpleNamespace
from .route_planner import adjacency,shortest_path
from .smap import path_record_geometry
from .avoidance import clear_segment
from .studio_core import collision_reason


def lane(model,a,b):
    record=next((r for r in getattr(model,'path_records',[]) if r.get('a')==a and r.get('b')==b),None)
    if record:return path_record_geometry(record.get('raw',{}))
    return [(model.nodes[a]['x'],model.nodes[a]['y']),(model.nodes[b]['x'],model.nodes[b]['y'])]


def split_reference(start,reference):
    candidates=[]
    for i,(a,b) in enumerate(zip(reference,reference[1:])):
        dx,dy=b[0]-a[0],b[1]-a[1];sq=dx*dx+dy*dy
        t=max(0,min(1,((start[0]-a[0])*dx+(start[1]-a[1])*dy)/sq)) if sq else 0
        point=(a[0]+t*dx,a[1]+t*dy);candidates.append((math.dist(start,point),i,point))
    if not candidates:return [],[]
    _,i,p=min(candidates)
    return [start,p]+list(reference[i+1:]),[start,p]+list(reversed(reference[:i+1]))


def alternate_route(model,start,goal,a,b,reference,radius,include_dynamic=False,safety_margin=.04,optimize=False):
    # Dynamic actors are waited for at execution, rather than closing map lanes.
    scene=SimpleNamespace(walls=model.walls,virtual_walls=model.virtual_walls,area_records=model.area_records,
        obstacles=[o for o in model.obstacles if include_dynamic or not o.get('dynamic')])
    original=adjacency(model);graph={key:[] for key in original}
    def clear(points):return bool(points) and all(clear_segment(scene,x,y,radius+safety_margin) for x,y in zip(points,points[1:]))
    def clear_prefix(points):
        if clear(points):return True
        # A close standstill can be inside the extra 4cm buffer without
        # physical overlap. Allow a checked escape along the existing lane.
        if collision_reason(scene,*start,radius) or not collision_reason(scene,*start,radius+.04):return False
        return bool(points) and all(clear_segment(scene,x,y,radius) for x,y in zip(points,points[1:]))
    for key,edges in original.items():
        for nxt,distance,seconds in edges:
            if clear(lane(model,key,nxt)):
                if optimize:
                    from .navigation_quality import route_cost
                    speed=distance/max(.001,seconds)
                    seconds=route_cost(model,lane(model,key,nxt),radius,speed)
                graph[key].append((nxt,distance,seconds))
    anchors=[];anchor_speeds={}
    def lane_speed(source,dest):
        edge=next(((d,t) for nxt,d,t in original.get(source,[]) if nxt==dest),None)
        return edge[0]/max(.001,edge[1]) if edge else model.robot_model['maxspeed']
    for key,node in model.nodes.items():
        if math.dist(start,(node['x'],node['y']))<1e-6 and not collision_reason(scene,*start,radius):anchors.append((key,[start]))
    forward,backward=split_reference(start,reference)
    if b in graph and forward and math.dist(forward[-1],(model.nodes[b]['x'],model.nodes[b]['y']))<=1e-5 and clear_prefix(forward):anchors.append((b,forward));anchor_speeds[id(forward)]=lane_speed(a,b)
    # Backtracking a directed lane requires an explicitly available reverse lane.
    if a in graph and b in original and any(nxt==a for nxt,_,_ in original[b]) and clear_prefix(backward):
        reverse,_=split_reference(start,lane(model,b,a))
        if reverse and math.dist(start,reverse[1])<=.05 and clear_prefix(reverse):anchors.append((a,reverse));anchor_speeds[id(reverse)]=lane_speed(b,a)
    # Recover from any directed lane containing the current pose, including
    # an interrupted return-to-anchor segment after earlier replanning.
    for key,edges in original.items():
        for nxt,_,_ in edges:
            suffix,_=split_reference(start,lane(model,key,nxt))
            if suffix and math.dist(start,suffix[1])<=.08 and clear_prefix(suffix):anchors.append((nxt,suffix));anchor_speeds[id(suffix)]=lane_speed(key,nxt)
    options=[]
    reachable=[]
    for anchor in graph:
        try:result=shortest_path(graph,anchor,goal,policy='time' if optimize else 'distance')
        except ValueError:continue
        reachable.append((anchor,result))
    routes=dict(reachable)
    for anchor,prefix in anchors:
        result=routes.get(anchor)
        if result is None:continue
        length=sum(math.dist(x,y) for x,y in zip(prefix,prefix[1:]))
        score=length+result['distance']
        if optimize:
            from .navigation_quality import route_cost
            score=route_cost(model,prefix,radius,anchor_speeds.get(id(prefix),model.robot_model['maxspeed']))+result['seconds']
        options.append((score,anchor,prefix,result['nodes']))
    if not options and safety_margin>0:
        # Try all lanes again without the extra buffer, never reducing the
        # actual robot footprint. This recovers physically passable corridors.
        result=alternate_route(model,start,goal,a,b,reference,radius,include_dynamic,safety_margin=0,optimize=optimize)
        if result:result['reduced_margin']=True
        return result
    if not options and reference:
        # A robot already in a free-space detour may be off every map lane.
        # Find a checked connector to a node that can actually reach the goal.
        from .studio_core import point_segment_distance
        on_lane=any(min((point_segment_distance(*start,x,y) for x,y in zip(lane(model,key,nxt),lane(model,key,nxt)[1:])),default=float('inf'))<=.08 for key,edges in original.items() for nxt,_,_ in edges)
        if not on_lane:
            from .avoidance import detour
            scene.nodes=model.nodes
            ranked=sorted(reachable,key=lambda pair:math.dist(start,(model.nodes[pair[0]]['x'],model.nodes[pair[0]]['y']))+pair[1]['distance'])
            for budget in (8000,24000):
                for anchor,result in ranked:
                    node=model.nodes[anchor];prefix=detour(scene,start,(node['x'],node['y']),radius,max_cells=budget,safety_margin=safety_margin,risk_aware=optimize)
                    if prefix:
                        length=sum(math.dist(x,y) for x,y in zip(prefix,prefix[1:]))
                        score=length+result['distance']
                        if optimize:
                            from .navigation_quality import route_cost
                            score=route_cost(model,prefix,radius,model.robot_model['maxspeed'])+result['seconds']
                        options.append((score,anchor,prefix,result['nodes']))
                if options:break
    if not options:return None
    _,anchor,prefix,nodes=min(options)
    return dict(anchor=anchor,prefix=prefix,nodes=nodes,reduced_margin=not clear(prefix),checked_edges=sum(len(edges) for edges in original.values()),anchor_candidates=len(anchors))
