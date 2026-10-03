"""SIM graph rerouting: follow directed lanes, never fabricate diagonal links."""
import math
from types import SimpleNamespace
from .route_planner import adjacency,shortest_path
from .smap import path_record_geometry
from .avoidance import clear_segment


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


def alternate_route(model,start,goal,a,b,reference,radius,include_dynamic=False):
    # Dynamic actors are waited for at execution, rather than closing map lanes.
    scene=SimpleNamespace(walls=model.walls,virtual_walls=model.virtual_walls,area_records=model.area_records,
        obstacles=[o for o in model.obstacles if include_dynamic or not o.get('dynamic')])
    original=adjacency(model);graph={key:[] for key in original}
    def clear(points):return bool(points) and all(clear_segment(scene,x,y,radius+.04) for x,y in zip(points,points[1:]))
    for key,edges in original.items():
        for nxt,distance,seconds in edges:
            if clear(lane(model,key,nxt)):graph[key].append((nxt,distance,seconds))
    anchors=[]
    for key,node in model.nodes.items():
        if math.dist(start,(node['x'],node['y']))<1e-6:anchors.append((key,[start]))
    forward,backward=split_reference(start,reference)
    if b in graph and clear(forward):anchors.append((b,forward))
    # Backtracking a directed lane requires an explicitly available reverse lane.
    if a in graph and b in original and any(nxt==a for nxt,_,_ in original[b]) and clear(backward):
        reverse,_=split_reference(start,lane(model,b,a))
        if reverse and math.dist(start,reverse[1])<=.05 and clear(reverse):anchors.append((a,reverse))
    options=[]
    for anchor,prefix in anchors:
        try:result=shortest_path(graph,anchor,goal)
        except ValueError:continue
        length=sum(math.dist(x,y) for x,y in zip(prefix,prefix[1:]))
        options.append((length+result['distance'],anchor,prefix,result['nodes']))
    if not options:return None
    _,anchor,prefix,nodes=min(options)
    return dict(anchor=anchor,prefix=prefix,nodes=nodes)
