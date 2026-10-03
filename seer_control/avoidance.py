"""Bounded local A* for SIM only. Walls and forbidden areas remain solid."""
import heapq
import math
from .studio_core import collision_reason,path_clearance


def clear_segment(model,a,b,radius):
    return not path_clearance(model,a,[b],radius,math.dist(a,b))[1]


def detour(model,start,goal,radius,max_cells=24000):
    """Return collision-checked world points; None means wait, never teleport."""
    radius+=.04
    if collision_reason(model,*start,radius) or collision_reason(model,*goal,radius):return None
    if clear_segment(model,start,goal,radius):return [start,goal]
    points=[start,goal]+[(n['x'],n['y']) for n in model.nodes.values()]
    points.extend((w[i],w[i+1]) for w in list(model.walls)+list(getattr(model,'virtual_walls',[])) for i in (0,2))
    points.extend((o['x'],o['y']) for o in getattr(model,'obstacles',[]))
    margin=max(1.,radius*3);xs,ys=zip(*points)
    x0,y0=min(xs)-margin,min(ys)-margin;x1,y1=max(xs)+margin,max(ys)+margin
    step=max(.12,max(x1-x0,y1-y0)/150)
    nx,ny=math.ceil((x1-x0)/step),math.ceil((y1-y0)/step)
    def world(cell):return x0+cell[0]*step,y0+cell[1]*step
    def cell(p):return round((p[0]-x0)/step),round((p[1]-y0)/step)
    occupied={};edges={}
    def free(c):
        if c not in occupied:occupied[c]=0<=c[0]<=nx and 0<=c[1]<=ny and not collision_reason(model,*world(c),radius)
        return occupied[c]
    def edge(a,b):
        key=tuple(sorted((a,b)))
        if key not in edges:edges[key]=clear_segment(model,world(a),world(b),radius)
        return edges[key]
    def connect(p):
        c=cell(p)
        candidates=[(c[0]+dx,c[1]+dy) for dx in range(-2,3) for dy in range(-2,3)]
        return [v for v in sorted(candidates,key=lambda v:math.dist(p,world(v))) if free(v) and clear_segment(model,p,world(v),radius)][:4]
    starts=connect(start);ends=set(connect(goal))
    if not starts or not ends:return None
    costs={c:math.dist(start,world(c)) for c in starts};parents={c:None for c in starts}
    queue=[(costs[c]+math.dist(world(c),goal),costs[c],c) for c in starts];heapq.heapify(queue)
    visited=set();finish=None
    while queue and len(visited)<max_cells:
        _,cost,c=heapq.heappop(queue)
        if c in visited:continue
        visited.add(c)
        if c in ends:finish=c;break
        for dx,dy in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
            nxt=(c[0]+dx,c[1]+dy)
            if nxt in visited or not free(nxt) or not edge(c,nxt):continue
            value=cost+step*math.hypot(dx,dy)
            if value<costs.get(nxt,float('inf')):
                costs[nxt]=value;parents[nxt]=c;heapq.heappush(queue,(value+math.dist(world(nxt),goal),value,nxt))
    if finish is None:return None
    path=[goal];c=finish
    while c is not None:path.append(world(c));c=parents[c]
    path.append(start);path.reverse()
    # Remove grid corners only when the whole swept segment remains free.
    simplified=[path[0]];i=0
    while i<len(path)-1:
        j=len(path)-1
        while j>i+1 and not clear_segment(model,path[i],path[j],radius):j-=1
        simplified.append(path[j]);i=j
    return simplified
