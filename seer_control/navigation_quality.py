"""Bounded motion prediction and route costs for SIM navigation."""
import math
from .studio_core import point_segment_distance,collision_reason

def moving_obstacles(model):
    for o in model.obstacles:
        vx,vy=o.get('_observed_velocity',(0.,0.))
        if math.hypot(vx,vy)>=.04:yield o,vx,vy

def forecast_risk(model,p,radius):
    # Soft costs only: a forecast never replaces physical collision checks.
    risk=0.
    for o,vx,vy in moving_obstacles(model):
        start=(o['x'],o['y']);end=(o['x']+vx*2.5,o['y']+vy*2.5)
        clearance=point_segment_distance(*p,start,end)-radius-o['radius']
        risk=max(risk,max(0.,1.-clearance/.5))
    return min(3.,risk)

def projected_conflict(model,start,points,radius,speed,horizon=2.5):
    actors=list(moving_obstacles(model))
    if not actors or speed<=.001 or not points:return None
    path=[start]+list(points);index=1;ego=start;t=0.;distance=0.
    while t<horizon and index<len(path):
        dt=min(.15,horizon-t);travel=speed*dt;previous=ego
        while travel>1e-9 and index<len(path):
            goal=path[index];d=math.dist(ego,goal)
            if d<=travel:ego=goal;travel-=d;index+=1
            else:ego=(ego[0]+(goal[0]-ego[0])*travel/d,ego[1]+(goal[1]-ego[1])*travel/d);travel=0.
        for o,vx,vy in actors:
            rel0=(previous[0]-o['x']-vx*t,previous[1]-o['y']-vy*t)
            rel1=(ego[0]-o['x']-vx*(t+dt),ego[1]-o['y']-vy*(t+dt))
            if point_segment_distance(0.,0.,rel0,rel1)<=radius+o['radius']+.08:
                return dict(obstacle=o,time=t+dt,distance=distance+math.dist(previous,ego))
        distance+=math.dist(previous,ego);t+=dt
    return None

def route_cost(model,points,radius,speed=.45,heading=None,maxrot=.8):
    """Seconds plus soft exposure/turn costs; smaller is preferred."""
    length=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
    turn=0.;risk=0.;near=0.;previous=heading
    for a,b in zip(points,points[1:]):
        d=math.dist(a,b)
        if d<1e-8:continue
        angle=math.atan2(b[1]-a[1],b[0]-a[0])
        if previous is not None:turn+=abs(math.atan2(math.sin(angle-previous),math.cos(angle-previous)))
        previous=angle
        samples=max(1,math.ceil(d/.3))
        for i in range(1,samples+1):
            p=(a[0]+(b[0]-a[0])*i/samples,a[1]+(b[1]-a[1])*i/samples)
            risk+=forecast_risk(model,p,radius)*d/samples
            near+=(.8 if collision_reason(model,*p,radius+.18) else 0.)*d/samples
    return length/max(.05,speed)+turn/max(.1,maxrot)*.3+risk*3.+near/max(.05,speed)
