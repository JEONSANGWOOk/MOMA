"""Pose verification in metres/radians, independent of controller path units."""
import math

def arrival_pose(node,state,distance_tolerance=.05,angle_tolerance=math.radians(1)):
    def valid(value):return type(value) in (int,float) and math.isfinite(value)
    heading=node.get('r',node.get('angle',node.get('theta')))
    theta=state.get('theta',state.get('angle'))
    values=(node.get('x'),node.get('y'),heading,state.get('x'),state.get('y'),theta)
    if not all(valid(v) for v in values):return dict(ok=False,distance_m=None,angle_rad=None)
    distance=math.hypot(state['x']-node['x'],state['y']-node['y'])
    delta=theta-heading;error=abs(math.atan2(math.sin(delta),math.cos(delta)))
    return dict(ok=distance<=distance_tolerance and error<=angle_tolerance,distance_m=distance,angle_rad=error)
