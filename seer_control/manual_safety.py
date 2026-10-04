"""Manual command guards; SIM footprint checks and real controller interlocks."""
import math
from .studio_core import path_clearance

def arc_pose(pose,v,w,seconds,scale=1.):
    x,y,theta=pose;velocity=v*scale;angle=theta+w*seconds
    if abs(w)<1e-8:return x+velocity*seconds*math.cos(theta),y+velocity*seconds*math.sin(theta),angle
    return x+velocity/w*(math.sin(angle)-math.sin(theta)),y-velocity/w*(math.cos(angle)-math.cos(theta)),angle

def manual_motion_reason(model,pose,v,w,radius):
    if abs(v)<1e-8:return ''  # Circular SIM footprint does not change in place.
    cfg=model.robot_model;scale=cfg.get('wheel_scale',1.)
    speed=abs(v*scale);deceleration=max(.05,cfg.get('maxdec',.8))
    # Command lease + braking distance + 8cm stopping allowance.
    horizon=.3+speed/(2*deceleration)+.08/max(.01,speed)
    count=max(1,math.ceil(horizon/.03))
    points=[arc_pose(pose,v,w,horizon*i/count,scale)[:2] for i in range(1,count+1)]
    distance,reason=path_clearance(model,pose[:2],points,radius+.04,speed*horizon+.001)
    return reason

def controller_manual_reason(state,last_state,now):
    if now-last_state>3:return '상태 데이터 유효기간 초과'
    if state.get('emergency') is True or state.get('stopped') is True:return '비상정지 / 정지 상태'
    if state.get('motor') is False:return '모터 OFF'
    if state.get('blocked') is True:return '제어기 장애물 감지'
    return ''
