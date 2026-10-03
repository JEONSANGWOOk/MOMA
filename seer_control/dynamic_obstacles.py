"""SIM actors: repeat a taught segment, with swept collision checks."""
import math
from types import SimpleNamespace
from .studio_core import path_clearance

POLICIES={'wait':'대기 후 기존 경로로 진행','avoid':'장애물 피해서 가기',
          'stop':'정지하기 (수동 재개)','adaptive':'동적 대기 / 정적 우회',
          'reroute':'정적 대기 → 다른 경로 / 불가 목적지 패스'}


def validate_actor(obs):
    if 'dynamic' in obs and type(obs['dynamic']) is not bool:raise ValueError('dynamic은 true/false입니다.')
    if not obs.get('dynamic'):return
    if 'paused' in obs and type(obs['paused']) is not bool:raise ValueError('paused는 true/false입니다.')
    if obs.get('kind') not in ('person','amr'):raise ValueError('동적 장애물 종류: person / amr')
    pts=obs.get('motion_path')
    if not isinstance(pts,list) or len(pts)!=2:raise ValueError('동적 장애물 경로는 시작/끝 두 점입니다.')
    for p in pts:
        if not isinstance(p,(list,tuple)) or len(p)!=2 or not all(math.isfinite(float(v)) for v in p):raise ValueError('동적 장애물 경로 좌표 오류')
    if math.dist(*pts)<.05:raise ValueError('동적 장애물 이동 거리는 0.05m 이상입니다.')
    speed=float(obs.get('speed_mps',.5));dwell=float(obs.get('dwell_s',.5))
    if not math.isfinite(speed) or not .01<=speed<=2:raise ValueError('장애물 속도: 0.01~2m/s')
    if not math.isfinite(dwell) or not 0<=dwell<=60:raise ValueError('끝점 대기: 0~60초')
    obs['speed_mps']=speed;obs['dwell_s']=dwell
    obs['motion_path']=[[float(v) for v in p] for p in pts]


def actor_data(obs):
    result={k:v for k,v in obs.items() if not k.startswith('_')}
    if obs.get('dynamic'):result['x'],result['y']=obs['motion_path'][0]
    return result


def update_actors(model,dt,robot,radius):
    for obs in model.obstacles:
        if not obs.get('dynamic'):continue
        if getattr(model,'dynamic_paused',False) or obs.get('paused',False):
            obs['_motion']='일시정지';continue
        if obs.get('_dwell',0)>0:
            obs['_dwell']=max(0,obs['_dwell']-dt);obs['_motion']='끝점 대기';continue
        goal=obs['motion_path'][obs.get('_goal',1)]
        dx,dy=goal[0]-obs['x'],goal[1]-obs['y'];distance=math.hypot(dx,dy)
        if distance<1e-8:
            obs['_goal']=1-obs.get('_goal',1);obs['_dwell']=obs.get('dwell_s',.5);continue
        travel=min(distance,dt*obs.get('speed_mps',.5));start=(obs['x'],obs['y'])
        end=(obs['x']+dx*travel/distance,obs['y']+dy*travel/distance)
        obstacles=[o for o in model.obstacles if o is not obs]+[dict(x=robot.x,y=robot.y,radius=radius)]
        scene=SimpleNamespace(walls=model.walls,virtual_walls=model.virtual_walls,
            area_records=model.area_records,obstacles=obstacles)
        _,reason=path_clearance(scene,start,[end],obs['radius'],travel)
        if reason:obs['_motion']='통행 대기';continue
        obs['x'],obs['y']=end;obs['_heading']=math.atan2(dy,dx)
        obs['_motion']='걷는 중' if obs['kind']=='person' else '주행 중'
        if travel>=distance-1e-8:
            obs['_goal']=1-obs.get('_goal',1);obs['_dwell']=obs.get('dwell_s',.5)
