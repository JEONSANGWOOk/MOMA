"""UI-independent editing, geometry, calibration, recording and action execution."""
import copy
import json
import math
from pathlib import Path

DEFAULT_MODEL = dict(length=.55, width=.45, radius=.23, maxspeed=.45, maxacc=.6,
                     maxdec=.8, maxrot=.8, wheel_scale=1., laser_x=0., laser_y=0., laser_yaw=0.)
ACTION_DEFAULTS = {
    'Path Nav': dict(goal='', delay_ms=0, timeout_s=120),
    'Translation': dict(distance_m=1., speed_mps=.1, timeout_s=30),
    'Rotation': dict(angle_deg=90., speed_dps=30., timeout_s=30),
    'Wait': dict(duration_s=1., timeout_s=60),
    'Wait DI Trigger': dict(channel=0, value=True, timeout_s=30),
    'Set DO': dict(channel=0, value=True, timeout_s=10),
    'Branch DI': dict(channel=0, value=True, target=1, timeout_s=10),
    'Custom Action': dict(operation='', payload={}, timeout_s=30),
    'Arm Action': dict(operation='work', duration_s=2., timeout_s=60),
}


def finite(value, name='value'):
    if isinstance(value, bool):raise ValueError(f'{name}: 숫자가 필요합니다.')
    value=float(value)
    if not math.isfinite(value):raise ValueError(f'{name}: 유한한 숫자가 필요합니다.')
    return value


def validate_model(data):
    out=dict(DEFAULT_MODEL)
    for k in out:
        if k in data:out[k]=finite(data[k],k)
    for k in ('length','width','radius','maxspeed','maxacc','maxdec','maxrot','wheel_scale'):
        if out[k]<=0:raise ValueError(f'{k}: 0보다 커야 합니다.')
    return out


def point_segment_distance(x,y,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    t=max(0.,min(1.,((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy))) if dx or dy else 0.
    return math.hypot(x-a[0]-t*dx,y-a[1]-t*dy)


def in_polygon(x,y,pts):
    inside=False
    for a,b in zip(pts,pts[1:]+pts[:1]):
        if point_segment_distance(x,y,a,b)<1e-9:return True
        if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:inside=not inside
    return inside


def enabled(value):
    return value is True or str(value).lower() in ('1','true','yes')


def collision_reason(model,x,y,radius):
    for wall in list(model.walls)+list(getattr(model,'virtual_walls',[])):
        if point_segment_distance(x,y,wall[:2],wall[2:])<=radius:return '벽/가상벽'
    for obs in getattr(model,'obstacles',[]):
        if math.hypot(x-obs['x'],y-obs['y'])<=radius+obs.get('radius',.25):
            if obs.get('dynamic'):return '동적 장애물 · '+('사람' if obs.get('kind')=='person' else 'AMR')+' '+str(obs.get('id',''))
            return '배치 장애물'
    for area in getattr(model,'area_records',[]):
        pts=area.get('points',[])
        if len(pts)<3 or not enabled((area.get('properties') or {}).get('forbidden')):continue
        if in_polygon(x,y,pts) or any(point_segment_distance(x,y,a,b)<=radius for a,b in zip(pts,pts[1:]+pts[:1])):
            return '금지구역 '+str(area.get('id',''))
    return ''


def zone_limits(model,x,y,limits):
    out=dict(limits)
    for area in getattr(model,'area_records',[]):
        pts=area.get('points',[])
        if len(pts)>=3 and in_polygon(x,y,pts):
            for k in ('maxspeed','maxacc','maxdec','maxrot'):
                v=(area.get('properties') or {}).get(k)
                if v is not None:
                    value=max(0,finite(v,k))
                    if k=='maxrot':value=math.radians(value)
                    out[k]=min(out[k],value)
    return out


def path_clearance(model,start,points,radius,horizon):
    """Sample the swept circular footprint, including between curve samples."""
    traveled=0.;prev=start
    for point in points:
        distance=math.dist(prev,point)
        steps=max(1,math.ceil(min(distance,max(0,horizon-traveled))/.025))
        for i in range(steps+1):
            d=min(distance,max(0,horizon-traveled))*i/steps
            t=d/distance if distance else 0
            x=prev[0]+t*(point[0]-prev[0]);y=prev[1]+t*(point[1]-prev[1])
            reason=collision_reason(model,x,y,radius)
            if reason:return traveled+d,reason
        traveled+=distance
        if traveled>=horizon:break
        prev=point
    return float('inf'),''


class EditHistory:
    def __init__(self,limit=80):self.limit=limit;self.undo_stack=[];self.redo_stack=[]
    def capture(self,model):return copy.deepcopy(model.__dict__)
    def commit(self,before,model,label):
        after=self.capture(model)
        if before==after:return False
        self.undo_stack.append((before,after,label));self.undo_stack=self.undo_stack[-self.limit:]
        self.redo_stack.clear();return True
    def undo(self,model):
        if not self.undo_stack:return None
        item=self.undo_stack.pop();self.redo_stack.append(item)
        model.__dict__.clear();model.__dict__.update(copy.deepcopy(item[0]));return item[2]
    def redo(self,model):
        if not self.redo_stack:return None
        item=self.redo_stack.pop();self.undo_stack.append(item)
        model.__dict__.clear();model.__dict__.update(copy.deepcopy(item[1]));return item[2]


def rigid_calibration(pairs):
    """Least-squares 2D sensor-to-reference transform from paired coordinates."""
    if len(pairs)<2:raise ValueError('서로 다른 대응점이 2개 이상 필요합니다.')
    src=[(finite(p[0]),finite(p[1])) for p in pairs];dst=[(finite(p[2]),finite(p[3])) for p in pairs]
    sx=sum(p[0] for p in src)/len(src);sy=sum(p[1] for p in src)/len(src)
    tx=sum(p[0] for p in dst)/len(dst);ty=sum(p[1] for p in dst)/len(dst)
    dot=sum((a[0]-sx)*(b[0]-tx)+(a[1]-sy)*(b[1]-ty) for a,b in zip(src,dst))
    cross=sum((a[0]-sx)*(b[1]-ty)-(a[1]-sy)*(b[0]-tx) for a,b in zip(src,dst))
    if math.hypot(dot,cross)<1e-12:raise ValueError('보정점 배치가 퇴화되어 회전을 계산할 수 없습니다.')
    yaw=math.atan2(cross,dot);c,s=math.cos(yaw),math.sin(yaw)
    dx,dy=tx-c*sx+s*sy,ty-s*sx-c*sy
    rms=math.sqrt(sum((c*a[0]-s*a[1]+dx-b[0])**2+(s*a[0]+c*a[1]+dy-b[1])**2 for a,b in zip(src,dst))/len(src))
    return dict(x=dx,y=dy,yaw=yaw,rms=rms)


class Recorder:
    def __init__(self,limit=36000):self.frames=[];self.limit=limit;self.recording=False
    def append(self,t,state,extra=None):
        if self.recording:
            self.frames.append(dict(t=t,state=copy.deepcopy(state),extra=copy.deepcopy(extra or {})))
            if len(self.frames)>self.limit:del self.frames[:len(self.frames)-self.limit]
    def save(self,path):Path(path).write_text(json.dumps(dict(format='seer-studio-record-v1',frames=self.frames),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    def load(self,path):
        data=json.loads(Path(path).read_text(encoding='utf-8-sig'))
        if data.get('format')!='seer-studio-record-v1' or not isinstance(data.get('frames'),list):raise ValueError('운행 기록 형식이 아닙니다.')
        frames=data['frames']
        for f in frames:
            finite(f['t'],'t')
            for key in ('x','y','theta'):finite(f['state'][key],key)
        if any(b['t']<a['t'] for a,b in zip(frames,frames[1:])):raise ValueError('운행 기록 시간이 역순입니다.')
        self.frames=frames[-self.limit:];self.recording=False


def flatten_actions(chain):
    return [copy.deepcopy(a) for t in chain.get('tasks',[]) if t.get('checked',True)
            for g in t.get('groups',[]) if g.get('checked',True) for a in g.get('actions',[])]


def validate_actions(actions):
    if not actions:raise ValueError('실행할 Action이 없습니다.')
    out=[]
    for i,action in enumerate(actions):
        typ=action.get('type')
        if typ not in ACTION_DEFAULTS:raise ValueError(f'{i+1}: 지원하지 않는 Action {typ}')
        a=dict(type=typ,**ACTION_DEFAULTS[typ]);a.update(copy.deepcopy(action))
        a['timeout_s']=finite(a['timeout_s'],'timeout_s')
        if a['timeout_s']<=0:raise ValueError('timeout_s는 0보다 커야 합니다.')
        if typ in ('Set DO','Wait DI Trigger','Branch DI'):
            if type(a['channel']) is not int or not 0<=a['channel']<64:raise ValueError('I/O 채널: 0~63 정수')
            if type(a['value']) is not bool:raise ValueError('I/O value: true/false')
        if typ=='Branch DI' and (type(a['target']) is not int or not 1<=a['target']<=len(actions)):raise ValueError('분기 target: 1부터 시작하는 단계 번호')
        if typ=='Path Nav' and 'route_nodes' in a:
            route=a['route_nodes']
            if not isinstance(route,list) or len(route)<2 or any(not isinstance(key,str) or not key for key in route) or route[-1]!=a['goal']:
                raise ValueError('Path Nav route_nodes는 시작부터 목적지까지의 노드 목록이어야 합니다.')
        for key in ('distance_m','angle_deg','speed_mps','speed_dps','duration_s','delay_ms'):
            if key in a:a[key]=finite(a[key],key)
        if a.get('speed_mps',.1)<=0 or a.get('speed_dps',30)<=0 or a.get('duration_s',0)<0 or a.get('delay_ms',0)<0:raise ValueError('속도/시간 설정 오류')
        out.append(a)
    return out


class MissionRunner:
    """One action per transition. No automatic retry of device commands."""
    def __init__(self,adapter):
        self.adapter=adapter;self.actions=[];self.status='IDLE';self.index=0
        self.cycle=0;self.repeat=1;self.started=0.;self.dwell_until=None;self.entered=False
        self.error='';self.elapsed=0.;self.paused_at=None
        self.skipped=[];self.inactive_elapsed=None
    @property
    def active(self):return self.status in ('RUNNING','PAUSED')
    def start(self,actions,repeat=1,index=0):
        if self.active:raise ValueError('미션이 이미 실행 중입니다.')
        actions=validate_actions(actions)
        if not 0<=index<len(actions):raise ValueError('재개 단계가 범위를 벗어났습니다.')
        if type(repeat) is not int or repeat<0:raise ValueError('반복 횟수: 0 이상 정수')
        self.actions=actions;self.repeat=repeat;self.index=index;self.cycle=0
        self.status='RUNNING';self.entered=False;self.dwell_until=None;self.error='';self.elapsed=0
        self.skipped=[];self.inactive_elapsed=None
        for a in self.actions:a['status']='대기'
        hook=getattr(type(self.adapter),'mission_started',None)
        if hook:hook(self.adapter,index)
    def tick(self,now,dt):
        if self.status!='RUNNING':return
        a=self.actions[self.index]
        try:
            if not self.entered:
                self.started=now;self.entered=True;a['status']='실행 중'
                self.adapter.begin(a)
            self.elapsed=now-self.started
            timeout_elapsed=self.elapsed;timed_result=None
            hook=getattr(type(self.adapter),'navigation_timeout_elapsed',None)
            self.inactive_elapsed=None
            if hook:
                measured=hook(self.adapter,a,self.elapsed)
                if measured is not None:
                    timeout_elapsed=0. if self.dwell_until is not None else measured
                    self.inactive_elapsed=timeout_elapsed
            if timeout_elapsed>a['timeout_s']:
                handler=getattr(type(self.adapter),'navigation_timeout_result',None)
                if handler:timed_result=handler(self.adapter,a)
                if not timed_result:raise TimeoutError(f"{self.index+1}단계 {a['type']} 시간 초과")
            if timed_result is not None:done=timed_result
            elif self.dwell_until is not None:
                if now<self.dwell_until:return
                done=True
            else:
                done=self.adapter.poll(a,dt,self.elapsed)
            skipped=isinstance(done,dict) and done.get('skip') is True
            if done and not skipped and self.dwell_until is None and a.get('delay_ms',0):self.dwell_until=now+a['delay_ms']/1000.;return
            if not done:return
            skipped=isinstance(done,dict) and done.get('skip') is True
            a['status']='패스' if skipped else '완료'
            if skipped:
                self.skipped.append(dict(goal=done['goal'],reason=done['reason'],cycle=self.cycle+1,step=self.index+1))
                self.skipped=self.skipped[-300:]
            target=self.adapter.branch_target(a) if a['type']=='Branch DI' else None
            self.index=target-1 if target is not None else self.index+1
            if skipped:
                while self.index<len(self.actions) and self.actions[self.index]['type']!='Path Nav':
                    self.actions[self.index]['status']='목적지 미도착으로 생략';self.index+=1
            self.entered=False;self.dwell_until=None
            if self.index>=len(self.actions):
                self.cycle+=1
                if self.repeat and self.cycle>=self.repeat:self.status='COMPLETED';return
                self.index=0
                for item in self.actions:item['status']='대기'
        except Exception as e:
            self.error=str(e);a['status']='실패';self.status='FAILED'
            try:self.adapter.cancel()
            except Exception as stop_error:self.error+=' · 정지 요청 실패: '+str(stop_error)
    def pause(self,now):
        if self.status=='RUNNING':self.status='PAUSED';self.paused_at=now;self.adapter.pause()
    def resume(self,now):
        if self.status=='PAUSED':
            delta=now-self.paused_at;self.started+=delta
            if self.dwell_until is not None:self.dwell_until+=delta
            self.status='RUNNING';self.adapter.resume()
    def cancel(self):
        if self.active:self.actions[self.index]['status']='취소됨'
        try:self.adapter.cancel()
        finally:self.status='CANCELED';self.entered=False
