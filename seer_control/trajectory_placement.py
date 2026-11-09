"""Full-link measured recording and bounded virtual door fitting. No robot API writes."""
import copy
import json
import math
import queue
import threading
import time
from pathlib import Path
from .geometry3d import transform,multiply,point
from .arm_physics import inverse
from .equipment_placement import signature
from .marker_placement import scope,solve
from .door_geometry import door_definitions
from .door_feedback import SurfaceDoorFollower


def tool_offsets(view):
    if not getattr(view.tool_asset,'name','').startswith('EndEffector V3_EndOnly_Change1-org'):raise ValueError('검증된 후크/롤러 엔드이펙터 모델이 필요합니다.')
    for name,(lo,hi) in getattr(view.tool_asset,'component_bounds',{}).items():
        if 'Polymer_ROECRHA50-10-30-T5' in name:
            scale=view.scales.get('tool',1.)
            roller=tuple((lo[i]+hi[i])/2*scale for i in range(3))
            from .door_teaching import hook_contact_offset
            return hook_contact_offset(),point(transform((0,0,.1)),roller),max(hi[i]-lo[i] for i in range(3))*scale/2
    raise ValueError('롤러 중심 형상을 찾을 수 없습니다.')


def placement_scope(app):
    view=app.world3d
    from .door_teaching import reference
    return dict(door_io7_enabled=bool(getattr(view,'door_io7_enabled',False)),teaching_sha256=reference()['source_sha256'],base=scope(app,view),arm=getattr(view.arm_asset,'name',''),tool=getattr(view.tool_asset,'name',''),
                tool_scale=view.scales.get('tool',1.),assets=copy.deepcopy(getattr(app,'spatial_assets',{})))


class LinkRecording:
    def __init__(self,app):
        self.app=app;self.active=False;self.data=None;self.dropped=0;self.epoch=0
    def begin(self):
        app=self.app;view=app.world3d
        if self.active:raise ValueError('연동 전체 녹화가 이미 진행 중입니다.')
        if not app.real:raise ValueError('실기 연동에서만 측정 녹화합니다.')
        self.offsets=tool_offsets(view)
        if not door_definitions(view):raise ValueError('설비 문 모델을 불러오세요.')
        self.names=[j['name'] for j in view.arm_asset.movable()]
        runtime=getattr(app,'runtime_arm_feedback',None)
        if len(self.names)!=6 or not runtime or not runtime.joints(self.names,max_age=.5):raise ValueError('최신 Runtime 로봇팔 관절값이 필요합니다.')
        self.started=time.monotonic();self.dropped=0;self.active=True;self.epoch+=1
        self.data=dict(initial_origin=list(view.cad_origin),format='moma-full-link-v1',units='metres-radians',scope=placement_scope(app),
                       origin=list(view.cad_origin),frames=[],markers=[],complete=False,events=[])
        self._tick()
    def _tick(self):
        if not self.active:return
        app=self.app;view=app.world3d;now=time.monotonic()
        try:
            if not app.real or not app.connected or self.data['scope']!=placement_scope(app):raise ValueError('연결/지도/모델 설정 변경')
            if now-self.started>600 or len(self.data['frames'])>=20000:raise ValueError('전체 녹화 최대 10분 초과')
            state=app.current_state();runtime=app.runtime_arm_feedback
            joints=runtime.joints(self.names,max_age=.5)
            pose={k:state.get(k) for k in ('x','y','theta')}
            if now-app.last_state>1 or len(joints)!=6 or not all(type(v) in (int,float) and math.isfinite(v) for v in list(pose.values())+list(joints.values())):
                self.dropped+=1
            else:
                arm=multiply(transform((pose['x'],pose['y'],0),(0,0,pose['theta'])),transform(view.mount[:3],view.mount[3:]))
                j6=view.arm_asset.link_transforms(joints,arm).get('J6')
                if j6 is None:raise ValueError('J6 FK 좌표가 없습니다.')
                hook,roller,radius=self.offsets
                self.data['roller_radius']=radius
                from .door_io7 import read_opening_feedback
                signal,tool=read_opening_feedback(app,now)
                self.data['frames'].append(dict(t=now-self.started,pose=pose,q=[joints[n] for n in self.names],
                    hook=point(j6,hook),roller=point(j6,roller),confidence=app._confidence_value(state),
                    io={str(i):runtime.signal('robot.vacuum_io.outputs.'+str(i)) for i in (2,3,4,5,7)},j6_rotation=j6,door_tool=tool))
                self.data['frames'][-1]['io']['7']=signal
                sample=runtime.marker_observation()
                if sample and not any(m['id']==sample.get('capture_id') for m in self.data['markers']):
                    if abs(state.get('speed',1))<=.01:
                        self.data['markers'].append(dict(id=sample['capture_id'],t=now-self.started,matrix=multiply(arm,sample['matrix'])))
        except Exception as error:
            self.data['error']=str(error);self.finish(False);return
        epoch=self.epoch
        app.after(33,lambda:self._tick() if self.active and self.epoch==epoch else None)
    def event(self,kind,message):
        if self.active:self.data['events'].append(dict(t=time.monotonic()-self.started,kind=kind,message=message))
    def finish(self,complete):
        if not self.active:return None
        self.active=False;self.data['complete']=bool(complete and len(self.data['frames'])>=10 and not self.data.get('error'));self.data['dropped']=self.dropped
        from .app import USER_DIR
        folder=USER_DIR/'door_trajectory_sessions';folder.mkdir(parents=True,exist_ok=True)
        path=folder/(time.strftime('%Y%m%d-%H%M%S')+'-'+str(time.time_ns()%1000000)+'.json')
        self.data['duration']=time.monotonic()-self.started
        text=json.dumps(self.data,separators=(',',':'));path.write_text(text,encoding='utf-8')
        if self.data['complete']:
            temp=folder/'latest.tmp';temp.write_text(text,encoding='utf-8');temp.replace(folder/'latest.json')
        self.app.log('RECORD',f'연동 전체 녹화 저장: {path} · {len(self.data["frames"])} 프레임 · 완료={complete}')
        return path


def recorder_for(app):
    if not hasattr(app,'_link_recording'):app._link_recording=LinkRecording(app)
    return app._link_recording


def validate_record(data):
    if data.get('format')!='moma-full-link-v1' or data.get('complete') is not True:raise ValueError('정상 종료된 전체 연동 녹화가 필요합니다.')
    frames=data.get('frames',[])
    if not 10<=len(frames)<=20000:raise ValueError('녹화 프레임 수 오류')
    previous=-1.;max_gap=0.
    for f in frames:
        vals=[f.get('t')]+list(f.get('hook',[]))+list(f.get('roller',[]))+list(f.get('q',[]))+[f.get('pose',{}).get(k) for k in ('x','y','theta')]
        if len(vals)!=16 or any(type(v) not in (int,float) or not math.isfinite(v) for v in vals):raise ValueError('녹화 좌표 오류')
        if not 0<=f['t']<=600 or f['t']<=previous:raise ValueError('녹화 시간 순서 오류')
        if previous>=0:max_gap=max(max_gap,f['t']-previous)
        previous=f['t']
    if max_gap>.5 or data.get('dropped',0)>len(frames)*.1:raise ValueError('녹화 누락/수신 간격이 큽니다. 다시 녹화하세요.')
    return data


def fresh_follower(template):
    result=copy.copy(template)
    for key,value in template.__dict__.items():
        if key not in ('doors','_trees','_hook_trees','_triangles'):setattr(result,key,copy.deepcopy(value))
    return result


def evaluate(template,frames,origin,with_trace=False):
    follower=fresh_follower(template);world_to_cad=inverse(transform(origin[:3],origin[3:]))
    names=[n for n in follower.angles if n.endswith(':1')]
    if not names:raise ValueError('왼쪽 문 모델이 없습니다.')
    name=names[0];peak=0.;hook_peak=0.;roller_peak=0.;counts=dict(hook=0,roller=0,close=0)
    from .door_io7 import DoorIO7,motion_profile,contact_start_time
    controller=None
    if any('7' in f.get('io',{}) for f in frames):
        try:
            profile=motion_profile(dict(complete=True,frames=frames),contact_start_time(template,frames,origin))
            controller=DoorIO7(profile['hook_motion'],profile['roller_motion'],min(160.,profile['hook_motion']+profile['roller_motion']),profile['closing_motion'],measured_rotation=True)
        except (ValueError,KeyError):controller=DoorIO7(measured_rotation=True)
    last=-1.;trace=[]
    for i,frame in enumerate(frames):
        # Keep <=100ms samples for the same contact model used by live rendering.
        if frame['t']-last<.095 and i!=len(frames)-1:continue
        last=frame['t']
        hook=point(world_to_cad,frame['hook']);roller=point(world_to_cad,frame['roller'])
        if controller:controller.update(follower,hook,roller,frame.get('j6_rotation'),frame['t']+1.,frame.get('io',{}).get('7'),frame.get('door_tool'))
        else:follower.update_contacts(hook,roller,frame['t']+1.)
        angle=math.degrees(follower.angles[name]);peak=max(peak,angle)
        if with_trace:trace.append(dict(t=frame['t'],hook=frame['hook'],roller=frame['roller'],pose=frame['pose'],angle=angle))
        if follower.active==name:
            mode=follower.contact_mode
            if mode=='hook':counts['hook']+=1;hook_peak=max(hook_peak,angle)
            elif mode in ('push','open-angle'):counts['roller']+=1;roller_peak=max(roller_peak,angle)
            elif mode=='close-angle':counts['close']+=1
    final=math.degrees(follower.angles[name])
    ok=peak>=115 and counts['hook']>=3 and counts['roller']>=3 and roller_peak>=115 and roller_peak>=hook_peak+5 and counts['close']>=3 and final<=15
    grasp_error=0.
    if controller:
        first=next((f for f in frames if f.get('io',{}).get('7') is True),None)
        grasp_error=math.dist(point(world_to_cad,first['hook']),follower.doors[name]['handle']) if first else float('inf')
        ok=ok and grasp_error<=.03
    score=peak-200*grasp_error+min(counts['hook'],10)+min(counts['roller'],10)+min(counts['close'],10)-min(final,120)*.15
    result=dict(ok=ok,score=score,peak=peak,final=final,hook_peak=hook_peak,roller_peak=roller_peak,contacts=counts,initial_grasp_error_m=grasp_error,signal_driven=controller is not None)
    if with_trace:result['trace']=trace
    return result


def search(data,doors,origin,progress=lambda value:None):
    validate_record(data)
    # Do not fit only a partial trace to make a door appear successful.
    frames=data['frames'];conf=[f.get('confidence') for f in frames]
    if any(type(c) not in (int,float) or not math.isfinite(c) or c<.6 for c in conf):raise ValueError('낮은 AMR 위치 신뢰도가 포함돼 있습니다. 위치 정합 후 다시 녹화하세요.')
    template=SurfaceDoorFollower(doors,roller_radius=data.get('roller_radius',.025))
    best=None;seed=list(origin);tested=set();count=0
    for step,angle in ((.12,math.radians(6)),(.04,math.radians(2)),(.013333,math.radians(2/3))):
        local=[]
        for dx in (-step,0,step):
            for dy in (-step,0,step):
                for da in (-angle,0,angle):
                    p=list(seed);p[0]+=dx;p[1]+=dy;p[5]+=da
                    if math.hypot(p[0]-origin[0],p[1]-origin[1])>.25 or abs(p[5]-origin[5])>math.radians(12):continue
                    key=tuple(round(v,6) for v in p)
                    if key in tested:continue
                    tested.add(key);count+=1
                    metrics=evaluate(template,frames,p)
                    # Prefer a verified full sequence over high-angle partial fits.
                    rank=(metrics['ok'],metrics['score']-math.hypot(p[0]-origin[0],p[1]-origin[1])*10)
                    row=(rank,p,metrics);local.append(row)
                    if best is None or rank>best[0]:best=row
                    progress(count)
        if best:seed=list(best[1])
    if best is None:raise ValueError('탐색 후보가 없습니다.')
    trace=evaluate(template,frames,best[1],with_trace=True)['trace']
    door=next(d for d in doors if d['name'].endswith(':1'))
    return dict(origin=best[1],metrics=best[2],tested=count,scope=data['scope'],marker_reference=(data.get('markers') or [{}])[-1].get('matrix'),preview=trace,preview_door={k:door[k] for k in ('pivot','handle','sign')})


def apply_saved(app):
    link=getattr(app,'_departure_link',None)
    if link and link.thread and link.thread.is_alive():raise ValueError('연동 중에는 배치를 변경하지 않습니다.')
    config=getattr(app.world3d,'trajectory_placement',{})
    if not config.get('enabled'):return False
    if config.get('scope')!=placement_scope(app):raise ValueError('저장 배치와 지도/호스트/모델/장착 설정이 다릅니다. 다시 탐색하세요.')
    if config.get('metrics',{}).get('ok') is not True:raise ValueError('검증 조건을 충족한 배치가 아닙니다.')
    origin=config.get('origin',[])
    if len(origin)!=6 or any(type(v) not in (int,float) or not math.isfinite(v) for v in origin):raise ValueError('저장 배치 좌표 오류')
    view=app.world3d;view.cad_origin[:]=origin
    reference=config.get('marker_reference')
    # Subsequent main4 measurements relocate the learned placement as a fixed
    # marker->CAD transform. Without a marker keep the verified MAP placement.
    view.marker_placement=dict(enabled=bool(reference),mode='trajectory_registered',reference=reference,
        origin=list(config['origin']),scope=scope(app,view))
    view._marker_applied=None;view._marker_observation=None;view._marker_register_pending=False
    view.cad_placement_confirmation=dict(source='real_marker_registered',signature=signature(app,view))
    view.reset_real_doors();app._spatial_save()
    return True


def show_fit(app,auto_start=False,record_path=None):
    import tkinter as tk
    from tkinter import ttk,messagebox
    from .app import USER_DIR
    view=app.world3d;win=tk.Toplevel(app);win.title('전체 녹화 → 최초 설비 배치 탐색');win.geometry('780x620')
    status=tk.StringVar(value='정상 완료된 LM2→LM1 전체 녹화로 가상 문 열기/닫기를 평가합니다. 실제 로봇 성공을 보장하지 않습니다.')
    ttk.Label(win,textvariable=status,wraplength=700).pack(padx=15,pady=16)
    jobs=queue.Queue();state=dict(result=None,busy=False,scope=placement_scope(app))
    def start():
        if state['busy']:return
        try:
            path=Path(record_path) if record_path else USER_DIR/'door_trajectory_sessions'/'latest.json'
            data=validate_record(json.loads(path.read_text(encoding='utf-8')))
            if data['scope']!=placement_scope(app):raise ValueError('녹화 당시 지도/호스트/모델/장착 설정과 다릅니다.')
            doors=door_definitions(view);origin=list(data['origin'])
            if data.get('markers'):
                from .door_teaching import place_handle
                part=next((p for p in view.cad_asset.door_parts if p['name'].endswith(':1')),None)
                if part is None:raise ValueError('왼쪽 문 CAD 없음')
                origin=place_handle(data['markers'][-1]['matrix'],part,origin,view.scales['cad'])
            state.update(busy=True,result=None);status.set('탐색 중 · 실제 로봇에 명령하지 않습니다.')
            def work():
                try:jobs.put(('result',search(data,doors,origin,lambda n:jobs.put(('progress',n)))))
                except Exception as e:jobs.put(('error',str(e)))
            threading.Thread(target=work,daemon=True,name='door-placement-fit').start()
        except Exception as e:messagebox.showerror('배치 탐색',str(e),parent=win)
    def save():
        result=state['result']
        if not result or not result['metrics']['ok']:return
        if placement_scope(app)!=state['scope']:raise ValueError('설정 변경: 다시 탐색하세요.')
        old_config=getattr(view,'trajectory_placement',{})
        view.trajectory_placement=dict({k:v for k,v in result.items() if k not in ('preview','preview_door')},enabled=True)
        try:apply_saved(app)
        except Exception:
            view.trajectory_placement=old_config;raise
        view.refresh()
        status.set('검토된 배치를 저장했습니다. 다음 연동부터 자동 적용합니다. 마커가 기록된 경우 다음 main4에도 학습 관계를 유지합니다.')
    def safe_save():
        try:save()
        except Exception as e:messagebox.showerror('배치 저장',str(e),parent=win)
    ttk.Button(win,text='저장된 전체 녹화로 최초 위치 탐색',command=start).pack(pady=5)
    ttk.Button(win,text='탐색 결과 적용·저장 → 이후 연동 자동 배치',command=safe_save).pack(pady=5)
    ttk.Label(win,text='범위: 녹화 당시 배치 주변 XY 25cm / 각도 12° 이내. 높이 고정.\n기준: 후크 초기 열림 → 롤러 추가 열림 115° 이상 → 닫기 15° 이하.\n조건을 만족하지 못하면 자동 저장하지 않습니다. 가까운 초기 배치에서 다시 탐색하세요.',justify='left').pack(pady=15)
    canvas=tk.Canvas(win,height=240,bg='#f4f7fb');canvas.pack(fill='both',expand=True,padx=12,pady=4)
    cursor=tk.DoubleVar(value=0)
    def preview(value=None):
        canvas.delete('all');result=state['result']
        if not result or not result.get('preview'):return
        frames=result['preview'];index=max(0,min(len(frames)-1,int(cursor.get())));frame=frames[index]
        origin=result['origin'];world=transform(origin[:3],origin[3:]);door=result['preview_door']
        pivot=point(world,door['pivot']);handle=point(world,door['handle'])
        pts=[p for row in frames for p in (row['hook'],row['roller'])]+[pivot,handle]
        xmin,xmax=min(p[0] for p in pts),max(p[0] for p in pts);ymin,ymax=min(p[1] for p in pts),max(p[1] for p in pts)
        width,height=max(10,canvas.winfo_width()),max(10,canvas.winfo_height())
        scale=min((width-40)/max(.1,xmax-xmin),(height-40)/max(.1,ymax-ymin))
        def xy(p):return 20+(p[0]-xmin)*scale,height-20-(p[1]-ymin)*scale
        for field,color in (('hook','#cd812c'),('roller','#2394b4')):
            path=[v for row in frames for v in xy(row[field])]
            if len(path)>=4:canvas.create_line(*path,fill=color,width=1)
            x,y=xy(frame[field]);canvas.create_oval(x-4,y-4,x+4,y+4,fill=color,outline='')
        px,py=pivot[:2];hx,hy=handle[:2];angle=math.radians(frame['angle'])*door['sign'];c,s=math.cos(angle),math.sin(angle)
        end=(px+c*(hx-px)-s*(hy-py),py+s*(hx-px)+c*(hy-py))
        canvas.create_line(*xy(pivot),*xy(end),fill='#3a475a',width=5)
        bx,by=xy((frame['pose']['x'],frame['pose']['y']));canvas.create_oval(bx-5,by-5,bx+5,by+5,fill='#6284b8')
        canvas.create_text(8,8,anchor='nw',text=f"t={frame['t']:.1f}s · 문 {frame['angle']:.1f}° · 주황 후크 / 청록 롤러 / 파랑 AMR")
    slider=ttk.Scale(win,variable=cursor,from_=0,to=1,command=preview);slider.pack(fill='x',padx=15,pady=4)
    canvas.bind('<Configure>',preview)
    def poll():
        if not win.winfo_exists():return
        while not jobs.empty():
            kind,value=jobs.get_nowait()
            if kind=='progress':status.set(f'가상 궤적 평가 중 · {value}개 위치')
            elif kind=='error':state['busy']=False;status.set(value)
            else:
                state.update(result=value,busy=False);m=value['metrics'];p=value['origin']
                slider.configure(to=max(1,len(value.get('preview',[]))-1));cursor.set(0);preview()
                status.set(f'후보 X={p[0]:.3f} Y={p[1]:.3f} θ={math.degrees(p[5]):.2f}° · 최대 {m["peak"]:.1f}° / 마지막 {m["final"]:.1f}° · '+('가상 접촉 조건 충족 · 적용·저장 가능' if m['ok'] else '조건 미달 · 저장하지 않음'))
        win.after(100,poll)
    poll()
    if auto_start:win.after(100,start)
