"""Local map/trajectory archive. JSON data only; never executes remote Python."""
import copy
import json
import math
import threading
import time
from pathlib import Path
from concurrent.futures import Future


def latest_real_map(folder):
    folder=Path(folder);index=folder/'last_real_map.json'
    if index.exists():
        data=json.loads(index.read_text(encoding='utf-8'));candidate=Path(data['path'])
        if candidate.is_file() and candidate.suffix=='.smap':return candidate
    paths=list((folder/'maps').glob('*.smap'))
    if not paths:raise ValueError('저장된 실기 지도가 없습니다. 실기에서 지도를 한 번 다운로드하세요.')
    return max(paths,key=lambda p:p.stat().st_mtime_ns)


def use_last_real_map(app):
    from .app import USER_DIR
    from .smap import load_smap_data
    from .model import Simulator
    if app.real:raise ValueError('시뮬레이션 모드로 연결한 뒤 불러오세요.')
    if app.task_running or app.sim.route or app.arm_dev_sim.state in ('RUNNING','PAUSED'):raise ValueError('SIM 동작을 정지하고 지도를 변경하세요.')
    path=latest_real_map(USER_DIR)
    model=load_smap_data(json.loads(path.read_text(encoding='utf-8')),path.stem)
    app.map=model;app.sim=Simulator(model,decision_journal=getattr(app,'decision_journal',None));app.map_path=path
    app.tasks=[];app.task_running=False;app.scan_points=[];app.last_scan=0;app.current_robot_map=''
    if 'LM2' in model.nodes:
        node=model.nodes['LM2'];app.sim.state.x=node['x'];app.sim.state.y=node['y']
        app.sim.state.theta=float(node.get('r',node.get('angle',node.get('theta',0))) or 0)
        app.sim.state.last_node='LM2'
    app.refresh_tasks();app.refresh_nodes();app.fit_map();app._spatial_sync_cad_lidar()
    app.log('SIM','마지막 실기 지도 로컬 로드: '+str(path))


class ScenarioArchive:
    def __init__(self,folder):
        self.folder=Path(folder);self.lock=threading.Lock();self.active=None;self.frames=[];self.started=0.;self.metadata={}
    def path(self,name):
        if name not in ('main1','main2','main3'):raise ValueError('Unsupported scenario')
        return self.folder/(name+'.json')
    def begin(self,name):
        with self.lock:self.active=name;self.frames=[];self.started=time.monotonic();self.metadata={}
    def capture(self,view,pose,joints):
        with self.lock:
            if self.active is None:return
            if len(self.frames)>=36000:return
            stamp=time.monotonic()-self.started
            if self.frames and stamp-self.frames[-1]['t']<.03:return
            names=[j['name'] for j in view.arm_asset.movable()]
            if len(names)!=6 or not all(n in joints for n in names):return
            values=[joints[n] for n in names]
            if not all(type(v) in (int,float) and math.isfinite(v) for v in values):return
            if not self.metadata:
                self.metadata=dict(map_name=view.app.map.name,cad_origin=list(view.cad_origin),mount=list(view.mount),arm_model=view.arm_asset.name,door_initial_angles={('left' if part['name'].endswith(':1') else 'right'):math.degrees(view.cad_door_angle(part)) for part in getattr(view.cad_asset,'door_parts',())})
            controller=getattr(view,'_door_io7',None)
            if controller:self.metadata['do7_profile']=dict(hook_motion=math.degrees(controller.hook_motion),roller_motion=math.degrees(controller.roller_motion),roller_contact_gated=True,roller_target=controller.roller_target,closing_motion=math.degrees(controller.closing_motion),measured_rotation=controller.measured_rotation,closing_speed=controller.closing_speed)
            runtime=getattr(view.app,'runtime_arm_feedback',None)
            io={str(i):runtime.signal('robot.vacuum_io.outputs.'+str(i)) for i in (2,3,4,5,7)} if runtime else {}
            from .door_io7 import read_opening_feedback
            signal,tool=read_opening_feedback(view.app,time.monotonic());io['7']=signal
            self.frames.append(dict(t=stamp,q=values,pose={k:pose[k] for k in ('x','y','theta')},io=io,door_tool=tool))
    def finish(self,name,success):
        with self.lock:
            if self.active!=name:return
            frames=self.frames;self.active=None;self.frames=[]
        # Never replace a known good recording with a failed/empty execution.
        if not success or len(frames)<2:return False
        data=dict(format='moma-offline-replay-v1',scenario=name,units='metres-radians',metadata=dict(self.metadata),frames=frames)
        self.folder.mkdir(parents=True,exist_ok=True);path=self.path(name);temp=path.with_suffix('.tmp')
        temp.write_text(json.dumps(data,separators=(',',':')),encoding='utf-8');temp.replace(path)
        return True
    def load(self,name):
        path=self.path(name)
        if not path.exists():raise ValueError(name+' 실기 실행 기록이 없습니다. 예제로 대체하지 않습니다. 실기에서 3D 화면을 열고 한 번 실행·저장하거나 기록 JSON을 가져오세요.')
        data=json.loads(path.read_text(encoding='utf-8'));validate_recording(data,name);return data


def validate_recording(data,name=None):
    if data.get('format')!='moma-offline-replay-v1' or data.get('scenario') not in ('main1','main2','main3'):raise ValueError('오프라인 기록 형식 오류')
    if name and data['scenario']!=name:raise ValueError('시나리오 이름 불일치')
    frames=data.get('frames',[])
    if not 2<=len(frames)<=36000:raise ValueError('기록 프레임 수 오류')
    previous=-1.
    for frame in frames:
        stamp=frame.get('t');q=frame.get('q',[]);pose=frame.get('pose',{})
        values=[stamp]+list(q)+[pose.get(k) for k in ('x','y','theta')]
        if len(q)!=6 or not all(type(v) in (int,float) and math.isfinite(v) for v in values):raise ValueError('기록 좌표 오류')
        if not 0<=stamp<=3600 or stamp<=previous:raise ValueError('기록 시간 순서 오류')
        previous=stamp
    return data


def archive_for(app):
    if not hasattr(app,'_scenario_archive'):
        from .app import USER_DIR
        app._scenario_archive=ScenarioArchive(USER_DIR/'offline_scenarios')
    return app._scenario_archive


def start_replay(app,name):
    if app.real:raise ValueError('실기 모드에서 오프라인 재생을 할 수 없습니다.')
    data=archive_for(app).load(name);frames=data['frames'];arm=app.arm_dev_sim
    if arm.state in ('RUNNING','PAUSED'):raise ValueError('기존 SIM 팔 작업을 먼저 정지하세요.')
    metadata=data.get('metadata',{})
    if metadata.get('map_name') and metadata['map_name']!=app.map.name:raise ValueError('기록과 SIM 지도가 다릅니다. 마지막 실기 지도를 먼저 불러오세요.')
    for frame in frames:
        if any(abs(a-b)>1e-6 for a,b in zip(frame['q'],arm.kin.clamp(frame['q']))):raise ValueError('기록 관절값이 현재 SIM 모델 범위를 벗어납니다.')
    view=app.world3d
    view.sim_io7_feedback=any('7' in f.get('io',{}) for f in frames)
    view.sim_do7_profile=metadata.get('do7_profile',{})
    for key in ('cad_origin','mount'):
        values=metadata.get(key)
        if isinstance(values,list) and len(values)==6 and all(type(v) in (int,float) and math.isfinite(v) for v in values):setattr(view,key,list(values))
    initial=metadata.get('door_initial_angles',{})
    if initial:
        view.door_initial_angles={k:v for k,v in initial.items() if k in ('left','right') and type(v) in (int,float) and math.isfinite(v) and 0<=v<=160}
    if hasattr(view,'reset_real_doors'):view.reset_real_doors()
    # This is a local replay scene confirmation, never a real-LiDAR confirmation.
    if getattr(view,'cad_asset',None) is not None:
        from .equipment_placement import signature
        view.cad_placement_confirmation=dict(source='sim_operator_confirmed',signature=signature(app,view),time=time.time())
    future=Future();started=time.monotonic();sim=app.sim
    arm.state='RUNNING'
    app.arm_sim_owner='offline-replay';view.sim_contact_follow=True;view.cad_doors_auto.set(False)
    for key in ('arm_follow','real_doors_follow'):getattr(view,key).set(True)
    index=0
    def tick():
        nonlocal index
        if app.real or app.sim is not sim or app.arm_dev_sim is not arm or arm.state in ('CANCELED','FAILED'):
            app.arm_sim_owner='workspace';future.set_exception(ValueError('모드/지도 변경 · 오프라인 재생 취소'));return
        elapsed=time.monotonic()-started+frames[0]['t']
        while index+1<len(frames) and frames[index+1]['t']<=elapsed:index+=1
        frame=frames[index];values=frame['q']
        if index+1<len(frames):
            next_frame=frames[index+1];alpha=max(0.,min(1.,(elapsed-frame['t'])/(next_frame['t']-frame['t'])))
            values=[a+(b-a)*alpha for a,b in zip(values,next_frame['q'])]
        arm.q=arm.kin.clamp(values)
        if not sim.route:
            for k,v in frame['pose'].items():setattr(sim.state,k,v)
        view.sim_door_signal=frame.get('io',{}).get('7');view.sim_door_tool=frame.get('door_tool')
        for key,value in frame.get('io',{}).items():
            if key in ('2','3','4','5','7') and type(value) is bool:arm.io['DO'][int(key)]=int(value)
        app._aw_sync_main(render=False)
        if index==len(frames)-1:
            arm.state='COMPLETED';app.arm_sim_owner='workspace';future.set_result({'status':'success','source':'recorded-replay','scenario':name});return
        app.after(20,tick)
    app.after(0,tick);return future
