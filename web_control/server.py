"""Local MOMA control service. No automatic physical connection or command replay."""
import asyncio
import copy
import ipaddress
import json
import math
import os
import secrets
import time
from contextlib import asynccontextmanager
from pathlib import Path
from urllib.parse import urlsplit

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from seer_control.door_sync import read_snapshot as read_door_snapshot
from .camera import CameraHub, realsense_url
from seer_control.live import SeerClient, extract_laser_scans
from seer_control.model import MapModel, Simulator
from seer_control.smap import load_smap_data
from seer_control.runtime_feedback import RuntimeArmFeedback
from seer_control.fairino_api import profile as arm_profile
from seer_control.fairino_client import FairinoClient
from seer_control.studio_devices import seer_io_operations, tcp_operation, parse_io
from seer_control.geometry3d import load_glb, load_glb_robot, load_glb_static_robot, identity, transform, multiply, point
from seer_control.door_feedback import SurfaceDoorFollower

ROOT=Path(__file__).resolve().parents[1]
SETTINGS=Path(os.environ.get('MOMA_WEB_SETTINGS',str(Path.home()/'.seer_amr_console'/'web_control.json')))
DESKTOP=Path.home()/'.seer_amr_console'/'studio_settings.json'


def read_json(path,default):
    try:return json.loads(path.read_text(encoding='utf-8-sig'))
    except (OSError,ValueError):return copy.deepcopy(default)


def restore_equipment_placement(viewer):
    """Prefer the persisted confirmed pose over later unconfirmed edits."""
    viewer=copy.deepcopy(viewer) if isinstance(viewer,dict) else {}
    record=viewer.get('cad_placement_confirmation') or {}
    if not isinstance(record,dict):record={}
    signature=record.get('signature',{})
    if not isinstance(signature,dict):signature={}
    origin=signature.get('origin') if record.get('source')=='real_lidar_operator_confirmed' else None
    valid=lambda v:isinstance(v,(list,tuple)) and len(v)==6 and all(type(x) in (int,float) and math.isfinite(x) for x in v)
    if valid(origin):viewer['cad_origin']=list(origin)
    elif not valid(viewer.get('cad_origin')):viewer['cad_origin']=[0.,0.,0.,0.,0.,0.]
    return viewer


def clean(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {str(k):clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    return value


class Connect(BaseModel):
    mode: str='sim'
    host: str='192.168.30.13'
    runtime_url: str='http://192.168.30.13:8000/runtime.html'

class Command(BaseModel):
    name: str
    goal: str=''
    channel: int=Field(default=0,ge=0,le=63)
    value: bool=False

class Doors(BaseModel):
    left: float=Field(ge=0,le=120)
    right: float=Field(ge=0,le=120)
    follow: bool=False

class ArmConnect(BaseModel):
    ip: str='192.168.58.2'
    sdk_path: str=''
    version: str=''

class ArmOperation(BaseModel):
    operation: str

class Camera(BaseModel):
    device: int=Field(default=0,ge=0,le=9)
    url: str=''
    source: str='realsense_remote'

class Mission(BaseModel):
    goals: list[str]=Field(min_length=1,max_length=30)


class Control:
    def __init__(self):
        self.config=read_json(SETTINGS,{})
        self.desktop=read_json(DESKTOP,{})
        self.viewer=restore_equipment_placement(self.desktop.get('viewer3d',{}))
        self.mode='sim';self.host='';self.connected=True;self.epoch=0
        self.state={};self.raw={};self.rx=0.;self.error=''
        self.laser=[];self.laser_rx=0.;self.laser_source='';self.laser_error='LiDAR 수신 대기';self.laser_raw_count=0
        self.lidar_alignment=(0.,0.,0.);self.lidar_mounts=[]
        self.map=MapModel.load(ROOT/'maps/demo.json');self.map_version=0
        self.sim=Simulator(self.map)
        self.profile=read_json(ROOT/'config/api_profile.json',{})
        self.client=None;self.runtime=None;self.runtime_rx=0.;self.joints=[0.,-math.pi/3,math.pi/2,0.,math.pi/3,0.]
        self.arm=FairinoClient();self.arm_config=self.desktop.get('arm',arm_profile())
        self.arm_state={};self.arm_rx=0.;self.arm_error='';self.arm_source='none'
        self.io={'di':{},'do':{}};self.io_valid={};self.io_rx=0.;self.io_error=''
        initial=self.config.get('doors',self.viewer.get('door_initial_angles',{}))
        self.doors={s:float(initial.get(s,0)) for s in ('left','right')}
        self.door_follow=False;self.door_contact=None
        self.door_source='web';self.desktop_revision=None;self._ignored_desktop_revision=None
        self.desktop_door_rx=0.;self.desktop_door_mode=''
        self.events=[];self.serial=0;self.command_state={'status':'IDLE'}
        self.owner=None;self.owner_until=0.;self.jobs=[]
        self.connection_lock=asyncio.Lock();self.command_lock=asyncio.Lock()
        self.arm_lock=asyncio.Lock();self.map_lock=asyncio.Lock()
        self.scene_lock=asyncio.Lock();self.scene_data=None;self.arm_model=None
        self.follower=None;self.placement_confirmed=bool(self.viewer.get('cad_placement_confirmation'))
        self.mission={'status':'IDLE','goals':[],'index':0,'awaiting':False}
        self.navigation_cancel_seq=0
        self.camera_source=self.config.get('camera_source','realsense_remote')
        runtime_url=self.config.get('runtime_url',self.viewer.get('runtime_url','http://192.168.30.13:8000/runtime.html'))
        self.camera_url=(realsense_url(self.viewer,runtime_url) if self.camera_source=='realsense_remote' else self.config.get('camera_url',''))
        self.realsense=CameraHub(realsense_url(self.viewer,runtime_url))
        self.camera=CameraHub(self.camera_url if self.camera_source!='realsense_remote' else '')
        self.model_paths={}
        self.event('SYSTEM','관제 서버 준비 · SIM 모드 · 실기 자동 연결 없음')
    def event(self,kind,message):
        self.serial+=1;self.events.insert(0,dict(id=self.serial,time=time.strftime('%H:%M:%S'),kind=kind,message=str(message)))
        self.events=self.events[:100]
    def save(self):
        SETTINGS.parent.mkdir(parents=True,exist_ok=True)
        self.config.update(doors=self.doors,camera_url=self.camera_url,camera_source=self.camera_source)
        tmp=SETTINGS.with_suffix('.tmp');tmp.write_text(json.dumps(self.config,ensure_ascii=False,indent=2),encoding='utf-8');os.replace(tmp,SETTINGS)
    def require_owner(self,session):
        if not session or self.owner!=session or time.monotonic()>self.owner_until:
            raise ValueError('운영 제어권을 먼저 가져오세요.')
        self.owner_until=time.monotonic()+20
    def require_live(self,write=True):
        if self.mode=='sim':
            if not self.connected:raise ValueError('SIM 연결을 먼저 실행하세요.')
            return
        if write and self.mode!='control':raise ValueError('실기 조회 전용 모드입니다.')
        if not self.connected or time.monotonic()-self.rx>3:raise ValueError('로봇 상태 만료 · 최신 수신을 확인하세요.')
        if write and (self.state.get('emergency') is not False or self.state.get('blocked') is not False):
            raise ValueError('비상정지·장애물 상태가 정상인지 확인하세요.')
    def snapshot(self):
        now=time.monotonic();fresh=self.connected and (self.mode=='sim' or now-self.rx<=3)
        arm_fresh=(self.arm_source!='none' and now-self.arm_rx<=2)
        return clean(dict(mode=self.mode,host=self.host,connected=fresh,error=self.error,
            state=self.state,state_age=round(now-self.rx,2) if self.rx else None,
            joints=self.joints if arm_fresh or self.mode=='sim' else None,
            arm=dict(connected=arm_fresh,source='simulation' if self.mode=='sim' else self.arm_source,status=self.arm_state,error=self.arm_error,
                operations=list(self.arm_config.get('operations',{}))),
            io=self.io if self.mode=='sim' or now-self.io_rx<=3 else {'di':{},'do':{}},
            io_valid=self.io_valid,io_error=self.io_error,io_fresh=self.mode=='sim' or now-self.io_rx<=3,
            doors=self.doors,door_follow=self.door_follow,door_sync={'source':self.door_source,'revision':self.desktop_revision,'connected':self.door_source=='desktop' and -.5<=time.time()-self.desktop_door_rx<=2,'age_s':round(time.time()-self.desktop_door_rx,2) if self.desktop_door_rx else None,'mode':self.desktop_door_mode},door_contact=self.door_contact,door_contact_info={'roller_radius_m':self.follower.roller_radius,'angular_velocity_rad_s':self.follower.angular_velocity,'penetration_m':self.follower.penetration} if self.follower else None,
            placement_confirmed=self.placement_confirmed,map_version=self.map_version,
            mission=self.mission,command=self.command_state,events=self.events[:30],
            control_owned=bool(self.owner and now<self.owner_until),camera_enabled=bool(self.camera_url),camera=(self.realsense if self.camera_source=='realsense_remote' else self.camera).status(),camera_source=self.camera_source,realsense=self.realsense.status(),
            laser=self.laser if fresh and now-self.laser_rx<=2 else [],
            lidar={'connected':fresh and bool(self.laser_rx) and now-self.laser_rx<=2,'source':self.laser_source,'count':self.laser_raw_count,'displayed':len(self.laser) if fresh and now-self.laser_rx<=2 else 0,'age_s':round(now-self.laser_rx,2) if self.laser_rx else None,'error':self.laser_error,'simulated':self.mode=='sim'}))
    async def connect(self,cfg):
        if cfg.mode not in ('sim','read','control'):raise ValueError('연결 모드 오류')
        ipaddress.IPv4Address(cfg.host)
        parts=urlsplit(cfg.runtime_url)
        if parts.scheme not in ('http','https') or not parts.hostname:raise ValueError('Runtime HTTP 주소 오류')
        async with self.connection_lock:
            if self.command_lock.locked() or self.mission['status']=='RUNNING':raise ValueError('명령/미션 종료 후 연결을 변경하세요.')
            self.epoch+=1;self.connected=False;self.rx=0;self.error='';self.io_rx=0
            self.laser=[];self.laser_rx=0;self.laser_source='';self.laser_error='LiDAR 수신 대기';self.laser_raw_count=0
            self.lidar_alignment=(0.,0.,0.);self.load_lidar_mounts(cfg.host)
            self.mode=cfg.mode;self.host=cfg.host
            self.placement_confirmed=False
            self.runtime=RuntimeArmFeedback(cfg.runtime_url)
            self.realsense.configure(realsense_url(self.viewer,cfg.runtime_url))
            if self.camera_source=='realsense_remote':
                self.camera_url=realsense_url(self.viewer,cfg.runtime_url);self.camera.configure('')
            self.arm_source='none';self.arm_rx=0
            if self.client:self.client.close()
            self.client=None
            self.mission={'status':'IDLE','goals':[],'index':0,'awaiting':False}
            if cfg.mode=='sim':
                self.map=MapModel.load(ROOT/'maps/demo.json');self.map_version+=1;self.sim=Simulator(self.map)
                self.connected=True;self.io={'di':self.sim.di,'do':self.sim.do}
            else:
                self.map=MapModel.load(ROOT/'maps/demo.json');self.map_version+=1
                self.client=SeerClient(cfg.host,self.profile);self.client.persistent_status=True
                self.state={};self.raw={}
            self.event('CONNECT',f'{cfg.mode.upper()} · {cfg.host}')
    async def disconnect(self):
        async with self.connection_lock:
            if self.command_lock.locked():raise ValueError('진행 중인 명령 응답을 먼저 확인하세요.')
            self.epoch+=1;self.connected=False;self.rx=0;self.arm_rx=0;self.laser=[];self.laser_rx=0;self.laser_raw_count=0
            if self.client:self.client.close()
            self.client=None;self.event('CONNECT','AMR 연결 해제')
    async def poll(self):
        last=time.monotonic()
        while True:
            now=time.monotonic()
            if self.mode=='sim' and self.connected:
                self.sim.tick(min(.2,now-last));self.state=self.sim.status();self.rx=now
                self.io={'di':dict(self.sim.di),'do':dict(self.sim.do)};self.io_rx=now
            elif self.client:
                async with self.connection_lock:
                    try:
                        state,raw=await asyncio.to_thread(self.client.snapshot)
                        self.state=state;self.raw=raw;self.rx=time.monotonic();self.connected=True;self.error=''
                    except Exception as e:self.error=str(e);self.connected=False
            last=now
            await asyncio.sleep(.2)
    def load_lidar_mounts(self,host):
        data=read_json(Path.home()/'.seer_amr_console'/'lidar_mounts.json',{})
        record=data.get(host,{}) if isinstance(data,dict) else {}
        self.lidar_mounts=record.get('mounts',[]) if isinstance(record,dict) else []
    def receive_lidar(self,payload,source,epoch):
        if epoch!=self.epoch:return False
        pose=tuple(self.state.get(k) for k in ('x','y','theta'))
        if not self.connected or time.monotonic()-self.rx>3 or not all(type(v) in (int,float) and math.isfinite(v) for v in pose):
            self.laser_error='최신 AMR 위치 수신 대기';return False
        scans=extract_laser_scans(payload,pose,self.lidar_mounts)
        points=[]
        for scan in scans:
            for x,y in scan.get('points',[]):
                if not scan.get('official_world'):
                    dx,dy,angle=self.lidar_alignment;c,s=math.cos(angle),math.sin(angle)
                    x,y=pose[0]+c*(x-pose[0])-s*(y-pose[1])+dx,pose[1]+s*(x-pose[0])+c*(y-pose[1])+dy
                if math.isfinite(x) and math.isfinite(y):points.append((x,y))
        if not scans and not any(k in payload for k in ('laser_beams','lasers','ranges','beams')):
            self.laser_error='LiDAR 응답에 점군 필드가 없습니다.';return False
        self.laser_raw_count=len(points)
        self.laser=points[::max(1,math.ceil(len(points)/65536))]
        self.laser_rx=time.monotonic();self.laser_source=source+' · '+' + '.join(scan.get('source','') for scan in scans)
        self.laser_error='';return True
    async def read_status_api(self,api,epoch):
        # Some SEER builds stop serving secondary sockets while a persistent
        # status socket is open. Reuse that socket, with strictly serialized IO.
        async with self.connection_lock:
            if epoch!=self.epoch or self.client is None:raise ValueError('조회 중 연결 변경')
            result=await asyncio.to_thread(self.client.request,dict(port=19204,request_type=api,response_type=api+10000,read_only=True))
            return dict(result,_response_type=api+10000)
    async def poll_lidar(self):
        while True:
            if self.mode=='sim':
                if self.connected:
                    self.laser=self.sim.scan();self.laser_raw_count=len(self.laser)
                    self.laser_rx=time.monotonic();self.laser_source='SIM · 합성 스캔';self.laser_error=''
                await asyncio.sleep(.2);continue
            if not self.client or not self.connected:
                await asyncio.sleep(.2);continue
            epoch=self.epoch;success=False;errors=[]
            for api in (1009,1101):
                try:
                    payload=await self.read_status_api(api,epoch)
                    if epoch!=self.epoch:break
                    if self.receive_lidar(payload,str(api),epoch):success=True;break
                except Exception as e:errors.append(str(e))
            if epoch==self.epoch and not success:self.laser_error=' / '.join(errors) or self.laser_error
            await asyncio.sleep(.2 if success else .8)
    async def poll_io(self):
        while True:
            if self.mode!='sim' and self.connected:
                host,epoch=self.host,self.epoch
                try:
                    r=await self.read_status_api(1013,epoch)
                    io,valid=parse_io(r,seer_io_operations()['read_io'])
                    if epoch==self.epoch:self.io=io;self.io_valid=valid;self.io_rx=time.monotonic();self.io_error=''
                except Exception as e:self.io_error=str(e)
            await asyncio.sleep(1)
    async def poll_arm(self):
        while True:
            try:
                epoch=self.epoch
                if self.arm.connected:
                    async with self.arm_lock:r=await asyncio.to_thread(self.arm.call,self.arm_config,'status')
                    joints=r['joints_rad'];source='sdk';self.arm_state=r
                elif self.mode!='sim' and self.runtime:
                    await asyncio.to_thread(self.runtime.poll)
                    vals=self.runtime.joints([f'J{i}' for i in range(1,7)])
                    joints=list(vals.values());source='runtime'
                    if len(joints)!=6:raise ValueError('FR5 관절 피드백 대기')
                else:
                    await asyncio.sleep(.2);continue
                if epoch==self.epoch:
                    self.joints=joints;self.arm_source=source;self.arm_rx=time.monotonic();self.arm_error=''
                    self.update_doors()
            except Exception as e:
                self.arm_error=str(e)
                if self.follower:self.follower.detach()
            await asyncio.sleep(.15)
    def update_doors(self):
        if self.door_source=='desktop':return
        if not (self.door_follow and self.placement_confirmed and self.arm_model and self.follower
                and self.connected and time.monotonic()-self.rx<=3):return
        from seer_control.arm_physics import inverse
        mount=self.viewer.get('mount',[.34,0,.626,0,0,0]);origin=self.viewer.get('cad_origin',[0]*6)
        base=transform((self.state.get('x',0),self.state.get('y',0),0),(0,0,self.state.get('theta',0)))
        poses=self.arm_model.link_transforms(dict(zip([f'J{i}' for i in range(1,7)],self.joints)),multiply(base,transform(mount[:3],mount[3:])))
        cad_inv=inverse(transform(origin[:3],origin[3:]));j6=poses['J6']
        hook=point(cad_inv,point(j6,(0,-.14714,.148)))
        roller=point(cad_inv,point(j6,self.roller_tip)) if getattr(self,'roller_tip',None) else None
        self.follower.update_contacts(hook,roller,time.monotonic())
        self.door_contact=self.follower.contact_mode if self.follower.active else None
        for name,a in self.follower.angles.items():
            side='left' if name.endswith(':1') else 'right';self.doors[side]=math.degrees(a)
    async def send(self,cmd,mission=False):
        if cmd.name in ('cancel','pause'):
            self.navigation_cancel_seq+=1
        if self.command_lock.locked():raise ValueError('이전 명령 응답 대기 중')
        if cmd.name not in ('navigate','pause','resume','cancel','set_do','lock_control','unlock_control'):
            raise ValueError('지원하지 않는 명령')
        if self.mission['status']=='RUNNING' and cmd.name in ('navigate','set_do') and not mission:raise ValueError('미션 종료 후 수동 조작하세요.')
        self.require_live()
        if cmd.name=='navigate':
            if cmd.goal not in self.map.nodes:raise ValueError('현재 지도에 없는 목적지')
            if self.mode!='sim' and not hasattr(self.map,'smap_source'):raise ValueError('실제 로봇 지도를 먼저 받으세요.')
            if self.mode!='sim':
                self.arm_travel_guard()
                node=self.map.nodes[cmd.goal];heading=node.get('r',node.get('angle',node.get('theta')))
                if type(heading) not in (int,float) or not math.isfinite(heading):raise ValueError('목적지 세타 설정이 필요합니다.')
        async with self.command_lock:
            started=time.monotonic();self.command_state={'name':cmd.name,'status':'SENDING'}
            try:
                if self.mode=='sim':
                    if cmd.name=='navigate':self.sim.navigate(cmd.goal)
                    elif cmd.name=='set_do':self.sim.do[cmd.channel]=cmd.value
                    elif cmd.name in ('pause','resume'):self.sim.command(cmd.name)
                    elif cmd.name=='cancel':self.sim.stop()
                    result={'sim':True}
                else:
                    if cmd.name=='set_do':
                        if time.monotonic()-self.io_rx>3 or cmd.channel not in self.io['do']:raise ValueError('최신 DO 채널 수신 필요')
                        if abs(self.state.get('speed',0))>.02:raise ValueError('정지 후 DO를 변경하세요.')
                        result=await asyncio.to_thread(tcp_operation,self.host,{'operations':seer_io_operations()},'set_do',{'channel':cmd.channel,'value':cmd.value})
                        self.io_rx=0
                    else:
                        payload={'id':cmd.goal,'angle':self.map.nodes[cmd.goal].get('r',self.map.nodes[cmd.goal].get('angle',self.map.nodes[cmd.goal].get('theta'))),'spin':True} if cmd.name=='navigate' else {'nick_name':'MOMA Web Control'} if 'control' in cmd.name else {}
                        result=await self.command_request(cmd.name,payload)
                if cmd.name=='cancel':self.mission['status']='CANCELED'
                self.command_state={'name':cmd.name,'status':'ACK','elapsed_ms':round((time.monotonic()-started)*1000)}
                self.event('ACK',cmd.name+(' · '+cmd.goal if cmd.goal else '')+' · 요청 응답 확인')
                return result
            except Exception as e:
                self.command_state={'name':cmd.name,'status':'UNCERTAIN','error':str(e)}
                self.event('ERROR',f'{cmd.name}: {e} · 자동 재전송 없음');raise
    def arm_travel_guard(self):
        if self.arm_source!='sdk' or time.monotonic()-self.arm_rx>2:raise ValueError('실기 AMR 이동은 FR5 SDK 연결 및 안전 자세 확인이 필요합니다.')
        cfg=self.arm_config.get('operations',{}).get('safe_pose',{})
        state=self.arm_state
        if cfg.get('method')!='MoveJ' or len(cfg.get('target',[]))!=6:raise ValueError('기존 FR5 설정에 safe_pose MoveJ를 등록하세요.')
        if state.get('motion_done')!=1 or state.get('emergency') or any(state.get('safety_stop',[])) or any(state.get('errors',[])):
            raise ValueError('로봇팔 정지/안전 상태를 확인하세요.')
        if not all(abs(a-b)<=.5 for a,b in zip(state.get('joints_deg',[]),cfg['target'])) or len(state.get('joints_deg',[]))!=6:
            raise ValueError('로봇팔 safe_pose에서 AMR을 이동하세요.')
    async def command_request(self,name,payload,host=None):
        c=SeerClient(host or self.host,self.profile);c.timeout=8
        epoch=self.epoch;cancel_seq=self.navigation_cancel_seq
        def execute():
            if name=='navigate':
                def guard():
                    if self.epoch!=epoch or self.navigation_cancel_seq!=cancel_seq:
                        raise ValueError('연결/정지 상태 변경 · 대기 이동 취소')
                    self.require_live();self.arm_travel_guard()
                    if self.mode!='control':
                        raise ValueError('실기 제어 모드가 필요합니다.')
                guard()
            return c.command(name,payload)
        try:return await asyncio.to_thread(execute)
        finally:c.close()
    async def stop(self):
        self.navigation_cancel_seq+=1
        self.mission['status']='CANCELED'
        if self.mode=='sim':self.sim.stop();self.event('STOP','SIM 주행 정지');return {'stopped':True}
        if self.mode!='control':raise ValueError('실기 제어 모드에서 정지를 요청하세요.')
        if not self.host:raise ValueError('AMR 연결 주소 없음')
        host=self.host
        results=await asyncio.gather(self.command_request('cancel',{},host),self.command_request('stop_motion',{},host),return_exceptions=True)
        errors=[str(r) for r in results if isinstance(r,Exception)]
        self.event('STOP','소프트웨어 정지 요청 · '+('응답 불확실: '+' / '.join(errors) if errors else '응답 확인'))
        if errors:raise ValueError('정지 결과 불확실: '+' / '.join(errors))
        return {'ack':True}
    async def pull_map(self):
        if self.mode=='sim':return self.map.data()
        self.require_live(False)
        if self.map_lock.locked():raise ValueError('지도 수신 중')
        epoch=self.epoch;host=self.host
        async with self.map_lock:
            c=SeerClient(host,self.profile);c.timeout=20
            try:
                info=await asyncio.to_thread(c.query,1300)
                name=info.get('current_map')
                if not isinstance(name,str) or not name:raise ValueError('현재 지도 이름 미수신')
                data=await asyncio.to_thread(c.download_map,name)
                model=await asyncio.to_thread(load_smap_data,data,name)
                if epoch!=self.epoch:raise ValueError('지도 수신 중 연결 변경')
                self.map=model;self.map_version+=1
                alignment=read_json(Path.home()/'.seer_amr_console'/'lidar_map_alignment.json',{}).get(host,{}).get(name,{}).get('transform',[0.,0.,0.])
                self.lidar_alignment=tuple(alignment[:3]) if len(alignment)>=3 and all(type(v) in (int,float) and math.isfinite(v) for v in alignment[:3]) else (0.,0.,0.)
                confirmation=self.viewer.get('cad_placement_confirmation') or {}
                context=confirmation.get('signature',{}).get('context',{})
                self.placement_confirmed=(context.get('host')==host and context.get('map')==name)
                self.event('MAP',f'로봇 지도 수신 · {name}')
                return self.map.data()
            finally:c.close()
    async def poll_mission(self):
        while True:
            await self.mission_tick()
            await asyncio.sleep(.25)
    async def mission_tick(self):
        m=self.mission
        if m['status']!='RUNNING':return
        if not self.connected or time.monotonic()-self.rx>3:
            m['status']='FAILED';self.event('ERROR','미션 중 연결 만료 · 재전송하지 않음');return
        if m['awaiting']:
            if time.monotonic()-m['sent_at']>180:
                m['status']='FAILED';self.event('ERROR','미션 목적지 확인 시간 초과');return
            status=self.state.get('task')
            if status in ('FAILED','CANCELED'):
                m['status']='FAILED';return
            if self.mode=='sim':done=status=='완료'
            else:done=status=='COMPLETED' and self.rx>m.get('rx_at_send',0) and str(self.state.get('target'))==m['goals'][m['index']]
            if done and self.mode!='sim':
                from seer_control.arrival_pose import arrival_pose
                done=arrival_pose(self.map.nodes.get(m['goals'][m['index']],{}),self.state)['ok']
            if done:
                m['index']+=1;m['awaiting']=False
                if m['index']>=len(m['goals']):m['status']='COMPLETED';self.event('MISSION','모든 목적지 완료')
        elif m['index']<len(m['goals']):
            try:
                await self.send(Command(name='navigate',goal=m['goals'][m['index']]),mission=True)
                m['awaiting']=True;m['sent_at']=time.monotonic();m['rx_at_send']=self.rx
            except Exception as e:m['status']='FAILED';self.event('ERROR','미션: '+str(e))
    def sync_desktop_doors(self):
        sample=read_door_snapshot()
        if sample is None or sample['revision']==self._ignored_desktop_revision:return False
        expected=Path(self.viewer.get('assets',{}).get('cad',{}).get('path','SIMPLE_WHOLE.glb')).name
        if sample.get('asset') and sample['asset'].casefold()!=expected.casefold():return False
        changed=self.desktop_revision!=sample['revision'] or self.door_source!='desktop'
        self.doors=dict(sample['angles']);self.door_follow=sample['follow'];self.door_source='desktop'
        self.desktop_revision=sample['revision'];self.desktop_door_rx=sample['timestamp'];self.desktop_door_mode=sample.get('mode','')
        if self.follower:
            if changed:self.follower.detach()
            for name in self.follower.angles:self.follower.angles[name]=math.radians(self.doors['left' if name.endswith(':1') else 'right'])
        if changed:self.event('VIEW',f"데스크톱 문 표시 동기화 · L {self.doors['left']:.1f}° / R {self.doors['right']:.1f}°")
        return True
    async def poll_desktop_doors(self):
        while True:
            self.sync_desktop_doors()
            await asyncio.sleep(.15)
    def set_doors(self,cfg):
        current=read_door_snapshot()
        self._ignored_desktop_revision=current['revision'] if current else self.desktop_revision
        self.door_source='web';self.door_contact=None
        self.doors={'left':cfg.left,'right':cfg.right};self.door_follow=cfg.follow
        if self.follower:
            self.follower.detach()
            for n in self.follower.angles:self.follower.angles[n]=math.radians(self.doors['left' if n.endswith(':1') else 'right'])
        self.save();self.event('VIEW',f'문 초기값 · L {cfg.left}° / R {cfg.right}° · '+('관절 추종' if cfg.follow else '수동 표시'))
    async def scene(self):
        async with self.scene_lock:
            if self.scene_data is not None:return self.scene_data
            self.scene_data=await asyncio.to_thread(self.build_scene)
            return self.scene_data
    def build_scene(self):
        assets={};errors=[];specs=self.viewer.get('assets',{})
        defaults={'amr':'models/extctrl_linux/SDC-400EU-light.glb','arm':'models/extctrl_linux/FR5_Assembly.glb','tool':'models/extctrl_linux/gripper/EndEffector V3_EndOnly_Change1-org.glb','cad':'SIMPLE_WHOLE.glb'}
        for role,path in defaults.items():
            spec=specs.get(role,{})
            p=Path(spec.get('path',path));p=p if p.is_absolute() else ROOT/p
            try:
                if not p.is_file() or p.suffix.lower()!='.glb':raise ValueError('GLB 모델 파일이 없습니다.')
                self.model_paths[role]=p
                assets[role]=dict(url=f'/api/models/{role}.glb?v={p.stat().st_mtime_ns}',
                    rotation=spec.get('rotation',[0,0,0]),scale=spec.get('scale',1),detail='full-source',bytes=p.stat().st_size)
                if role=='arm':
                    # Geometry here is never sent to the renderer. Only joint
                    # transforms and FK are used; browser loads original GLB.
                    self.arm_model=load_glb_robot(p,max_faces=700)
                    assets[role].update(joints=self.arm_model.joints,roots=self.arm_model.roots)
                elif role=='cad':
                    asset=load_glb(p,max_faces=20000,rotation=tuple(spec.get('rotation',[0,0,0])))
                    assets[role]['doors']=[];doors=[]
                    for part in asset.door_parts:
                        ps=[p for face in part['asset'].faces for p in face]
                        handle=[p for face,mat in zip(part['asset'].faces,part['asset'].face_materials) if mat==36 for p in face]
                        lo=[min(p[i] for p in ps) for i in range(3)];hi=[max(p[i] for p in ps) for i in range(3)]
                        h=tuple((min(p[i] for p in handle)+max(p[i] for p in handle))/2 for i in range(3)) if handle else tuple((a+b)/2 for a,b in zip(lo,hi))
                        doors.append(dict(name=part['name'],pivot=part['pivot'],sign=part['sign'],handle=h,push=(lo[0]-.018,lo[1]-.018,lo[2]+.3*(hi[2]-lo[2])),surface_faces=part['asset'].faces,hook_open_only=True,hook_inside_exit=part['name'].endswith(':2'),hook_dwell=0.,hook_faces=[face for face,mat in zip(part['asset'].faces,part['asset'].face_materials) if mat in (33,36,37)]))
                        assets[role]['doors'].append(dict(name=part['name'],pivot=part['pivot'],sign=part['sign']))
                    self.follower=SurfaceDoorFollower(doors,roller_radius=getattr(self,'roller_radius',.025))
                    for n in self.follower.angles:self.follower.angles[n]=math.radians(self.doors['left' if n.endswith(':1') else 'right'])
                elif role=='tool':
                    tool_source=load_glb(p,max_faces=6000,rotation=tuple(spec.get('rotation',[0,0,0])))
                    points=[p for face,node in zip(tool_source.faces,tool_source.face_nodes) if 'Polymer_ROECRHA50-10-30-T5' in node for p in face]
                    self.roller_radius=max(max(p[i] for p in points)-min(p[i] for p in points) for i in range(3))/2 if points else .025
                    self.roller_tip=tuple((min(p[i] for p in points)+max(p[i] for p in points))/2+(.1 if i==2 else 0) for i in range(3)) if points else None
            except Exception as e:
                assets.pop(role,None);errors.append(role+': '+str(e))
        return dict(assets=assets,mount=self.viewer.get('mount',[.34,0,.626,0,0,0]),cad_origin=self.viewer['cad_origin'],errors=errors,render_detail='원본 GLB 전체 형상 · 표시용 면 축소 없음')


@asynccontextmanager
async def lifespan(app):
    c=app.state.control
    c.camera.start();c.realsense.start()
    c.jobs=[asyncio.create_task(c.poll()),asyncio.create_task(c.poll_io()),asyncio.create_task(c.poll_arm()),asyncio.create_task(c.poll_mission()),asyncio.create_task(c.poll_lidar()),asyncio.create_task(c.poll_desktop_doors())]
    yield
    for job in c.jobs:job.cancel()
    await asyncio.gather(*c.jobs,return_exceptions=True)
    if c.client:c.client.close()
    c.arm.close();c.camera.close();c.realsense.close()


def create_app(control=None):
    app=FastAPI(title='MOMA Mission Control',lifespan=lifespan)
    app.state.control=control or Control();c=app.state.control
    @app.middleware('http')
    async def guard(request:Request,call_next):
        if request.method not in ('GET','HEAD','OPTIONS'):
            origin=request.headers.get('origin')
            if origin and urlsplit(origin).netloc!=request.headers.get('host'):
                return Response('Origin rejected',403)
        response=await call_next(request)
        response.headers['X-Content-Type-Options']='nosniff'
        return response
    def owner(request):
        session=request.headers.get('x-moma-session','')
        c.require_owner(session);return session
    async def action(fn):
        try:return await fn()
        except (ValueError,OSError,RuntimeError) as e:raise HTTPException(409,str(e)) from e
    @app.get('/api/health')
    async def health():return {'service':'moma-web-control','version':'1.0.0'}
    @app.get('/api/status')
    async def status():return c.snapshot()
    @app.get('/api/map')
    async def map_data():return clean(c.map.data())
    @app.get('/api/scene')
    async def scene():return clean(await c.scene())
    @app.get('/api/models/{role}.glb')
    async def model_file(role:str):
        if role not in ('amr','arm','tool','cad'):raise HTTPException(404,'Unknown model')
        await c.scene()
        path=c.model_paths.get(role)
        if not path:raise HTTPException(404,'Model unavailable')
        return FileResponse(path,media_type='model/gltf-binary',headers={'Cache-Control':'private, max-age=3600'})
    @app.get('/api/config')
    async def config():return {'host':c.config.get('host','192.168.30.13'),'runtime_url':c.config.get('runtime_url','http://192.168.30.13:8000/runtime.html'),'camera_url':c.camera_url,'camera_source':c.camera_source,'arm_ip':c.arm_config.get('ip','192.168.58.2'),'sdk_path':c.arm_config.get('sdk_path',''),'version':c.arm_config.get('controller_version','')}
    @app.post('/api/control/acquire')
    async def acquire(request:Request):
        session=request.headers.get('x-moma-session','')
        if not 16<=len(session)<=128:raise HTTPException(400,'Session required')
        if c.owner!=session and c.owner_until>time.monotonic():raise HTTPException(409,'다른 운영 화면이 제어 중입니다.')
        c.owner=session;c.owner_until=time.monotonic()+20
        return {'owned':True}
    @app.post('/api/control/release')
    async def release(request:Request):
        try:owner(request)
        except ValueError as e:raise HTTPException(409,str(e))
        c.owner=None;c.owner_until=0
        return {'owned':False}
    @app.post('/api/control/renew')
    async def renew(request:Request):
        try:owner(request)
        except ValueError as e:raise HTTPException(409,str(e))
        return {'owned':True}
    @app.post('/api/connect')
    async def connect(cfg:Connect,request:Request):
        async def run():
            owner(request);await c.connect(cfg)
            c.config.update(host=cfg.host,runtime_url=cfg.runtime_url);c.save()
            return {'requested':True}
        return await action(run)
    @app.post('/api/disconnect')
    async def disconnect(request:Request):
        async def run():owner(request);await c.disconnect();return {'disconnected':True}
        return await action(run)
    @app.post('/api/command')
    async def command(cmd:Command,request:Request):
        async def run():owner(request);return await c.send(cmd)
        return await action(run)
    @app.post('/api/stop')
    async def stop(request:Request):
        async def run():owner(request);return await c.stop()
        return await action(run)
    @app.post('/api/map/pull')
    async def pull(request:Request):
        async def run():owner(request);await c.pull_map();return {'map_version':c.map_version}
        return await action(run)
    @app.post('/api/doors')
    async def doors(cfg:Doors,request:Request):
        async def run():owner(request);c.set_doors(cfg);return {'saved':True}
        return await action(run)
    @app.post('/api/doors/desktop')
    async def desktop_doors(request:Request):
        async def run():
            owner(request);c._ignored_desktop_revision=None
            if not c.sync_desktop_doors():raise ValueError('데스크톱 프로그램의 문 표시 수신을 기다리세요. 두 프로그램은 같은 PC에서 실행해야 합니다.')
            return {'synced':True}
        return await action(run)
    @app.post('/api/mission')
    async def mission(cfg:Mission,request:Request):
        async def run():
            owner(request);c.require_live()
            if c.mission['status']=='RUNNING':raise ValueError('미션 실행 중')
            if any(g not in c.map.nodes for g in cfg.goals):raise ValueError('지도에 없는 목적지')
            if c.mode!='sim':c.arm_travel_guard()
            c.mission={'status':'RUNNING','goals':cfg.goals,'index':0,'awaiting':False}
            c.event('MISSION','목적지 순차 미션 시작');return {'started':True}
        return await action(run)
    @app.post('/api/arm/connect')
    async def arm_connect(cfg:ArmConnect,request:Request):
        async def run():
            owner(request)
            if c.mode=='sim':raise ValueError('실기 모드에서 연결하세요.')
            ipaddress.IPv4Address(cfg.ip)
            base=copy.deepcopy(c.arm_config) if c.arm_config.get('driver')=='fairino' else arm_profile()
            base.update(ip=cfg.ip,sdk_path=cfg.sdk_path,controller_version=cfg.version)
            async with c.arm_lock:r=await asyncio.to_thread(c.arm.connect,base)
            c.arm_config=base;c.arm_state=r;c.arm_rx=time.monotonic();c.arm_source='sdk'
            c.event('ARM','FR5 SDK 연결 · 상태 조회');return r
        return await action(run)
    @app.post('/api/arm/execute')
    async def arm_execute(cfg:ArmOperation,request:Request):
        async def run():
            owner(request);c.require_live()
            if c.mode=='sim':raise ValueError('실제 팔 작업은 실기 제어 모드에서 실행하세요.')
            if c.mission['status']=='RUNNING' or abs(c.state.get('speed',0))>.02:raise ValueError('AMR 및 미션 정지 후 팔 작업을 실행하세요.')
            if not c.arm.connected or time.monotonic()-c.arm_rx>2:raise ValueError('최신 FR5 SDK 상태 수신 필요')
            if c.arm_lock.locked():raise ValueError('팔 요청 처리 중')
            if cfg.operation not in c.arm_config.get('operations',{}):raise ValueError('등록되지 않은 팔 작업')
            async with c.arm_lock:
                result=await asyncio.to_thread(c.arm.call,c.arm_config,'execute',cfg.operation)
            c.event('ARM',cfg.operation+' · 요청 응답 확인');return result
        return await action(run)
    @app.post('/api/arm/stop')
    async def arm_stop(request:Request):
        async def run():owner(request);return await asyncio.to_thread(c.arm.stop)
        return await action(run)
    @app.post('/api/camera')
    async def camera(cfg:Camera,request:Request):
        async def run():
            owner(request)
            if cfg.source not in ('realsense_remote','custom','webcam','off'):raise ValueError('카메라 소스 오류')
            url=(realsense_url(c.viewer,c.config.get('runtime_url',c.viewer.get('runtime_url','http://192.168.30.13:8000/runtime.html'))) if cfg.source=='realsense_remote' else f'webcam:{cfg.device}' if cfg.source=='webcam' else cfg.url if cfg.source=='custom' else '')
            p=urlsplit(url)
            if cfg.source!='webcam' and url and (p.scheme not in ('http','https') or not p.hostname or p.username):raise ValueError('HTTP 이미지/MJPEG 주소를 입력하세요.')
            c.camera_source=cfg.source;c.camera_url=url;c.camera.configure(url if cfg.source!='realsense_remote' else '');c.save()
            return {'saved':True,'url':url}
        return await action(run)
    @app.get('/api/camera/frame')
    async def frame(source:str='current'):
        if source not in ('current','realsense'):raise HTTPException(400,'Unknown camera')
        hub=c.realsense if source=='realsense' or c.camera_source=='realsense_remote' else c.camera
        image,kind,received,sequence=hub.latest()
        if not image or time.monotonic()-received>2:raise HTTPException(503,hub.status()['error'] or '영상 수신 대기')
        return Response(image,media_type=kind,headers={'Cache-Control':'no-store'})
    @app.get('/api/camera/stream')
    async def camera_stream(request:Request,source:str='current'):
        if source not in ('current','realsense'):raise HTTPException(400,'Unknown camera')
        async def frames():
            sequence=None
            while not await request.is_disconnected():
                hub=c.realsense if source=='realsense' or c.camera_source=='realsense_remote' else c.camera
                image,kind,received,current=hub.latest()
                if image and (id(hub),current)!=sequence and time.monotonic()-received<=2:
                    sequence=(id(hub),current)
                    yield b'--frame\r\nContent-Type: '+kind.encode('ascii')+b'\r\nContent-Length: '+str(len(image)).encode('ascii')+b'\r\n\r\n'+image+b'\r\n'
                await asyncio.sleep(.025)
        return StreamingResponse(frames(),media_type='multipart/x-mixed-replace; boundary=frame',headers={'Cache-Control':'no-store','X-Accel-Buffering':'no'})
    @app.websocket('/api/ws')
    async def ws(websocket:WebSocket):
        origin=websocket.headers.get('origin')
        if origin and urlsplit(origin).netloc!=websocket.headers.get('host'):
            await websocket.close(code=1008);return
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(c.snapshot());await asyncio.sleep(.15)
        except (WebSocketDisconnect,RuntimeError):pass
    dist=ROOT/'web_control/dist'
    if dist.exists():app.mount('/',StaticFiles(directory=dist,html=True),name='web')
    return app

app=create_app()

if __name__=='__main__':
    import argparse,uvicorn
    parser=argparse.ArgumentParser();parser.add_argument('--host',default='127.0.0.1');parser.add_argument('--port',type=int,default=8765)
    args=parser.parse_args();uvicorn.run(app,host=args.host,port=args.port)
