"""SEER facade for a verified openTCS loopback vehicle. No local motion simulation."""
import json
import math
import threading
import time
import uuid
from urllib.error import HTTPError
from urllib.parse import quote, urlsplit
from urllib.request import Request, build_opener, ProxyHandler

from .decision_log import DecisionJournal, decide
from .model import MapModel
from .robot_emulator import PORT_APIS, prepare_map
from .smap import build_smap_from_model, make_path_record

LOOPBACK = 'org.opentcs.virtualvehicle.LoopbackCommunicationAdapterDescription'
ACTIVE = {'RAW', 'ACTIVE', 'DISPATCHABLE', 'BEING_PROCESSED', 'WITHDRAWN'}


def fleet_states(vehicles, model):
    """Preserve every ACS vehicle and explain first pending resource conflicts."""
    owners={}
    for vehicle in vehicles:
        for group in vehicle.get('allocatedResources',[]):
            for resource in group:owners.setdefault(resource,set()).add(vehicle['name'])
    result=[]
    for v in vehicles:
        node=model.nodes.get(v.get('currentPosition'),{})
        precise=v.get('precisePosition')
        xy=(precise.get('x')/1000,precise.get('y')/1000) if isinstance(precise,dict) else (node.get('x'),node.get('y'))
        pending=v.get('claimedResources',[])
        conflicts={resource:sorted(owners.get(resource,set())-{v['name']}) for resource in (pending[0] if pending else [])}
        conflicts={k:value for k,value in conflicts.items() if value}
        waiting=sorted({name for names in conflicts.values() for name in names}) if v.get('state')=='IDLE' and v.get('transportOrder') else []
        result.append(dict(id=v['name'],x=xy[0],y=xy[1],theta=0.,node=v.get('currentPosition'),state=v.get('state'),
                    order=v.get('transportOrder'),paused=v.get('paused',False),waiting_for=waiting,conflicts=conflicts,
                    wait_reason=('통행 대기: '+', '.join(waiting)) if waiting else ('일시정지' if v.get('paused') else ''),
                    battery=v.get('energyLevel'),selected=False))
    return result


class OpenTCSAPI:
    def __init__(self, url='http://127.0.0.1:55200/v1'):
        parsed = urlsplit(url)
        if parsed.scheme != 'http' or parsed.hostname not in ('127.0.0.1', 'localhost') or parsed.username or parsed.password or parsed.query or parsed.fragment:
            raise ValueError('가상 ACS 연결은 localhost HTTP 주소만 지원합니다.')
        self.url = url.rstrip('/')
        self.opener = build_opener(ProxyHandler({}))

    def call(self, method, path, body=None):
        data = json.dumps(body, allow_nan=False).encode() if body is not None else None
        request = Request(self.url+'/'+path, data=data, method=method, headers={'Content-Type':'application/json'})
        try:
            with self.opener.open(request, timeout=3) as response:
                data = response.read()
                return json.loads(data) if data else None
        except HTTPError as error:
            raise ValueError(f'openTCS HTTP {error.code}: {error.read(4096).decode("utf-8", "replace")}') from error


def convert_map(source):
    nodes = [dict(id=p['name'], x=p['position']['x']/1000, y=p['position']['y']/1000,
                  kind='dock' if p['type']=='PARK_POSITION' else 'station') for p in source['points']]
    lookup = {n['id']:n for n in nodes}
    records = []
    for path in source['paths']:
        if path.get('locked'): continue
        a, b = path['srcPointName'], path['destPointName']
        directions = []
        if path.get('maxVelocity', 0)>0: directions.append((a,b))
        if path.get('maxReverseVelocity', 0)>0: directions.append((b,a))
        for src, dst in directions:
            raw = make_path_record(lookup[src], lookup[dst], 'StraightPath', {'opentcs_path':path['name']})
            records.append(dict(a=src,b=dst,raw=raw,properties={}))
    model = MapModel(dict(format='amr-console-map-v1', name='opentcs_virtual', nodes=nodes,
                          edges=[[r['a'],r['b']] for r in records], path_records=records))
    return prepare_map(model, 'opentcs_virtual')


class OpenTCSRobot:
    def __init__(self, api=None, vehicle='AGV-01', log_dir=None):
        self.api = api or OpenTCSAPI()
        self.vehicle = vehicle
        self.lock = threading.RLock()
        self.offline_until = 0.; self.delay_s = 0.
        self.control_owner = ''; self.order_name = ''; self.target = ''; self.canceled = False
        self.vehicle_data = {}; self.order_data = {}; self.observed_order = {}; self.fleet = []; self.last_error = ''; self.updated_at = 0.
        self.stop_event = threading.Event()
        self.parking_orders = {}
        self.decision_journal = DecisionJournal(log_dir)
        self.decision_journal.bind('OPENTCS')
        self.source_map = self.api.call('GET', 'plantModel')
        self.map = convert_map(self.source_map)
        self.verify_virtual()
        self.refresh()
        self.thread = None

    def vehicle_path(self): return 'vehicles/'+quote(self.vehicle, safe='')

    def verify_virtual(self):
        info = self.api.call('GET', self.vehicle_path()+'/commAdapter/attachmentInformation')
        if info.get('attachedCommAdapter') != LOOPBACK:
            raise ValueError('선택한 AGV가 openTCS Loopback 가상 차량이 아닙니다.')

    def record(self, source, evidence, conclusion, action, **kwargs):
        decide(self, 'openTCS.'+source, dict(vehicle=self.vehicle,order=self.order_name,target=self.target),
               evidence, conclusion, action, **kwargs)

    def refresh(self):
        # Serialize HTTP state reads with commands so an old sample cannot overwrite a new order.
        with self.lock:
            vehicles = self.api.call('GET', 'vehicles')
            vehicle = next((v for v in vehicles if v['name']==self.vehicle),None)
            if vehicle is None:raise ValueError('ACS에 선택한 차량이 없습니다: '+self.vehicle)
            order = self.api.call('GET','transportOrders/'+quote(self.order_name,safe='')) if self.order_name else {}
            external = vehicle.get('transportOrder')
            observed = self.api.call('GET','transportOrders/'+quote(external,safe='')) if external and external!=self.order_name else order
            position = vehicle.get('currentPosition')
            precise = vehicle.get('precisePosition')
            if position not in self.map.nodes and not isinstance(precise, dict):
                raise ValueError('가상 AGV 위치가 아직 초기화되지 않았습니다.')
            self.vehicle_data = vehicle; self.order_data = order; self.observed_order = observed
            self.fleet=fleet_states(vehicles,self.map)
            for entry in self.fleet:
                entry['selected']=entry['id']==self.vehicle
                if entry['waiting_for']:
                    decide(self,'openTCS.통행 대기',dict(vehicle=entry['id'],node=entry['node'],order=entry['order']),
                           dict(conflicting_resources=entry['conflicts']),entry['wait_reason'],'ACS 자원 할당 대기 유지',
                           key=('acs-wait',entry['id']),identity=(entry['node'],tuple(entry['waiting_for'])))
            self.updated_at = time.monotonic(); self.last_error = ''
            state = observed.get('state', 'NONE')
            self.record('상태 판정',dict(acs_state=state,vehicle_state=vehicle.get('state'),
                        position=position,processing_vehicle=observed.get('processingVehicle'),observed_order=external or self.order_name,paused=vehicle.get('paused')),
                        'ACS가 보고한 상태 수신', '지도 위치와 미션 진행 상태 갱신',
                        identity=(state,position,vehicle.get('paused'),vehicle.get('state')))

    def start(self):
        def poll():
            while not self.stop_event.wait(.25):
                try: self.refresh()
                except Exception as error:
                    with self.lock:
                        self.last_error = str(error)
                        self.record('통신 오류',dict(error=str(error)),'ACS 상태 확인 불가','성공/완료로 판단하지 않음',identity=str(error))
        self.thread = threading.Thread(target=poll, daemon=True, name='opentcs-status')
        self.thread.start()
        return self

    def status(self):
        with self.lock:
            if self.last_error or time.monotonic()-self.updated_at>3:
                raise ValueError('openTCS 상태 확인 실패: '+(self.last_error or '상태가 오래됨'))
            v = self.vehicle_data; state = self.observed_order.get('state')
            task = {'RAW':1,'ACTIVE':1,'DISPATCHABLE':1,'BEING_PROCESSED':2,'WITHDRAWN':2,
                    'FINISHED':4,'FAILED':5,'UNROUTABLE':5}.get(state,0)
            external = v.get('transportOrder') and v.get('transportOrder')!=self.order_name
            if self.canceled and not external and state in ('FAILED','FINISHED'): task=6
            if v.get('paused') and task in (1,2): task=3
            position = v.get('currentPosition'); node = self.map.nodes.get(position,{})
            precise = v.get('precisePosition')
            x,y = (precise['x']/1000,precise['y']/1000) if isinstance(precise,dict) else (node['x'],node['y'])
            angle = v.get('orientationAngle',0)
            angle = math.radians(angle) if isinstance(angle,(float,int)) and math.isfinite(angle) else 0.
            current_order = v.get('transportOrder')
            warnings = ['openTCS 가상 차량 · 실제 센서 없음', '노드 도착 위치 표시 · 연속 속도 미제공']
            if current_order and current_order!=self.order_name:
                warnings.append('외부 ACS 작업 수행 중: '+current_order)
            return dict(ret_code=0,x=x,y=y,angle=angle,vx=0.,vy=0.,w=0.,confidence=1.,
                        battery_level=v.get('energyLevel',100)/100,charging=v.get('state')=='CHARGING',
                        blocked=v.get('state') in ('ERROR','UNAVAILABLE'),emergency=False,stopped=False,
                        motor=True,mode=1,task_status=task,target_id='' if external else self.target,last_station=position,
                        current_map=self.map.name,reloc_status=1,loadmap_status=1,control_owner=self.control_owner,
                        robot_model='openTCS Virtual AGV',robot_id=self.vehicle,robokit_version='openTCS-7',
                        is_emulator=True,laser_beams=[],path=[],unfinished_path=[],errors=[],fatals=[],warnings=warnings,
                        acs_order=current_order or self.order_name,acs_state=state,pose_source='precise' if precise else 'node',speed_available=False,
                        fleet=self.fleet,acs_wait_reason=next((f['wait_reason'] for f in self.fleet if f['selected']),''))

    def clear_idle_blockers(self):
        """Park only idle virtual vehicles actually holding another job's next resource."""
        with self.lock:
            self.refresh()
            blockers={name for v in self.fleet for name in v['waiting_for']}
            occupied={v['node'] for v in self.fleet}
            parking_vehicles=set()
            for name,entry in list(self.parking_orders.items()):
                state=self.api.call('GET','transportOrders/'+name).get('state')
                if state not in ACTIVE:del self.parking_orders[name]
                else:occupied.add(entry['goal']);parking_vehicles.add(entry['vehicle'])
            submitted=[]
            for entry in self.fleet:
                if entry['id'] not in blockers or entry['id'] in parking_vehicles or entry['order'] or entry['paused'] or entry['state']!='IDLE':continue
                options=[]
                for key,node in self.map.nodes.items():
                    if node.get('kind')!='dock' or key in occupied:continue
                    try:route=self.map.route(entry['node'],key)
                    except ValueError:continue
                    options.append((sum(self.map.distance(a,b) for a,b in zip(route,route[1:])),key))
                if not options:continue
                path='vehicles/'+quote(entry['id'],safe='')
                attachment=self.api.call('GET',path+'/commAdapter/attachmentInformation')
                current=self.api.call('GET',path)
                if attachment.get('attachedCommAdapter')!=LOOPBACK or current.get('transportOrder') or current.get('paused') or current.get('state')!='IDLE':continue
                goal=min(options)[1];name='MOMA-Park-'+uuid.uuid4().hex
                self.api.call('POST','transportOrders/'+name,dict(intendedVehicle=entry['id'],type='Park',destinations=[dict(locationName=goal,operation='MOVE')]))
                reservation=dict(vehicle=entry['id'],goal=goal,order=name)
                self.parking_orders[name]=reservation
                occupied.add(goal);submitted.append(reservation)
                decide(self,'openTCS.통로 확보',dict(vehicle=entry['id'],node=entry['node']),
                       dict(blocked_vehicles=[v['id'] for v in self.fleet if entry['id'] in v['waiting_for']],goal=goal,acs_order=name),
                       '실행 작업 없는 Loopback 차량이 통행 자원을 점유','ACS에 주차 작업 등록 · 완료는 ACS 상태로 확인',force=True)
            return submitted

    def start_fleet_demo(self):
        """Submit one short real ACS MOVE order per available virtual vehicle."""
        with self.lock:
            self.refresh();goals=set();submitted=[]
            occupied={v['node'] for v in self.fleet}
            for entry in self.fleet:
                if entry['order'] or entry['state']!='IDLE' or entry['paused']:continue
                choices=[r['b'] for r in self.map.path_records if r['a']==entry['node'] and r['b'] not in occupied|goals]
                if not choices:continue
                path='vehicles/'+quote(entry['id'],safe='')
                if self.api.call('GET',path+'/commAdapter/attachmentInformation').get('attachedCommAdapter')!=LOOPBACK:continue
                current=self.api.call('GET',path)
                if current.get('transportOrder') or current.get('state')!='IDLE' or current.get('paused'):continue
                goal=min(choices,key=lambda node:self.map.distance(entry['node'],node));name='MOMA-Fleet-'+uuid.uuid4().hex
                self.api.call('POST','transportOrders/'+name,dict(intendedVehicle=entry['id'],destinations=[dict(locationName=goal,operation='MOVE')]))
                goals.add(goal);submitted.append(dict(vehicle=entry['id'],goal=goal,order=name))
                decide(self,'openTCS.차량 주행 시험',dict(vehicle=entry['id'],node=entry['node']),
                       dict(goal=goal,acs_order=name),'유휴 Loopback 차량과 비점유 목적지 확인','ACS에 이동 작업 등록 · 실제 완료는 상태로 확인',force=True)
            return submitted

    def request(self, api, payload=None, port=None):
        with self.lock:
            try:
                if api not in set().union(*PORT_APIS.values()) or port is not None and api not in PORT_APIS[port]:
                    raise ValueError('지원하지 않는 API')
                p = {} if payload is None else payload
                if not isinstance(p,dict): raise ValueError('JSON object가 필요합니다.')
                if api in (1100,1002,1007,1020): return self.status()
                if api==1000: return dict(ret_code=0,robot_model='openTCS Virtual AGV',robot_id=self.vehicle,is_emulator=True)
                if api in (1009,1101): return dict(ret_code=0,laser_beams=[],sensor_available=False)
                if api in (1021,1022): return dict(ret_code=0,reloc_status=1,loadmap_status=1,confidence=1.)
                if api==1300: return dict(ret_code=0,current_map=self.map.name,maps=[self.map.name])
                if api==1301: return dict(ret_code=0,stations=[dict(id=k,x=n['x'],y=n['y'],r=0.,spin=True) for k,n in self.map.nodes.items()])
                if api==4011:
                    if p.get('map_name')!=self.map.name: raise ValueError('지도 이름 불일치')
                    return build_smap_from_model(self.map)
                if api==4005:
                    nick = p.get('nick_name')
                    if not isinstance(nick,str) or not nick.strip(): raise ValueError('제어자 이름이 필요합니다.')
                    if self.control_owner and self.control_owner!=nick: raise ValueError('다른 제어자가 사용 중입니다.')
                    self.verify_virtual(); self.control_owner=nick
                elif api==4006:
                    if not self.control_owner or p.get('nick_name')!=self.control_owner: raise ValueError('제어권 소유자 불일치')
                    if self.order_data.get('state') in ACTIVE: raise ValueError('미션 취소 또는 완료 후 제어권을 해제하세요.')
                    self.control_owner=''
                else:
                    if not self.control_owner: raise ValueError('제어권 획득이 필요합니다.')
                    self.verify_virtual(); self.refresh()
                    if api in (3050,3051):
                        goal = p.get('id')
                        if goal not in self.map.nodes: raise ValueError('등록된 openTCS Point ID만 지원합니다.')
                        if self.order_data.get('state') in ACTIVE or self.vehicle_data.get('transportOrder'):
                            raise ValueError('선택한 AGV의 기존 작업이 아직 종료되지 않았습니다.')
                        name = 'MOMA-'+uuid.uuid4().hex
                        order = self.api.call('POST','transportOrders/'+name,
                              dict(intendedVehicle=self.vehicle,destinations=[dict(locationName=goal,operation='MOVE')]))
                        self.order_name=name; self.target=goal; self.order_data=order or {'state':'RAW'}; self.observed_order=self.order_data; self.canceled=False
                        self.record('작업 생성',dict(goal=goal,vehicle=self.vehicle,acs_order=name,operation='MOVE'),
                                    '등록된 노드·가상 차량·기존 작업 종료 확인','ACS에 운송 작업 등록 · 완료는 별도 상태로 확인',force=True)
                    elif api in (3001,3002):
                        if self.order_data.get('state') not in ACTIVE: raise ValueError('진행 중인 MOMA 작업이 없습니다.')
                        paused = api==3001
                        self.api.call('PUT',self.vehicle_path()+'/paused?newValue='+str(paused).lower())
                        self.vehicle_data['paused']=paused
                    elif api in (3003,2000):
                        if self.order_name and self.order_data.get('state') in ACTIVE:
                            self.api.call('POST','transportOrders/'+self.order_name+'/withdrawal?immediate=true')
                            self.canceled=True
                            self.api.call('PUT',self.vehicle_path()+'/paused?newValue=false')
                            self.refresh()
                    elif api==4000:
                        if p.get('mode')!=1: raise ValueError('openTCS 가상 연동은 자동 모드만 지원합니다.')
                    elif api==2022:
                        if p.get('map_name')!=self.map.name: raise ValueError('지도 이름 불일치')
                    else: raise ValueError('openTCS 연동에서 지도 업로드·조그·재위치·SLAM은 지원하지 않습니다.')
                self.record('명령 처리',dict(api=api,payload=p),'ACS 요청 성공','다음 상태 조회로 결과 확인',force=True)
                return dict(ret_code=0,is_emulator=True,acs_order=self.order_name)
            except Exception as error:
                self.record('명령 거부',dict(api=api,error=str(error)),'요청 또는 통신 실패','명령 자동 재시도 없음',force=True)
                return dict(ret_code=9001,err_msg=str(error),is_emulator=True)

    def close(self):
        self.stop_event.set()
        if self.thread: self.thread.join(timeout=4)
        self.decision_journal.close()
