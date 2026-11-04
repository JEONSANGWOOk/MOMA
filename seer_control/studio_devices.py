"""Explicitly configured peripheral adapters. Network work never touches Tk."""
import copy
import json
import queue
import threading
import urllib.parse
import xmlrpc.client
from .transport import PersistentSeerSession


def substitute(value,values):
    if isinstance(value,dict):return {k:substitute(v,values) for k,v in value.items()}
    if isinstance(value,list):return [substitute(v,values) for v in value]
    if isinstance(value,str) and value.startswith('${') and value.endswith('}'):
        key=value[2:-1]
        if key not in values:raise ValueError(f'명령 변수 누락: {key}')
        return values[key]
    return value


def validate_operation(config,name):
    spec=config.get('operations',{}).get(name)
    if not isinstance(spec,dict) or spec.get('verified') is not True:
        raise ValueError(f'{name}: 장비 문서로 확인한 API 설정이 필요합니다.')
    for k in ('api','port','response_type'):
        if type(spec.get(k)) is not int or not 1<=spec[k]<=65535:raise ValueError(f'{name}.{k}: 1~65535 정수')
    if spec.get('role') not in ('read','write'):raise ValueError('operation role: read/write')
    if not isinstance(spec.get('payload',{}),dict):raise ValueError('payload는 JSON object입니다.')
    return spec


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self,host):
        conn=super().make_connection(host);conn.timeout=3.;return conn


class TimeoutSafeTransport(xmlrpc.client.SafeTransport):
    def make_connection(self,host):
        conn=super().make_connection(host);conn.timeout=3.;return conn


def arm_call(config,kind,operation=None):
    if config.get('verified') is not True:raise ValueError('로봇팔 연결 정보가 미검증입니다.')
    url=config.get('endpoint','');parts=urllib.parse.urlparse(url)
    if parts.scheme not in ('http','https') or not parts.hostname:raise ValueError('로봇팔 XML-RPC 주소가 필요합니다.')
    method=config.get(kind+'_method','')
    if not method or any(not part.isidentifier() for part in method.split('.')):raise ValueError(f'{kind}_method 설정 오류')
    args=config.get(kind+'_args',[])
    if kind=='execute':
        operations=config.get('operations',{})
        if operation not in operations:raise ValueError('등록되지 않은 로봇팔 작업입니다.')
        args=operations[operation]
    if not isinstance(args,list):raise ValueError('XML-RPC 인수는 JSON 배열입니다.')
    transport=TimeoutSafeTransport() if parts.scheme=='https' else TimeoutTransport()
    with xmlrpc.client.ServerProxy(url,transport=transport,allow_none=True) as proxy:
        call=proxy
        for part in method.split('.'):call=getattr(call,part)
        result=call(*args)
    if kind=='execute':
        code=get_field(result,config['success_path']) if config.get('success_path') else result
        if code!=config.get('success_value',0):raise ValueError(f'로봇팔 명령 거부: {result}')
    if kind=='status' and config.get('status_path'):result=get_field(result,config['status_path'])
    return result


class DeviceBridge:
    def __init__(self):
        self.requests=queue.Queue(maxsize=8);self.results=queue.Queue();self.serial=0
        self.stop_event=threading.Event()
        self.canceled=set()
        threading.Thread(target=self._work,daemon=True,name='seer-studio-device').start()
    def submit(self,fn):
        self.serial+=1;token=self.serial
        self.requests.put_nowait((token,fn));return token
    def _work(self):
        while not self.stop_event.is_set():
            try:token,fn=self.requests.get(timeout=.1)
            except queue.Empty:continue
            if token in self.canceled:self.results.put((token,False,'요청 취소됨'));self.canceled.discard(token);continue
            try:self.results.put((token,True,fn()))
            except Exception as e:self.results.put((token,False,str(e)))
    def close(self):self.stop_event.set()
    def cancel(self,token):
        if token is not None:self.canceled.add(token)


def tcp_operation(host,config,name,values):
    spec=copy.deepcopy(validate_operation(config,name))
    payload=substitute(spec.get('payload',{}),values)
    session=PersistentSeerSession(host,spec['port'],timeout=3.)
    try:
        response=session.request(spec['api'],payload)
        if response['_response_type']!=spec['response_type']:raise ValueError('장비 응답 API 불일치')
        return response
    finally:session.close()


def get_field(value,path):
    for key in path.split('.'):
        value=value[int(key)] if isinstance(value,list) else value[key]
    return value


def seer_io_operations():
    """SEER NetProtocol L 1.2.1, IO query and ordinary digital output."""
    return {
        'read_io': dict(verified=True, role='read', api=1013, port=19204,
                        response_type=11013, payload={}, di_path='DI', do_path='DO',
                        di_valid_path='DI_valid'),
        'set_do': dict(verified=True, role='write', api=6001, port=19210,
                       response_type=16001, payload={'id':'${channel}', 'status':'${value}'}),
    }


def io_channels(value):
    """Preserve channel IDs; never interpret strings or records by truthiness."""
    if not isinstance(value, (dict, list)):
        raise ValueError('I/O 응답은 객체 또는 배열이어야 합니다.')
    out = {}
    for key, item in (value.items() if isinstance(value, dict) else enumerate(value)):
        if isinstance(item, dict):
            key, item = item.get('id', key), item.get('status')
        if isinstance(key, bool) or not str(key).isdigit():
            raise ValueError('I/O 채널 번호 오류')
        channel = int(key)
        if not 0 <= channel < 64:
            raise ValueError('I/O 채널: 0~63')
        if type(item) is not bool and not (type(item) is int and item in (0, 1)):
            raise ValueError('I/O 상태는 boolean 또는 0/1이어야 합니다.')
        if channel in out:
            raise ValueError('I/O 채널 중복')
        out[channel] = bool(item)
    return out


def parse_io(result, spec):
    parsed = {}
    for kind in ('di', 'do'):
        path = spec.get(kind + '_path', kind)
        try:
            value = get_field(result, path)
        except KeyError:
            value = []  # Optional banks are unknown, not OFF.
        parsed[kind] = io_channels(value)
    valid_path = spec.get('di_valid_path')
    valid = io_channels(get_field(result, valid_path)) if valid_path and valid_path in result else {}
    return parsed, valid
