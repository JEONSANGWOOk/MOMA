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
