"""Loopback-only SEER protocol fixture. Never connects to physical hardware."""
import json, socketserver, sys, threading, time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from seer_control.transport import HEADER, receive_exact
class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address=True
    daemon_threads=True
class Handler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(3)
        h=HEADER.unpack(receive_exact(self.request,16));body=receive_exact(self.request,h[3])
        p=json.loads(body) if body else {};api=h[4]
        with self.server.lock:
            state=self.server.state
            out={'ret_code':0}
            if api==1100:out=dict(state)
            elif api==1300:out={'current_map':state.get('current_map','fixture'),'maps':['fixture','floor2']}
            elif api==2022:state.update(current_map=p['map_name'],loadmap_status=1)
            elif api==1301:out={'stations':[dict(id='LM1',x=2,y=2,type='LandMark'),dict(id='LM2',x=4,y=2,type='LandMark')]}
            elif api==4011:
                out={'header':{'mapName':p.get('map_name','fixture')},
                     'normalPosList':[{'x':0,'y':0},{'x':0,'y':4},{'x':6,'y':4},{'x':6,'y':0}],
                     'advancedPointList':[
                         {'className':'LandMark','instanceName':'LM1','pos':{'x':2,'y':2}},
                         {'className':'LandMark','instanceName':'LM2','pos':{'x':4,'y':2}}],
                     'advancedCurveList':[{'className':'LinePath','startPos':{'pos':{'x':2,'y':2}},'endPos':{'pos':{'x':4,'y':2}}}]}
            elif api==3051:state.update(task_status=2,target_id=p['id'],path=[[2,2],[4,2]])
            elif api in (3001,3002,3003):state['task_status']={3001:3,3002:2,3003:6}[api]
            elif api==2002:state.update(x=p['x'],y=p['y'],angle=p['angle'])
            elif api==4000:state['mode']=p['mode']
            elif api!=2003:out={'ret_code':999,'err_msg':'fixture: unsupported API'}
        data=json.dumps(out).encode();packet=HEADER.pack(0x5a,1,h[2],len(data),api+10000,b'\0'*6)+data
        self.request.sendall(packet[:7]);self.request.sendall(packet[7:])
def start(ports=(19204,19205,19206,19207)):
    state=dict(x=2.,y=2.,angle=0.,confidence=.99,battery_level=.82,vx=0.,mode=1,emergency=False,blocked=False,task_status=0,laser_beams=[[1,1],[1,2],[1,3]],path=[],slam_status=0)
    lock=threading.Lock();servers=[]
    try:
        for port in ports:
            s=Server(('127.0.0.1',port),Handler);s.state=state;s.lock=lock
            threading.Thread(target=s.serve_forever,daemon=True).start();servers.append(s)
    except Exception:
        for s in servers:s.shutdown();s.server_close()
        raise
    return servers
if __name__=='__main__':
    servers=start();print('MOCK ONLY: GUI IP=127.0.0.1. Ctrl+C 종료.',flush=True)
    try:
        while True:time.sleep(1)
    except KeyboardInterrupt:
        for s in servers:s.shutdown();s.server_close()
