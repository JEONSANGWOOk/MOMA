"""Isolated official SDK feedback process. No control/servo/motion/output API."""
import contextlib
import json
import math
import os
from pathlib import Path
import select
import socketserver
import sys
import threading
import time


class SDKForward:
    def __init__(self,config):self.config=config;self.servers=[];self.client=None
    def start(self):
        import paramiko
        cfg=self.config;self.client=paramiko.SSHClient();self.client.load_system_host_keys()
        self.client.connect(cfg['ssh_host'],username=cfg['ssh_user'],key_filename=cfg['ssh_key'],look_for_keys=False,allow_agent=False,timeout=5)
        transport=self.client.get_transport();transport.set_keepalive(15)
        destination=cfg['controller_ip']
        class Server(socketserver.ThreadingTCPServer):
            allow_reuse_address=False;daemon_threads=True
        for port in (20003,20005):
            class Handler(socketserver.BaseRequestHandler):
                def handle(handler):
                    channel=None
                    try:
                        channel=transport.open_channel('direct-tcpip',(destination,handler.server.server_address[1]),handler.client_address,timeout=5)
                        while True:
                            ready,_,_=select.select([handler.request,channel],[],[],1)
                            for source in ready:
                                data=source.recv(65536)
                                if not data:return
                                (channel if source is handler.request else handler.request).sendall(data)
                    except Exception:pass
                    finally:
                        if channel is not None:channel.close()
            server=Server(('127.0.0.1',port),Handler);self.servers.append(server)
            threading.Thread(target=server.serve_forever,daemon=True,name='sdk-readonly-forward').start()
    def close(self):
        for s in self.servers:s.shutdown();s.server_close()
        if self.client:self.client.close()


def snapshot(robot,received):
    from .fairino_api import checked,read_output_io
    stamp=received['stamp']
    if received['count']<3 or time.monotonic()-stamp>.5:raise ValueError('SDK CNDE fresh feedback not yet available')
    joints=checked(robot.GetActualJointPosDegree(),'GetActualJointPosDegree',True)
    tool=checked(robot.GetActualTCPNum(),'GetActualTCPNum',True)
    if len(joints)!=6 or any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>1000 for v in joints):raise ValueError('Invalid measured joints')
    io,error=read_output_io(robot)
    if type(io.get('DO',{}).get('7')) is not bool:raise ValueError('SDK DO7 feedback unavailable: '+error)
    return dict(valid=True,driver='fairino',source='official-sdk-direct',read_only=True,
        sample_monotonic=stamp,updated=time.time(),io=io,tool_index=tool,
        joints_deg=joints,joints_rad=[math.radians(v) for v in joints],io_error=error)


def write_state(folder,data):
    path=folder/'fr5_direct_feedback.json';temp=folder/'fr5_direct_feedback.tmp'
    temp.write_text(json.dumps(data,separators=(',',':')),encoding='utf-8')
    for attempt in range(6):
        try:
            temp.replace(path);return True
        except PermissionError:
            # Windows readers can hold the destination briefly. Keep the SDK
            # connection alive and skip a frame rather than reconnecting.
            time.sleep(.005*(attempt+1))
    return False


def main():
    folder=Path.home()/'.seer_amr_console';folder.mkdir(exist_ok=True)
    config=json.loads((folder/'fr5_readonly.json').read_text(encoding='utf-8'))
    if config.get('enabled') is not True:return
    lock=open(folder/'fr5_readonly.lock','a+b')
    if os.name=='nt':
        import msvcrt
        lock.seek(0);lock.write(b'1');lock.flush();lock.seek(0)
        try:msvcrt.locking(lock.fileno(),msvcrt.LK_NBLCK,1)
        except OSError:return
    else:
        import fcntl
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except OSError:return
    (folder/'fr5_readonly.pid').write_text(str(os.getpid()))
    robot=None;tunnel=SDKForward(config)
    try:
        write_state(folder,dict(valid=False,error='Official SDK connection initializing',updated=time.time()))
        tunnel.start()
        sdk_path=Path(config['sdk_path']);sys.path.insert(0,str(sdk_path))
        from fairino import Robot
        runtime=folder/'fr5_sdk_runtime';runtime.mkdir(exist_ok=True);os.chdir(runtime)
        robot=Robot.RPC('127.0.0.1')
        from .fairino_api import checked
        version=checked(robot.GetSoftwareVersion(),'GetSoftwareVersion',True)
        if not isinstance(version,(list,tuple)) or config['controller_version'] not in version:raise ValueError('Controller/SDK version mismatch: '+str(version))
        received=dict(stamp=0.,count=0)
        client=robot._cnde_client;parse=client._parse_cnde_state_data
        def observed(data):
            parse(data);received['stamp']=time.monotonic();received['count']+=1
        client._parse_cnde_state_data=observed
        last_ok=time.monotonic()
        while True:
            try:
                data=snapshot(robot,received);data.update(controller_ip=config['controller_ip'],controller_version=version,sdk_tag=config['sdk_tag'],pid=os.getpid())
                write_state(folder,data);last_ok=time.monotonic()
            except Exception as error:
                write_state(folder,dict(valid=False,error=str(error),updated=time.time(),pid=os.getpid()))
                if time.monotonic()-last_ok>15:raise
            time.sleep(.05)
    except Exception as error:
        write_state(folder,dict(valid=False,error=str(error),updated=time.time(),pid=os.getpid()))
        print(type(error).__name__,str(error),flush=True)
    finally:
        if robot is not None:
            try:robot.CloseRPC()
            except Exception:pass
        tunnel.close();lock.close()

if __name__=='__main__':main()
