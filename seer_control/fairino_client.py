"""Bounded SDK calls; independent manufacturer StopMotion channel."""
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import xmlrpc.client
from .fairino_api import validate, checked
from .studio_devices import TimeoutTransport


class FairinoClient:
    def __init__(self):
        self.process=None;self.connected=False;self.config=None
        self.lock=threading.Lock();self.responses=None

    def connect(self,config):
        validate(config)
        with self.lock:
            self.close();self.config=dict(config)
            if getattr(sys,'frozen',False):raise RuntimeError('FR5 SDK 연결은 Python 소스 실행에서 지원됩니다.')
            env=dict(os.environ,PYTHONIOENCODING='utf-8',PYTHONUNBUFFERED='1')
            self.process=subprocess.Popen([sys.executable,'-X','utf8','-m','seer_control.fairino_worker'],
                cwd=str(Path(__file__).resolve().parents[1]),env=env,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,text=True,encoding='utf-8',
                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            process=self.process;responses=self.responses=queue.Queue()
            def read():
                try:
                    for line in process.stdout:
                        try:
                            response=json.loads(line)
                            if isinstance(response,dict) and 'ok' in response:responses.put(response)
                        except ValueError:continue
                finally:
                    process.stdout.close()
                    responses.put(dict(ok=False,error='FR5 SDK 프로세스 종료'))
            threading.Thread(target=read,daemon=True,name='fairino-feedback').start()
            self.connected=True
            return self._request('status',timeout=15)

    def _request(self,kind,operation=None,timeout=6):
        if not self.connected or self.process is None:raise ValueError('FR5 연결 시험을 먼저 실행하세요.')
        try:
            self.process.stdin.write(json.dumps(dict(config=self.config,kind=kind,operation=operation))+'\n')
            self.process.stdin.flush();response=self.responses.get(timeout=timeout)
            if not response['ok']:raise RuntimeError(response['error'])
            return response['result']
        except queue.Empty as e:
            self.close()
            raise TimeoutError('FR5 SDK 응답 시간 초과 · 다시 연결하기 전에 제어기 상태를 확인하세요.') from e
        except Exception:
            self.close()
            raise

    def call(self,config,kind,operation=None):
        with self.lock:
            if config!=self.config:raise ValueError('FR5 설정 변경 후 다시 연결하세요.')
            return self._request(kind,operation)

    def stop(self):
        # Terminate retry loops first; do not queue a stop behind SDK motion calls.
        config=self.config
        self.close()
        if not config:raise ValueError('FR5 연결 정보 없음')
        with xmlrpc.client.ServerProxy('http://'+config['ip']+':20003',transport=TimeoutTransport()) as rpc:
            checked(rpc.StopMotion(),'StopMotion')
        return dict(status='STOPPED',reconnect_required=True)

    def close(self):
        self.connected=False
        process=self.process;self.process=None
        if process is not None:
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=.5)
                except subprocess.TimeoutExpired:process.kill()
            # Reader releases stdout at EOF; never wait for SDK reconnect on Tk.
            if process.stdin:
                try:process.stdin.close()
                except (OSError,ValueError):pass
