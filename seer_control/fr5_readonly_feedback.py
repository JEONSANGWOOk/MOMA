"""Cached read-only SDK feedback; keep the control driver's lifecycle separate."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time


def alive(pid):
    if type(pid) is not int or pid<=0:return False
    if os.name=='nt':
        import ctypes
        kernel=ctypes.WinDLL('kernel32',use_last_error=True)
        kernel.OpenProcess.restype=ctypes.c_void_p
        handle=kernel.OpenProcess(0x1000,False,pid)
        if not handle:return False
        code=ctypes.c_ulong()
        try:return bool(kernel.GetExitCodeProcess(ctypes.c_void_p(handle),ctypes.byref(code))) and code.value==259
        finally:kernel.CloseHandle(ctypes.c_void_p(handle))
    try:os.kill(pid,0);return True
    except OSError:return False


def ensure(folder):
    folder=Path(folder);config=folder/'fr5_readonly.json'
    if not config.exists() or json.loads(config.read_text(encoding='utf-8')).get('enabled') is not True:return False
    pid=folder/'fr5_readonly.pid'
    try:
        if alive(int(pid.read_text())):return True
    except (OSError,ValueError):pass
    log=open(folder/'fr5_readonly_service.log','ab')
    try:
        process=subprocess.Popen([sys.executable,'-u','-X','utf8','-m','seer_control.fr5_readonly_service'],
            cwd=str(Path(__file__).resolve().parents[1]),stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        pid.write_text(str(process.pid));return True
    finally:log.close()


def read(app,now):
    if not getattr(app,'real',False):return None
    from .app import USER_DIR
    cache=getattr(app,'_fr5_readonly_cache',None)
    if cache and now-cache[0]<.03:return cache[1]
    data=None
    try:
        cfg=json.loads((USER_DIR/'fr5_readonly.json').read_text(encoding='utf-8'))
        if cfg.get('enabled') is not True:return None
        host=getattr(app,'host',None)
        host=host.get().strip() if host is not None else ''
        from urllib.parse import urlsplit
        runtime=getattr(app,'runtime_arm_feedback',None)
        runtime_host=urlsplit(getattr(runtime,'base','')).hostname
        if host not in cfg.get('amr_hosts',[cfg['ssh_host']]) and runtime_host!=cfg['ssh_host']:return None
        if now-getattr(app,'_fr5_readonly_check',0)>10:
            app._fr5_readonly_check=now;ensure(USER_DIR)
        candidate=json.loads((USER_DIR/'fr5_direct_feedback.json').read_text(encoding='utf-8'))
        stamp=candidate.get('sample_monotonic');checked_now=time.monotonic()
        receiver_pid=int((USER_DIR/'fr5_readonly.pid').read_text())
        if (candidate.get('valid') is True and candidate.get('read_only') is True
            and candidate.get('controller_ip')==cfg['controller_ip'] and candidate.get('pid')==receiver_pid
            and type(stamp) in (int,float) and math.isfinite(stamp) and 0<=checked_now-stamp<=.8):data=candidate
    except (OSError,ValueError,KeyError):pass
    app._fr5_readonly_cache=(now,data)
    return data
