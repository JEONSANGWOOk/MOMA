"""Same-PC desktop -> web door display handoff. Never sends robot commands."""
import json
import math
import os
from pathlib import Path
import time
import uuid


def state_path():
    return Path(os.environ.get('MOMA_DOOR_SYNC_PATH',str(Path.home()/'.seer_amr_console'/'door_display_live.json')))


def read_snapshot(path=None,now=None):
    try:
        data=json.loads((path or state_path()).read_text(encoding='utf-8'))
        age=(time.time() if now is None else now)-data['timestamp']
        if data.get('version')!=1 or not -.5<=age<=2:return None
        if not isinstance(data.get('revision'),str) or not data['revision']:return None
        if type(data.get('follow')) is not bool:return None
        if not isinstance(data.get('asset',''),str) or not isinstance(data.get('mode',''),str):return None
        if any(type(data['angles'].get(s)) not in (int,float) or not math.isfinite(data['angles'][s]) or not 0<=data['angles'][s]<=120 for s in ('left','right')):return None
        return data
    except (OSError,ValueError,KeyError,TypeError,AttributeError):return None


def publish(app,command=False):
    view=app.world3d;now=time.monotonic()
    if not command and now-getattr(app,'_door_sync_at',0)<.15:return False
    parts=getattr(getattr(view,'cad_asset',None),'door_parts',())
    angles={}
    for part in parts:
        name=part.get('name','');side='left' if name.endswith(':1') else 'right' if name.endswith(':2') else None
        if side:
            angle=math.degrees(view.cad_door_angle(part))
            if not math.isfinite(angle):return False
            angles[side]=round(max(0.,min(120.,angle)),9)
    if len(angles)!=2:return False
    if command or not getattr(app,'_door_sync_revision',None):app._door_sync_revision=uuid.uuid4().hex
    manual=getattr(view,'manual_real_door_display',None)
    real=bool(app.real)
    webcam=getattr(view,'webcam_doors_enabled',None)
    camera=bool(webcam and webcam.get())
    follow=bool(real and view.real_doors_follow.get() and manual is None and not camera)
    mode='manual' if manual is not None else 'camera' if real and camera else 'tracked' if follow else 'simulation' if not real else 'manual'
    data=dict(version=1,timestamp=time.time(),revision=app._door_sync_revision,angles=angles,
              follow=follow,mode=mode,asset=getattr(view.cad_asset,'name',''))
    path=state_path()
    try:
        path.parent.mkdir(parents=True,exist_ok=True)
        pending=path.with_suffix(f'.{os.getpid()}.tmp')
        pending.write_text(json.dumps(data,ensure_ascii=False,allow_nan=False),encoding='utf-8')
        os.replace(pending,path);app._door_sync_at=now
        return True
    except OSError:return False
