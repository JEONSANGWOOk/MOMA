"""Copy beside scenario.py and viewer_moma.py. Read-only marker telemetry."""
import threading
import time
import uuid

_lock=threading.Lock()
_sample=None


def capture(moma,marker):
    """marker is camera.target_pose(... WORLD), before door motion.

    Convert using SDK matrices; no Euler convention/unit guessing for rotation.
    This SDK exposes Cartesian xyz in millimetres. Convert translation only.
    """
    global _sample
    try:
        import sdc_extctrlpy as exc
        relative=moma.GetTCP(exc.COORD.RBASE)*moma.GetTCP(exc.COORD.WORLD).inverse()*marker
        # Rotation columns are obtained with SDK Vector3 transforms by translating
        # the frame along its local axes; xyz() exposes .x/.y/.z in this SDK.
        p=relative.xyz();origin=[p.x,p.y,p.z]
        columns=[]
        for axis in ((1,0,0),(0,1,0),(0,0,1)):
            q=(relative*exc.Matrix(exc.Vector3(*axis))).xyz()
            columns.append([q.x-origin[0],q.y-origin[1],q.z-origin[2]])
        matrix=[[columns[j][i] for j in range(3)]+[origin[i]/1000.] for i in range(3)]+[[0,0,0,1]]
        def pack_native(value):
            p=value.xyz();o=[p.x,p.y,p.z];cols=[]
            for axis in ((1,0,0),(0,1,0),(0,0,1)):
                q=(value*exc.Matrix(exc.Vector3(*axis))).xyz();cols.append([q.x-o[0],q.y-o[1],q.z-o[2]])
            return [[cols[j][i] for j in range(3)]+[o[i]/1000.] for i in range(3)]+[[0,0,0,1]]
        map_matrix=pack_native(marker)
        # Importable without the robot SDK until called. Never writes IO or moves.
        with _lock:_sample=dict(valid=True,capture_id=uuid.uuid4().hex,marker_id=0,frame='rbase_m',matrix=matrix,map_frame='controller_world_m',map_matrix=map_matrix,_at=time.monotonic())
    except Exception as error:
        with _lock:_sample=dict(valid=False,error=str(error),_at=time.monotonic())


def status():
    with _lock:data=dict(_sample) if _sample else dict(valid=False)
    at=data.pop('_at',None)
    if at is not None:data['age_s']=time.monotonic()-at
    return data
