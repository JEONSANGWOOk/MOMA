"""Keep measured and simulated robot poses distinct; never synthesize REAL data."""
import math,time

def real_arm_positions(app,joints,now=None):
 now=time.monotonic() if now is None else now
 client=getattr(app,'fr5_client',None);feedback=getattr(app,'fr5_feedback',{})
 values=feedback.get('joints_rad',[])
 if not client or not client.connected or now-getattr(app,'fr5_rx',0)>2:return None
 if len(values)!=len(joints) or len(values)!=6 or any(type(v) not in (float,int) or not math.isfinite(v) for v in values):return None
 return {joint['name']:value for joint,value in zip(joints,values)}
