import json, math
from pathlib import Path
from datetime import datetime


def normalized_pose_record(map_name, x, y, theta, confidence=None, source='REAL', timestamp=None):
    rec={
        'map_name':str(map_name or ''),
        'x':float(x),'y':float(y),'theta':float(theta),
        'source':str(source or 'REAL'),
        'timestamp':timestamp or datetime.now().isoformat(timespec='seconds')
    }
    if confidence is not None:
        try: rec['confidence']=float(confidence)
        except Exception: pass
    return rec


def save_pose(path, record):
    p=Path(path); p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8')


def load_pose(path):
    p=Path(path)
    if not p.exists(): return None
    data=json.loads(p.read_text(encoding='utf-8'))
    for k in ('map_name','x','y','theta'):
        if k not in data: raise ValueError(f'last pose missing: {k}')
    data['x']=float(data['x']); data['y']=float(data['y']); data['theta']=float(data['theta'])
    return data


def map_matches(saved_map, current_map):
    return bool(saved_map and current_map and str(saved_map)==str(current_map))


def confidence_ok(value, threshold):
    try: return float(value) >= float(threshold)
    except Exception: return False


def nearest_node(nodes, x, y, allowed_ids=None):
    allowed=set(allowed_ids) if allowed_ids is not None else None
    best=None
    for key,n in nodes.items():
        if allowed is not None and key not in allowed: continue
        try: d=math.hypot(float(n['x'])-float(x),float(n['y'])-float(y))
        except Exception: continue
        if best is None or d<best[1]: best=(key,d)
    return best


def nearest_reachable_node(map_model, x, y):
    nodes=getattr(map_model,'nodes',{}) or {}
    if not nodes: return None
    records=getattr(map_model,'path_records',[]) or []
    if records:
        ids=set()
        for r in records:
            a,b=r.get('a'),r.get('b')
            if a in nodes: ids.add(a)
            if b in nodes: ids.add(b)
        return nearest_node(nodes,x,y,ids or None)
    edges=getattr(map_model,'edges',[]) or []
    if edges:
        ids={k for e in edges for k in e if k in nodes}
        return nearest_node(nodes,x,y,ids or None)
    return nearest_node(nodes,x,y)
