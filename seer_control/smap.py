"""Read SEER JSON SMAP without altering source properties or robot paths."""
import json
import math
from pathlib import Path
from .model import MapModel, number


def _unwrap_map(data):
    """Accept direct SMAP JSON and a few common API wrapper shapes."""
    if isinstance(data,str):
        data=json.loads(data)
    if not isinstance(data,dict):
        raise ValueError('지도 응답이 JSON object가 아닙니다.')
    if isinstance(data.get('header'),dict):
        return data
    for key in ('map','data','map_data','content'):
        value=data.get(key)
        if isinstance(value,str):
            try:value=json.loads(value)
            except json.JSONDecodeError:continue
        if isinstance(value,dict) and isinstance(value.get('header'),dict):
            return value
    raise ValueError('JSON SMAP header가 없습니다. 로봇 펌웨어의 4011 응답 형식을 확인하세요.')


def _point(value, default=(0.0,0.0)):
    """Read the several point wrappers seen in JSON SMAP exports."""
    if not isinstance(value,dict):
        return default
    # Some RoboShop JSON exports wrap coordinates under pos/point/value.
    for key in ('pos','point','value'):
        if isinstance(value.get(key),dict):
            value=value[key]
            break
    try:
        dx,dy=(0.0,0.0) if default is None else default
        if 'x' not in value or 'y' not in value:
            return default
        return (number(value.get('x',dx)), number(value.get('y',dy)))
    except Exception:
        return default


def _first_point(obj, names, default):
    for name in names:
        if name in obj:
            p=_point(obj.get(name),None)
            if p is not None:
                return p
    return default


def _bezier(a,u,v,b,segments=48):
    out=[]
    for i in range(segments+1):
        t=i/segments
        mt=1.0-t
        x=mt**3*a[0]+3*mt**2*t*u[0]+3*mt*t*t*v[0]+t**3*b[0]
        y=mt**3*a[1]+3*mt**2*t*u[1]+3*mt*t*t*v[1]+t**3*b[1]
        if math.isfinite(x) and math.isfinite(y):out.append((x,y))
    return out




def _properties(items):
    """Convert protobuf-JSON MapProperty[] into a normal {key: value} dict."""
    out={}
    if not isinstance(items,list):
        return out
    typed=('boolValue','int32Value','uint32Value','int64Value','uint64Value',
           'floatValue','doubleValue','stringValue','bytesValue')
    for item in items:
        if not isinstance(item,dict) or not item.get('key'):
            continue
        value=None
        for field in typed:
            if field in item:
                value=item[field]; break
        if value is None and 'value' in item:
            value=item.get('value')
        out[str(item['key'])]=value
    return out

def load_smap_data(data, source_name='robot_map'):
    data=_unwrap_map(data)
    nodes=[]
    for p in data.get('advancedPointList',[]):
        if p.get('className') not in ('LandMark','LocationMark','SpecialLocation','ChargePoint','RobotHome','ReturnPoint','GyrocaliPoint','GyroCaliPoint'): continue
        pos=p.get('pos',{})
        props=_properties(p.get('property',[]))
        node=dict(p)
        node.update(props)
        node.update(id=p['instanceName'], x=number(pos.get('x',0)), y=number(pos.get('y',0)),
                    kind='dock' if p.get('className')=='ChargePoint' else 'station',
                    station_type=p.get('className','LocationMark'), properties=props, smap_raw=dict(p))
        # Official SMAP AdvancedPoint stores its heading as `dir`.  Some live
        # station APIs expose the same value as `r`; normalize both to `r`.
        raw_r=p.get('dir', p.get('r', pos.get('r')))
        if isinstance(raw_r,(int,float)) and not isinstance(raw_r,bool):
            node['r']=float(raw_r)
            node['angle']=float(raw_r)
        ignore=p.get('ignoreDir',p.get('ignore_dir'))
        if ignore is not None:
            node['ignoreDir']=bool(ignore)
        nodes.append(node)
    model=MapModel(dict(format='amr-console-map-v1',name=data['header'].get('mapName',Path(source_name).stem),nodes=nodes or [dict(id='_display_origin',x=0,y=0)],edges=[],walls=[]))
    if not nodes:model.nodes={}
    normal=data.get('normalPosList',[])
    if not isinstance(normal,list): normal=[]
    step=max(1,len(normal)//12000)
    model.cloud=[(number(p.get('x',0)),number(p.get('y',0))) for p in normal[::step] if isinstance(p,dict)]

    # Preserve RoboShop path geometry.  Earlier versions only handled one exact
    # Bezier JSON spelling, so valid curves from firmware/RoboShop variants could
    # silently fall back to a straight start->end line.
    model.curves=[]
    model.path_records=[]
    model.edges=[]
    class_counts={}
    curved=0
    straight=0
    for c in data.get('advancedCurveList',[]):
        if not isinstance(c,dict):continue
        cname=str(c.get('className') or c.get('type') or 'Unknown')
        class_counts[cname]=class_counts.get(cname,0)+1
        a=_first_point(c,('startPos','startPoint','start','sourcePos'),(0.0,0.0))
        b=_first_point(c,('endPos','endPoint','end','targetPos'),a)

        lower=cname.lower()
        looks_bezier=('bezier' in lower or 'curve' in lower)
        u=_first_point(c,('controlPos1','controlPoint1','ctrlPos1','ctrlPoint1','p1'),None)
        v=_first_point(c,('controlPos2','controlPoint2','ctrlPos2','ctrlPoint2','p2'),None)

        # A few exports keep control points in a list.
        controls=c.get('controlPoints') or c.get('ctrlPoints')
        if isinstance(controls,list):
            if u is None and len(controls)>0:u=_point(controls[0],None)
            if v is None and len(controls)>1:v=_point(controls[1],None)

        if looks_bezier or (u is not None and v is not None):
            # Missing one handle: use the endpoint as the neutral handle instead
            # of throwing away the remaining curvature information.
            u=a if u is None else u
            v=b if v is None else v
            line=_bezier(a,u,v,b)
            curved+=1
        else:
            line=[a,b]
            straight+=1
        if len(line)>=2:
            model.curves.append(line)
            sa=c.get('startPos',{}) if isinstance(c.get('startPos'),dict) else {}
            sb=c.get('endPos',{}) if isinstance(c.get('endPos'),dict) else {}
            aid=str(sa.get('instanceName','')); bid=str(sb.get('instanceName',''))
            props=_properties(c.get('property',[]))
            rec={'a':aid,'b':bid,'className':cname,'instanceName':c.get('instanceName') or f'{aid}-{bid}',
                 'controls':[u or a,v or b],'properties':props,'raw':dict(c),'draft':False}
            model.path_records.append(rec)
            if aid in model.nodes and bid in model.nodes and aid!=bid:
                model.edges.append([aid,bid])

    # Advanced Area: RoboShop work zones with speed / obstacle / sensor overrides.
    model.area_records=[]
    for idx,area in enumerate(data.get('advancedAreaList',[]) or []):
        if not isinstance(area,dict):
            continue
        pts=[]
        for pos in area.get('posGroup',[]) or []:
            pt=_point(pos,None)
            if pt is not None: pts.append(pt)
        if len(pts)<3:
            continue
        props=_properties(area.get('property',[]))
        model.area_records.append({'id':str(area.get('instanceName') or f'Area{idx+1}'),
                                   'className':str(area.get('className') or 'AdvancedArea'),
                                   'points':pts,'properties':props,'raw':dict(area),'draft':False})

    summary=', '.join(f'{k}:{v}' for k,v in sorted(class_counts.items())) or 'none'
    model.curve_diagnostics=f'advancedCurveList={len(model.curves)} · curved={curved} · straight={straight} · types=[{summary}]'
    model.smap_source=data
    return model


def load_smap(path):
    data=json.loads(Path(path).read_text(encoding='utf-8-sig'))
    return load_smap_data(data, path)


# ---------------- Editable SMAP helpers ----------------
def _prop_value(prop):
    if not isinstance(prop,dict): return None
    for k in ('boolValue','int32Value','uint32Value','int64Value','uint64Value','floatValue','doubleValue','stringValue'):
        if k in prop:return prop[k]
    return prop.get('value')


def _set_property(items,key,value):
    """Preserve an existing property's wire type where possible."""
    if not isinstance(items,list): items=[]
    target=None
    for item in items:
        if isinstance(item,dict) and str(item.get('key'))==key:
            target=item; break
    if target is None:
        target={'key':key}; items.append(target)
    if isinstance(value,bool):
        target.update(type='bool',boolValue=value,value='dHJ1ZQ==' if value else 'ZmFsc2U=')
        for k in ('int32Value','uint32Value','floatValue','doubleValue','stringValue'): target.pop(k,None)
    elif isinstance(value,int) and not isinstance(value,bool):
        import base64
        target.update(type='int32',int32Value=int(value),value=base64.b64encode(str(int(value)).encode()).decode())
        for k in ('boolValue','floatValue','doubleValue','stringValue'): target.pop(k,None)
    elif isinstance(value,(int,float)):
        target.update(type='double',doubleValue=float(value))
    else:
        target.update(type='string',stringValue=str(value))
    return items


def _station_class(node):
    # The live 1301 API may call a point `LocationMark` even when the SMAP wire
    # class is LandMark.  Existing points must retain the exact className pulled
    # from the robot; otherwise a round-trip can silently rewrite every point.
    raw=node.get('smap_raw') if isinstance(node,dict) else None
    if isinstance(raw,dict) and isinstance(raw.get('className'),str) and raw.get('className'):
        return raw['className']
    t=str(node.get('className') or node.get('station_type') or node.get('type') or 'LandMark')
    aliases={'OrdinaryMark':'LandMark','SpecialLocation':'LandMark',
             'LandMark':'LandMark','LocationMark':'LocationMark','ChargePoint':'ChargePoint',
             'RobotHome':'RobotHome','ReturnPoint':'ReturnPoint','GyroCaliPoint':'GyroCaliPoint'}
    return aliases.get(t,t)


def _point_json(node, template=None):
    import copy
    raw=copy.deepcopy(template if isinstance(template,dict) else node.get('smap_raw') if isinstance(node.get('smap_raw'),dict) else {})
    raw['className']=_station_class(node)
    raw['instanceName']=str(node['id'])
    raw['pos']={'x':float(node['x']),'y':float(node['y'])}
    r=node.get('r',node.get('angle',0.0))
    if isinstance(r,(int,float)) and math.isfinite(r): raw['dir']=float(r)
    raw['property']=_set_property(raw.get('property',[]),'spin',bool(node.get('spin',True)))
    desc=node.get('desc')
    if desc not in (None,''):
        # Some firmware stores desc as string, some as protobuf bytes/base64. Do not
        # overwrite an existing encoded desc unless it is already textual.
        if not isinstance(raw.get('desc'),str) or not raw.get('desc'):
            raw['desc']=str(desc)
    return raw


def _advanced_ref(node):
    return {'className':_station_class(node),'instanceName':str(node['id']),
            'pos':{'x':float(node['x']),'y':float(node['y'])}}


def make_path_record(a_node,b_node,curve_type='BezierPath',properties=None,template=None):
    """Create an editable SMAP path record using official AdvancedCurve fields."""
    import copy
    a=(float(a_node['x']),float(a_node['y'])); b=(float(b_node['x']),float(b_node['y']))
    dx,dy=b[0]-a[0],b[1]-a[1]
    raw=copy.deepcopy(template) if isinstance(template,dict) else {}
    raw['className']=curve_type
    raw['instanceName']=f"{a_node['id']}-{b_node['id']}"
    raw['startPos']=_advanced_ref(a_node); raw['endPos']=_advanced_ref(b_node)
    raw['controlPos1']={'x':a[0]+dx/3.0,'y':a[1]+dy/3.0}
    raw['controlPos2']={'x':a[0]+2.0*dx/3.0,'y':a[1]+2.0*dy/3.0}
    raw.setdefault('property',[]); raw.setdefault('devices',[])
    for k,v in (properties or {}).items(): raw['property']=_set_property(raw['property'],k,v)
    return raw


def path_record_geometry(raw):
    a=_first_point(raw,('startPos','startPoint','start','sourcePos'),(0.0,0.0))
    b=_first_point(raw,('endPos','endPoint','end','targetPos'),a)
    u=_first_point(raw,('controlPos1','controlPoint1','ctrlPos1','ctrlPoint1','p1'),None)
    v=_first_point(raw,('controlPos2','controlPoint2','ctrlPos2','ctrlPoint2','p2'),None)
    lower=str(raw.get('className','')).lower()
    if 'straight' in lower:return [a,b]
    if u is None:u=a
    if v is None:v=b
    return _bezier(a,u,v,b)


def make_area_record(area_id, points, properties=None, template=None):
    """Create a SEER AdvancedArea record. points are map/world XY vertices."""
    import copy
    raw=copy.deepcopy(template) if isinstance(template,dict) else {}
    raw['className']='AdvancedArea'
    raw['instanceName']=str(area_id)
    raw['posGroup']=[{'x':float(x),'y':float(y)} for x,y in points]
    raw.setdefault('property',[]); raw.setdefault('devices',[])
    for k,v in (properties or {}).items():
        raw['property']=_set_property(raw.get('property',[]),k,v)
    return raw


def validate_smap_edit(model):
    errors=[]
    if not hasattr(model,'smap_source') or not isinstance(model.smap_source,dict):
        return ['Pull Map으로 받은 SMAP 원본이 없습니다.']
    if not model.nodes:errors.append('Point가 하나도 없습니다.')
    for key,n in model.nodes.items():
        if not key or not isinstance(key,str):errors.append('빈 Point 이름이 있습니다.')
        for k in ('x','y'):
            if not isinstance(n.get(k),(int,float)) or not math.isfinite(float(n[k])):errors.append(f'{key}: {k} 좌표가 비정상입니다.')
    seen=set()
    for rec in getattr(model,'path_records',[]):
        a=rec.get('a'); b=rec.get('b')
        if a not in model.nodes or b not in model.nodes:errors.append(f'Path {a}->{b}: Point가 없습니다.')
        if a==b:errors.append(f'Path {a}: 자기 자신 연결입니다.')
        pair=(a,b)
        if pair in seen:errors.append(f'Path {a}->{b}: 중복입니다.')
        seen.add(pair)
    area_ids=set()
    for area in getattr(model,'area_records',[]):
        aid=str(area.get('id','')).strip()
        if not aid: errors.append('Advanced Area 이름이 비어 있습니다.'); continue
        if aid in area_ids: errors.append(f'Advanced Area {aid}: 이름 중복입니다.')
        area_ids.add(aid)
        pts=area.get('points',[])
        if not isinstance(pts,list) or len(pts)<3: errors.append(f'Advanced Area {aid}: 꼭짓점이 부족합니다.')
        for pt in pts if isinstance(pts,list) else []:
            if not (isinstance(pt,(tuple,list)) and len(pt)>=2 and all(isinstance(v,(int,float)) and math.isfinite(float(v)) for v in pt[:2])):
                errors.append(f'Advanced Area {aid}: 좌표가 비정상입니다.'); break
    return errors


def build_smap_from_model(model):
    """Apply edited Point/Path graph to a copy of the pulled SMAP, preserving all unrelated layers."""
    import copy
    errors=validate_smap_edit(model)
    if errors: raise ValueError('SMAP 검증 실패: '+' / '.join(errors[:6]))
    source=copy.deepcopy(model.smap_source)
    original_points={str(p.get('instanceName')):p for p in source.get('advancedPointList',[]) if isinstance(p,dict)}
    # Preserve template/firmware-specific fields for existing nodes. New nodes use a
    # LandMark template if one exists and then overwrite the identity/pose/properties.
    point_template=next((p for p in source.get('advancedPointList',[]) if isinstance(p,dict) and p.get('className') in ('LandMark','LocationMark')),None)
    new_points=[]
    for key,n in model.nodes.items():
        tmpl=original_points.get(key) or point_template
        new_points.append(_point_json(n,tmpl))
    source['advancedPointList']=new_points

    raw_paths=[]
    original_curves=[c for c in source.get('advancedCurveList',[]) if isinstance(c,dict)]
    curve_template=next((c for c in original_curves if 'bezier' in str(c.get('className','')).lower()), None) or (original_curves[0] if original_curves else None)
    for rec in getattr(model,'path_records',[]):
        a=model.nodes[rec['a']]; b=model.nodes[rec['b']]
        raw=copy.deepcopy(rec.get('raw')) if isinstance(rec.get('raw'),dict) else make_path_record(a,b,rec.get('className','BezierPath'),rec.get('properties'),curve_template)
        raw['className']=rec.get('className') or raw.get('className') or 'BezierPath'
        raw['instanceName']=rec.get('instanceName') or f"{a['id']}-{b['id']}"
        raw['startPos']=_advanced_ref(a); raw['endPos']=_advanced_ref(b)
        cps=rec.get('controls')
        if isinstance(cps,(list,tuple)) and len(cps)>=2:
            raw['controlPos1']={'x':float(cps[0][0]),'y':float(cps[0][1])}
            raw['controlPos2']={'x':float(cps[1][0]),'y':float(cps[1][1])}
        elif 'straight' not in str(raw['className']).lower():
            dx,dy=b['x']-a['x'],b['y']-a['y']
            raw['controlPos1']={'x':a['x']+dx/3.0,'y':a['y']+dy/3.0}
            raw['controlPos2']={'x':a['x']+2*dx/3.0,'y':a['y']+2*dy/3.0}
        props=rec.get('properties') or {}
        for k,v in props.items():raw['property']=_set_property(raw.get('property',[]),k,v)
        raw_paths.append(raw)
    source['advancedCurveList']=raw_paths

    # Apply Advanced Area editor records while preserving firmware-specific fields.
    original_areas=[a for a in source.get('advancedAreaList',[]) if isinstance(a,dict)]
    original_area_by_id={str(a.get('instanceName')):a for a in original_areas}
    area_template=original_areas[0] if original_areas else None
    raw_areas=[]
    for area in getattr(model,'area_records',[]):
        aid=str(area.get('id'))
        raw=copy.deepcopy(area.get('raw')) if isinstance(area.get('raw'),dict) else copy.deepcopy(original_area_by_id.get(aid) or area_template or {})
        raw['className']=area.get('className') or 'AdvancedArea'
        raw['instanceName']=aid
        raw['posGroup']=[{'x':float(x),'y':float(y)} for x,y in area.get('points',[])]
        raw.setdefault('property',[]); raw.setdefault('devices',[])
        for k,v in (area.get('properties') or {}).items(): raw['property']=_set_property(raw.get('property',[]),k,v)
        raw_areas.append(raw)
    source['advancedAreaList']=raw_areas
    # Export locally drawn virtual walls as thin forbidden Advanced Areas.
    for i,(x1,y1,x2,y2) in enumerate(getattr(model,'virtual_walls',[])):
        distance=math.hypot(x2-x1,y2-y1)
        if distance<1e-9:continue
        nx,ny=-(y2-y1)/distance*.025,(x2-x1)/distance*.025
        points=[(x1+nx,y1+ny),(x2+nx,y2+ny),(x2-nx,y2-ny),(x1-nx,y1-ny)]
        name=f'StudioVirtualWall_{i+1}'
        while any(a.get('instanceName')==name for a in raw_areas):name+='_'
        raw_areas.append(make_area_record(name,points,{'forbidden':True}))

    # Keep header bounds consistent when an editor adds points/areas outside the old extent.
    xs=[];ys=[]
    for n in model.nodes.values():xs.append(float(n['x']));ys.append(float(n['y']))
    for raw in raw_paths:
        for key in ('controlPos1','controlPos2'):
            pos=raw.get(key)
            if isinstance(pos,dict) and isinstance(pos.get('x'),(int,float)) and isinstance(pos.get('y'),(int,float)):
                xs.append(float(pos['x']));ys.append(float(pos['y']))
    for raw in source.get('advancedAreaList',[]):
        for pos in raw.get('posGroup',[]) if isinstance(raw,dict) else []:
            if isinstance(pos,dict) and isinstance(pos.get('x'),(int,float)) and isinstance(pos.get('y'),(int,float)):
                xs.append(float(pos['x']));ys.append(float(pos['y']))
    if xs and ys:
        header=source.setdefault('header',{})
        oldmin=header.get('minPos') if isinstance(header.get('minPos'),dict) else {}
        oldmax=header.get('maxPos') if isinstance(header.get('maxPos'),dict) else {}
        header['minPos']={'x':min([min(xs),float(oldmin.get('x',min(xs)))]),'y':min([min(ys),float(oldmin.get('y',min(ys)))])}
        header['maxPos']={'x':max([max(xs),float(oldmax.get('x',max(xs)))]),'y':max([max(ys),float(oldmax.get('y',max(ys)))])}
    return source
