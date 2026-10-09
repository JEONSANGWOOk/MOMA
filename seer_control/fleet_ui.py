"""Render the complete ACS fleet from telemetry, without inventing movement."""
import math


def get_fleet(app):
    if not getattr(app,'real',False) or not getattr(app,'connected',False):return []
    raw=getattr(app,'raw',{})
    if not isinstance(raw,dict):return []
    response=raw.get('all',{})
    return response.get('fleet',[]) if isinstance(response,dict) and response.get('robot_model')=='openTCS Virtual AGV' else []


def positioned(fleet):
    return [v for v in fleet if all(type(v.get(k)) in (int,float) and math.isfinite(v[k]) for k in ('x','y'))]


def color(vehicle):
    return '#cf7900' if vehicle.get('waiting_for') or vehicle.get('paused') else '#287de0' if vehicle.get('selected') else '#168d66'


def draw_fleet(canvas,fleet,xy):
    for v in positioned(fleet):
        x,y=xy(v['x'],v['y']);shade=color(v);tag='acs_fleet_'+v['id']
        canvas.create_oval(x-10,y-10,x+10,y+10,fill=shade,outline='white',width=2,tags=('acs_fleet',tag))
        if v.get('selected'):canvas.create_oval(x-14,y-14,x+14,y+14,outline=shade,width=2,tags=('acs_fleet',tag))
        caption=v['id']+(' · 통행 대기' if v.get('waiting_for') else ' · 정지' if v.get('paused') else ' · 주행' if v.get('state')=='EXECUTING' else '')
        label=canvas.create_text(x,y-20,text=caption,fill=shade,font=('Malgun Gothic',9,'bold'),tags=('acs_fleet_label',tag))
        bounds=canvas.bbox(label)
        if bounds:
            background=canvas.create_rectangle(bounds[0]-3,bounds[1]-1,bounds[2]+3,bounds[3]+1,fill='white',outline='',tags=('acs_fleet',tag))
            canvas.tag_lower(background,label)
