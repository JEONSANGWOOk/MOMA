"""Short-window filtering for camera displays; raw data remains authoritative."""
import copy
import math
import statistics
from collections import deque


class VisionFilter:
    def __init__(self):self.states={}
    def clear(self):self.states.clear()

    def vector(self,key,value,now,deadband=.0005):
        if value is None:self.states.pop(key,None);return None
        state=self.states.get(key)
        if state is None or now-state['time']>.7:
            self.states[key]=dict(time=now,history=deque([list(value)],maxlen=5),ema=list(value),shown=list(value))
            return list(value)
        state['history'].append(list(value))
        median=[statistics.median(v[i] for v in state['history']) for i in range(len(value))]
        alpha=1-math.exp(-max(0.,now-state['time'])/.25);state['time']=now
        state['ema']=[a+(b-a)*alpha for a,b in zip(state['ema'],median)]
        if math.dist(state['ema'],state['shown'])>deadband:state['shown']=list(state['ema'])
        return list(state['shown'])

    def rotation(self,key,value,now):
        if value is None:self.states.pop(key,None);return None
        angle=math.sqrt(sum(v*v for v in value))
        q=[math.cos(angle/2)]+[v*math.sin(angle/2)/angle if angle>1e-12 else 0. for v in value]
        previous=self.states.get(key)
        if previous and sum(a*b for a,b in zip(previous['ema'],q))<0:q=[-v for v in q]
        filtered=self.vector(key,q,now,deadband=math.sin(math.radians(.4)/2))
        norm=math.sqrt(sum(v*v for v in filtered));q=[v/norm for v in filtered]
        length=math.sqrt(sum(v*v for v in q[1:]))
        if length<1e-12:return [0.,0.,0.]
        angle=2*math.atan2(length,q[0]);return [v*angle/length for v in q[1:]]

    def markers(self,markers,now,orientation):
        result=copy.deepcopy(markers);counts={}
        for m in result:
            key=(m['dictionary'],m['id']);counts[key]=counts.get(key,0)+1
        active=set()
        for m in result:
            prefix=('marker',m['dictionary'],m['id'])
            if counts[(m['dictionary'],m['id'])]!=1:continue
            for field in ('camera_xyz_m','depth_camera_xyz_m'):
                key=prefix+(field,);active.add(key);m[field]=self.vector(key,m.get(field),now)
            key=prefix+('rotation',);active.add(key)
            m['rotation_vector_rad']=self.rotation(key,m.get('rotation_vector_rad'),now)
            if m['rotation_vector_rad'] is not None:m['orientation_deg']=orientation(m['rotation_vector_rad'])
            if m.get('depth_camera_xyz_m') is not None:m['depth_z_m']=m['depth_camera_xyz_m'][2]
        for key in list(self.states):
            if key[0]=='marker' and key not in active:self.states.pop(key,None)
        return result

    def board(self,board,now,orientation):
        result=copy.deepcopy(board);prefix=('board',board.get('revision'))
        for key in list(self.states):
            if key[0]=='board' and (key[:2]!=prefix or not board.get('valid')):self.states.pop(key,None)
        if board.get('valid'):
            result['camera_xyz_m']=self.vector(prefix+('position',),board['camera_xyz_m'],now)
            result['rotation_vector_rad']=self.rotation(prefix+('rotation',),board['rotation_vector_rad'],now)
            result['orientation_deg']=orientation(result['rotation_vector_rad'])
        return result

    def pair(self,pair,now):
        if pair is None:
            for key in list(self.states):
                if key[0]=='pair':self.states.pop(key,None)
            return None
        result=copy.deepcopy(pair);prefix=('pair',pair.get('source'),tuple(pair['marker_indices']))
        for key in list(self.states):
            if key[0]=='pair' and key[:3]!=prefix:self.states.pop(key,None)
        for field,band in [('center_distance_m',.0005),('line_depth_deg',.5),('image_line_deg',.2)]:
            if pair.get(field) is not None:
                result[field]=self.vector(prefix+(field,),[pair[field]],now,band)[0]
        return result
