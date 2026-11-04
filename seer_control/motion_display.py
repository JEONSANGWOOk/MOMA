"""Bounded visual smoothing only; never feed these values to robot control."""
import math

class MotionDisplay:
    def __init__(self):
        self.values=None;self.stamp=None;self.context=None
    def update(self,values,now,context=None,angular=(),jump=1.):
        values=dict(values)
        if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values.values()):return values
        dt=now-self.stamp if self.stamp is not None else 0
        reset=(self.values is None or self.context!=context or self.values.keys()!=values.keys() or not 0<dt<=.5)
        if not reset and jump is not None:
            reset=any(abs(v-self.values[k])>jump for k,v in values.items() if k not in angular)
        if reset:self.values=values
        else:
            alpha=1-math.exp(-dt/.08)
            for key,target in values.items():
                delta=target-self.values[key]
                if key in angular:delta=math.atan2(math.sin(delta),math.cos(delta))
                self.values[key]+=delta*alpha
                if abs(delta)<1e-5:self.values[key]=target
        self.stamp=now;self.context=context
        return dict(self.values)
