"""Shared render clock for all mirrored poses; no robot commands or extrapolation."""
from collections import deque

class PoseTimeline:
    def __init__(self,delay=.08):
        self.delay=delay;self.samples=deque(maxlen=12)
    def clear(self):self.samples.clear()
    def push(self,now,poses):
        if self.samples and now-self.samples[-1][0]>.5:self.clear()
        self.samples.append((now,dict(poses)))
    def sample(self,now):
        if not self.samples:return {},{},1.
        target=now-self.delay
        while len(self.samples)>2 and self.samples[1][0]<=target:self.samples.popleft()
        a=self.samples[0]
        if target<=a[0]:return a[1],a[1],1.
        for b in list(self.samples)[1:]:
            if target<=b[0]:return a[1],b[1],max(0.,min(1.,(target-a[0])/max(1e-9,b[0]-a[0])))
            a=b
        return a[1],a[1],1.
