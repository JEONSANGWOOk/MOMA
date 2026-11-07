"""Read-only joint feedback from the MoMa runtime viewer's HTTP API."""
import json
from collections import deque
import math
import threading
import time
import urllib.request
from urllib.parse import urlsplit


def parse_joints(payload):
    joints = payload['robot']['joint']
    values = [joints[f'J{i}'] for i in range(1, 7)]
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError('Runtime requires six finite joint angles in degrees')
    return tuple(math.radians(v) for v in values)


class RuntimeArmFeedback:
    def __init__(self, url, timeout=.8, period=1/30):
        parts = urlsplit(url)
        if parts.scheme not in ('http', 'https') or not parts.hostname:
            raise ValueError('Invalid runtime viewer URL')
        self.base = f'{parts.scheme}://{parts.netloc}'
        self.timeout, self.period = timeout, period
        self._lock = threading.Lock()
        self._payload = None
        self._sample = None
        self._history=deque(maxlen=12)
        self.error = ''
        # The viewer is a local robot service; avoid system HTTP proxy detours.
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def _get(self, path):
        with self.opener.open(self.base + path, timeout=self.timeout) as response:
            data = response.read(262145)
        if len(data) > 262144:
            raise ValueError('Runtime status exceeds size limit')
        return json.loads(data)

    def poll(self):
        # Reject cached robot data when the viewer has lost its robot connection.
        if self._get('/api/connection_status').get('connected') is not True:
            raise ConnectionError('Runtime viewer is disconnected from robot')
        payload=self._get('/api/moma_status')
        values = parse_joints(payload)
        with self._lock:
            stamp=time.monotonic()
            if self._history and stamp-self._history[-1][0]>.5:self._history.clear()
            self._history.append((stamp,values))
            self._sample = (stamp, values)
            self._payload = payload
            token=payload.get('robot',{}).get('vacuum_io',{}).get('sample_time')
            if token is not None and token!=getattr(self,'_io_token',None):
                self._io_token=token;self._io_changed=time.monotonic()
            self.error = ''

    def run(self, stop):
        failures = 0
        try:
            while not stop.is_set():
                started = time.monotonic()
                try:
                    self.poll()
                    failures = 0
                except Exception as error:
                    with self._lock:
                        self._sample = None
                        self._history.clear()
                        self.error = str(error)
                    failures += 1
                delay = min(2., .2 * 2**min(failures-1, 4)) if failures else self.period
                stop.wait(max(0., delay - (time.monotonic()-started)))
        finally:
            with self._lock:
                self._sample = None

    def joints(self, names, max_age=2.):
        names = tuple(names)
        with self._lock:
            sample = self._sample
        if sample is None or len(names) != 6 or time.monotonic()-sample[0] > max_age:
            return {}
        return dict(zip(names, sample[1]))

    def signal(self,path,max_age=1.):
        with self._lock:
            sample=self._sample;data=self._payload
        if not path or sample is None or time.monotonic()-sample[0]>max_age:return None
        try:
            for key in path.split('.'):
                data=data[int(key)] if isinstance(data,list) else data[key]
        except (KeyError,IndexError,TypeError,ValueError):return None
        if type(data) is bool:return data
        if type(data) is int and data in (0,1):return bool(data)
        return None

    def vacuum_state(self,max_age=1.):
        from .vacuum_panel_display import vacuum_output_state
        with self._lock:
            sample=self._sample;data=self._payload
            changed=getattr(self,'_io_changed',0.)
        now=time.monotonic()
        if sample is None or now-sample[0]>max_age or now-changed>max_age:return None
        io=(data or {}).get('robot',{}).get('vacuum_io',{})
        if io.get('valid') is not True:return None
        return vacuum_output_state(io.get('outputs',{}))

    def visual_joints(self,names,delay=.065,max_age=1.):
        """Short delayed interpolation for display only; no prediction or control."""
        names=tuple(names);now=time.monotonic()
        with self._lock:
            sample=self._sample;history=list(self._history)
        if sample is None or len(names)!=6 or now-sample[0]>max_age or not history:return {}
        target=now-delay
        values=history[0][1]
        for index,(stamp,angles) in enumerate(history):
            if stamp>=target:
                if index:
                    before,old=history[index-1]
                    alpha=max(0.,min(1.,(target-before)/max(1e-9,stamp-before)))
                    # FR5 joints have bounded ranges; interpolate numeric angles
                    # directly instead of wrapping to an invalid shortest arc.
                    values=tuple(a+(b-a)*alpha for a,b in zip(old,angles))
                break
            values=angles
        return dict(zip(names,values))
