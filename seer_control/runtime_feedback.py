"""Read-only joint feedback from the MoMa runtime viewer's HTTP API."""
import json
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
    def __init__(self, url, timeout=.8, period=.1):
        parts = urlsplit(url)
        if parts.scheme not in ('http', 'https') or not parts.hostname:
            raise ValueError('Invalid runtime viewer URL')
        self.base = f'{parts.scheme}://{parts.netloc}'
        self.timeout, self.period = timeout, period
        self._lock = threading.Lock()
        self._sample = None
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
        values = parse_joints(self._get('/api/moma_status'))
        with self._lock:
            self._sample = (time.monotonic(), values)
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
