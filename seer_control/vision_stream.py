"""Atomic latest-frame handoff, independent of the slower archived camera log."""
import json
import time
from pathlib import Path
from .aruco_arm_follow import latest_record


def publish_frame(path,record):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    pending=path.with_suffix('.tmp')
    pending.write_text(json.dumps(record,allow_nan=False),encoding='utf-8')
    pending.replace(path)


def read_frame(log_path,live_path=None,now=None,previous=None):
    now=time.time() if now is None else now
    record=None
    if live_path is not None:
        try:
            candidate=json.loads(Path(live_path).read_text(encoding='utf-8'))
            if -.1<=now-candidate['timestamp']<=.7:record=candidate
        except (OSError,ValueError,KeyError,TypeError):pass
    if record is None:record=latest_record(log_path)
    # A Windows snapshot read can briefly fail during replacement. An older archive
    # must not rewind the controller clock; keep the last fresh frame without moving.
    if previous is not None and -.1<=now-previous['timestamp']<=.7:
        if record is None or record.get('timestamp',0)<previous['timestamp']:record=previous
    if record and 'control_board' in record:
        record=dict(record,board=record['control_board'])
    return record
