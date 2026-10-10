"""Process lifetime lock prevents multiple D455 producers from opening the device."""
import os
from contextlib import contextmanager
from pathlib import Path


@contextmanager
def camera_lease(path):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    handle=path.open('a+b');locked=False
    try:
        if handle.seek(0,2)==0:handle.write(b'0');handle.flush()
        handle.seek(0)
        try:
            if os.name=='nt':
                import msvcrt
                msvcrt.locking(handle.fileno(),msvcrt.LK_NBLCK,1)
            else:
                import fcntl
                fcntl.flock(handle.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
            locked=True
        except OSError as exc:raise RuntimeError('D455 인식기가 이미 실행 중입니다. 기존 영상을 공유하거나 먼저 종료하세요.') from exc
        yield
    finally:
        if locked:
            handle.seek(0)
            if os.name=='nt':msvcrt.locking(handle.fileno(),msvcrt.LK_UNLCK,1)
            else:fcntl.flock(handle.fileno(),fcntl.LOCK_UN)
        handle.close()
