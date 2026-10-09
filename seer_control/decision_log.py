"""Structured runtime decisions. File IO runs off the control/Tk thread."""
from collections import deque
from datetime import datetime
from functools import wraps
import json
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import queue
import threading
import time
import uuid


def clean(value, depth=0):
    if depth > 5:
        return str(value)[:500]
    if isinstance(value, dict):
        return {str(k): ('[REDACTED]' if any(s in str(k).lower() for s in
                ('password', 'secret', 'access_token', 'authorization')) else clean(v, depth+1))
                for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v, depth+1) for v in value[:100]]
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        import math
        return round(value, 6) if math.isfinite(value) else str(value)
    return str(value)[:500]


def decision_text(event):
    compact=lambda v: json.dumps(v, ensure_ascii=False, separators=(',', ':'))
    return (f"#{event['id']} [{event['backend']}/{event['source']}] "
            f"상황: {compact(event['situation'])} → 근거: {compact(event['evidence'])} "
            f"→ 결론: {event['conclusion']} → 동작: {event['action']}")


class DecisionJournal:
    def __deepcopy__(self, memo):
        # Map undo/redo copies model state; a session writer keeps its identity.
        return self

    def __init__(self, folder=None, repeat_seconds=5., max_bytes=8*1024*1024, backups=20):
        self.session=uuid.uuid4().hex
        self.context={'backend': 'SIM', 'mission': None, 'loop': None, 'step': None}
        self.report=None; self.repeat_seconds=repeat_seconds
        self.records=deque(maxlen=2000); self.pending=queue.SimpleQueue()
        self._writes=queue.SimpleQueue(); self._lock=threading.RLock()
        self._states={}; self._sequence=0; self._closed=False; self.error=''; self.failed_writes=0
        self.folder=Path(folder) if folder is not None else None
        self._handlers=[]; self._thread=None
        if self.folder is not None:
            try:
                self.folder.mkdir(parents=True, exist_ok=True)
                for name in ('decisions.jsonl', 'decisions.txt'):
                    h=RotatingFileHandler(self.folder/name, maxBytes=max_bytes,
                                          backupCount=backups, encoding='utf-8')
                    self._handlers.append(h)
                self._thread=threading.Thread(target=self._writer, daemon=True, name='decision-log-writer')
                self._thread.start()
            except OSError as exc:
                self.error=str(exc)
                for h in self._handlers: h.close()
                self._handlers=[]

    def bind(self, backend, report=None, loop=None, step=None):
        with self._lock:
            self.report=report
            self.context=dict(backend=backend, mission=getattr(report, 'started', None), loop=loop, step=step)

    def emit(self, source, situation, evidence, conclusion, action, *, key=None, identity=None, force=False):
        situation,evidence=clean(situation),clean(evidence)
        with self._lock:
            if self._closed: return None
            now=time.monotonic()
            token=(self.context['backend'],self.context['mission'],self.context['loop'],self.context['step'],source,key or source)
            fingerprint=json.dumps([conclusion, action, clean(identity)], ensure_ascii=False, sort_keys=True)
            previous=self._states.get(token)
            if previous and previous['fingerprint']==fingerprint and not force:
                previous['repeated']+=1
                if now-previous['at']<self.repeat_seconds: return None
            self._sequence+=1
            context=dict(self.context)
            context['host_backend']=context['backend']
            if source.startswith(('SIM.', 'REAL.')):context['backend']=source.split('.')[0]
            event=dict(format='moma-decision-v1', session=self.session, id=self._sequence,
                time=datetime.now().astimezone().isoformat(timespec='milliseconds'),
                **context, source=source, situation=situation, evidence=evidence,
                conclusion=str(conclusion), action=str(action),
                repeated=previous['repeated'] if previous else 0)
            self._states[token]=dict(fingerprint=fingerprint, at=now, repeated=0)
            # Bound dedup memory even for long-running infinite missions.
            if len(self._states)>4096: self._states.pop(next(iter(self._states)))
            self.records.append(event); self.pending.put(event)
            if self.report is not None and self.report.ended is None:
                self.report.add_decision(event)
            if self._thread is not None: self._writes.put(event)
            return event

    def _writer(self):
        while True:
            event=self._writes.get()
            try:
                if event is None: return
                if isinstance(event, threading.Event): event.set(); continue
                for handler, content in zip(self._handlers, (
                    json.dumps(event, ensure_ascii=False, allow_nan=False),
                    event['time']+' | '+decision_text(event)+f" | 동일 판단 반복 {event['repeated']}회")):
                    # handleError normally hides IO failures; report them explicitly.
                    if handler.shouldRollover(logging.makeLogRecord({'msg': content})):
                        handler.doRollover()
                    handler.stream.write(content+'\n'); handler.flush()
            except Exception as exc:
                self.error=str(exc); self.failed_writes+=1

    def flush(self, timeout=3.):
        if self._thread is None: return not self.error
        marker=threading.Event(); self._writes.put(marker)
        return marker.wait(timeout) and not self.error

    def close(self):
        with self._lock:
            if self._closed: return
            self._closed=True
        if self._thread:
            self._writes.put(None); self._thread.join(3.)
        if self._thread is None or not self._thread.is_alive():
            for handler in self._handlers: handler.close()


def journal_for(obj):
    values=getattr(obj, '__dict__', {})
    journal=values.get('decision_journal')
    if isinstance(journal, DecisionJournal): return journal
    console=values.get('c')
    if console is not None:
        journal=getattr(console, '__dict__', {}).get('decision_journal')
        if isinstance(journal, DecisionJournal): return journal
    return None


def decide(obj, source, situation, evidence, conclusion, action, **kwargs):
    journal=journal_for(obj)
    if journal is not None:
        return journal.emit(source, situation, evidence, conclusion, action, **kwargs)


def snapshot(obj):
    values=getattr(obj, '__dict__', {})
    console=values.get('c', obj)
    state=values.get('state')
    if isinstance(state, str): return dict(state=state, step=values.get('index', 0)+1, elapsed=values.get('elapsed'))
    if state is not None and hasattr(state, '__dict__'): return dict(state.__dict__)
    fn=getattr(type(console), 'current_state', None)
    if fn:
        try: return clean(fn(console))
        except Exception: pass
    return dict(status=values.get('status'), step=values.get('index', 0)+1)


def audited(source, action=None):
    """Record validation failures at command boundaries without changing exceptions."""
    def decorate(fn):
        @wraps(fn)
        def wrapped(self, *args, **kwargs):
            try:
                result=fn(self, *args, **kwargs)
            except Exception as exc:
                decide(self, source, snapshot(self), dict(operation=fn.__name__, reason=str(exc)),
                       '검증/안전 조건 불충족', '요청 중단 · '+str(exc), force=True)
                raise
            if action:
                decide(self, source, snapshot(self), dict(operation=fn.__name__, arguments=clean(args), keyword_arguments=clean(kwargs)),
                       '검증 통과', action, force=True)
            return result
        return wrapped
    return decorate
