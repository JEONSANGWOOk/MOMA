"""Mission audit records, separate from mutable action status."""
from datetime import datetime
from collections import Counter
import json

class MissionReport:
    def __init__(self, repeat):
        self.started=datetime.now().astimezone().isoformat(timespec='seconds')
        self.ended=None; self.status='RUNNING'; self.repeat=repeat
        self.records=[]; self.seconds=0.; self.completed_loops=0; self.metadata={}
    def add(self,cycle,step,kind,target='',detail='',duration=None):
        self.records.append(dict(time=datetime.now().astimezone().isoformat(timespec='seconds'),
            elapsed=round(self.seconds,2),loop=cycle+1,step=step+1,kind=kind,
            target=str(target),detail=str(detail),duration=round(duration,2) if duration is not None else None))
    def finish(self,status):
        self.status=status
        if self.ended is None:self.ended=datetime.now().astimezone().isoformat(timespec='seconds')
    def data(self):
        counts=Counter(r['kind'] for r in self.records)
        return dict(started=self.started,ended=self.ended,status=self.status,repeat=self.repeat,
            completed_loops=self.completed_loops,seconds=round(self.seconds,2),metadata=self.metadata,
            success=counts['성공'],failure=counts['실패'],skipped=counts['패스'],
            omitted=counts['생략'],canceled=counts['취소'],records=self.records)
    def text(self):
        d=self.data()
        lines=['미션 수행 보고서',f"상태: {d['status']} | 완료 루프: {d['completed_loops']} | 수행 시간: {d['seconds']:.1f}초",
            f"작업 결과: 성공 {d['success']} / 실패 {d['failure']} / 패스 {d['skipped']} / 생략 {d['omitted']} / 취소 {d['canceled']}",
            '※ 횟수는 실행된 작업 단계 기준입니다. 패스는 도달 불가 목적지, 생략은 그 목적지의 후속 작업입니다.',
            '시작: '+self.started, '종료: '+str(self.ended or '수행 중')]
        if self.metadata:lines.insert(1,'실행 환경: '+json.dumps(self.metadata,ensure_ascii=False,sort_keys=True))
        for cycle in sorted(set(r['loop'] for r in self.records)):
            rows=[r for r in self.records if r['loop']==cycle]; c=Counter(r['kind'] for r in rows)
            lines += ['',f"{cycle}번째 루프 — 성공 {c['성공']} / 실패 {c['실패']} / 패스 {c['패스']} / 생략 {c['생략']}"]
            for r in rows:
                duration=f" ({r['duration']:.1f}초)" if r['duration'] is not None else ''
                lines.append(f"  {r['elapsed']:8.1f}초 | 단계 {r['step']} | {r['kind']} | {r['target']} | {r['detail']}{duration}")
        return '\n'.join(lines)
    def save(self,path):
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(self.data(),ensure_ascii=False,indent=2),encoding='utf-8')
        path.with_suffix('.txt').write_text(self.text(),encoding='utf-8')
