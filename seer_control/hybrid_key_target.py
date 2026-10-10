"""Conservative SIM fusion gate; marker pose supplies the surface orientation."""
import math


class HybridKeyTarget:
    def __init__(self):self.revision=None;self.offset=None;self.status='구멍 등록 대기';self.accepted_stamp=None
    def reset(self):self.revision=None;self.offset=None;self.accepted_stamp=None;self.status='구멍 등록 대기'
    def update(self,record,board_revision,now):
        h=(record or {}).get('handle_target') or {}
        self.status=h.get('reason') or '구멍 미검출 · 삽입 보류'
        if not record or not -.1<=now-record.get('timestamp',0)<=.25:return False
        if not h.get('valid') or h.get('board_revision')!=board_revision or h.get('confidence',0)<.78:return False
        xyz=h.get('board_xyz_mm');revision=h.get('profile_revision')
        if not isinstance(xyz,list) or len(xyz)!=3 or any(type(v) not in (float,int) or not math.isfinite(v) for v in xyz) or not revision:return False
        if self.revision is not None and revision!=self.revision:self.status='구멍 등록 변경 · 정지 후 다시 시작하세요.';return False
        if self.offset is not None and math.dist(xyz[:2],self.offset) >2:self.status='구멍 중심 급변 · 삽입 보류';return False
        self.revision=revision;self.offset=list(xyz[:2]);self.accepted_stamp=record['timestamp'];self.status='마커 자세 + 실제 구멍 중심 추종';return True
