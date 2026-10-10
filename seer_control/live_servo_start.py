"""An explicit live-only SIM start cannot consume the model preview camera."""
import time


class LiveServoStart:
    def __init__(self):self.pending=False
    def request(self,mode):
        if mode!='SIM':raise ValueError('D455 추종 시험은 SIM 모드에서 시작하세요. 실기는 선택 모드 작업 시작을 사용하세요.')
        self.pending=True
    def cancel(self):self.pending=False
    def tick(self,record,ready,start,now=None):
        if not self.pending or not ready or not isinstance(record,dict):return False
        now=time.time() if now is None else now;board=record.get('board') or {}
        photo=record.get('vision_source')=='photo_screen_sim' and board.get('geometry_source')=='photo_screen_estimate' and board.get('markers_used')==0
        if record.get('vision_source')=='built_in_demo' or not -.1<=now-record.get('timestamp',0)<=.25 or not board.get('valid') or (board.get('markers_used')!=2 and not photo):return False
        self.pending=False;start();return True
