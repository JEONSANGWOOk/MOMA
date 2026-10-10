"""Explicit SIM adapter: a recognized photograph never counts as ArUco markers."""
import copy


def photo_record(camera_record,revision):
    if not camera_record:return None
    raw=copy.deepcopy(camera_record.get('photo_target') or dict(valid=False,reason='화면 사진 미검출'))
    board=copy.deepcopy(camera_record.get('photo_control_target') or raw)
    if board.get('revision')!=revision:board=dict(valid=False,reason='사진 등록 적용 대기')
    board.update(revision=revision,markers_used=0,geometry_source='photo_screen_estimate')
    raw_valid=raw.get('valid',False) and raw.get('revision')==revision
    raw.update(valid=raw_valid,revision=revision,markers_used=0,geometry_source='photo_screen_estimate')
    return dict(timestamp=camera_record['timestamp'],vision_source='photo_screen_sim',board=board,raw_board=raw)
