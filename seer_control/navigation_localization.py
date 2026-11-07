"""Controller localization before navigation; never replay motion commands."""
import math
import time


def checked(response):
    if not isinstance(response,dict) or response.get('ret_code',0)!=0:
        raise ValueError(f'위치 재추정 API 거부: {response}')
    return response


def pose(state):
    values=tuple(state.get(k) for k in ('x','y','angle'))
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
        raise ValueError('유효한 로봇 위치/방향 수신이 필요합니다.')
    return values


def validate_state(state,seed=None):
    pose_now=pose(state)
    for key in ('vx','vy','w'):
        value=state.get(key)
        if type(value) not in (int,float) or not math.isfinite(value) or abs(value)>.01:
            raise ValueError('위치 재추정은 로봇 정지 후 실행합니다.')
    if state.get('emergency') or state.get('blocked'):
        raise ValueError('비상정지/장애물 상태에서는 자동 출발하지 않습니다.')
    if state.get('task_status') in (2,3):
        raise ValueError('기존 주행 작업 완료 또는 취소 후 이동하세요.')
    if seed:
        delta=math.atan2(math.sin(pose_now[2]-seed[2]),math.cos(pose_now[2]-seed[2]))
        if math.dist(pose_now[:2],seed[:2])>.25 or abs(delta)>math.radians(15):
            raise ValueError('위치 재추정 보정량이 큽니다. 수동 정합 확인이 필요합니다.')
    return pose_now


def prepare_navigation(client,guard,progress=lambda message:None,timeout=20.,clock=time.monotonic,sleep=time.sleep):
    """2002 -> fresh 1021/1100 samples -> 2003 ACK -> post-check.

    Confidence is a controller quality indicator, not a physical accuracy guarantee.
    The current measured pose is the seed; never substitute a destination pose.
    """
    guard()
    map_name=checked(client.query(1300)).get('current_map')
    if not map_name:raise ValueError('현재 로봇 지도 이름 수신이 필요합니다.')
    initial_status=checked(client.query(1021)).get('reloc_status')
    if type(initial_status) is not int or initial_status not in (0,1,3):
        raise ValueError('위치 재추정 상태 API 미지원 또는 기존 재추정 진행 중')
    seed=validate_state(checked(client.query(1100)))
    progress('이동 전 실제 위치 재추정 · 2002 요청')
    guard()
    checked(client.command('relocate',dict(x=seed[0],y=seed[1],angle=seed[2])))
    started=clock();stable=0;stable_since=None;saw_relocating=False
    while clock()-started<timeout:
        guard()
        status=checked(client.query(1021)).get('reloc_status')
        state=checked(client.query(1100));validate_state(state,seed)
        confidence=state.get('confidence',state.get('localization_confidence'))
        good=(type(confidence) in (int,float) and math.isfinite(confidence) and .85<=confidence<=1.)
        if status==0:raise ValueError('로봇 위치 재추정 실패 · 출발 차단')
        if status==2:saw_relocating=True
        # Poll sooner without accepting ACK or a single quality sample as success.
        # A witnessed RELOCING transition allows a shorter settling period.
        settling=.3 if saw_relocating else .6
        eligible=status==1 and good and clock()-started>=settling
        if eligible:
            if stable_since is None:stable_since=clock()
            stable+=1
        else:
            stable=0;stable_since=None
        if stable>=3 and clock()-stable_since>=.2-1e-9:break
        sleep(.1)
    else:raise TimeoutError('위치 재추정 시간 초과 또는 정합 불충분 · 출발 차단')
    guard()
    if checked(client.query(1300)).get('current_map')!=map_name:
        raise ValueError('위치 재추정 중 지도 변경 · 출발 차단')
    checked(client.command('confirm_loc',{}))
    guard()
    if checked(client.query(1021)).get('reloc_status')!=1:
        raise ValueError('위치 확정 후 상태 확인 실패 · 출발 차단')
    final=checked(client.query(1100));validate_state(final,seed)
    confidence=final.get('confidence',final.get('localization_confidence'))
    if type(confidence) not in (int,float) or not math.isfinite(confidence) or not .85<=confidence<=1.:
        raise ValueError('위치 확정 후 신뢰도 저하 · 출발 차단')
    guard();progress('위치 재추정·정합 확인 완료 · 이동 허용')
    return dict(map_name=map_name,pose=pose(final),confidence=confidence)
