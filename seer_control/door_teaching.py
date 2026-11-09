"""Controller-native teaching matrices exported read-only from the actual SDK."""
import json
import math
from pathlib import Path
from functools import lru_cache
from .geometry3d import multiply,transform,point

@lru_cache(maxsize=1)
def reference():
    data=json.loads((Path(__file__).resolve().parents[1]/'config/door_teaching_reference.json').read_text(encoding='utf-8'))
    if data.get('format')!='moma-door-teaching-v1':raise ValueError('후크 티칭 자료 형식 오류')
    from .marker_placement import rigid
    for key in ('hook_tool','marker_handle','marker_hinge','marker_grasp_tcp'):rigid(data[key])
    return data


def hook_contact_offset():
    # FR5 GLB J6 origin is 100 mm behind the SDK flange. Verified against the
    # live SDK TCP with the measured six joints, not inferred from CAD colour.
    return point(multiply(transform((0,0,.1)),reference()['hook_tool']),(0,0,0))


def handle_center(part):
    pts=[p for face,mat in zip(part['asset'].faces,part['asset'].face_materials) if mat==36 for p in face]
    if not pts:raise ValueError('왼쪽 손잡이 내부 형상을 찾을 수 없습니다.')
    return tuple((min(p[i] for p in pts)+max(p[i] for p in pts))/2 for i in range(3))


def handle_contact_anchor(part,scale=1.):
    """TCP anchor ahead of the solid handle front face, in CAD coordinates.

    The SDK TCP is inside the hook end geometry; placing it at the handle's
    volume centre puts both solids through each other. Keep X/Z centred,
    and separate the hook envelope from the handle front face along CAD Y.
    Distances are metres, independent of the asset scale.
    """
    if not math.isfinite(scale) or scale<=0:raise ValueError('CAD 배율 오류')
    center=handle_center(part)
    front=min(p[1] for face,mat in zip(part['asset'].faces,part['asset'].face_materials) if mat==36 for p in face)
    cfg=reference().get('cad_handle_contact',{})
    gap=float(cfg.get('hook_normal_extent_m',0.))+float(cfg.get('clearance_m',0.))
    if not math.isfinite(gap) or not 0<=gap<=.03:raise ValueError('후크 손잡이 깊이 보정 범위 오류')
    return (center[0],front-gap/scale,center[2])


def place_handle(marker,part,origin,scale=1.):
    from .marker_placement import rigid,solve_anchor
    rigid(marker)
    # Reuse the verified marker horizontal/vertical axis checks.
    solve_anchor(marker,(0,0,0),origin,scale)
    center=handle_contact_anchor(part,scale)
    target=multiply(marker,reference()['marker_grasp_tcp'])
    yaw=math.atan2(marker[1][0],marker[0][0]);c,s=math.cos(yaw),math.sin(yaw)
    x,y,z=(v*scale for v in center)
    return [target[0][3]-c*x+s*y,target[1][3]-s*x-c*y,target[2][3]-z,0.,0.,yaw]


def apply_pending_profile(config,folder=None):
    """Consume the user-requested display alignment once on program restart.

    The old running program saves its in-memory profile when closing, so a
    pending file keeps the calculated candidate from being overwritten there.
    """
    if folder is None:
        from .app import USER_DIR
        folder=USER_DIR
    path=Path(folder)/'pending_hook_alignment.json'
    if not path.exists():return config
    data=json.loads(path.read_text(encoding='utf-8'))
    if data.get('format')!='moma-pending-hook-alignment-v1':raise ValueError('후크 배치 대기 자료 형식 오류')
    if config.get('mount')!=data['expected_mount'] or config.get('assets')!=data['expected_assets']:raise ValueError('장착/모델 변경으로 후크 배치 자동 적용을 보류합니다.')
    patch=data['patch'];origin=patch['cad_origin']
    if len(origin)!=6 or not all(type(v) in (int,float) and math.isfinite(v) for v in origin):raise ValueError('후크 배치 좌표 오류')
    if patch['marker_placement']['teaching_sha256']!=reference()['source_sha256']:raise ValueError('후크 티칭 자료 변경으로 배치 적용 보류')
    result=dict(config,**patch)
    path.replace(path.with_name('applied_hook_alignment.json'))
    return result


def apply_taught_hinge(view):
    """Correct the visual left hinge from the controller's taught geometry.

    Keep the grasp-centred equipment pose fixed. Never moves the real robot.
    """
    if not getattr(getattr(view,'cad_asset',None),'name','').upper().startswith('SIMPLE_WHOLE'):return False
    delta=reference().get('cad_left_hinge_from_handle')
    if not isinstance(delta,list) or len(delta)!=3:return False
    scale=view.scales['cad']
    if not math.isfinite(scale) or scale<=0:return False
    changed=False
    for part in view.cad_asset.door_parts:
        if not part['name'].endswith(':1'):continue
        center=handle_contact_anchor(part,scale);original=part.setdefault('authored_pivot',part['pivot'])
        pivot=(center[0]+delta[0]/scale,center[1]+delta[1]/scale,original[2])
        if math.dist(pivot[:2],original[:2])*scale>.05:raise ValueError('CAD 힌지와 티칭 축 차이가 5cm를 초과합니다. 모델을 확인하세요.')
        if tuple(part['pivot'])!=pivot:part['pivot']=pivot;changed=True
        for lod in getattr(view.cad_asset,'lod_door_parts',()):
            if lod['name']==part['name'] and tuple(lod['pivot'])!=pivot:
                lod['pivot']=pivot;changed=True
    return changed
