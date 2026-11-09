"""Read-only DO7 phase gate. No IO writes, robot commands or robot-side door actuation."""
import math
from time import time_ns

def twist_delta(rotation,old,sign):
    r=[[sum(rotation[i][k]*old[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    return math.atan2(r[1][0]-r[0][1],r[0][0]+r[1][1])*sign


def motion_profile(data,roller_start_time=None):
    if data.get('complete') is not True:raise ValueError('정상 완료 전체 녹화가 필요합니다.')
    segments=[];rows=[];last=False
    for frame in data.get('frames',[]):
        value=frame.get('io',{}).get('7')
        if type(value) is not bool:raise ValueError('DO7 누락 기록은 각도 보정에 사용할 수 없습니다.')
        if value:rows.append(frame)
        if last and not value:segments.append(rows);rows=[]
        last=value
    if len(segments)!=3 or last:raise ValueError('후크/롤러/닫기 세 ON/OFF 구간이 필요합니다.')
    spans=[]
    for index,segment in enumerate(segments[:2]):
        if index==1 and roller_start_time is not None:segment=[f for f in segment if f['t']>=roller_start_time]
        if len(segment)<3:raise ValueError('DO7 구간 자세 표본 부족')
        progress=peak=0.
        for before,after in zip(segment,segment[1:]):
            progress+=twist_delta(after['j6_rotation'],before['j6_rotation'],-1)
            peak=max(peak,progress)
        spans.append(math.degrees(peak))
    if not 40<=spans[0]<=80 or not 15<=spans[1]<=180:raise ValueError('ON 구간의 실제 회전량이 예상 범위를 벗어났습니다.')
    closing=peak=0.
    for before,after in zip(segments[2],segments[2][1:]):
        closing-=twist_delta(after['j6_rotation'],before['j6_rotation'],-1)
        peak=max(peak,closing)
    closing_motion=math.degrees(peak)
    if not 30<=closing_motion<=180:raise ValueError('닫기 구간 회전량이 예상 범위를 벗어났습니다.')
    return dict(hook_motion=spans[0],roller_motion=spans[1],closing_motion=closing_motion,closing_speed=closing_motion/max(.01,segments[2][-1]['t']-segments[2][0]['t']),scope=data.get('scope'),roller_contact_gated=roller_start_time is not None,roller_start_time=roller_start_time,format='moma-do7-angle-profile-v2')


def roller_push_start(follower,name,previous,roller,angle):
    if previous is None or roller is None:return False
    near=follower._nearest(name,roller,angle,follower.roller_radius+.015)
    if near is None:return False
    old=follower._nearest(name,previous,angle,.20)
    return old is not None and near[0]<old[0]-.0005


def contact_start_time(template,frames,origin,hook_angle=60.):
    from .arm_physics import inverse
    from .geometry3d import transform,point
    world_to_cad=inverse(transform(origin[:3],origin[3:]))
    name=next(n for n in template.angles if n.endswith(':1'))
    pulse=0;last=False;previous=None
    for frame in frames:
        on=frame.get('io',{}).get('7') is True
        if on and not last:pulse+=1;previous=None
        if pulse==2 and on:
            roller=point(world_to_cad,frame['roller'])
            if frame.get('door_tool')!='hook' and roller_push_start(template,name,previous,roller,math.radians(hook_angle)):return frame['t']
            previous=roller
        last=on
    return None


def contact_profile(data,view):
    from .door_geometry import door_definitions
    from .door_feedback import SurfaceDoorFollower
    from .door_teaching import place_handle
    origin=list(data['origin'])
    if data.get('markers'):
        part=next(p for p in view.cad_asset.door_parts if p['name'].endswith(':1'))
        origin=place_handle(data['markers'][-1]['matrix'],part,origin,view.scales['cad'])
    template=SurfaceDoorFollower(door_definitions(view),roller_radius=data.get('roller_radius',.025))
    start=contact_start_time(template,data['frames'],origin,motion_profile(data)['hook_motion'])
    if start is None:raise ValueError('녹화에서 롤러 접근 후 첫 밀기 접촉을 확인하지 못했습니다. 설비 배치를 확인하세요.')
    profile=motion_profile(data,start)
    profile['roller_target']=min(160.,profile['hook_motion']+profile['roller_motion'])
    profile['measured_rotation']=True
    profile['format']='moma-do7-angle-profile-v3'
    return profile


class DoorIO7:
    def __init__(self,hook_motion=60.,roller_motion=100.,roller_target=120.,closing_motion=150.,measured_rotation=False,closing_speed=40.):
        self.measured_rotation=bool(measured_rotation);self.closing_speed=max(1.,float(closing_speed))
        self.roller_target=max(60.,min(160.,float(roller_target)))
        self.phase='waiting';self.engaged=False;self.rotation=None;self.stamp=None;self.progress=0.;self.start=0.;self.wait_roller=None;self.wait_stamp=None
        self.hook_motion=math.radians(hook_motion);self.roller_motion=math.radians(roller_motion)
        self.closing_motion=math.radians(max(1.,float(closing_motion)))
        self.close_elapsed=0.;self.close_from=0.;self.close_duration=1.;self.close_stamp=None
        self.status=f'DO7 세 구간 대기 · 후크 0~60 / 롤러 60~{self.roller_target:g} / 닫기'
    def release(self,follower):
        self.engaged=False;self.rotation=None;self.stamp=None;self.wait_roller=None;self.wait_stamp=None;self.roller_position=None;self.roller_time=None
        follower.detach();follower._roller_previous=None;follower._roller_stamp=None
    def roller_plane_target(self,follower,name,roller,closing=False):
        door=follower.doors[name];plane=door.get('roller_plane_y')
        if roller is None or plane is None:return None
        p=door['pivot'];x,y=roller[0]-p[0],roller[1]-p[1];radius=math.hypot(x,y)
        if not .05<radius<.45:return None
        offset=plane-p[1]+(-1 if closing else 1)*follower.roller_radius
        if abs(offset)>=radius:return None
        phi=math.atan2(y,x);beta=math.asin(offset/radius);choices=[]
        for branch in (beta,math.pi-beta):
            for turns in (-1,0,1):
                a=(phi-branch+turns*2*math.pi)*door['sign']
                if 0<=a<=math.radians(170):choices.append(a)
        return min(choices,key=lambda a:abs(a-follower.angles[name])) if choices else None

    def follow_roller_surface(self,follower,name,roller,now,closing=False):
        """Roller centre around the hinge, bounded by actual travel, not wrist twist.

        Fit the finite CAD surface within that travel budget; never jump to a
        distant contact branch or force an endpoint angle.
        """
        if roller is None or not all(math.isfinite(v) for v in roller):
            self.status='롤러 위치 미수신 · 문 각도 유지';return
        previous=getattr(self,'roller_position',None);stamp=getattr(self,'roller_time',None)
        self.roller_position=tuple(roller);self.roller_time=now
        if previous is None or stamp is None or not 0<now-stamp<=.5:return
        p=follower.doors[name]['pivot'];a=(previous[0]-p[0],previous[1]-p[1]);b=(roller[0]-p[0],roller[1]-p[1])
        if min(math.hypot(*a),math.hypot(*b))<.05:return
        delta=math.atan2(a[0]*b[1]-a[1]*b[0],a[0]*b[0]+a[1]*b[1])*follower.doors[name]['sign']
        if (closing and delta>=0) or (not closing and delta<=0):return
        # Reject telemetry jumps rather than letting them fling the door.
        if abs(delta)>math.radians(15):return
        if closing and follower.doors[name].get('roller_plane_y') is not None:
            plane=follower.doors[name]['roller_plane_y']-p[1]-follower.roller_radius
            radius=math.hypot(*b)
            if abs(plane)>=radius:return
            # The front contact plane is tangent to the roller circle. Solve
            # absolute pose each frame; never integrate a clipped speed budget.
            phi=math.atan2(b[1],b[0]);offset=math.asin(plane/radius)
            choices=[]
            for branch in (offset,math.pi-offset):
                for turns in (-1,0,1):
                    angle=(phi-branch+turns*2*math.pi)*follower.doors[name]['sign']
                    if -1e-8<=angle<=math.radians(170):choices.append(max(0.,angle))
            if not choices:return
            before=follower.angles[name]
            angle=min(choices,key=lambda a:abs(a-before))
            if angle>before+1e-8:return  # Roller has not reached the open leaf.
            follower.angles[name]=angle;follower.active=name;follower.contact_mode='close-angle'
            self.status='닫기 · 롤러 앞면 접선 각도 직접 동기화';return
        if not closing and follower.doors[name].get('roller_plane_y') is not None:
            target=self.roller_plane_target(follower,name,roller)
            if target is None:return
            before=follower.angles[name]
            # Advance only by the current roller travel; no distant branch snap.
            follower.angles[name]=min(math.radians(self.roller_target),before+max(0.,min(abs(delta),target-before)))
            follower.active=name;follower.contact_mode='open-angle'
            self.status='롤러 앞면 도달 후 연속 추가 열기';return
        budget=abs(delta)
        before=follower.angles[name]
        target=max(0.,before-budget) if closing else min(math.radians(self.roller_target),before+budget)
        candidates=[before+(target-before)*i/6 for i in range(7)]
        best=None
        for angle in candidates:
            near=follower._nearest(name,roller,angle,follower.roller_radius+.08)
            if near is None:continue
            error=abs(near[0]-follower.roller_radius)
            # Prefer advancing with the roller when surface errors are equal.
            score=(round(error,7),abs(angle-target))
            if best is None or score<best[0]:best=(score,angle)
        follower.angles[name]=best[1] if best else target
        follower.active=name;follower.contact_mode='close-angle' if closing else 'open-angle'
        self.status='롤러 표면 밀착 '+('닫기' if closing else '열기')+' · 실제 이동량 제한'

    def update(self,follower,hook,roller,rotation,now,on,tool):
        self._update(follower,hook,roller,rotation,now,on,tool)
        if getattr(self,'audit_folder',None) is None:return
        try:self._record(follower,hook,roller,rotation,now,on,tool)
        except (OSError,ValueError,TypeError) as error:self.audit_error=str(error)
    def _record(self,follower,hook,roller,rotation,now,on,tool):
        import json
        from pathlib import Path
        if on is True and getattr(self,'audit_rows',None) is None:
            self.audit_rows=[];self.audit_start=now;self.audit_previous=None;self.audit_sequence=time_ns()
        rows=getattr(self,'audit_rows',None)
        if rows is None:return
        edge=on is not self.audit_previous
        if len(rows)<20000 and (not rows or now-self.audit_start-rows[-1]['t']>=.03 or edge):
            rows.append(dict(t=now-self.audit_start,io={'7':on},door_tool=tool,phase=self.phase,
                hook=hook,roller=roller,j6_rotation=rotation,angles=dict(follower.angles),status=self.status))
        self.audit_previous=on
        complete=self.phase=='waiting' and on is False
        if edge or complete:
            folder=Path(self.audit_folder);folder.mkdir(parents=True,exist_ok=True)
            data=dict(format='moma-door-gate-cycle-v1',complete=complete,frames=rows,
                hook_motion=math.degrees(self.hook_motion),roller_motion=math.degrees(self.roller_motion))
            text=json.dumps(data,separators=(',',':'));temp=folder/'latest.tmp';temp.write_text(text,encoding='utf-8');temp.replace(folder/'latest.json')
            if complete:
                (folder/(str(self.audit_sequence)+'.json')).write_text(text,encoding='utf-8');self.audit_rows=None
    def _update(self,follower,hook,roller,rotation,now,on,tool):
        left=next((n for n in follower.angles if n.endswith(':1')),None)
        if left is None:return
        if type(on) is not bool:
            self.close_stamp=now;self.status='DO7 미수신 · 각도 및 구간 기준 유지';return
        if self.phase=='auto_close':
            if on:self.phase='closing';self.release(follower)
            else:
                dt=max(0.,min(.1,now-self.close_stamp));self.close_stamp=now;self.close_elapsed+=dt
                t=min(1.,self.close_elapsed/self.close_duration);ease=t*t*(3-2*t)
                follower.angles[left]=self.close_from*(1-ease)
                self.status='닫기 OFF · 잔여 각도 부드럽게 닫기'
                if t>=1.:
                    self.phase='waiting';follower.angles[left]=0.;follower._roller_gate.pop(left,None)
                return
        if not on:
            if self.phase=='hook':
                if not self.measured_rotation and math.radians(58)<=follower.angles[left]<=math.radians(60):follower.angles[left]=math.radians(60)
                self.phase='handoff';self.release(follower)
            elif self.phase=='roller':
                if not self.measured_rotation and math.radians(self.roller_target-2)<=follower.angles[left]<=math.radians(self.roller_target):follower.angles[left]=math.radians(self.roller_target)
                self.phase='open_hold';self.release(follower)
            elif self.phase=='closing':
                self.release(follower);self.phase='auto_close';self.close_from=follower.angles[left]
                self.close_duration=(max(.6,1.5*math.degrees(self.close_from)/self.closing_speed) if self.measured_rotation else max(.6,min(3.,math.degrees(self.close_from)/40.)));self.close_elapsed=0.;self.close_stamp=now
            self.status='DO7 OFF · 열린 각도 유지' if self.phase!='auto_close' else '닫기 OFF · 자동 마무리 시작'
            return
        # The three DO7 pulses define the phases, independently of Tool metadata.
        if self.phase in ('waiting','handoff','open_hold'):
            phase={'waiting':'hook','handoff':'roller','open_hold':'closing'}[self.phase]
            self.release(follower);self.phase=phase
        if self.phase=='closing' and self.measured_rotation and roller is not None:
            self.engaged=True
            self.follow_roller_surface(follower,left,roller,now,closing=True);return
        if self.phase=='closing':
            # The third pulse explicitly marks the taught roller closing move.
            # Keep its rotation anchor throughout ON instead of losing CAD
            # contact and waiting until OFF to close the remaining door.
            if rotation is None:
                self.status='닫기 ON · 실제 자세 대기 / 각도 유지';return
            if not self.engaged:
                self.engaged=True;self.rotation=rotation;self.start=follower.angles[left];self.progress=0.;self.stamp=now
            else:
                delta=-twist_delta(rotation,self.rotation,follower.doors[left]['sign'])
                if not math.isfinite(delta):self.status='닫기 회전값 대기';return
                self.progress+=delta;self.rotation=rotation;self.stamp=now
                value=max(0.,self.start-max(0.,self.progress)) if self.measured_rotation else self.start*(1-max(0.,min(1.,self.progress/self.closing_motion)))
                follower.angles[left]=max(0.,min(follower.angles[left],value))
            follower.active=left;follower.contact_mode='close-angle'
            self.status='닫기 DO7 ON · 롤러 회전 연속 추종 / OFF 후 잔여 마무리';return
        if rotation is None:
            self.status='DO7 ON · 실제 자세 대기 / 구간 유지';return
        if not self.engaged and self.phase=='roller':
            previous=self.wait_roller
            if self.wait_stamp is None or not 0<now-self.wait_stamp<=.5:previous=None
            self.wait_roller=roller;self.wait_stamp=now
            plane_target=self.roller_plane_target(follower,left,roller)
            plane_start=False
            if previous is not None and plane_target is not None:
                old_target=self.roller_plane_target(follower,left,previous)
                plane_start=(old_target is not None and plane_target>old_target+1e-5 and plane_target>=follower.angles[left] and plane_target-follower.angles[left]<math.radians(15))
            if tool=='hook' or not (plane_start or roller_push_start(follower,left,previous,roller,follower.angles[left])):
                self.status='두 번째 DO7 ON · 롤러 접근 대기 / 문 각도 유지';return
        if not self.engaged:
            # First contact starts the roller segment; subsequent ON samples
            # stay latched and do not re-test proximity.
            # ON is the operator's explicit engagement signal. Do not re-test
            # CAD contact while the hook/roller is engaged in this segment.
            self.engaged=True;self.rotation=rotation;self.stamp=now;self.progress=0.
            self.start=follower.angles[left]
            self.hook_position=tuple(hook) if hook is not None else None
            self.roller_position=tuple(roller) if roller is not None else None;self.roller_time=now
            if not self.measured_rotation and self.phase=='roller' and self.start>math.radians(50):
                self.start=math.radians(60);follower.angles[left]=self.start
        elif self.measured_rotation and self.phase=='roller' and roller is not None:
            self.follow_roller_surface(follower,left,roller,now);return
        else:
            delta=twist_delta(rotation,self.rotation,follower.doors[left]['sign'])
            if self.measured_rotation and self.phase=='hook' and hook is not None:
                old=getattr(self,'hook_position',None);self.hook_position=tuple(hook)
                if old is not None:
                    pivot=follower.doors[left]['pivot'];a=(old[0]-pivot[0],old[1]-pivot[1]);b=(hook[0]-pivot[0],hook[1]-pivot[1])
                    delta=math.atan2(a[0]*b[1]-a[1]*b[0],a[0]*b[0]+a[1]*b[1])*follower.doors[left]['sign']
                    if abs(delta)>math.radians(15):return
            if not math.isfinite(delta):self.status='유효한 회전값 대기';return
            # Accumulate signed motion; small reversals do not release or
            # repeatedly add positive jitter. Telemetry gaps keep the anchor.
            self.progress+=delta;self.rotation=rotation;self.stamp=now
            maximum=(self.hook_motion if self.measured_rotation else math.radians(60)) if self.phase=='hook' else math.radians(self.roller_target)
            follower.max_angle=max(follower.max_angle,math.radians(self.roller_target))
            span=self.hook_motion if self.phase=='hook' else self.roller_motion
            available=max(0.,maximum-self.start)
            value=min(maximum,self.start+max(0.,self.progress)) if self.measured_rotation else self.start+available*max(0.,min(1.,self.progress/max(1e-9,span)))
            follower.angles[left]=min(follower.max_angle,max(follower.angles[left],value))
        follower.active=left;follower.contact_mode='hook' if self.phase=='hook' else 'open-angle'
        self.status=('DO7 ON · 실제 회전 1:1 추종 · 추가 각도 확대 없음' if self.measured_rotation else 'DO7 ON · '+('후크 고정 추종 0~60°' if self.phase=='hook' else f'롤러 고정 추종 60~{self.roller_target:g}°')+' · 접촉 해제 안 함')


def save_motion_profile(app):
    import json
    from .app import USER_DIR
    from .marker_placement import scope
    view=app.world3d;controller=getattr(view,'_door_io7',None)
    if controller and controller.phase!='waiting':raise ValueError('문 동작이 끝난 뒤 각도 기준을 갱신하세요.')
    data=json.loads((USER_DIR/'door_trajectory_sessions/latest.json').read_text(encoding='utf-8'))
    profile=contact_profile(data,view)
    if profile.get('scope',{}).get('base')!=scope(app,view):raise ValueError('현재 지도/호스트/장착/CAD와 녹화 기준이 다릅니다.')
    path=USER_DIR/'do7_angle_profile.json';temp=path.with_suffix('.tmp')
    temp.write_text(json.dumps(profile,ensure_ascii=False,indent=2),encoding='utf-8');temp.replace(path)
    view._door_io7=None
    view.marker_placement_status=f"DO7 실측 회전 기준 · 후크 {profile['hook_motion']:.2f}° + 롤러 {profile['roller_motion']:.2f}° · 최대 {profile['roller_target']:.2f}°"
    return profile


def configured_controller(view):
    import json
    try:
        if not view.app.real:
            data=getattr(view,'sim_do7_profile',{})
            return DoorIO7(data.get('hook_motion',60.),data.get('roller_motion',100.) if data.get('roller_contact_gated') else 100.,data.get('roller_target',160.),data.get('closing_motion',150.),measured_rotation=True,closing_speed=data.get('closing_speed',40.))
        from .app import USER_DIR
        from .marker_placement import scope
        data=json.loads((USER_DIR/'do7_angle_profile.json').read_text(encoding='utf-8'))
        if data.get('scope',{}).get('base')!=scope(view.app,view):return DoorIO7(roller_target=160.,measured_rotation=True)
        controller=DoorIO7(data['hook_motion'],data['roller_motion'] if data.get('roller_contact_gated') else 100.,data.get('roller_target',160.),data.get('closing_motion',150.),measured_rotation=True,closing_speed=data.get('closing_speed',40.))
        controller.audit_folder=USER_DIR/'do7_cycles'
        return controller
    except (OSError,ValueError,KeyError,AttributeError):return DoorIO7(roller_target=160.,measured_rotation=True)


def read_opening_feedback(app,now):
    """Prefer an already-connected manufacturer SDK. Never opens a connection."""
    runtime=getattr(app,'runtime_arm_feedback',None)
    from .fr5_readonly_feedback import read
    direct=read(app,now)
    if direct is not None:
        value=direct.get('io',{}).get('DO',{}).get('7')
        if type(value) is bool:
            kind={1:'hook',2:'roller'}.get(direct.get('tool_index'))
            if kind is None and runtime and hasattr(runtime,'tool_kind'):kind=runtime.tool_kind()
            app.door_io7_source='공식 FR5 SDK 직접 수신 · 읽기 전용'
            return value,kind
    client=getattr(app,'fr5_client',None)
    if client and client.connected and now-getattr(app,'fr5_rx',0)<=.8:
        data=getattr(app,'fr5_feedback',{})
        value=data.get('io',{}).get('DO',{}).get('7')
        if data.get('driver')=='fairino' and type(value) is bool:
            kind={1:'hook',2:'roller'}.get(data.get('tool_index'))
            if kind is None:kind=runtime.tool_kind() if runtime and hasattr(runtime,'tool_kind') else None
            # Direct tool index if the SDK profile provides one; do not guess
            # from a motion program's name or a stale arm pose.
            if kind is None:kind={1:'hook',2:'roller'}.get(data.get('tool_index'))
            app.door_io7_source='로봇팔 제어기 SDK 직접 수신'
            return value,kind
    app.door_io7_source='Runtime 중계 수신'
    return runtime.opening_feedback() if runtime and hasattr(runtime,'opening_feedback') else (None,None)
