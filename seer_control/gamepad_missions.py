"""Explicit DualSense mission buttons; neutral/fresh input and one press only."""
import math
from .arm_gamepad import ButtonEdges

class MissionButtons:
    def __init__(self):self.edges=ButtonEdges()
    def update(self,sample,now,allowed,enabled,zone=.15,forward='Y',turn='X'):
        edges=self.edges.update(sample,now,allowed)
        pressed=edges&3  # WinMM: square=button 1, cross=button 2
        if not pressed:return None,''
        if not enabled:return None,'스틱 중앙에서 L1으로 AMR 조종을 활성화한 뒤 실행하세요.'
        if pressed==3:return None,'×와 □ 중 한 버튼만 누르세요.'
        if any(k not in sample.axes or abs(sample.axes[k])>zone for k in (forward,turn)):
            return None,'스틱을 중앙에 놓은 뒤 연동/복귀 버튼을 누르세요.'
        return ('link' if pressed==2 else 'return'),''


def execute(app,action):
    if action not in ('link','return'):raise ValueError('지원하지 않는 리모컨 동작')
    state=app.current_state()
    speed=state.get('speed')
    if type(speed) not in (int,float) or not math.isfinite(speed) or abs(speed)>.01:raise ValueError('AMR이 정지한 뒤 실행하세요.')
    if app.task_running or getattr(app.studio_runner,'active',False):raise ValueError('진행 중인 작업을 먼저 종료하세요.')
    link=getattr(app,'_departure_link',None);thread=getattr(link,'thread',None)
    if (thread and thread.is_alive()) or getattr(link,'active',False):raise ValueError('LM2→LM1 연동이 이미 실행 중입니다.')
    manager=getattr(app,'_runtime_scenario',None)
    if manager and (manager.uncertain or manager.future and not manager.future.done()):raise ValueError('로봇팔 시나리오 상태를 먼저 확인하세요.')
    expected='LM2' if action=='link' else 'LM1'
    station=state.get('current_station',state.get('last_node'))
    if app.real:
        if not app.connected or not app.control_enabled:raise ValueError('실기 · 제어 모드로 연결하세요.')
        raw=getattr(app,'raw',{})
        for row in [raw]+[v for v in raw.values() if isinstance(v,dict)]:
            if 'current_station' in row:station=row['current_station'];break
    if station!=expected:raise ValueError(expected+' 정지 상태에서만 실행할 수 있습니다. 현재 위치를 확인하세요.')
    if not app.studio_arm_safe:raise ValueError('팔 안전 자세 확인이 필요합니다.')
    if action=='return' and app.real and 'LM2' not in app.robot_stations:raise ValueError('로봇 Stations에서 LM2를 조회하세요.')
    app._pad_stop();app.release_drive();app.pad_enabled.set(False);app.pad_gate.reset();app._pad_mission_stop=True;app._pad_mission_authorized=True
    app.log('GAMEPAD','× LM2→LM1 · main2→main3 실행' if action=='link' else '□ LM1→LM2 이동 요청')
    if action=='link':
        from .departure_link import show_departure_link
        show_departure_link(app,autostart=True)
    else:
        app.target.set('LM2');app.navigate()
