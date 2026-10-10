"""Host the shared camera/panel workspace in the existing MOMA notebook."""
import math,time
from types import SimpleNamespace
from tkinter import ttk


def register_workspace(app):
    page=ttk.Frame(app.tabs);app.vision_panel_page=page;app.vision_panel_workspace=None
    app.tabs.add(page,text='비전 · 판넬 작업');app._register_navigation(page,'비전 · 판넬 작업','◉')
    def open_page(event=None):
        if app.tabs.select()!=str(page) or app.vision_panel_workspace is not None:return
        from .key_panel_workspace import build_workspace
        flags=SimpleNamespace(demo=False,smoke=False,smoke_ui=False,smoke_compare=False)
        workspace=build_workspace(page,args=flags);app.vision_panel_workspace=workspace
        link=workspace.key_panel_link
        def connection_guard():
            if app.fr5_client.connected or app.fr5_pending or app.studio_runner.active:
                raise ValueError('기존 MOMA 팔 연결·요청·미션을 종료한 뒤 판넬 FR5를 연결하세요.')
        def motion_guard():
            connection_guard()
            if app.held or app.studio_runner.active or app.task_running:
                raise ValueError('MOMA 주행·미션·수동 조작 종료 후 판넬 작업을 시작하세요.')
            if app.real:
                state=app.current_state();speed=state.get('speed')
                if not app.connected or time.monotonic()-app.last_state>3 or type(speed) not in (float,int) or not math.isfinite(speed) or abs(speed)>.005:
                    raise ValueError('실제 AMR 최신 정지 상태 확인 필요')
        link.connection_guard=connection_guard;link.motion_guard=motion_guard
        last_status=[None]
        def status_changed(*unused):
            message=workspace.key_panel_status.get()
            if message!=last_status[0]:
                last_status[0]=message;app.log('INFO','비전·판넬: '+message)
        workspace.key_panel_status.trace_add('write',status_changed)
        app.log('INFO','ArUco 영상 · 판넬 SIM/REAL 작업을 MOMA 내부 화면에서 시작했습니다.')
    app.tabs.bind('<<NotebookTabChanged>>',open_page,add='+')


def panel_link(app):
    workspace=getattr(app,'vision_panel_workspace',None)
    return workspace.key_panel_link if workspace is not None else None


def ensure_arm_available(app):
    link=panel_link(app)
    if link and (link.client.connected or link.busy or link.plan):
        raise ValueError('비전·판넬 FR5 연결을 해제한 뒤 기존 로봇팔 제어를 사용하세요.')


def stop_panel(app):
    workspace=getattr(app,'vision_panel_workspace',None)
    if workspace is not None:workspace.key_panel_stop()


def close_panel(app):
    workspace=getattr(app,'vision_panel_workspace',None)
    if workspace is None:return True
    if workspace.key_panel_close():app.vision_panel_workspace=None;return True
    return False
