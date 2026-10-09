"""Actual Console REAL TCP path against an offline virtual robot on loopback."""
import sys
import tempfile
import time
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

with tempfile.TemporaryDirectory() as folder,patch.object(Path,'home',return_value=Path(folder)):
    import seer_control.app as am
    import seer_control.studio_ui as ui
    from seer_control.model import MapModel
    from seer_control.robot_emulator import VirtualRobot,EmulatorServer,run_window
    import tkinter as tk
    am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json'
    model=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=2,y=0)],
                        edges=[['A','B']],walls=[[-2,-2,4,-2],[4,-2,4,4],[4,4,-2,4],[-2,4,-2,-2]]))
    robot=VirtualRobot(model,Path(folder)/'emulator_logs')
    server=EmulatorServer(robot).start();app=None;errors=[]
    try:
        app=am.Console();app.withdraw();app.pose_autosave.set(False);app.pose_autorecover.set(False)
        app.report_callback_exception=lambda t,v,b:errors.append(str(v))
        def wait_for(check,timeout=10):
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                app.update()
                if errors:raise AssertionError(errors)
                if check():return
                time.sleep(.02)
            raise AssertionError('Timed out: '+str(app.live)+' / runner '+app.studio_runner.status+' / '+app.studio_runner.error)
        app.host.set('127.0.0.1');app.mode.set('실기 · 제어');app.connect()
        wait_for(lambda:app.connected and app.last_state>0)
        app.send_command('lock_control',dict(nick_name=app.control_nick))
        wait_for(lambda:robot.control_owner==app.control_nick)
        app.pull_robot_map()
        wait_for(lambda:not app.downloading_map and getattr(app.map,'smap_source',None) and 'B' in app.map.nodes)
        assert app.map.route('A','B')==['A','B']
        app.ui_role.set('사용자');app._role_apply();app._operator_open_route();app._operator_map_add('B')
        app._operator_preview();app._operator_start()
        wait_for(lambda:robot.task_status==2 and robot.status()['x']>.05)
        wait_for(lambda:app.live.get('speed',0)>.01)
        app.action('pause');wait_for(lambda:robot.task_status==3)
        pose=robot.status()['x'];time.sleep(.25);app.update()
        assert abs(robot.status()['x']-pose)<1e-6
        app.action('resume');wait_for(lambda:robot.task_status==2)
        robot.inject('blocked');wait_for(lambda:app.live.get('blocked') is True)
        pose=robot.status()['x'];time.sleep(.2);app.update();assert abs(robot.status()['x']-pose)<1e-6
        robot.inject('blocked')
        wait_for(lambda:app.studio_runner.status=='COMPLETED',timeout=15)
        assert abs(app.live['x']-2)<.05 and app.live['task_status']==4
        assert app.real and app.studio_runner.report.metadata['controller']['model']=='LAN Virtual AMR'
        assert robot.decision_journal.flush()
        assert (Path(folder)/'emulator_logs'/'decisions.jsonl').stat().st_size>0
        assert any(e['source']=='REAL.주행 완료 판정' for e in app.decision_journal.records)
        print('PASS: GUI REAL TCP connection, control acquisition, SMAP pull, user-mode mission, real position/speed feedback, pause/resume, BLOCKED and completion/report logs')
        # This smoke creates a second Tk interpreter; cancel the first one's timers.
        for job in app.tk.call('after','info'):app.tk.call('after','cancel',job)
        app.close();app=None
        # Check the remote laptop's Tk view and a real click callback.
        original_tk=tk.Tk
        def make_root():
            root=original_tk();root.withdraw()
            root.report_callback_exception=lambda t,v,b:errors.append(str(v))
            def check_remote():
                try:
                    canvas=next(w for w in root.winfo_children() if isinstance(w,tk.Canvas))
                    canvas.redraw();x,y=canvas.xy(1,1)
                    canvas.click(type('Event',(),dict(x=x,y=y))())
                    assert robot.map.obstacles
                finally:root.destroy()
            root.after(400,check_remote)
            return root
        with patch.object(tk,'Tk',side_effect=make_root):run_window(robot,server)
        assert not errors,errors
        print('PASS: remote emulator Tk map, local obstacle click and clean window shutdown')
    finally:
        try:
            if app:app.close()
        finally:server.close();robot.close()
