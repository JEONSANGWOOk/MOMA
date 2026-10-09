"""Live openTCS + original MOMA Console integration smoke. Virtual AGV only."""
import sys
import tempfile
import time
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

with tempfile.TemporaryDirectory() as folder,patch.object(Path,'home',return_value=Path(folder)):
    import seer_control.app as am
    import seer_control.studio_ui as ui
    from seer_control.opentcs_bridge import OpenTCSRobot
    from seer_control.robot_emulator import EmulatorServer
    am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'studio.json'
    robot=OpenTCSRobot(vehicle='AGV-04',log_dir=Path(folder)/'logs').start()
    server=EmulatorServer(robot).start(physics=False)
    app=None;errors=[]
    try:
        app=am.Console();app.withdraw();app.pose_autosave.set(False);app.pose_autorecover.set(False)
        app.report_callback_exception=lambda t,v,b:errors.append(str(v))
        def wait(check,timeout=30):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                app.update()
                if errors:raise AssertionError(errors)
                if check():return
                time.sleep(.02)
            raise AssertionError('Timed out: '+str(app.live)+' / '+app.studio_runner.status+' / '+app.studio_runner.error)
        app.host.set('127.0.0.1');app.mode.set('실기 · 제어');app.connect()
        wait(lambda:app.connected and app.last_state>0)
        app.send_command('lock_control',dict(nick_name=app.control_nick));wait(lambda:robot.control_owner==app.control_nick)
        app.pull_robot_map();wait(lambda:'Point-0011' in app.map.nodes and not app.downloading_map)
        assert len(app.map.nodes)==59
        assert len(robot.fleet)==4
        start=robot.vehicle_data['currentPosition']
        goal=min((r['b'] for r in robot.map.path_records if r['a']==start),key=lambda node:robot.map.distance(start,node))
        app.ui_role.set('사용자');app._role_apply();app._operator_open_route();app._operator_map_add(goal)
        app.operator_map.redraw()
        assert len(app.operator_map.find_withtag('acs_fleet_label'))==4
        initial=(robot.status()['x'],robot.status()['y'])
        app._operator_preview();app._operator_start()
        wait(lambda:robot.order_data.get('state')=='BEING_PROCESSED')
        order=robot.order_name
        assert robot.order_data['processingVehicle']=='AGV-04'
        app.action('pause');wait(lambda:robot.vehicle_data.get('paused'))
        app.action('resume');wait(lambda:not robot.vehicle_data.get('paused'))
        wait(lambda:app.studio_runner.status=='COMPLETED',timeout=60)
        assert robot.order_data['state']=='FINISHED'
        assert robot.vehicle_data['currentPosition']==goal
        assert abs(app.live['x']-robot.map.nodes[goal]['x'])<.05
        assert abs(app.live['y']-robot.map.nodes[goal]['y'])<.05
        assert (app.live['x'],app.live['y'])!=initial
        app.operator_map.redraw()
        assert len(app.operator_map.find_withtag('acs_fleet_label'))==4
        assert any(e['source']=='REAL.주행 완료 판정' for e in app.decision_journal.records)
        print('PASS: actual ACS order '+order+' / 4 fleet markers / changed real node position / AGV-04 / 59 nodes / user-mode mission / pause-resume / FINISHED / decision logs')
        next_goal=next(r['b'] for r in robot.map.path_records if r['a']==goal)
        app.send_command('navigate',dict(id=next_goal))
        wait(lambda:robot.order_name!=order and robot.order_data.get('state')=='BEING_PROCESSED')
        app.send_command('cancel');wait(lambda:app.live.get('task_status')==6)
        assert robot.order_data['state']=='FAILED'
        print('PASS: cancellation acknowledged by ACS terminal state and mapped to CANCELED')
    finally:
        if app:
            for job in app.tk.call('after','info'):app.tk.call('after','cancel',job)
            app.close()
        if robot.order_data.get('state') in ('RAW','ACTIVE','DISPATCHABLE','BEING_PROCESSED','WITHDRAWN'):robot.request(3003)
        server.close();robot.close()
