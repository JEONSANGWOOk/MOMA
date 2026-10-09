"""FR5 GUI and mission integration with synthetic SDK only; no network."""
import math
from pathlib import Path
import sys
import tempfile
import time
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as app_module
import seer_control.studio_ui as studio_module
from seer_control.fairino_api import profile,SDKEngine
from window_capture import capture_window


with tempfile.TemporaryDirectory() as folder:
    app_module.USER_DIR=Path(folder);studio_module.SETTINGS=Path(folder)/'settings.json'
    app=app_module.Console();errors=[]
    app.report_callback_exception=lambda typ,value,tb:errors.append(str(value))
    try:
        app.pose_autosave.set(False)
        cfg=profile();cfg.update(verified=True,version_confirmed=True,controller_version='test',operations={
            'work':dict(method='MoveJ',target=[0,-90,90,-90,-90,0],vel=10),
            'safe_pose':dict(method='MoveJ',target=[0,-120,130,-100,-90,0],vel=10)})
        app.studio_config['arm']=cfg
        app.studio_runner.start([dict(type='Arm Action',operation='work',duration_s=1),dict(type='Arm Action',operation='safe_pose',duration_s=1)])
        clock=time.monotonic()
        for i in range(40):app.studio_runner.tick(clock+i*.1,.1)
        assert app.studio_runner.status=='COMPLETED'
        assert app.studio_arm_safe
        expected=[math.radians(v) for v in cfg['operations']['safe_pose']['target']]
        assert list(app.sim.arm['joint_positions'].values())==expected
        # Operator mode is the startup page; select the world viewport before
        # checking rendered poses, rather than rendering a hidden 1px canvas.
        app.tabs.select(app.operation_page);app._sync_navigation()
        app.view_mode.set('3D');app._spatial_switch();app.update();app._spatial_render()
        assert list(app.world3d.scene_joint_values.values())==expected, (app.world3d.scene_joint_values, expected, app.arm_dev_sim.q, app.arm_dev_sim.state)
        # REAL feedback rendering from a synthetic in-memory sample (no connect).
        app.real=True;app.fr5_client.connected=True
        app.fr5_feedback=dict(status='IDLE',joints_deg=[0]*6,joints_rad=[0]*6,tcp_mm_deg=[0]*6,motion_done=1)
        app.fr5_rx=time.monotonic();app._spatial_render()
        assert list(app.world3d.scene_joint_values.values())==[0]*6
        app.fr5_client.connected=False;app.real=False
        app.studio_config['arm']=profile()
        app._fr5_dialog();app.update()
        windows=[w for w in app.winfo_children() if w.winfo_class()=='Toplevel']
        assert windows
        capture_window(windows[-1],Path(__file__).resolve().parents[1]/'artifacts/fr5_api_settings.png')
        windows[-1].destroy()
        assert not errors,errors
        print('PASS: FR5 settings UI, SIM taught MoveJ mission, safe pose, REAL synthetic 3D feedback; no hardware/network')
    finally:
        app.real=False;app.fr5_client.connected=False;app.close()
