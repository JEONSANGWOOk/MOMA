"""Own-window SIM wall editor and avoidance integration; no hardware/network."""
import sys
import tempfile
import math
import time
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as app_module
import seer_control.studio_ui as ui_module
from seer_control.model import MapModel,Simulator
from seer_control.studio_core import collision_reason
from window_capture import capture_window

with tempfile.TemporaryDirectory() as folder:
    app_module.USER_DIR=Path(folder);ui_module.SETTINGS=Path(folder)/'settings.json'
    app=app_module.Console();errors=[]
    app.report_callback_exception=lambda typ,value,tb:errors.append(str(value))
    try:
        app.pose_autosave.set(False)
        model=MapModel(dict(format='amr-console-map-v1',name='Wall / avoidance SIM',nodes=[dict(id='START',x=0,y=0),dict(id='GOAL',x=4,y=0)],edges=[['START','GOAL']],walls=[[-2,-2,6,-2],[6,-2,6,2],[6,2,-2,2],[-2,2,-2,-2]]))
        app.map=model;app.sim=Simulator(model);app.sim.collision_radius=.61;app.refresh_nodes()
        app.tabs.select(app.nodes_page);app.edit_mode.set('벽 만들기');app.update();app.draw_map()
        for x,y in ((2,-.4),(2,.4)):
            sx,sy=app.xy(x,y);app.map_click(SimpleNamespace(x=sx,y=sy))
        assert len(model.walls)==5
        app.tabs.select(app.operation_page);app.update()
        app.sim_obstacle_policy.set('정지 / 대기');app._sim_obstacle_change();app.sim.navigate('GOAL')
        for _ in range(200):app.sim.tick(.1)
        assert app.sim.state.blocked and app.sim.state.x<1.4
        app.sim_obstacle_policy.set('우회 주행');app._sim_obstacle_change()
        for _ in range(30):app.sim.tick(.1)
        assert '우회' in app.sim.avoidance_status
        assert max(abs(p[1]) for p in app.sim.navigation_points())>1
        app.draw_map();app.update()
        capture_window(app,Path(__file__).resolve().parents[1]/'artifacts/sim_wall_avoidance.png')
        for _ in range(1000):
            app.sim.tick(.1)
            assert not collision_reason(model,app.sim.state.x,app.sim.state.y,app.sim.collision_radius)
        assert app.sim.state.last_node=='GOAL'
        assert app.studio_config['sim_obstacle_policy']=='avoid' and ui_module.SETTINGS.exists()
        # Operation layer switches, live status and rear-follow camera.
        assert set(app.operation_layer_buttons)==set(app.layers)
        for name,button in app.operation_layer_buttons.items():assert str(button.cget('variable'))==str(app.layers[name])
        app.layers['LiDAR'].set(True);app.view_mode.set('3D');app._spatial_switch();app.update();app.draw_map()
        assert app.world3d.find_withtag('world_lidar')
        assert 'LiDAR SIM' in app.operation_sensor_label.cget('text')
        app.layers['LiDAR'].set(False);app.draw_map();assert not app.world3d.find_withtag('world_lidar')
        for layer,tag in (('벽','world_map'),('노드','world_node'),('경로','world_path'),('그리드','world_grid')):
            app.layers[layer].set(False);app.draw_map();assert not app.world3d.find_withtag(tag)
            app.layers[layer].set(True)
        app.layers['LiDAR'].set(True);app.camera_view.set('뒤따라 보기');app._spatial_camera();app.update()
        for heading in (0.,math.pi/2,-math.pi/2):
            app.sim.state.theta=heading;app.sim.state.x=0.;app.draw_map()
            view=app.world3d;eye,*_=view.camera.basis();target=view.camera.target
            assert (eye[0]-target[0])*math.cos(heading)+(eye[1]-target[1])*math.sin(heading)<0
            assert target[:2]==[app.sim.state.x,app.sim.state.y]
        app.sim.state.theta=0.;app.draw_map()
        capture_window(app,Path(__file__).resolve().parents[1]/'artifacts/world3d_chase_lidar.png')
        app.real=True;app.live.update(x=0.,y=0.,theta=0.,blocked=True)
        app.real_laser_points=[(0.,1.)];app.lidar_source='1101.laser_beams(WORLD)';app.last_laser_rx=time.monotonic()
        app.draw_map();assert 'LiDAR 수신 1점' in app.operation_sensor_label.cget('text') and '정지 감지' in app.operation_sensor_label.cget('text')
        app.last_laser_rx=time.monotonic()-10;app.draw_map()
        assert '수신 지연' in app.operation_sensor_label.cget('text') and not app.world3d.find_withtag('world_lidar')
        app.real=False
        assert not errors,errors
        print('SIM wall / avoidance GUI: PASS (wall editing, wait, switch to detour, collision-free arrival, route preview, persisted policy)')
    finally:
        if app.pad_after:app.after_cancel(app.pad_after)
        app.studio_bridge.close();app.destroy()
