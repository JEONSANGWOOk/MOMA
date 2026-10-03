"""Own-window SIM policy and moving actor integration. No hardware/network."""
import math
from pathlib import Path
import sys
import tempfile
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as app_module
import seer_control.studio_ui as studio_module
from seer_control.model import MapModel,Simulator
from seer_control.dynamic_obstacles import POLICIES
from window_capture import capture_window


def buttons(widget):
    found=[]
    if widget.winfo_class()=='Button':found.append(widget)
    for child in widget.winfo_children():found.extend(buttons(child))
    return found


with tempfile.TemporaryDirectory() as folder:
    app_module.USER_DIR=Path(folder);studio_module.SETTINGS=Path(folder)/'settings.json'
    app=app_module.Console();errors=[]
    app.report_callback_exception=lambda typ,value,tb:errors.append(str(value))
    try:
        app.pose_autosave.set(False)
        model=MapModel(dict(format='amr-console-map-v1',name='Dynamic people / AMR SIM',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=6,y=0),dict(id='C',x=4,y=1.7),dict(id='D',x=6,y=1.7)],
            edges=[['A','B'],['C','D']],walls=[[-2,-3,8,-3],[8,-3,8,3],[8,3,-2,3],[-2,3,-2,-3]]))
        app.map=model;app.sim=Simulator(model);app.sim.collision_radius=.61;app.refresh_nodes()
        for kind,start,end in [('person',[2,0],[2,2]),('amr',[4,1.7],[6,1.7])]:
            app._actor_dialog(kind,start,end);app.update()
            win=next(w for w in app.winfo_children() if w.winfo_class()=='Toplevel')
            if kind=='amr':
                def widgets(w):
                    for child in w.winfo_children():
                        yield child
                        yield from widgets(child)
                choice=next(w for w in widgets(win) if w.winfo_class()=='TCombobox')
                for key in ['C','D']:
                    choice.set(key);next(b for b in buttons(win) if b.cget('text')=='포인트 추가').invoke()
            next(b for b in buttons(win) if b.cget('text')=='저장 / 배치').invoke()
        assert len(model.obstacles)==2
        app.sim_obstacle_policy.set(POLICIES['adaptive']);app._sim_obstacle_change()
        assert app.studio_config['sim_obstacle_policy']=='adaptive'
        actor=model.obstacles[0];actor['paused']=True
        app.sim.navigate('B')
        for _ in range(200):app.sim.tick(.1)
        assert app.sim.state.blocked and '동적' in app.sim.avoidance_status
        app.tabs.select(app.operation_page);app.update();app.draw_map();app.update()
        artifacts=Path(__file__).resolve().parents[1]/'artifacts'
        capture_window(app,artifacts/'dynamic_obstacles_2d.png')
        app.view_mode.set('3D');app._spatial_switch();app.draw_map();app.update()
        assert app.world3d.find_withtag('world_actor')
        box=app.world3d.bbox('world_actor')
        assert box[3]-box[1]>10,box
        capture_window(app,artifacts/'dynamic_obstacles_3d.png')
        actor['paused']=False;actor['dwell_s']=60
        for _ in range(600):
            app.sim.tick(.1)
            assert abs(app.sim.state.y)<1e-6
            for obs in model.obstacles:
                assert math.dist((obs['x'],obs['y']),(app.sim.state.x,app.sim.state.y))>obs['radius']+.61
        assert app.sim.state.last_node=='B'
        app.dynamic_paused.set(True);app._actors_pause();before=[(o['x'],o['y']) for o in model.obstacles]
        for _ in range(20):app.sim.tick(.1)
        assert before==[(o['x'],o['y']) for o in model.obstacles]
        app._actors_manage();app.update()
        for win in [w for w in app.winfo_children() if w.winfo_class()=='Toplevel']:win.destroy()
        assert not errors,errors
        print('PASS: main policy save, UI people/AMR creation, dynamic wait/resume, no overlap, 2D/3D actors, pause and management')
    finally:app.close()
