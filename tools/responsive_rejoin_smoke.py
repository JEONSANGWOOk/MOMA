"""Compact desktop and rejoin preview, without hardware/network."""
import sys,time,tempfile
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as app_module
import seer_control.studio_ui as ui_module
from seer_control.model import MapModel,Simulator
from seer_control.dynamic_obstacles import POLICIES
from window_capture import capture_window

with tempfile.TemporaryDirectory() as folder:
    app_module.USER_DIR=Path(folder);ui_module.SETTINGS=Path(folder)/'settings.json'
    app=app_module.Console();errors=[]
    app.report_callback_exception=lambda typ,value,tb:errors.append(str(value))
    try:
        app.pose_autosave.set(False)
        for width,height in [(1366,720),(1024,640)]:
            app.geometry(f'{width}x{height}');app.update();app._ui_scale_apply();app.update()
            assert .65<=app.ui_scale_ratio<=.85
            assert app.operation_canvas.winfo_width()>300
            canvas=app.operation_right_canvas
            canvas.yview_moveto(0)
            button=next(w for w in canvas.master.winfo_children() if w.winfo_class()=='TScrollbar')
            assert button.winfo_ismapped(),(width,height,app.winfo_width(),canvas.winfo_width(),button.winfo_width())
            before=canvas.yview()
            app._ui_panel_wheel(SimpleNamespace(widget=app.sim_avoidance_label,delta=-120))
            assert canvas.yview()[0]>before[0]
            canvas.yview_moveto(1);app.update()
            assert canvas.yview()[1]>=.99
            canvas.yview_moveto(0);app.update()
            capture_window(app,Path(__file__).resolve().parents[1]/f'artifacts/ui_compact_{width}.png')
        app.ui_scale.set('100%');app._ui_scale_apply();assert app.ui_scale_ratio==1.
        app.ui_scale.set('75%');app._ui_scale_apply();assert app.ui_scale_ratio==.75
        assert app.studio_config['ui_scale']=='75%'
        app.geometry('1366x720');app.operation_right_canvas.yview_moveto(0)
        m=MapModel(dict(format='amr-console-map-v1',name='Early return to original lane',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=8,y=0)],
            edges=[['A','B']],walls=[[-2,-3,10,-3],[10,-3,10,3],[10,3,-2,3],[-2,3,-2,-3]],obstacles=[dict(x=2,y=0,radius=.3)]))
        app.map=m;app.sim=Simulator(m);app.sim.collision_radius=.61;app.refresh_nodes();app.sim_obstacle_policy.set(POLICIES['avoid']);app._sim_obstacle_change();app.sim.navigate('B')
        for _ in range(80):
            app.sim.tick(.1)
            if '복귀' in app.sim.avoidance_status:break
        assert '복귀' in app.sim.avoidance_status
        points=app.sim.navigation_points();peak=max(range(len(points)),key=lambda i:abs(points[i][1]))
        rejoin=next(p for p in points[peak+1:] if abs(p[1])<1e-8)
        assert rejoin[0]<3.5
        app.draw_map();app.update()
        capture_window(app,Path(__file__).resolve().parents[1]/'artifacts/obstacle_early_rejoin.png')
        assert not errors,errors
        print('PASS: 1366/1024 compact layout, wheel on panel children, reach panel bottom, manual scale save, early lane rejoin preview')
    finally:app.close()
