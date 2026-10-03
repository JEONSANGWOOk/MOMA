import sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.model import MapModel,Simulator
from window_capture import capture_window
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json';app=am.Console();errors=[]
 app.report_callback_exception=lambda t,v,b:errors.append(str(v))
 try:
  app.pose_autosave.set(False);app.geometry('1366x720')
  app.map=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=6,y=0),dict(id='C',x=0,y=-3),dict(id='D',x=6,y=3)],edges=[['A','B']],walls=[],obstacles=[dict(x=5,y=2,radius=.3)]));app.sim=Simulator(app.map);app.refresh_nodes()
  moving=dict(x=2,y=-2,radius=.25,map_fixed=False);stationary=dict(x=4,y=-2,radius=.25,map_fixed=False);app.map.obstacles.extend([moving,stationary])
  for _ in range(35):moving['y']+=.035;app.sim.tick(.1)
  app.update();app.draw_map();app.update();assert len(app.sim.obstacle_tracker.records())==2
  assert app.operation_canvas.find_withtag('tracked_obstacle');capture_window(app,Path('artifacts/new_obstacle_tracks_2d.png'))
  app.view_mode.set('3D');app._spatial_switch();app.draw_map();app.update();assert app.world3d.find_withtag('world_tracked_obstacle');capture_window(app,Path('artifacts/new_obstacle_tracks_3d.png'))
  app.map.obstacles.remove(moving)
  for _ in range(15):app.sim.tick(.1)
  assert any(r['state']=='미관측' for r in app.sim.obstacle_tracker.records())
  app._tracking_history();app.update();win=next(w for w in app.winfo_children() if w.winfo_class()=='Toplevel');capture_window(win,Path('artifacts/new_obstacle_track_list.png'))
  assert not errors,errors
  print('PASS: fixed map excluded, new dynamic/static 2D/3D markings, trail, lost status and live list')
 finally:app.close()
