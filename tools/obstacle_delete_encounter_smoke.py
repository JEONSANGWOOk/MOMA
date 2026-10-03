import sys,tempfile,math
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
  app.map=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=8,y=0)],edges=[['A','B']],walls=[[0,-4,10,-4]],obstacles=[dict(x=3,y=0,radius=.3)]));app.sim=Simulator(app.map);app.refresh_nodes();app.sim.navigate('B')
  app._fixed_obstacles_manage();app.update();win=next(w for w in app.winfo_children() if w.winfo_class()=='Toplevel');tree=next(w for w in win.winfo_children() if w.winfo_class()=='Treeview');tree.selection_set(tree.get_children()[0]);capture_window(win,Path('artifacts/fixed_obstacles_delete.png'))
  next(w for w in win.winfo_children() if w.winfo_class()=='Button').invoke();assert not app.map.obstacles and app.sim.route and app.sim.state.target=='B'
  tree.selection_set(tree.get_children()[0]);next(w for w in win.winfo_children() if w.winfo_class()=='Button').invoke();assert not app.map.walls;win.destroy()
  for _ in range(1000):app.sim.tick(.1)
  assert app.sim.state.last_node=='B'
  actors=[dict(id=key,dynamic=True,map_fixed=False,kind='person',x=x,y=-2,radius=.3,motion_path=[[x,-2],[end,-2]],speed_mps=.5,dwell_s=0,encounter_wait_s=.3) for key,x,end in [('P1',2,6),('P2',6,2)]];app.map.obstacles=actors;turned=False
  for _ in range(100):
   app.sim.tick(.1);turned|=any(a.get('_direction')==-1 for a in actors)
   assert math.dist((actors[0]['x'],actors[0]['y']),(actors[1]['x'],actors[1]['y']))>.6-1e-6
  assert turned;app.draw_map();app.update();capture_window(app,Path('artifacts/actors_encounter_reverse.png'));assert not errors,errors
  print('PASS: fixed obstacle/wall delete while navigating, route retained and B arrival; actor encounter direction reversal without overlap')
 finally:app.close()
