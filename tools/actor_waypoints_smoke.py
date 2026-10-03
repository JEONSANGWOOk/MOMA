import sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.model import MapModel,Simulator
from window_capture import capture_window

def children(w):
 for child in w.winfo_children():
  yield child
  yield from children(child)
def button(w,label):return next(c for c in children(w) if c.winfo_class()=='Button' and c.cget('text')==label)
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json';app=am.Console();errors=[]
 app.report_callback_exception=lambda t,v,b:errors.append(str(v))
 try:
  app.pose_autosave.set(False)
  app.map=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id=k,x=x,y=y) for k,x,y in [('A',0,0),('B',4,0),('C',4,3),('D',7,3)]],edges=[['A','B'],['B','C'],['C','D']],walls=[],obstacles=[]));app.sim=Simulator(app.map);app.refresh_nodes()
  app._actor_dialog('amr');app.update();win=next(w for w in app.winfo_children() if w.winfo_class()=='Toplevel')
  choice=next(c for c in children(win) if c.winfo_class()=='TCombobox')
  for key in ['B','C','D']:choice.set(key);button(win,'포인트 추가').invoke()
  app.update();capture_window(win,Path('artifacts/amr_node_waypoints.png'));button(win,'저장 / 배치').invoke()
  assert app.map.obstacles[0]['motion_nodes']==['B','C','D']
  app._actor_dialog('person',[2,-2],[4,-2]);app.update();win=next(w for w in app.winfo_children() if w.winfo_class()=='Toplevel')
  button(win,'포인트 추가').invoke();app.update();capture_window(win,Path('artifacts/person_waypoints.png'));button(win,'저장 / 배치').invoke()
  assert len(app.map.obstacles[1]['motion_path'])==3
  app.view_mode.set('3D');app._spatial_switch();app.draw_map();app.update();assert not errors,errors
  print('PASS: AMR 3 existing nodes, person 3 coordinate points, dialogs save, 3D renders')
 finally:app.close()
