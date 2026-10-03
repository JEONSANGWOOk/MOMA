import sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.model import MapModel,Simulator
from seer_control.obstacle_tracking import SCENARIOS
from seer_control.dynamic_obstacles import POLICIES
from window_capture import capture_window

def widgets(w):
 for c in w.winfo_children():
  yield c
  yield from widgets(c)
def combos(w):return [c for c in widgets(w) if c.winfo_class()=='TCombobox']
def save(w):next(c for c in widgets(w) if c.winfo_class()=='Button' and c.cget('text')=='설정 저장').invoke()
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json';app=am.Console();errors=[]
 app.report_callback_exception=lambda t,v,b:errors.append(str(v))
 try:
  app.pose_autosave.set(False);app.geometry('1366x720')
  app.map=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id='A',x=0,y=0),dict(id='B',x=3,y=0)],edges=[['A','B']],walls=[],obstacles=[dict(id='BOX',x=1.5,y=0,radius=.3)]));app.sim=Simulator(app.map);app.refresh_nodes()
  app._auto_scenario_dialog();app.update();win=next(w for w in app.winfo_children() if w.winfo_class()=='Toplevel');cc=combos(win);cc[2].set(SCENARIOS['wait']);save(win)
  assert app.studio_config['auto_scenarios']['static']=='wait'
  app._auto_scenario_dialog();app.update();win=next(w for w in app.winfo_children() if w.winfo_class()=='Toplevel');cc=combos(win);cc[0].set('1. BOX')
  next(c for c in widgets(win) if c.winfo_class()=='Checkbutton').invoke();cc[2].set(SCENARIOS['wait_avoid'])
  app.update();capture_window(win,Path('artifacts/obstacle_scenario_settings.png'));save(win)
  assert app.map.obstacles[0]['auto_scenarios']['static']=='wait_avoid'
  app.sim_obstacle_policy.set(POLICIES['auto']);app._sim_obstacle_change();app.sim.navigate('B')
  for _ in range(35):app.sim.tick(.1)
  app.update();app.draw_map();app.update();capture_window(app,Path('artifacts/obstacle_auto_classification.png'))
  assert '정적' in app.sim.auto_obstacle_status
  assert not errors,errors
  print('PASS: global scenario settings, per-obstacle override save, automatic static status, UI no callback errors')
 finally:app.close()
