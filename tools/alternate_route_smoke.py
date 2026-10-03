import sys,time,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.model import MapModel,Simulator
from seer_control.dynamic_obstacles import POLICIES
from window_capture import capture_window
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json'
 app=am.Console();errors=[];app.report_callback_exception=lambda t,v,b:errors.append(str(v))
 try:
  app.pose_autosave.set(False);app.geometry('1366x720');app.update()
  m=MapModel(dict(format='amr-console-map-v1',name='Alternate route and skipped goal',nodes=[dict(id=k,x=x,y=y) for k,x,y in [('A',0,0),('B',3,0),('C',0,2),('D',3,2),('E',6,0)]],edges=[['A','B'],['A','C'],['C','D'],['D','B'],['D','E']],walls=[],obstacles=[dict(x=3,y=0,radius=.4)]))
  app.map=m;app.sim=Simulator(m);app.refresh_nodes()
  app.sim_obstacle_policy.set(POLICIES['reroute']);app._sim_obstacle_change()
  app.reroute_wait.set('.5');app.reroute_attempts.set('2');app._reroute_settings()
  assert app.studio_config['reroute_attempts']==2
  app.sim.navigate('B')
  for i in range(500):
   app.sim.tick(.1)
   if app.sim.skipped_goals:break
  assert 'B' in app.sim.skipped_goals
  app.draw_map();app.update();capture_window(app,Path('artifacts/skipped_goals_2d.png'))
  app.view_mode.set('3D');app._spatial_switch();app.draw_map();app.update();capture_window(app,Path('artifacts/skipped_goals_3d.png'))
  app._skipped_history();app.update()
  assert not errors,errors
  print('PASS: main reroute settings persist, unreachable goal marked in 2D/3D, history opens without Tk errors')
 finally:app.close()
