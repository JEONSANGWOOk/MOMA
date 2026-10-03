import sys,tempfile,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.model import MapModel,Simulator
from seer_control.dynamic_obstacles import POLICIES
from window_capture import capture_window

def widgets(w):
 for c in w.winfo_children():
  yield c
  yield from widgets(c)
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json';app=am.Console();errors=[]
 app.report_callback_exception=lambda t,v,b:errors.append(str(v))
 try:
  app.pose_autosave.set(False);app.geometry('1366x720')
  app.map=MapModel(dict(format='amr-console-map-v1',nodes=[dict(id=k,x=x,y=y) for k,x,y in [('A',0,0),('B',8,0),('BOUND1',0,-2),('BOUND2',8,2)]],edges=[['A','B']],walls=[],obstacles=[dict(id='OTHER',dynamic=True,map_fixed=False,kind='amr',x=8,y=0,radius=.61,motion_nodes=['B','A'],motion_path=[[8,0],[0,0]],speed_mps=.5,dwell_s=0)]));app.sim=Simulator(app.map);app.sim.collision_radius=.61;app.refresh_nodes()
  app.sim_obstacle_policy.set(POLICIES['reroute']);app._sim_obstacle_change()
  next(w for w in widgets(app) if w.winfo_class()=='Button' and w.cget('text')=='상호 정지 자동 해소 적용').invoke()
  assert app.studio_config['sim_obstacle_policy']=='auto';assert app.sim.auto_scenarios['static']=='wait_recover'
  app.studio_config.update(auto_static_s=.5,auto_wait_s=.5);app._auto_apply_settings();app.sim.navigate('B')
  both=False;captured=False
  for _ in range(2200):
   app.sim.tick(.1);o=app.map.obstacles[0]
   both|=app.sim.state.blocked and o.get('_motion')=='통행 대기'
   assert math.dist((app.sim.state.x,app.sim.state.y),(o['x'],o['y']))>=app.sim.collision_radius+o['radius']-1e-6
   if not captured and abs(app.sim.state.y)>.5:
    app.draw_map();app.update();capture_window(app,Path('artifacts/head_on_amr_recovery.png'));captured=True
   if app.sim.state.last_node=='B':break
  assert both and captured and app.sim.state.last_node=='B'
  assert not errors,errors
  print('PASS: recovery preset saved, both AMRs stopped, local detour with no overlap, final B arrival')
 finally:app.close()
