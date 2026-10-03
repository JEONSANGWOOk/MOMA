import sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.model import MapModel,Simulator
from seer_control.dynamic_obstacles import POLICIES
from seer_control.studio_core import collision_reason
from window_capture import capture_window
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json';app=am.Console();errors=[]
 app.report_callback_exception=lambda t,v,b:errors.append(str(v))
 try:
  app.pose_autosave.set(False);app.geometry('1366x720')
  m=MapModel.load('maps/demo.json');m.obstacles.append(dict(id='AMR1',dynamic=True,map_fixed=False,kind='amr',x=4.23,y=2,radius=.61,paused=True,motion_path=[[4.23,2],[2,2]],speed_mps=.5,dwell_s=0));app.map=m;app.sim=Simulator(m);app.refresh_nodes()
  s=app.sim;s.state.x=5.48;s.state.y=2;s.state.last_node='LM2';s.collision_radius=.6045
  app.studio_config.update(reroute_wait_s=.1,auto_static_s=.5);app._auto_apply_settings();s.reroute_wait_s=.1
  app.sim_obstacle_policy.set(POLICIES['reroute']);app._sim_obstacle_change();s.navigate('LM8');s.route=['LM1','CP1','LM8'];s._segment_start='LM2';s._waypoints=[(2,2)];s._reference_waypoints=[(7,2),(2,2)]
  for _ in range(100):
   s.tick(.1)
   if s.route==['LM2','LM7','LM8']:break
  assert s.route==['LM2','LM7','LM8'],(s.route,s.avoidance_status)
  app.draw_map();app.update();capture_window(app,Path('artifacts/lm2_lm7_lm8_return_route.png'))
  for _ in range(1800):
   s.tick(.1);assert not collision_reason(m,s.state.x,s.state.y,s.collision_radius)
   if s.state.last_node=='LM8':break
  assert s.state.last_node=='LM8' and not s.skipped_goals;assert not errors,errors
  print('PASS: screenshot pose 5.48/2, blocked LM1 lane, LM2-LM7-LM8 route shown, collision-free arrival, no skip')
 finally:app.close()
