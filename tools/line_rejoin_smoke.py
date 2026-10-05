"""Main UI line-priority default and short bypass/original line regression."""
import sys,tempfile
from pathlib import Path
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[1]/'tests')]
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.model import Simulator
from test_nearest_rejoin import scene
from window_capture import capture_window
from seer_control.studio_core import collision_reason
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json'
 app=am.Console();errors=[];app.report_callback_exception=lambda t,v,b:errors.append(str(v))
 try:
  app.geometry('1366x768');app.update();assert app.prefer_line_rejoin.get() and app.sim.prefer_line_rejoin
  app.map=scene();app.sim=Simulator(app.map);s=app.sim;s.collision_radius=.36;s.prefer_line_rejoin=True;s.prefer_graph_routes=True;s.obstacle_policy='auto'
  s.auto_scenarios={k:'avoid' for k in s.auto_scenarios};s.navigate('B');app.fit_map()
  nominal=[(0.,0.),(10.,0.)]
  for _ in range(500):
   s.tick(.05)
   assert not collision_reason(app.map,s.state.x,s.state.y,.36)
   if abs(s.state.y)>.4:break
  assert s._reference_waypoints==nominal and s._local_avoidance_active,(s._reference_waypoints,s._local_avoidance_active,s.state.x,s.state.y,s.avoidance_status,s.obstacle_policy)
  app.last_tick=__import__('time').monotonic();app.tick();app.draw_map();app.update();capture_window(app,Path('artifacts/line_rejoin_bypass.png'))
  for _ in range(1800):
   s.tick(.05);assert not collision_reason(app.map,s.state.x,s.state.y,.36)
   if s.state.x>3.3:assert abs(s.state.y)<1e-5
   if s.state.last_node=='B':break
  assert s.state.last_node=='B'
  app._line_rejoin_save();assert app.studio_config['prefer_line_rejoin']
  assert not errors,errors
  print('PASS main default, immutable original line, near-obstacle rejoin and straight tail, collision-free arrival, saved setting')
 finally:app.close()
