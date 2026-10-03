import sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.dynamic_obstacles import POLICIES
from window_capture import capture_window
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json';app=am.Console();errors=[]
 app.report_callback_exception=lambda t,v,b:errors.append(str(v))
 try:
  app.pose_autosave.set(False);app.geometry('1280x720');app.update()
  s=app.sim;s.map.obstacles=[dict(id='AMR1',dynamic=True,kind='amr',x=4.23,y=2,radius=.61,paused=True,motion_path=[[4.23,2],[2,2]],speed_mps=.5)]
  app.sim_obstacle_policy.set(POLICIES['reroute']);s.obstacle_policy='reroute';s.collision_radius=.6045;s.state.x=5.48;s.state.y=2;s.state.last_node='LM2'
  s.navigate('LM1');s.route=['LM1'];s._segment_start='LM2';s._waypoints=[(2,2)];s._reference_waypoints=[(7,2),(2,2)]
  assert s.try_recovery_detour();assert len(s.navigation_points())>2;assert not s.skipped_goals
  app._map_focus_toggle();app.draw_map();app.update()
  capture_window(app,Path('artifacts/final_detour.png'))
  assert not errors,errors
  print('PASS: full footprint autonomous bypass drawn on map, no skipped goal')
 finally:app.close()
