import sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as am
import seer_control.studio_ui as ui
from window_capture import capture_window
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json';app=am.Console();errors=[]
 app.report_callback_exception=lambda t,v,b:errors.append(str(v))
 try:
  app.pose_autosave.set(False);app.geometry('1280x720');app.update()
  app.connected=True;app.sim_powered=True;app.manual.set(True);app.speed.set('.3')
  s=app.sim;s.state.x=2;s.state.y=2;s.state.theta=0;s.collision_radius=.6045
  s.map.obstacles=[dict(x=2.9,y=2,radius=.2,map_fixed=False)]
  app.press_drive('forward');app.update()
  assert s.state.x==2;assert s.state.blocked;assert app.held is None
  assert '수동 정지' in app.command_status.cget('text')
  app.draw_map();capture_window(app,Path('artifacts/manual_safety.png'))
  app.press_drive('back');s.tick(.1);assert s.state.x<2;assert not s.state.blocked
  app.release_drive();assert not errors,errors
  print('PASS: keyboard/button collision stop without modal dialog, persistent BLOCKED, checked reverse retreat')
 finally:app.close()
