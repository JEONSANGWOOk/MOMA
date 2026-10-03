import sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.model import MapModel,Simulator
from seer_control.dynamic_obstacles import POLICIES
from window_capture import capture_window
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json';app=am.Console();errors=[]
 app.report_callback_exception=lambda t,v,b:errors.append(str(v))
 try:
  app.pose_autosave.set(False);app.geometry('1280x720');app.update();normal=app.operation_canvas.winfo_width()
  assert app.map_info.winfo_height()<25;assert len(app.map_info.cget('text'))<100
  app._map_focus_toggle();app.update();assert app.operation_canvas.winfo_width()>normal+150;assert not app.operation_right_outer.winfo_ismapped();assert app.map_focus_stop.winfo_ismapped()
  capture_window(app,Path('artifacts/compact_map_focus.png'));app._map_focus_toggle();app.update();assert app.operation_right_outer.winfo_ismapped()
  app.sim.skipped_goals={'LM8':dict(goal='LM8')};app.studio_runner.start([dict(type='Path Nav',goal='LM8')],repeat=0);assert not app.sim.skipped_goals;app.studio_runner.cancel()
  assert not errors,errors
  print('PASS: compact one-line map status, wider map focus and restore, stop stays visible, new mission mark reset')
 finally:app.close()
