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
  app.pose_autosave.set(False);app.update();app._studio_real_capabilities();app.update()
  import tkinter as tk
  win=next(w for w in app.winfo_children() if isinstance(w,tk.Toplevel) and w.title()=='실기 연동 현황')
  capture_window(win,Path('artifacts/hardware_capabilities.png'));assert not errors,errors
  print('PASS: real capability audit GUI, no hardware connection or motion')
 finally:app.close()
