"""Offline task editor QA. Does not connect to any robot."""
import sys,tempfile,tkinter as tk
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as am
import seer_control.studio_ui as ui
from window_capture import capture_window
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json';app=am.Console();errors=[]
 app.report_callback_exception=lambda t,v,b:errors.append(str(v))
 try:
  app.pose_autosave.set(False);app.update();app._fr5_development();app.update()
  win=next(w for w in app.winfo_children() if isinstance(w,tk.Toplevel) and w.title().startswith('FR5 작업 개발'))
  def all_widgets(w):
   yield w
   for c in w.winfo_children():yield from all_widgets(c)
  widgets=list(all_widgets(win));tabs=next(w for w in widgets if isinstance(w,__import__('tkinter').ttk.Notebook));tabs.select(1);app.update()
  button=next(w for w in widgets if w.winfo_class() in ('Button','TButton') and w.cget('text')=='템플릿 생성');button.invoke();app.update()
  tree=next(w for w in widgets if w.winfo_class()=='Treeview');assert len(tree.get_children())==12
  assert not app.fr5_client.connected;assert not app.studio_runner.active
  capture_window(win,Path('artifacts/fr5_program_development.png'));assert not errors,errors
  print('PASS: FR5 task workspace and pick/place draft, no hardware connection or commands')
 finally:app.close()
