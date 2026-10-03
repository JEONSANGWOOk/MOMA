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
  app.pose_autosave.set(False);app.update();r=app.studio_runner
  r.start([dict(type='Path Nav',goal='LM2')],repeat=3)
  for cycle in range(3):
   r.report.add(cycle,0,'시작','LM2','Path Nav')
   r.report.add(cycle,0,'장애물 / 경로','LM2','LM2 → LM7 → LM8')
   r.report.add(cycle,0,'성공','LM2','목적지 도착',3.5)
  r.report.completed_loops=3;r.status='COMPLETED';r.report.finish(r.status)
  app._studio_report_update();app.update()
  assert list(Path(folder,'mission_reports').glob('*.json'))
  assert list(Path(folder,'mission_reports').glob('*.txt'))
  assert not errors,errors
  import tkinter as tk
  win=next(w for w in app.winfo_children() if isinstance(w,tk.Toplevel) and w.title()=='미션 수행 보고서')
  capture_window(win,Path('artifacts/mission_report.png'))
  print('PASS: automatic report dialog, loop lists, TXT/JSON saved')
 finally:app.close()
