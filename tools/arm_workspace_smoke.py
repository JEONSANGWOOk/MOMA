"""Arm-only workspace GUI QA: no robot connections or hardware commands."""
import sys,tempfile,time,math
from pathlib import Path
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as am
import seer_control.studio_ui as ui
from window_capture import capture_window
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json';app=am.Console();errors=[]
 app.report_callback_exception=lambda t,v,b:errors.append(str(v))
 try:
  app.pose_autosave.set(False);app.tabs.select(app.arm_workspace_page);app.geometry('1366x768');app.update()
  assert any(app.tabs.tab(tab,'text')=='로봇팔 개발' for tab in app.tabs.tabs())
  assert str(app.arm_workspace_page) in app.navigation_buttons
  amr_pose=(app.sim.state.x,app.sim.state.y,app.sim.state.theta);amr_arm=dict(app.sim.arm);api_cfg=dict(app.studio_config['arm'])
  app._arm_call=Mock(side_effect=AssertionError('Unexpected hardware request'))
  app.arm_dev_program.set('관절 이동 예제');app._aw_play()
  for i in range(100):app.arm_dev_sim.tick(.1)
  assert app.arm_dev_sim.state=='COMPLETED',app.arm_dev_sim.error
  assert amr_pose==(app.sim.state.x,app.sim.state.y,app.sim.state.theta);assert app.sim.arm==amr_arm;assert app.studio_config['arm']==api_cfg;app._arm_call.assert_not_called()
  app.arm_dev_canvas.render();app.update();assert app.arm_dev_canvas.find_withtag('arm_mesh');assert not app.arm_dev_canvas.find_withtag('world_amr')
  for _ in range(5):app.update();time.sleep(.03)
  capture_window(app,Path('artifacts/arm_workspace_1366.png'))
  app.geometry('1100x650');app.update();capture_window(app,Path('artifacts/arm_workspace_1100.png'))
  from tkinter import ttk
  def widgets(w):
   yield w
   for child in w.winfo_children():yield from widgets(child)
  notebook=next(w for w in widgets(app.arm_workspace_page) if isinstance(w,ttk.Notebook));notebook.select(1);app.update();capture_window(app,Path('artifacts/arm_workspace_program.png'))
  app._aw_save();saved=app.studio_config['arm_workspace'];assert len(saved['joints_rad'])==6;assert saved['programs']
  assert not errors,errors
  print('PASS: arm workspace navigation, independent MoveJ program, 1366/1100 GUI, persistence; no SDK calls or AMR changes')
 finally:app.close()
