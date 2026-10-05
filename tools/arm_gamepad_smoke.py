"""Synthetic DualSense arm input in a real offline Tk workspace."""
import sys,tempfile,time,math
from pathlib import Path
from unittest.mock import Mock
sys.path[:0]=[str(Path(__file__).resolve().parents[1])]
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.gamepad import Sample
from window_capture import capture_window
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json'
 app=am.Console();errors=[];app.report_callback_exception=lambda t,v,b:errors.append(str(v));app._arm_call=Mock(side_effect=AssertionError('hardware'))
 try:
  app.geometry('1366x768');app.update();app.tabs.select(app.arm_workspace_page);app.pad_target.set('로봇팔');app._pad_toggle();app.manual.set(True);app.pad_enabled.set(True)
  app.focus_force();app.update();app.focus_get=lambda:app
  app.pad_device=(0,type('Caps',(),dict(name='DualSense synthetic'))());app.pad_backend.read=lambda *args:Sample(dict(X=0.,Y=0.),0,time.monotonic(),('synthetic',))
  def sample(v=0,w=0,held=False):return Sample(dict(X=w,Y=-v),16 if held else 0,time.monotonic(),('synthetic',))
  start=list(app.arm_dev_sim.q);app._pad_process(sample(),time.monotonic())
  for _ in range(20):app.pad_arm_time=time.monotonic()-.05;app._pad_process(sample(1,.3,True),time.monotonic())
  assert app.arm_dev_sim.q!=start
  assert app.sim.arm['joint_positions']==app.arm_dev_sim.kin.positions(app.arm_dev_sim.q)
  q=list(app.arm_dev_sim.q);app._pad_process(sample(1,.3,False),time.monotonic());assert app.arm_dev_sim.q==q and not app.pad_arm_active
  app.pad_arm_group.set('J3 / J4');app._pad_toggle();app._pad_process(sample(),time.monotonic());app._pad_process(sample(.5,0,True),time.monotonic());assert app.arm_dev_sim.q[3]!=q[3]
  app._pad_process(sample(),time.monotonic())
  mode=sample();mode.buttons=32;app._pad_process(mode,time.monotonic());assert app.pad_arm_group.get()=='J5 / J6'
  app._pad_process(sample(),time.monotonic());speed=sample();speed.buttons=1<<32;app._pad_process(speed,time.monotonic());assert app.pad_arm_speed.get()==75
  app._pad_process(sample(),time.monotonic());app.arm_dev_sim.physics.configure(dict(kind='finger'));grip=sample(held=True);grip.buttons|=8;app._pad_process(grip,time.monotonic());assert app.arm_dev_sim.io['ToolDO'][0]==1
  app._pad_process(sample(),time.monotonic());app.update();capture_window(app,Path('artifacts/arm_gamepad_workspace.png'))
  app.pad_target.set('AMR');app._pad_toggle();assert not app.pad_gate.armed
  app._arm_call.assert_not_called();assert not errors,errors
  print('PASS offline Tk arm joystick groups, neutral/deadman release, main shared joints, mode change; no hardware calls')
 finally:app.close()
