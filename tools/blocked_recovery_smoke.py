"""Offline GUI/default-setting and collision-free BLOCKED recovery smoke."""
import sys,tempfile
from pathlib import Path
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[1]/'tests')]
import seer_control.app as am
import seer_control.studio_ui as ui
from test_blocked_recovery import scene
from window_capture import capture_window
from seer_control.studio_core import collision_reason
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json'
 app=am.Console();errors=[];app.report_callback_exception=lambda t,v,b:errors.append(str(v))
 try:
  app.geometry('1366x768');app.update()
  assert app.blocked_recovery_enabled.get() and app.sim.blocked_recovery.enabled
  for rear in (False,True):
   app.sim=scene(rear);app.map=app.sim.map;app.sim.collision_radius=.36
   app.sim.state.blocked=True;app.sim._collision_blocked=True;app.sim.block_reason='배치 장애물'
   if rear:app.sim.blocked_recovery.begin(app.sim,'배치 장애물')
   app.sim.tick(.05);app.last_tick=__import__('time').monotonic();app.tick();app.draw_map();app.update()
   capture_window(app,Path('artifacts')/('blocked_recovery_back.png' if rear else 'blocked_recovery_current.png'))
   for _ in range(1600):
    app.sim.tick(.05)
    assert not collision_reason(app.map,app.sim.state.x,app.sim.state.y,app.sim.blocked_recovery.radius(app.sim))
    if app.sim.state.last_node=='B':break
   assert app.sim.state.last_node=='B'
  app.blocked_recovery_enabled.set(False);app._blocked_recovery_save()
  assert not app.sim.blocked_recovery.enabled and app.studio_config['blocked_recovery_enabled'] is False
  assert not errors,errors
  print('PASS GUI default enabled, current-position bypass/short back preview, collision-free arrival, persisted toggle; offline SIM only')
 finally:app.close()
