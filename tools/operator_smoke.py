"""Real offline Tk operator role + recipes; no hardware or microphone access."""
import sys,tempfile,time,json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.voice_gui import GuiRegistry
from window_capture import capture_window
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json'
 app=am.Console();errors=[];app.report_callback_exception=lambda t,v,b:errors.append(str(v));app._arm_call=Mock(side_effect=AssertionError('hardware'));app.voice_talk.set(False)
 try:
  app.geometry('1366x768');app.update();dev=GuiRegistry(app);target=next(r for r in dev.targets.values() if r['kind']=='choice' and 'J1 / J2' in r.get('values',[]))
  # Density refresh must never remap hidden 2D/3D views or role controls.
  app.view_mode.set('3D');app._spatial_switch();app.update()
  app._ui_scale_last=None;app._ui_scale_apply();app.update()
  assert not app.operation_canvas.winfo_manager();assert app.world3d.winfo_manager()=='pack'
  app.view_mode.set('2D');app._spatial_switch();app._ui_scale_last=None;app._ui_scale_apply();app.update()
  assert not app.world3d.winfo_manager();assert app.operation_canvas.winfo_manager()=='pack'
  # Developer recipe reorder uses the same move routine as drag-and-drop.
  app.recipe_selection.set('순환 운반');app._recipe_load();app.recipe_list.selection_set(0);app._recipe_move(1);assert app.recipe_blocks[1]['value']=='픽업';app._recipe_move(-1)
  app.ui_role.set('사용자');app._role_apply();app.update();assert app.tabs.tab(app.nodes_page,'state')=='hidden';assert app.tabs.tab(app.operator_page,'state')=='normal'
  app._ui_scale_last=None;app._ui_scale_apply();app.update();assert not app.navigation_buttons[str(app.nodes_page)].winfo_manager()
  gui=GuiRegistry(app);assert 'map::좌표 클릭' not in gui.targets
  try:dev.execute(dict(action='gui',target=target['id'],operation='set',value='J3 / J4'));raise AssertionError('stale developer GUI privilege')
  except ValueError:pass
  for fn in (app.editable,app._recipe_save):
   try:fn();raise AssertionError('developer access allowed')
   except ValueError:pass
  try:app._voice_dispatch(dict(action='arm_action',operation='safe_pose'));raise AssertionError('direct arm operation allowed')
  except ValueError:pass
  app.operator_destination.set('LM3');app._operator_preview();app.update();capture_window(app,Path('artifacts/operator_workspace.png'));app._operator_start();assert app.studio_runner.active
  app.ui_role.set('개발자')
  try:app._role_apply();raise AssertionError('active mission role change')
  except ValueError:pass
  assert app.role_active=='사용자'
  now=time.monotonic()
  for _ in range(1800):
   app.sim.tick(.05);now+=.05;app._studio_tick(now,.05)
   if not app.studio_runner.active:break
  assert app.studio_runner.status=='COMPLETED',app.studio_runner.error;assert app.sim.state.last_node=='LM3';assert app.studio_runner.report.metadata['operator_recipe']=='이동만'
  app._operator_tick();app._arm_call.assert_not_called();assert not errors,errors
  # A changed form must invalidate preview even without a widget event.
  app.operator_repeat.set('1');app._operator_preview();app.operator_repeat.set('2')
  try:app._operator_start();raise AssertionError('stale form accepted')
  except ValueError:pass
  # Voice order fills the operator recipe preview, never starts a mission.
  app._voice_submit('LM1에서 LM3로 순환 운반 두 번');assert app.voice_pending[0]['action']=='operator_order';app._voice_execute();assert app.operator_repeat.get()=='2';assert not app.studio_runner.active
  app._operator_open_cycle();app.update();assert app.operator_cycle_panel.winfo_ismapped();assert 'LM1' in app.operator_fields[2]['values'];app.operator_destination.set('LM2');app._operator_cycle_add();app.operator_destination.set('LM1');app._operator_cycle_add();assert list(app.operator_cycle_nodes.get(0,'end'))==['LM2','LM1'];app.operator_repeat.set('2');plan=app._operator_preview();assert plan['actions'][-1]['goal']=='LM3';app._operator_start()
  now=time.monotonic()
  for _ in range(3600):
   app.sim.tick(.05);now+=.05;app._studio_tick(now,.05)
   if not app.studio_runner.active:break
  assert app.studio_runner.status=='COMPLETED',app.studio_runner.error
  assert app.sim.state.last_node=='LM3';assert app.studio_runner.cycle==2
  app._operator_history();app.update();assert not errors,errors
  app.ui_role.set('개발자');app._role_apply();assert app.tabs.tab(app.nodes_page,'state')=='normal';app.tabs.select(app.recipe_page);app.update();capture_window(app,Path('artifacts/operator_recipe_workspace.png'))
  print('PASS operator navigation/recipe preview/completion/report, developer drag reorder, role guard including stale GUI and direct arm, voice recipe fill, no hardware calls')
 finally:app.close()
