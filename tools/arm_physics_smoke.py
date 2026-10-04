"""Offline UI, I/O, grasp, transport, release, collision and restore verification."""
import sys,tempfile,math
from pathlib import Path
from unittest.mock import Mock
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parent)]
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.fairino_model import install
from window_capture import capture_window
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json'
 install(folder,Path(__file__).resolve().parents[2]/'.delivery/frcobot_ros2')
 app=am.Console();errors=[];app.report_callback_exception=lambda t,v,b:errors.append(str(v));app._arm_call=Mock(side_effect=AssertionError('hardware call'))
 try:
  app.geometry('1366x768');app.tabs.select(app.arm_workspace_page);app.update()
  for kind in ('2지 그리퍼','전동 1포트 진공'):
   sim=app.arm_dev_sim;sim.q=[math.radians(v) for v in [0,-90,90,-90,-90,0]];sim.state='IDLE';sim.physics.objects=[];sim.physics.held=None;sim.physics.last_position=None;sim.physics.last_velocity=[0,0,0];sim.physics.pressure=0;sim.physics.opening=.085
   app.arm_gripper_kind.set(kind);app._aps_apply();app._aps_demo();app._aw_play()
   for n in range(6000):
    sim.tick(.01)
    if sim.state not in ('RUNNING','PAUSED'):break
   print(kind,sim.state,sim.error,sim.physics.events,flush=True)
   assert sim.state=='COMPLETED',sim.error
   assert any(e.get('event')=='파지' for e in sim.physics.events)
   assert any(e.get('event')=='I/O OFF 해제' for e in sim.physics.events)
   assert not sim.physics.held
   obj=next(o for o in sim.physics.objects if o['name']=='demo_workpiece');target=next(o for o in sim.physics.objects if o['name']=='demo_place_table')
   for _ in range(100):sim.tick(.01)
   assert math.dist([obj['matrix'][i][3] for i in (0,1)],[target['matrix'][i][3] for i in (0,1)])<.02
   print('Final contacts:',sim.physics.risk,sim.physics.near,flush=True)
   app.arm_physics_tabs.select(app.arm_physics_tab)
   app._aps_refresh();app.arm_dev_canvas.render();app.update();capture_window(app,Path('artifacts/arm_'+('vacuum' if '진공' in kind else 'finger')+'_pickplace.png'))
  # A manual move into a new virtual obstacle must keep the original joint state.
  from seer_control.geometry3d import point,multiply,transform
  original=list(app.arm_dev_sim.q);goal=list(original);goal[0]+=.4
  p=app.arm_dev_sim.physics;leaf=app.arm_dev_sim.kin.tip;center=next(center for name,center,size in p.bounds if name==leaf)
  position=point(app.arm_dev_sim.kin.asset.link_transforms(app.arm_dev_sim.kin.positions(goal))[leaf],center)
  p.add_object('collision_demo','obstacle',[.025,.025,.025],1,[*position,0,0,0],True)
  try:app.arm_dev_sim.set_joints(goal);raise AssertionError('collision was not blocked')
  except ValueError:pass
  assert app.arm_dev_sim.q==original
  app.arm_dev_sim.tick(.01);app._aps_refresh();app.arm_dev_canvas.render();app.update();capture_window(app,Path('artifacts/arm_collision_preview.png'))
  app._aw_save();assert 'physics' in app.studio_config['arm_workspace'];app._arm_call.assert_not_called();assert not errors,errors
  print('PASS both grippers: virtual I/O, confirmed grasp, lift, transfer, release, gravity support, settings save; no hardware calls')
 finally:app.close()
