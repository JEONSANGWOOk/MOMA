"""Regression: persisted centre mount, shared joints, moving AMR, mission arm I/O."""
import sys,tempfile,math,copy
from pathlib import Path
from unittest.mock import Mock
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parent)]
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.fairino_model import install
from window_capture import capture_window
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json';install(folder,Path(__file__).resolve().parents[2]/'.delivery/frcobot_ros2')
 ui.SETTINGS.write_text(__import__('json').dumps(dict(viewer3d=dict(mount=[0,0,.42,0,0,0]))),encoding='utf-8')
 app=am.Console();errors=[];app.report_callback_exception=lambda t,v,b:errors.append(str(v));app._arm_call=Mock(side_effect=AssertionError('hardware'))
 try:
  app.geometry('1366x768');app.update();assert app.world3d.mount==[.34,0.,.626,0.,0.,0.]
  app.camera_view.set('사진형');app._spatial_camera();app.update()
  q=list(app.arm_dev_sim.q);q[0]+=.15;app._aw_set_synced_joints(q);app._spatial_render()
  assert app.world3d.scene_joint_values==app.arm_dev_sim.kin.positions(q)
  assert app.sim.arm['joint_positions']==app.world3d.scene_joint_values
  app.sim.navigate('LM2');app.arm_dev_program.set('관절 이동 예제');app._aw_play();start=app.sim.state.x;seen=set()
  for _ in range(100):
   app.sim.tick(.05);app.arm_dev_sim.tick(.05);app._aw_sync_main();app._spatial_render();seen.add(tuple(app.world3d.scene_joint_values.values()))
   assert app.world3d.scene_joint_values==app.arm_dev_sim.kin.positions(app.arm_dev_sim.q)
  assert app.sim.state.x>start+.1;assert len(seen)>20
  app.arm_dev_sim.stop();app.sim.stop();app._aw_sync_main()
  # A mission Arm Action operates exactly the same simulator and virtual I/O.
  cfg=copy.deepcopy(app.arm_dev_config);cfg['driver']='fairino';cfg['operations']['shared_on']=dict(method='SetToolDO',id=1,status=1)
  app.studio_config['arm']=cfg;a=dict(type='Arm Action',operation='shared_on',duration_s=.2,timeout_s=10)
  adapter=ui.ConsoleAdapter(app);adapter.begin(a);assert adapter.poll(a,.02,.02)
  assert app.arm_dev_sim.io['ToolDO'][1]==1;assert app.arm_sim_owner=='workspace'
  # Reverse adjustment through shared setter is reflected back into workspace.
  q=list(app.arm_dev_sim.q);q[0]-=.05;app._aw_set_synced_joints(q);app._spatial_render()
  assert app.world3d.scene_joint_values==app.arm_dev_sim.kin.positions(q)
  app._aw_save();app._spatial_save();assert app.studio_config['viewer3d']['assembly_version']=='front-sync-20261005'
  app.update();capture_window(app,Path('artifacts/mobile_arm_synchronized.png'))
  app._arm_call.assert_not_called();assert not errors,errors
  print('PASS centre 0.42m mount migration, shared manual/program joints, simultaneous AMR motion, shared mission I/O and save, no hardware calls')
 finally:app.close()
