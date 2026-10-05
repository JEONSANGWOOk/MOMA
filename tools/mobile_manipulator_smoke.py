"""Verify photo-based mobile assembly and its forward arm transform, offline."""
import sys,tempfile
from pathlib import Path
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parent)]
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.fairino_model import install
from window_capture import capture_window
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json';install(folder,Path(__file__).resolve().parents[2]/'.delivery/frcobot_ros2')
 app=am.Console();errors=[];app.report_callback_exception=lambda t,v,b:errors.append(str(v))
 try:
  app.geometry('1366x768');app.update();app.view_mode.set('3D');app._spatial_switch();v=app.world3d
  assert v.body_asset.name.endswith('Mobile Manipulator')
  assert v.mount==[.31,0.,.608,0.,0.,0.]
  frame=v.body_asset.link_transforms({})['arm_mount_link'];assert all(abs(frame[i][3]-v.mount[i])<1e-9 for i in range(3))
  for legacy in (.182,.308,.4285):
   app.studio_config['viewer3d']=dict(mount=[0,0,legacy,0,0,0]);app._spatial_restore();assert v.mount==[.31,0.,.608,0.,0.,0.]
  app.studio_config['viewer3d']=dict(mount=[.25,0,.62,0,0,0]);app._spatial_restore();assert v.mount==[.25,0,.62,0,0,0]
  app.camera_view.set('사진형');app._spatial_camera()
  v.mount=[.31,0.,.608,0.,0.,0.];v.layers['지도'].set(False);v.layers['좌표축'].set(False)
  v.camera.target=[app.sim.state.x,app.sim.state.y,.75];v.camera.distance=2.6;v.camera.yaw=135;v.camera.pitch=18
  app._spatial_render();app.update();capture_window(app,Path('artifacts/photo_mobile_manipulator.png'))
  assert not errors,errors
  print('PASS photo cabinet assembly, forward mount, legacy migration, custom calibration preserved, offline GPU rendering')
 finally:app.close()
