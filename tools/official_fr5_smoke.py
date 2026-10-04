"""Render and animate the actual downloaded official ROS2 FR5 model, offline."""
import sys,tempfile,time,math,argparse
from pathlib import Path
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.fairino_model import install,installed
from window_capture import capture_window
parser=argparse.ArgumentParser();parser.add_argument('--repo',required=True);args=parser.parse_args()
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json';path=install(folder,args.repo);app=am.Console();errors=[]
 app.report_callback_exception=lambda t,v,b:errors.append(str(v))
 try:
  app.pose_autosave.set(False);app.tabs.select(app.arm_workspace_page);app.geometry('1366x768');app.update()
  assert app.arm_dev_sim.kin.asset.name=='fairino5_v6_robot';assert len(app.arm_dev_sim.kin.joints)==6
  assert all('메시 단순화' in v for v in app.arm_dev_sim.kin.asset.warnings)
  assert all(app.arm_dev_sim.kin.asset.links.values());app._arm_call=Mock(side_effect=AssertionError('Hardware call'))
  app.arm_dev_program.set('관절 이동 예제');app._aw_play()
  for _ in range(100):app.arm_dev_sim.tick(.1)
  assert app.arm_dev_sim.state=='COMPLETED',app.arm_dev_sim.error
  current=app.arm_dev_sim.kin.pose(app.arm_dev_sim.q);target=list(current);target[2]+=10
  app.arm_dev_tcp.set(','.join(str(v) for v in target));app._aw_tcp_move()
  for _ in range(100):app.arm_dev_sim.tick(.03)
  assert app.arm_dev_sim.state=='COMPLETED',app.arm_dev_sim.error
  assert math.dist(app.arm_dev_sim.kin.pose(app.arm_dev_sim.q)[:3],target[:3])<1
  app._aw_use_official(path);app.update();app.arm_dev_canvas.render()
  for _ in range(3):app.update();time.sleep(.03)
  capture_window(app,Path('artifacts/official_fr5_workspace.png'))
  app._arm_call.assert_not_called();assert not errors,errors
  print('PASS: official FR5 package paths and seven meshes, 6 joints, MoveJ/MoveL, default and shared 3D model; no hardware calls')
 finally:app.close()
