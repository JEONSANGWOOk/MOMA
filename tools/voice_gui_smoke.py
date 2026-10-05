"""Offline real Tk GUI capability checks; hardware calls forbidden."""
import sys,tempfile,time
from pathlib import Path
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.voice_gui import GuiRegistry
from seer_control.voice_runtime import interpret
from seer_control.voice_setup import start_server,stop_server
from window_capture import capture_window
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json'
 app=am.Console();errors=[];app.report_callback_exception=lambda t,v,b:errors.append(str(v));app._arm_call=Mock(side_effect=AssertionError('hardware'));app.voice_talk.set(False)
 server=None
 try:
  app.geometry('1366x768');app.update();app.voice_gui=GuiRegistry(app);gui=app.voice_gui
  print('GUI targets:',len(gui.targets),'kinds:',sorted(set(r['kind'] for r in gui.targets.values())))
  row=next(r for r in gui.targets.values() if r['kind']=='choice' and set(['2D','3D']).issubset(r.get('values',[]))) if any(r['kind']=='choice' and set(['2D','3D']).issubset(r.get('values',[])) for r in gui.targets.values()) else None
  row=next(r for r in gui.targets.values() if r['kind']=='screen' and r['label'].endswith('노드 / 경로'))
  cmd=dict(action='gui',target=row['id'],operation='select');app._voice_accept(cmd,'GUI 화면 테스트');assert app.voice_pending;app._voice_execute();app.update();assert app.tabs.select()==str(app.nodes_page)
  target=next(r for r in gui.targets.values() if r['kind']=='choice' and 'J1 / J2' in r.get('values',[]))
  app._voice_accept(dict(action='gui',target=target['id'],operation='set',value='J3 / J4'),'팔 모드 테스트');assert app.pad_arm_group.get()=='J1 / J2';app._voice_execute();assert app.pad_arm_group.get()=='J3 / J4'
  target=gui.targets['map::노드 선택'];app._voice_accept(dict(action='gui',target=target['id'],operation='set',value='LM3'),'노드 선택 테스트');app._voice_execute()
  cache=Path.home()/'.seer_amr_console/voice';server=start_server(cache,'http://127.0.0.1:12634')
  text='수행 보고서 보여줘';candidates=gui.candidates(text);cmd=interpret('http://127.0.0.1:12634','qwen3:1.7b',text,list(app.map.nodes),['safe_pose'],gui,candidates)
  assert cmd['action']=='gui',cmd;print('CPU LLM GUI:',gui.describe(cmd))
  # Do not open a modal dialog during smoke; prove preview cannot auto-invoke it.
  app._voice_accept(cmd,'LLM GUI 해석');assert app.voice_pending
  app.tabs.select(app.voice_page);app.update();capture_window(app,Path('artifacts/voice_gui_workspace.png'))
  app._voice_gui_catalog();app.update();assert not errors,errors;app._arm_call.assert_not_called()
  print('PASS screen navigation, GUI value preview/apply, node selection, CPU LLM GUI interpretation; no hardware calls')
 finally:stop_server(server);app.close()
