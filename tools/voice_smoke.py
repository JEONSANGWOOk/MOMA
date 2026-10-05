"""Offline Tk integration. Synthetic command WAV, real CPU STT/LLM, no microphone/hardware calls."""
import sys,time,tempfile,json,subprocess
from pathlib import Path
from unittest.mock import Mock
sys.path[:0]=[str(Path(__file__).resolve().parents[1])]
import seer_control.app as am
import seer_control.studio_ui as ui
from seer_control.voice_runtime import transcribe,NO_WINDOW
from seer_control.voice_commands import parse_exact
from window_capture import capture_window
cache=Path.home()/'.seer_amr_console/voice'
with tempfile.TemporaryDirectory() as folder:
 am.USER_DIR=Path(folder);ui.SETTINGS=Path(folder)/'settings.json'
 app=am.Console();errors=[];app.report_callback_exception=lambda t,v,b:errors.append(str(v));app._arm_call=Mock(side_effect=AssertionError('hardware'))
 try:
  app.geometry('1366x768');app.update();app.voice_folder=cache;app.voice_talk.set(False);app.tabs.select(app.voice_page)
  # Generate Korean speech to a WAV file only, never the speaker/microphone.
  wav=Path(folder)/'synthetic.wav';code="[Console]::InputEncoding=[System.Text.UTF8Encoding]::new($false); Add-Type -AssemblyName System.Speech; $d=[Console]::In.ReadToEnd() | ConvertFrom-Json; $s=New-Object System.Speech.Synthesis.SpeechSynthesizer; $s.SelectVoice('Microsoft Heami Desktop'); $fmt=New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo -ArgumentList 16000,([System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen),([System.Speech.AudioFormat.AudioChannel]::Mono); $s.SetOutputToWaveFile($d.path,$fmt); $s.Speak($d.text); $s.Dispose()"
  r=subprocess.run(['powershell','-NoProfile','-NonInteractive','-Command',code],input=json.dumps(dict(path=str(wav),text='엘엠 일에서 엘엠 삼으로 이동해'),ensure_ascii=False),text=True,encoding='utf-8',capture_output=True,creationflags=NO_WINDOW);assert r.returncode==0,r.stderr
  t=time.monotonic();text=transcribe(cache,wav);stt=time.monotonic()-t;print('STT:',text,'seconds',round(stt,2))
  cmd=parse_exact(text,list(app.map.nodes),['safe_pose']);assert cmd and cmd['nodes']==['LM3'],cmd
  app.voice_input.set(text);app._voice_submit(text);assert app.studio_runner.active
  app._studio_tick(time.monotonic(),.05);app.sim.tick(.05)
  app.voice_map.redraw();assert app.voice_map.find_withtag('voice_live_route');assert app.voice_map.find_withtag('voice_robot')
  now=time.monotonic()
  for _ in range(1800):
   app.sim.tick(.05);now+=.05;app._studio_tick(now,.05)
   if not app.studio_runner.active:break
  assert app.studio_runner.status=='COMPLETED',app.studio_runner.error;assert app.sim.state.last_node=='LM3';app._voice_tick()
  # Real local LLM inference uses no SDK/network robotics calls.
  app.voice_auto_sim.set(False);app.voice_input.set('목적지를 LM4로 설정하고 출발 부탁해');app._voice_submit(app.voice_input.get())
  deadline=time.monotonic()+60
  while app.voice_busy and time.monotonic()<deadline:app.update();time.sleep(.02)
  assert app.voice_pending and app.voice_pending[0]==dict(action='goto',nodes=['LM4']),app.voice_status.get()
  app.update();capture_window(app,Path('artifacts/voice_llm_workspace.png'))
  app._voice_execute();assert app.studio_runner.active;app.action('cancel');app.arm_dev_sim.stop()
  app._arm_call.assert_not_called();assert not errors,errors
  # Wrong start cannot teleport or launch a mission.
  before=(app.sim.state.x,app.sim.state.y)
  try:app._voice_dispatch(dict(action='goto',nodes=['LM4'],start='LM1'));raise AssertionError('start mismatch accepted')
  except ValueError:pass
  assert (app.sim.state.x,app.sim.state.y)==before
  # Stop invalidates any in-flight LLM result.
  old=app.voice_epoch;app.voice_busy=True;ctx=app._voice_context();app._voice_dispatch({'action':'stop'});app.voice_queue.put((old,ctx,'command',True,({'action':'goto','nodes':['LM4']},.1)));app._voice_tick();assert not app.studio_runner.active and not app.voice_pending and not app.voice_busy
  print('PASS synthetic Korean STT -> LM3 mission completion; CPU LLM -> LM4 preview; wrong-start/stale-stop protection; no microphone/hardware calls')
 finally:app.close()
