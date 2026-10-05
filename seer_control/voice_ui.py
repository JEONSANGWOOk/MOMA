"""Push-to-talk / text commands: validated plans feed existing mission adapters."""
import copy,json,math,queue,tempfile,threading,time
from pathlib import Path
import tkinter as tk
from tkinter import ttk
from .voice_commands import parse_exact,validate_command,describe
from .voice_runtime import Recorder,transcribe,interpret,request,speak
from .voice_setup import install_whisper,install_ollama,start_server,pull,stop_server
from .fairino_api import validate as validate_fr5
from .fairino_programs import program_actions
from .theme import PANEL,INK,MUTED,BLUE,RED
class VoiceMixin:
 def _voice_build(self):
  from .app import USER_DIR
  self.voice_folder=Path(USER_DIR)/'voice';self.voice_recorder=Recorder();self.voice_queue=queue.Queue();self.voice_epoch=0;self.voice_busy=False;self.voice_closed=False;self.voice_server=None;self.voice_tts=None;self.voice_pending=None;self.voice_started=0.;self.voice_record_context=None;self.voice_active_mission=False
  cfg=self.studio_config.get('voice',{})
  self.voice_input=tk.StringVar();self.voice_status=tk.StringVar(value='마이크 버튼을 누르고 말하세요. 모델은 처음 한 번 설치합니다.')
  self.voice_llm=tk.BooleanVar(value=cfg.get('llm',True));self.voice_auto_sim=tk.BooleanVar(value=cfg.get('auto_sim',True));self.voice_talk=tk.BooleanVar(value=cfg.get('talk',True));self.voice_real=tk.BooleanVar(value=False)
  self.voice_url=tk.StringVar(value=cfg.get('url','http://127.0.0.1:12634'));self.voice_model=tk.StringVar(value=cfg.get('model','qwen3:1.7b'))
  page=ttk.Frame(self.tabs);self.tabs.add(page,text='음성 / LLM 명령');self._register_navigation(page,'음성 / LLM 명령','🎙');self._sync_navigation();self.voice_page=page
  holder,body,_=self._scrollable_frame(page,bg=PANEL);holder.pack(fill='both',expand=True)
  self._studio_note(body,'내장 마이크로 입력 → 명령/노드 확인 → 기존 주행·팔 작업 실행. 스피커로 응답. 상시 녹음하지 않습니다. 마이크 녹음은 최대 10초입니다.')
  row=self._studio_row(body)
  self.voice_record_button=self.button(row,'🎙 녹음 시작 / 인식',lambda:self.guarded(self._voice_record),BLUE);self.voice_record_button.pack(side='left',padx=3)
  self.button(row,'모두 정지',lambda:self.guarded(lambda:self._voice_dispatch({'action':'stop'})),RED).pack(side='left',padx=3)
  self.button(row,'지도 보기',lambda:self.tabs.select(self.operation_page)).pack(side='left',padx=3)
  ttk.Label(body,textvariable=self.voice_status,wraplength=1100).pack(fill='x',padx=14,pady=8)
  row=self._studio_row(body);ttk.Entry(row,textvariable=self.voice_input,width=65).pack(side='left',fill='x',expand=True)
  self.button(row,'명령 해석',lambda:self.guarded(lambda:self._voice_submit(self.voice_input.get()))).pack(side='left',padx=3)
  self.button(row,'미리보기 명령 실행',lambda:self.guarded(self._voice_execute),BLUE).pack(side='left',padx=3)
  for title,var in [('불분명한 표현은 로컬 LLM으로 해석',self.voice_llm),('SIM 명령 자동 실행',self.voice_auto_sim),('스피커 음성 응답',self.voice_talk),('이번 실행에서 REAL 음성 명령 허용 (미리보기 후 실행 버튼)',self.voice_real)]:
   tk.Checkbutton(body,text=title,variable=var,bg=PANEL,command=self._voice_save).pack(anchor='w',padx=14)
  self._studio_note(body,'예: LM3로 이동해 / LM1에서 LM3로 이동해 / LM2, LM7, LM8 순서로 이동해 / LM1, LM3 세 바퀴 돌아 / 현재 위치 알려줘 / 미션 일시정지 / 미션 재개 / 미션 취소 / 로봇팔 안전 자세로 이동해. 그 밖의 팔 작업은 등록된 작업명으로 요청하세요.')
  self.voice_preview=tk.Text(body,height=6,wrap='word',bg='#ffffff',fg=INK);self.voice_preview.pack(fill='x',padx=14,pady=5);self.voice_preview.configure(state='disabled')
  row=self._studio_row(body)
  self.button(row,'음성 인식 설치/확인',lambda:self.guarded(lambda:self._voice_install(False))).pack(side='left',padx=3)
  self.button(row,'로컬 LLM 설치/모델 준비',lambda:self.guarded(lambda:self._voice_install(True))).pack(side='left',padx=3)
  self.button(row,'LLM 연결 확인',lambda:self.guarded(self._voice_check)).pack(side='left',padx=3)
  row=self._studio_row(body);ttk.Label(row,text='로컬 LLM 주소').pack(side='left');ttk.Entry(row,textvariable=self.voice_url,width=30).pack(side='left',padx=4);ttk.Label(row,text='모델').pack(side='left');ttk.Entry(row,textvariable=self.voice_model,width=22).pack(side='left',padx=4);self.button(row,'설정 저장',self._voice_save).pack(side='left',padx=4)
  self._studio_note(body,'기본: 한국어 Whisper small + Qwen3 1.7B CPU. 설치 시 인터넷 다운로드, 이후 인식/명령 해석은 노트북에서 처리합니다. 음성 파일은 인식 후 삭제합니다. LLM 주소는 로컬만 허용합니다. REAL은 기존 제어권·등록 작업·안전 조건을 유지합니다. 음성 정지는 물리 비상정지 장치를 대신하지 않습니다.')
  if (self.voice_folder/'ggml-small-q5_1.bin').is_file():self.voice_status.set('한국어 음성 인식 준비됨 · 녹음 버튼을 누르고 말하세요.')
  self.voice_after=self.after(100,self._voice_tick)
 def _voice_save(self):
  self.studio_config['voice']=dict(llm=self.voice_llm.get(),auto_sim=self.voice_auto_sim.get(),talk=self.voice_talk.get(),url=self.voice_url.get(),model=self.voice_model.get());self._studio_save_settings()
 def _voice_context(self):return (self.real,self.generation,id(self.map),self.host.get())
 def _voice_operations(self):
  cfg=self.studio_config['arm'] if self.real or self.studio_config['arm'].get('driver')=='fairino' else self.arm_dev_config
  return cfg,list(cfg.get('operations',{}))+list(cfg.get('programs',{}))
 def _voice_job(self,fn,kind):
  if self.voice_busy:raise ValueError('음성/LLM 처리 중입니다. 정지는 언제든 누를 수 있습니다.')
  self.voice_busy=True;self.voice_epoch+=1;epoch=self.voice_epoch;context=self._voice_context()
  def work():
   try:self.voice_queue.put((epoch,context,kind,True,fn()))
   except Exception as e:self.voice_queue.put((epoch,context,kind,False,str(e)))
  threading.Thread(target=work,daemon=True,name='voice-'+kind).start()
 def _voice_install(self,llm):
  url=self.voice_url.get();model=self.voice_model.get();epoch=self.voice_epoch+1
  def progress(text):self.voice_queue.put((epoch,None,'progress',True,text))
  def work():
   install_whisper(self.voice_folder,progress)
   if self.voice_closed:raise ValueError('종료됨')
   if llm:
    install_ollama(self.voice_folder,progress)
    if self.voice_closed:raise ValueError('종료됨')
    server=start_server(self.voice_folder,url)
    if self.voice_closed:
     stop_server(server);raise ValueError('종료됨')
    if server:self.voice_server=server
    pull(url,model,progress)
   return '로컬 LLM 모델 준비 완료' if llm else '한국어 음성 인식 준비 완료'
  self.voice_status.set('모델 설치 중 · 최초 다운로드에 시간이 걸립니다.');self._voice_job(work,'setup')
 def _voice_check(self):
  url=self.voice_url.get();model=self.voice_model.get()
  def work():
   server=start_server(self.voice_folder,url)
   if self.voice_closed:
    stop_server(server);raise ValueError('종료됨')
   if server:self.voice_server=server
   tags=request(url,'/api/tags',timeout=10);names=[m['name'] for m in tags.get('models',[])]
   if model not in names:raise ValueError('서버 연결됨 / 모델 없음 · 로컬 LLM 설치/모델 준비를 누르세요.')
   return 'LLM 연결됨 · '+model
  self._voice_job(work,'setup')
 def _voice_record(self):
  if self.voice_recorder.handle:return self._voice_finish_record()
  if self.voice_busy:raise ValueError('현재 처리가 끝난 뒤 녹음하세요.')
  if not next(iter((self.voice_folder/'whisper').rglob('whisper-cli.exe')),None) or not (self.voice_folder/'ggml-small-q5_1.bin').is_file():raise ValueError('음성 인식 설치/확인 버튼을 먼저 누르세요.')
  if self.voice_tts and self.voice_tts.poll() is None:self.voice_tts.terminate()
  self.voice_recorder.start();self.voice_record_context=self._voice_context();self.voice_started=time.monotonic();self.voice_status.set('녹음 중 · 명령을 말하고 버튼을 다시 누르세요. 최대 10초.');self.voice_record_button.configure(text='녹음 완료 → 인식')
 def _voice_finish_record(self):
  if self.voice_record_context!=self._voice_context():
   self.voice_recorder.close();self.voice_record_button.configure(text='🎙 녹음 시작 / 인식');raise ValueError('녹음 중 모드/지도/연결 변경 · 다시 녹음하세요.')
  folder=tempfile.TemporaryDirectory();wav=Path(folder.name)/'command.wav'
  try:self.voice_recorder.finish(wav)
  except Exception:folder.cleanup();raise
  finally:self.voice_record_button.configure(text='🎙 녹음 시작 / 인식')
  def work():
   try:return transcribe(self.voice_folder,wav)
   finally:folder.cleanup()
  self.voice_status.set('한국어 음성 인식 중…');self._voice_job(work,'speech')
 def _voice_submit(self,text):
  nodes=list(self.map.nodes);_,operations=self._voice_operations();command=parse_exact(text,nodes,operations)
  if command and command['action']=='stop':return self._voice_dispatch(command)
  if self.voice_busy:raise ValueError('이전 명령을 처리 중입니다.')
  self.voice_pending=None
  if command:return self._voice_accept(command,'명령 직접 인식')
  if not self.voice_llm.get():raise ValueError('명령을 명확히 다시 말하거나 로컬 LLM 해석을 켜세요.')
  url=self.voice_url.get();model=self.voice_model.get();self.voice_status.set('로컬 LLM이 명령을 해석 중…')
  def work():
   server=start_server(self.voice_folder,url)
   if self.voice_closed:
    stop_server(server);raise ValueError('종료됨')
   if server:self.voice_server=server
   start=time.monotonic();command=interpret(url,model,text,nodes,operations)
   return command,round(time.monotonic()-start,2)
  self._voice_job(work,'command')
 def _voice_accept(self,command,source):
  _,ops=self._voice_operations();command=validate_command(command,self.map.nodes,ops)
  self.voice_pending=(command,self._voice_context(),time.monotonic()+30)
  self.voice_preview.configure(state='normal');self.voice_preview.delete('1.0','end');self.voice_preview.insert('1.0',source+'\n'+describe(command,self.real));self.voice_preview.configure(state='disabled')
  self.voice_status.set(source+' · 명령 미리보기 준비 (30초 유효)')
  if command['action'] in ('stop','status'):return self._voice_execute()
  if not self.real and self.voice_auto_sim.get():self._voice_execute()
 def _voice_execute(self):
  if not self.voice_pending:raise ValueError('실행할 명령을 먼저 해석하세요.')
  command,context,expires=self.voice_pending
  if context!=self._voice_context() or time.monotonic()>expires:self.voice_pending=None;raise ValueError('모드/지도/연결 변경 또는 명령 유효시간 초과. 다시 해석하세요.')
  _,ops=self._voice_operations();command=validate_command(command,self.map.nodes,ops)
  if self.real and command['action'] not in ('stop','status') and not self.voice_real.get():raise ValueError('REAL 음성 명령 허용을 먼저 켜세요. 명령 미리보기를 확인하고 실행합니다.')
  self._voice_dispatch(command);self.voice_pending=None
 def _voice_response(self,text):
  self.voice_status.set(text)
  if self.voice_talk.get() and not self.voice_recorder.handle:
   if self.voice_tts and self.voice_tts.poll() is None:self.voice_tts.terminate()
   self.voice_tts=speak(text);self.voice_tts.stdin.write(text);self.voice_tts.stdin.close()
 def _voice_dispatch(self,command):
  _,ops=self._voice_operations();command=validate_command(command,self.map.nodes,ops);action=command['action']
  if action=='stop':
   self.voice_epoch+=1;self.voice_pending=None;self.voice_recorder.close();self.voice_record_button.configure(text='🎙 녹음 시작 / 인식');self._pad_stop();self.action('stop')
   if self.real and self.fr5_client.connected:self._fr5_stop()
   if not self.real:self.arm_dev_sim.stop();self._aw_sync_main()
   return self._voice_response('정지 명령을 요청했습니다.')
  if action=='status':
   if not self.connected or self.real and time.monotonic()-self.last_state>3:return self._voice_response('현재 상태 수신이 없습니다. 로봇 연결을 확인하세요.')
   state=self.current_state();node=state.get('last_node') or self.map.nearest(state.get('x',0),state.get('y',0))
   return self._voice_response(f"현재 위치 {node}. 배터리 {state.get('battery','알 수 없음')} 퍼센트. 상태 {state.get('mode','알 수 없음')}.")
  if action in ('pause','resume','cancel'):
   self.action(action);return self._voice_response({'pause':'일시정지 요청','resume':'재개 요청','cancel':'미션 취소 요청'}[action])
  if not self.connected or not self.real and not self.sim_powered:raise ValueError('로봇 연결/전원이 꺼져 있습니다.')
  if self.studio_runner.active or self.task_running or self.sim.route or self.held or self.arm_dev_sim.state in ('RUNNING','PAUSED'):raise ValueError('현재 주행/팔 작업을 종료한 뒤 새 명령을 실행하세요.')
  state=self.current_state()
  if self.real:
   if not self.control_enabled or time.monotonic()-self.last_state>3:raise ValueError('최신 실기 상태와 제어권이 필요합니다.')
   if state.get('emergency') or state.get('stopped') or state.get('task') in ('RUNNING','WAITING','SUSPENDED') or abs(float(state.get('speed',0)))>.02:raise ValueError('실기 정지/안전 상태를 먼저 확인하세요.')
  start=command.get('start')
  if start:
   node=self.map.nodes[start]
   if math.hypot(state['x']-node['x'],state['y']-node['y'])>.25:raise ValueError(start+'에 있지 않습니다. 경유하려면 "'+start+'를 거쳐 이동"이라고 말하세요.')
  if action=='arm_action':
   cfg,_=self._voice_operations()
   if self.real:
    validate_fr5(cfg,True)
    if not self.fr5_client.connected or time.monotonic()-self.fr5_rx>2:raise ValueError('FR5 연결/최신 상태가 필요합니다.')
   name=command['operation'];actions=program_actions(cfg,name) if name in cfg.get('programs',{}) else [dict(type='Arm Action',operation=name,timeout_s=120)];repeat=1;message=name+' 팔 작업을 등록했습니다.'
  else:
   if self.real and not self.studio_arm_safe:raise ValueError('로봇팔 안전 자세를 확인하세요.')
   nodes=list(command['nodes'])
   if self.real and any(n not in self.robot_stations for n in nodes):raise ValueError('실기 Stations에 없는 노드입니다.')
   if action=='loop' and len(nodes)<2:raise ValueError('순환은 노드 두 개 이상을 말하세요.')
   if action=='loop' and nodes[-1]!=nodes[0]:nodes.append(nodes[0])
   actions=[dict(type='Path Nav',goal=n,timeout_s=300) for n in nodes];repeat=command.get('repeats',1);message=' → '.join(nodes)+' 이동 명령을 등록했습니다.'
  self._pad_stop();self.pad_enabled.set(False);self.pad_gate.reset();self.studio_runner.start(actions,repeat);self.task_running=True;self.studio_charge_inhibit=False;self.voice_active_mission=True;self._voice_response(message)
 def _voice_tick(self):
  if self.voice_closed:return
  try:
   if self.voice_recorder.handle and (self.voice_recorder.done() or time.monotonic()-self.voice_started>=10):self._voice_finish_record()
   while True:
    try:epoch,context,kind,ok,result=self.voice_queue.get_nowait()
    except queue.Empty:break
    if epoch!=self.voice_epoch:
     if kind!='progress':self.voice_busy=False
     continue
    if kind=='progress':self.voice_status.set(result);continue
    self.voice_busy=False
    if context!=self._voice_context():self.voice_status.set('처리 중 지도/모드/연결 변경 · 명령 폐기');continue
    if not ok:self.voice_status.set(result);continue
    if kind=='speech':self.voice_input.set(result);self._voice_submit(result)
    elif kind=='command':self._voice_accept(result[0],f'로컬 LLM 해석 {result[1]}초')
    else:self.voice_status.set(result)
   if self.voice_active_mission and not self.studio_runner.active:
    self.voice_active_mission=False;self._voice_response('미션 '+self.studio_runner.status+(' · '+self.studio_runner.error if self.studio_runner.error else ''))
  except Exception as e:self.voice_status.set(str(e))
  self.voice_after=self.after(100,self._voice_tick)
 def _voice_close(self):
  if not hasattr(self,'voice_closed'):return
  self.voice_closed=True;self.voice_epoch+=1;self.voice_recorder.close()
  if self.voice_after:self.after_cancel(self.voice_after)
  if self.voice_tts and self.voice_tts.poll() is None:self.voice_tts.terminate()
  stop_server(self.voice_server)
