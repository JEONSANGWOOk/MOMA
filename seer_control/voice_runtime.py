"""Offline Windows WAV capture, whisper.cpp recognition and loopback Ollama."""
import array,ctypes,json,math,os,subprocess,sys,tempfile,time,urllib.request,urllib.parse,wave
from pathlib import Path
from .voice_commands import command_schema,llm_messages,validate_command,ground_command
NO_WINDOW=getattr(subprocess,'CREATE_NO_WINDOW',0)
class WaveFormat(ctypes.Structure):
 _fields_=[('tag',ctypes.c_ushort),('channels',ctypes.c_ushort),('rate',ctypes.c_uint32),('bytes_per_sec',ctypes.c_uint32),('align',ctypes.c_ushort),('bits',ctypes.c_ushort),('extra',ctypes.c_ushort)]
class WaveHeader(ctypes.Structure):
 _fields_=[('data',ctypes.c_void_p),('length',ctypes.c_uint32),('recorded',ctypes.c_uint32),('user',ctypes.c_size_t),('flags',ctypes.c_uint32),('loops',ctypes.c_uint32),('next',ctypes.c_void_p),('reserved',ctypes.c_size_t)]
class Recorder:
 def __init__(self):self.handle=None;self.api=None;self.buffer=None;self.header=None
 def start(self):
  if sys.platform!='win32':raise ValueError('내장 마이크 녹음은 Windows에서 지원합니다.')
  if self.handle:raise ValueError('이미 녹음 중입니다.')
  self.api=ctypes.WinDLL('winmm');self.api.waveInOpen.argtypes=[ctypes.POINTER(ctypes.c_void_p),ctypes.c_uint32,ctypes.POINTER(WaveFormat),ctypes.c_size_t,ctypes.c_size_t,ctypes.c_uint32]
  for name in ('waveInPrepareHeader','waveInAddBuffer','waveInUnprepareHeader'):getattr(self.api,name).argtypes=[ctypes.c_void_p,ctypes.POINTER(WaveHeader),ctypes.c_uint32]
  for name in ('waveInStart','waveInStop','waveInReset','waveInClose'):getattr(self.api,name).argtypes=[ctypes.c_void_p]
  fmt=WaveFormat(1,1,16000,32000,2,16,0);handle=ctypes.c_void_p()
  if self.api.waveInOpen(ctypes.byref(handle),0xffffffff,ctypes.byref(fmt),0,0,0):raise ValueError('기본 마이크를 열 수 없습니다. Windows 마이크 권한/입력 장치를 확인하세요.')
  self.handle=handle;self.buffer=ctypes.create_string_buffer(32000*10);self.header=WaveHeader(ctypes.cast(self.buffer,ctypes.c_void_p),len(self.buffer),0,0,0,0,None,0)
  try:
   for name in ('waveInPrepareHeader','waveInAddBuffer'):
    if getattr(self.api,name)(handle,ctypes.byref(self.header),ctypes.sizeof(self.header)):raise ValueError('마이크 버퍼 준비 실패')
   if self.api.waveInStart(handle):raise ValueError('녹음 시작 실패')
  except Exception:self.close();raise
 def finish(self,path):
  if not self.handle:raise ValueError('녹음 중이 아닙니다.')
  self.api.waveInStop(self.handle);self.api.waveInReset(self.handle);count=self.header.recorded;pcm=self.buffer.raw[:count-count%2];self.close()
  if len(pcm)<3200:raise ValueError('녹음이 너무 짧습니다. 명령을 말한 뒤 인식 버튼을 누르세요.')
  samples=array.array('h',pcm)
  if math.sqrt(sum(v*v for v in samples)/len(samples))<40:raise ValueError('마이크 입력이 너무 작습니다. 입력 장치와 음량을 확인하세요.')
  with wave.open(str(path),'wb') as f:f.setnchannels(1);f.setsampwidth(2);f.setframerate(16000);f.writeframes(pcm)
 def close(self):
  if self.handle:
   self.api.waveInReset(self.handle)
   if self.header:self.api.waveInUnprepareHeader(self.handle,ctypes.byref(self.header),ctypes.sizeof(self.header))
   self.api.waveInClose(self.handle);self.handle=None
 def done(self):return bool(self.header and self.header.flags&1)
def transcribe(folder,wav):
 folder=Path(folder);exe=next(iter((folder/'whisper').rglob('whisper-cli.exe')),None);model=folder/'ggml-small-q5_1.bin'
 if not exe or not model.is_file():raise ValueError('먼저 음성 인식 설치 버튼을 누르세요.')
 with tempfile.TemporaryDirectory() as tmp:
  prefix=Path(tmp)/'transcript'
  args=[str(exe),'-m',str(model),'-f',str(wav),'-l','ko','-t','4','-bs','5','-bo','5','-tp','0','-otxt','-of',str(prefix),'-nt','-np','--prompt','AMR 로봇 명령. 노드 LM1 LM2 LM3 LM4 LM5 LM6 LM7 LM8 CP1. 이동, 정지, 미션 취소, 로봇팔 안전 자세.']
  result=subprocess.run(args,capture_output=True,timeout=90,creationflags=NO_WINDOW)
  if result.returncode:raise ValueError('음성 인식 실패: '+result.stderr.decode('utf-8',errors='replace')[-400:])
  target=prefix.with_suffix('.txt');text=target.read_text(encoding='utf-8').strip() if target.is_file() else ''
  if not text:raise ValueError('음성을 인식하지 못했습니다. 마이크 가까이서 다시 말하세요.')
  return text
def local_url(url):
 p=urllib.parse.urlsplit(url)
 if p.scheme!='http' or p.hostname not in ('127.0.0.1','localhost','::1') or p.username or p.password or p.query or p.fragment:raise ValueError('LLM 주소는 로컬 HTTP만 지원합니다.')
 return url.rstrip('/')
def request(url,path,data=None,timeout=120):
 body=json.dumps(data).encode() if data is not None else None
 req=urllib.request.Request(local_url(url)+path,data=body,headers={'Content-Type':'application/json'})
 with urllib.request.urlopen(req,timeout=timeout) as r:return json.load(r)
def interpret(url,model,text,nodes,operations):
 result=request(url,'/api/chat',dict(model=model,messages=llm_messages(text,nodes,operations),stream=False,think=False,format=command_schema(nodes,operations),options=dict(temperature=0,num_predict=180,num_ctx=2048,num_thread=4,num_gpu=0),keep_alive='2m'))
 return ground_command(validate_command(json.loads(result['message']['content']),nodes,operations),text)
def speak(text):
 # Feed plain data on stdin; operator text is never interpolated into PowerShell code.
 code="[Console]::InputEncoding=[System.Text.UTF8Encoding]::new($false); Add-Type -AssemblyName System.Speech; $voiceText=[Console]::In.ReadToEnd(); $s=New-Object System.Speech.Synthesis.SpeechSynthesizer; $v=$s.GetInstalledVoices() | Where-Object {$_.VoiceInfo.Culture.Name -eq 'ko-KR'} | Select-Object -First 1; if($v){$s.SelectVoice($v.VoiceInfo.Name)}; $s.Speak($voiceText); $s.Dispose()"
 return subprocess.Popen(['powershell','-NoProfile','-NonInteractive','-Command',code],stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,text=True,encoding='utf-8',creationflags=NO_WINDOW)
