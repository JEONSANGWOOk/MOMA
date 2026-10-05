"""Official portable CPU runtimes in user cache; no system installer/autostart."""
import hashlib,io,json,os,shutil,subprocess,time,urllib.request,urllib.parse,zipfile
from pathlib import Path,PurePosixPath
from .voice_runtime import NO_WINDOW,request,local_url
WHISPER='v1.9.2';OLLAMA='v0.35.1'
def json_url(url):
 with urllib.request.urlopen(urllib.request.Request(url,headers={'User-Agent':'MOMA-voice-setup'}),timeout=30) as f:return json.load(f)
def asset(repo,version,name):
 value=json_url('https://api.github.com/repos/'+repo+'/releases/tags/'+version)
 return next(a for a in value['assets'] if a['name']==name)
def download(url,target,progress=lambda s:None,sha=None):
 target=Path(target);target.parent.mkdir(parents=True,exist_ok=True);tmp=target.with_suffix(target.suffix+'.part')
 h=hashlib.sha256();last=-1
 try:
  with urllib.request.urlopen(url,timeout=120) as f,tmp.open('wb') as out:
   size=int(f.headers.get('Content-Length',0));total=0
   while data:=f.read(1024*1024):
    out.write(data);h.update(data);total+=len(data);percent=int(total*100/size) if size else int(total/1048576)
    if percent!=last:progress(f'다운로드 {percent}'+('%' if size else ' MB'));last=percent
  if sha and h.hexdigest()!=sha:raise ValueError('다운로드 SHA256 검증 실패')
  tmp.replace(target)
 finally:
  if tmp.exists():tmp.unlink()
 return h.hexdigest()
def safe_target(folder,name):
 p=PurePosixPath(name.replace('\\','/'))
 if p.is_absolute() or '..' in p.parts or ':' in name:raise ValueError('ZIP 경로 오류')
 return Path(folder).joinpath(*p.parts)
class RangeZip(io.RawIOBase):
 def __init__(self,url,size):self.url=url;self.size=size;self.position=0;self.cache={}
 def seekable(self):return True
 def readable(self):return True
 def tell(self):return self.position
 def seek(self,n,whence=0):
  self.position=n if whence==0 else self.position+n if whence==1 else self.size+n
  return self.position
 def read(self,n=-1):
  n=min(self.size-self.position,n if n>=0 else self.size-self.position)
  if n<=0:return b''
  start=self.position;end=start+n-1;key=(start,end)
  if key not in self.cache:
   req=urllib.request.Request(self.url+'?moma_range='+str(start)+'_'+str(end),headers={'Range':f'bytes={start}-{end}'})
   with urllib.request.urlopen(req,timeout=120) as f:
    if f.status!=206 or f.headers.get('Content-Range','').split('/')[0]!=f'bytes {start}-{end}':raise ValueError('부분 다운로드를 지원하지 않는 서버입니다.')
    value=f.read(n+1)
    if len(value)!=n:raise ValueError('부분 다운로드 길이 오류')
   if n<2*1024*1024:self.cache[key]=value
  else:value=self.cache[key]
  self.position+=n;return value
 def close(self):self.cache.clear();super().close()
def install_whisper(folder,progress=lambda s:None):
 folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
 if not next(iter((folder/'whisper').rglob('whisper-cli.exe')),None):
  item=asset('ggml-org/whisper.cpp',WHISPER,'whisper-bin-x64.zip');archive=folder/'whisper.zip'
  digest=item.get('digest','').removeprefix('sha256:') or None
  download(item['browser_download_url'],archive,progress,digest)
  with zipfile.ZipFile(archive) as z:
   for entry in z.infolist():
    if entry.is_dir():continue
    target=safe_target(folder/'whisper',entry.filename);target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(z.read(entry))
  archive.unlink()
 model=folder/'ggml-base-q5_1.bin'
 if not model.is_file():
  values=json_url('https://huggingface.co/api/models/ggerganov/whisper.cpp/tree/main')
  entry=next(x for x in values if x.get('path')==model.name);digest=entry.get('lfs',{}).get('oid')
  download('https://huggingface.co/ggerganov/whisper.cpp/resolve/main/'+model.name,model,progress,digest)
 return folder

def cpu_entry(name):
 p=PurePosixPath(name);low=name.lower()
 if low=='ollama.exe' or low=='lib/ollama/llama-server.exe':return True
 if p.parent.as_posix()=='lib/ollama' and p.suffix.lower()=='.dll':return not any(x in p.name.lower() for x in ('cuda','cublas','cudart','hip','rocm','vulkan','mlx'))
 return False

def install_ollama(folder,progress=lambda s:None):
 folder=Path(folder);exe=folder/'ollama'/'ollama.exe'
 if exe.is_file() and (exe.parent/'lib/ollama/llama-server.exe').is_file():return exe
 item=asset('ollama/ollama',OLLAMA,'ollama-windows-amd64.zip')
 with RangeZip(item['browser_download_url'],item['size']) as remote,zipfile.ZipFile(remote) as z:
  entries=[e for e in z.infolist() if cpu_entry(e.filename)]
  if not any(e.filename=='ollama.exe' for e in entries):raise ValueError('Ollama 실행 파일 없음')
  for e in entries:
   progress('Ollama CPU 설치 · '+e.filename);data=z.read(e);target=safe_target(exe.parent,e.filename);target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
 (exe.parent/'portable_cpu.json').write_text(json.dumps(dict(version=OLLAMA,files=[e.filename for e in entries])),encoding='utf-8')
 return exe

def start_server(folder,url):
 url=local_url(url)
 try:request(url,'/api/version',timeout=2);return None
 except Exception:pass
 exe=Path(folder)/'ollama/ollama.exe'
 if not exe.is_file():raise ValueError('먼저 로컬 LLM 설치를 완료하세요.')
 parsed=urllib.parse.urlsplit(url);host=parsed.netloc
 env=dict(os.environ,OLLAMA_HOST=host,OLLAMA_MODELS=str(Path(folder)/'ollama_models'),OLLAMA_NUM_PARALLEL='1',OLLAMA_MAX_LOADED_MODELS='1',OLLAMA_KEEP_ALIVE='2m',OLLAMA_VULKAN='0',CUDA_VISIBLE_DEVICES='-1')
 process=subprocess.Popen([str(exe),'serve'],cwd=exe.parent,env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,creationflags=NO_WINDOW)
 for _ in range(60):
  try:request(url,'/api/version',timeout=1);return process
  except Exception:
   if process.poll() is not None:raise ValueError('Ollama CPU 서버 시작 실패')
   time.sleep(.2)
 process.terminate();raise ValueError('Ollama 서버 응답 없음')
def pull(url,model,progress=lambda s:None):
 req=urllib.request.Request(local_url(url)+'/api/pull',data=json.dumps(dict(model=model,stream=True)).encode(),headers={'Content-Type':'application/json'})
 last=''
 with urllib.request.urlopen(req,timeout=180) as f:
  for line in f:
   value=json.loads(line)
   if value.get('error'):raise ValueError(value['error'])
   total=value.get('total',0);done=value.get('completed',0);status=value.get('status','')
   text=status+(f' {int(done*100/total)}%' if total else '')
   if text!=last:progress(text);last=text
 if last!='success':raise ValueError('LLM 모델 다운로드 완료 확인 실패')

def stop_server(process):
 if process and process.poll() is None:
  if os.name=='nt':subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=8,creationflags=NO_WINDOW)
  else:process.terminate()
