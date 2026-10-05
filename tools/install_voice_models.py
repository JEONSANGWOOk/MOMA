"""Install official offline voice runtimes and optional 1.7B local LLM."""
import argparse,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from seer_control.voice_setup import install_whisper,install_ollama,start_server,pull,stop_server
p=argparse.ArgumentParser();p.add_argument('--target',default=str(Path.home()/'.seer_amr_console/voice'));p.add_argument('--llm',action='store_true');a=p.parse_args()
install_whisper(a.target,print)
if a.llm:
 install_ollama(a.target,print);server=start_server(a.target,'http://127.0.0.1:12634')
 try:pull('http://127.0.0.1:12634','qwen3:1.7b',print)
 finally:
  stop_server(server)
print('Voice models ready')
