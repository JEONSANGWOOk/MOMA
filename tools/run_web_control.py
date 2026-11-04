"""Start the local browser workspace; build once when needed."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
import urllib.request
import webbrowser

ROOT=Path(__file__).resolve().parents[1]
os.chdir(ROOT)
sys.path.insert(0,str(ROOT))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--build',action='store_true')
    args=parser.parse_args()
    url=f'http://127.0.0.1:{args.port}'
    try:
        with urllib.request.urlopen(url+'/api/health',timeout=.6) as response:health=json.load(response)
        if health.get('service')=='moma-web-control':
            print('MOMA Web Control is already running:',url)
            if not args.no_browser:webbrowser.open(url)
            return
    except (OSError,ValueError):pass
    missing=[name for name in ('fastapi','uvicorn','websockets','httpx') if importlib.util.find_spec(name) is None]
    if missing:
        subprocess.run([sys.executable,'-m','pip','install','-r',str(ROOT/'web_control/requirements.txt')],check=True)
    if args.build or not (ROOT/'web_control/dist/index.html').exists():
        node=shutil.which('node')
        if not node:
            portable=sorted((ROOT/'.tools').glob('node-*-win-x64/node.exe'))
            node=str(portable[-1]) if portable else None
        if not node:
            raise SystemExit('Node.js 22.12+ is needed for the first build. Install it, then run again: https://nodejs.org/')
        node_dir=Path(node).parent
        npm=node_dir/'npm.cmd' if os.name=='nt' else Path(shutil.which('npm') or '')
        env=dict(os.environ);env['PATH']=str(node_dir)+os.pathsep+env.get('PATH','')
        web=ROOT/'web_control'
        subprocess.run([str(npm),'ci','--no-audit','--no-fund'],cwd=web,env=env,check=True)
        subprocess.run([str(npm),'run','build'],cwd=web,env=env,check=True)
    if not args.no_browser:
        def open_when_ready():
            for _ in range(40):
                try:
                    with urllib.request.urlopen(url+'/api/health',timeout=.5):pass
                    webbrowser.open(url);return
                except OSError:time.sleep(.25)
        threading.Thread(target=open_when_ready,daemon=True).start()
    print('\nMOMA Mission Control:',url,'\nStarts in SIM. Close this terminal to stop the service.\n')
    import uvicorn
    from web_control.server import create_app
    uvicorn.run(create_app(),host=args.host,port=args.port,log_level='warning')

if __name__=='__main__':main()
