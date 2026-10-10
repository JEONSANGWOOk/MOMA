"""Install a pinned official pure Python Windows SDK locally; no robot calls."""
import argparse,hashlib,json,sys,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--tag',default='v2.2.8_robot_v3.9.8');args=p.parse_args()
    if any(x not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-' for x in args.tag):raise ValueError('공식 SDK tag 오류')
    repo='https://api.github.com/repos/FAIR-INNOVATION/fairino-python-sdk'
    meta=json.load(urllib.request.urlopen(repo+'/commits/'+args.tag,timeout=15));sha=meta['sha']
    url='https://raw.githubusercontent.com/FAIR-INNOVATION/fairino-python-sdk/'+sha+'/windows/fairino/Robot.py'
    data=urllib.request.urlopen(url,timeout=20).read();compile(data.decode('utf-8-sig'),'Robot.py','exec')
    folder=ROOT/'.delivery/fairino-sdk/windows';package=folder/'fairino';package.mkdir(parents=True,exist_ok=True)
    (package/'Robot.py.tmp').write_bytes(data);(package/'Robot.py.tmp').replace(package/'Robot.py')
    (package/'__init__.py').write_text('',encoding='utf-8')
    (folder/'provenance.json').write_text(json.dumps(dict(source=url,tag=args.tag,commit=sha,sha256=hashlib.sha256(data).hexdigest()),indent=2),encoding='utf-8')
    print('Official SDK saved:',folder,'tag:',args.tag)
    print('Choose the SDK tag matching the actual controller version. No robot connection/motion was performed.')


if __name__=='__main__':main()
