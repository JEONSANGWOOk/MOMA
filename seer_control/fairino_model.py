"""Pinned official ROS2 model installer. Never runs ROS or robot code."""
import hashlib,json,os
from pathlib import Path
import urllib.request
from .geometry3d import RobotDescription
MANIFEST=Path(__file__).resolve().parents[1]/'config/fairino_model_manifest.json'
RELATIVE='fairino_description/urdf/fairino5_v6.urdf'

def manifest():return json.loads(MANIFEST.read_text(encoding='utf-8'))
def model_root(user_dir):return Path(user_dir)/'models'/'frcobot_ros2'/manifest()['revision']
def installed(user_dir):
 root=model_root(user_dir);info=manifest()
 for name,digest in info['files'].items():
  path=root/name
  if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:return None
 return root/RELATIVE

def install(user_dir,repository=None):
 info=manifest();root=model_root(user_dir);root.mkdir(parents=True,exist_ok=True)
 for name,digest in info['files'].items():
  path=root/name
  if path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest()==digest:continue
  if repository:data=(Path(repository)/name).read_bytes()
  else:
   url='https://raw.githubusercontent.com/FAIR-INNOVATION/frcobot_ros2/'+info['revision']+'/'+name
   with urllib.request.urlopen(url,timeout=30) as response:data=response.read(10*1024*1024+1)
  if len(data)>10*1024*1024 or hashlib.sha256(data).hexdigest()!=digest:raise ValueError('공식 FR5 모델 파일 검증 실패: '+name)
  path.parent.mkdir(parents=True,exist_ok=True);temp=path.with_suffix(path.suffix+'.download');temp.write_bytes(data);os.replace(temp,path)
 path=root/RELATIVE;asset=RobotDescription.load(path)
 missing=[v for v in asset.warnings if '메시 단순화' not in v]
 if missing or len(asset.movable())!=6:raise ValueError('FR5 모델 로드 실패: '+str(missing))
 (root/'SOURCE.json').write_text(json.dumps(info,indent=2),encoding='utf-8')
 return path
