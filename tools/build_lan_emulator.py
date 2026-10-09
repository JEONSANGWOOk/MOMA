"""Create Windows EXE and a separate source bundle using an isolated build env."""
import hashlib
import json
import subprocess
import sys
import zipfile
from pathlib import Path

project=Path(__file__).resolve().parents[1]
delivery=project.parent/'.delivery'
output=delivery/'lan-emulator-dist'
subprocess.run([sys.executable,'-m','PyInstaller','--noconfirm','--onefile','--console',
    '--hide-console','hide-early','--noupx','--name','LAN_Robot_Emulator',
    '--python-option','X utf8','--paths',str(project),
    '--add-data',str(project/'maps/demo.json')+':maps',
    '--distpath',str(output),'--workpath',str(delivery/'lan-emulator-build'),
    '--specpath',str(delivery),str(project/'tools/lan_robot_emulator.py')],check=True,cwd=project)
exe=output/'LAN_Robot_Emulator.exe'
assert exe.is_file()
portable=delivery/'LAN_Robot_Emulator_Windows.zip'
with zipfile.ZipFile(portable,'w',zipfile.ZIP_DEFLATED) as archive:
    archive.write(exe,exe.name)
    archive.write(project/'docs/LAN_ROBOT_EMULATOR_KO.md','사용방법.md')
    archive.writestr('먼저읽기.txt','1. 다른 Windows 노트북에 압축을 풉니다.\n2. LAN_Robot_Emulator.exe를 실행합니다. Python 설치는 필요 없습니다.\n3. TCP 19204~19207을 개인 네트워크에서 허용합니다.\n4. 현재 GUI에서 상대 노트북 IP로 실기 · 제어 연결 → 제어권 가져오기 → Pull Map → 미션 시작.\n자세한 방법과 VS Code 연결은 사용방법.md를 확인하세요.\n')
source=delivery/'LAN_Robot_Emulator_Source.zip'
with zipfile.ZipFile(source,'w',zipfile.ZIP_DEFLATED) as archive:
    for name in ['LAN_Robot_Emulator.cmd','docs/LAN_ROBOT_EMULATOR_KO.md','maps/demo.json']:
        archive.write(project/name,name)
    for file in sorted((project/'seer_control').glob('*.py')):
        archive.write(file,file.relative_to(project).as_posix())
manifest={p.name:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in [exe,portable,source]}
(delivery/'lan_emulator_artifacts.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print(json.dumps(manifest,indent=2))
