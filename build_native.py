"""Optional native packaging. Requires PyInstaller on the target OS."""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
root=Path(__file__).resolve().parent
if not importlib.util.find_spec('PyInstaller'):
    raise SystemExit('먼저 실행하세요: python -m pip install pyinstaller')
subprocess.run([sys.executable,'-m','PyInstaller','--noconfirm','--clean','--windowed',
    '--onedir','--name','AMRControlStudio',
    '--add-data',f'{root / "maps"}{os.pathsep}maps',
    '--add-data',f'{root / "examples"}{os.pathsep}examples',
    '--add-data',f'{root / "config"}{os.pathsep}config',str(root/'main.py')],cwd=root,check=True)
print('결과: dist/AMRControlStudio (현재 OS 전용, 다른 OS에서는 다시 빌드하세요.)')
