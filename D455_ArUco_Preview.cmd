@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv-vision\Scripts\python.exe" (
  echo Vision environment is missing. See docs\D455_ARUCO_KO.md.
  pause
  exit /b 1
)
".venv-vision\Scripts\python.exe" -X utf8 "tools\d455_aruco_preview.py" --marker-mm 100 --log ".delivery\d455_aruco.jsonl"
if errorlevel 1 pause
