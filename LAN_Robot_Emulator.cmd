@echo off
setlocal
cd /d "%~dp0"
where py >nul 2>nul
if not errorlevel 1 (
  py -3 -X utf8 -m seer_control.robot_emulator
) else (
  python -X utf8 -m seer_control.robot_emulator
)
if errorlevel 1 (
  echo See docs\LAN_ROBOT_EMULATOR_KO.md for setup and port conflicts.
  pause
)
