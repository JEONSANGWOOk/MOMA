@echo off
cd /d "%~dp0"
py -3 -X utf8 tools\d455_board_mission.py --hold
if errorlevel 1 pause
