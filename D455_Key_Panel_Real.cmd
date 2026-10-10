@echo off
cd /d "%~dp0"
py -3 tools\d455_key_panel_real.py --ip 192.168.57.2
if errorlevel 1 pause
