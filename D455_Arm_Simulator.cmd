@echo off
cd /d "%~dp0"
py -3 -X utf8 tools\d455_arm_sim.py
if errorlevel 1 pause
