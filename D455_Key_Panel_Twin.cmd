@echo off
cd /d "%~dp0"
py -3 tools\d455_key_panel.py
if errorlevel 1 pause
