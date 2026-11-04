@echo off
setlocal
cd /d "%~dp0"
python -c "import sys;sys.exit(0 if sys.version_info >= (3,10) else 1)" >nul 2>&1
if not errorlevel 1 goto python_run
py -3 -c "import sys;sys.exit(0 if sys.version_info >= (3,10) else 1)" >nul 2>&1
if not errorlevel 1 goto py_run
echo Python 3.10+ is required.
pause
exit /b 1
:python_run
python -X utf8 tools\run_web_control.py %*
goto finished
:py_run
py -3 -X utf8 tools\run_web_control.py %*
:finished
if not errorlevel 1 exit /b 0
echo Web control startup failed. See the message above.
pause
exit /b 1
