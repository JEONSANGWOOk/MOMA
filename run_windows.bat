@echo off
setlocal
cd /d "%~dp0"

rem Probe actual execution, not only PATH entries (Windows aliases may exist).
python -c "import sys, tkinter; sys.exit(0 if sys.version_info >= (3,10) else 1)" >nul 2>&1
if not errorlevel 1 goto run_python
py -3 -c "import sys, tkinter; sys.exit(0 if sys.version_info >= (3,10) else 1)" >nul 2>&1
if not errorlevel 1 goto run_py

echo.
echo Python 3.10+ with Tcl/Tk was not found.
echo Install Python for Windows: https://www.python.org/downloads/windows/
echo Enable Add python.exe to PATH and Tcl/Tk when offered by the installer.
echo Then close this window, open a new CMD window and try again.
echo.
echo Diagnostic commands: python --version   and   py -3 --version
pause
exit /b 1

:run_python
python -c "import PIL" >nul 2>&1
if errorlevel 1 python -m pip install -r requirements.txt
python -X utf8 main.py
goto finished

:run_py
py -3 -c "import PIL" >nul 2>&1
if errorlevel 1 py -3 -m pip install -r requirements.txt
py -3 -X utf8 main.py

:finished
set "app_exit_code=%errorlevel%"
if "%app_exit_code%"=="0" exit /b 0
echo.
echo Application exited with code %app_exit_code%. See the error above.
pause
exit /b %app_exit_code%
