@echo off
setlocal
cd /d "%~dp0"

rem Probe actual execution, not only PATH entries (Windows aliases may exist).
python -c "import sys, tkinter; tkinter.Tcl().eval('info patchlevel'); sys.exit(0 if sys.version_info >= (3,10) else 1)" >nul 2>&1
if not errorlevel 1 goto run_python
py -3 -c "import sys, tkinter; tkinter.Tcl().eval('info patchlevel'); sys.exit(0 if sys.version_info >= (3,10) else 1)" >nul 2>&1
if not errorlevel 1 goto run_py

echo.
echo Python 3.10+ with a working Tcl/Tk runtime was not found.
echo Install Python for Windows: https://www.python.org/downloads/windows/
echo Repair or reinstall Python and its Tcl/Tk runtime.
echo Then close this window, open a new CMD window and try again.
echo.
echo Diagnostic commands: python --version   and   py -3 --version
pause
exit /b 1

:run_python
set "PYTHON_CMD=python"
goto check_dependencies

:run_py
set "PYTHON_CMD=py -3"

:check_dependencies
%PYTHON_CMD% -c "import PIL" >nul 2>&1
if errorlevel 1 goto install_dependencies
goto launch_app

:install_dependencies
echo.
echo Required Python packages are missing. Installing requirements.txt...
%PYTHON_CMD% -m pip install -r requirements.txt
if errorlevel 1 goto dependency_install_failed

:launch_app
%PYTHON_CMD% -X utf8 main.py
set "app_exit_code=%errorlevel%"
goto finished

:dependency_install_failed
echo.
echo Dependency installation failed. The application was not started.
echo Check your network or proxy certificate configuration, then retry.
echo Do not disable TLS certificate verification.
pause
exit /b 1

:finished
if "%app_exit_code%"=="0" exit /b 0
echo.
echo Application exited with code %app_exit_code%. See the error above.
pause
exit /b %app_exit_code%
