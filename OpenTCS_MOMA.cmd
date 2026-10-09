@echo off
cd /d "%~dp0"
set "ACS_SCRIPT=%~dp0..\..\ACS_Test_openTCS\control.ps1"
if exist "%ACS_SCRIPT%" (
    powershell -NoProfile -ExecutionPolicy Bypass -File "%ACS_SCRIPT%" -Action Start
    if errorlevel 1 goto failed
)
where py >nul 2>nul
if not errorlevel 1 (
    py -3 -X utf8 tools\opentcs_integration.py %*
) else (
    python -X utf8 tools\opentcs_integration.py %*
)
if errorlevel 1 goto failed
exit /b 0
:failed
pause
exit /b 1
