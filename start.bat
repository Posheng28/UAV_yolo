@echo off
rem UAV_yolo ground station launcher - SIM mode (no hardware needed)
rem ASCII-only + CRLF on purpose; see start_guide.md for the Chinese notes.
title UAV_yolo Launcher (SIM)
cd /d "%~dp0"

set "PY=C:\Users\user\miniconda3\python.exe"
set "PORT=8610"

if not exist "%PY%" (
  echo [ERROR] Python not found at %PY%
  echo Edit start.bat and set PY to your python.exe path.
  pause
  exit /b 1
)

echo Starting UAV_yolo ground station in SIM mode ...
echo URL: http://localhost:%PORT%
echo A server window will open. Close it to stop the server.

start "UAV_yolo Server (SIM)" "%PY%" run.py --sim --port %PORT%
timeout /t 5 >nul
start "" "http://localhost:%PORT%"
exit /b 0
