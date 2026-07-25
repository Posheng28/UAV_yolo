@echo off
rem UAV_yolo ground station launcher.
rem Mode (SIM vs LIVE) is chosen in the web UI (Settings -> mode -> Restart engine),
rem NOT here - one launcher, the toggle is the single source of truth.
rem ASCII-only + CRLF on purpose; see start_guide.md for the Chinese notes.
title UAV_yolo Launcher
cd /d "%~dp0"

set "PY=C:\Users\user\miniconda3\python.exe"
set "PORT=8610"

if not exist "%PY%" (
  echo [ERROR] Python not found at %PY%
  echo Edit start.bat and set PY to your python.exe path.
  pause
  exit /b 1
)

echo Starting UAV_yolo ground station ...
echo URL: http://localhost:%PORT%
echo A server window will open. Close it to stop the server.
echo Switch SIM / LIVE inside the web UI (Settings tab), then Restart engine.

start "UAV_yolo Server" "%PY%" run.py --port %PORT%
timeout /t 5 >nul
start "" "http://localhost:%PORT%"
exit /b 0
