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
echo Log: data\server.log  (OpenCV / driver messages land here)

if not exist "data" mkdir "data"
echo. >> "data\server.log"
echo ===== session start %DATE% %TIME% ===== >> "data\server.log"
rem Redirect with /B (no new console). A plain `start "title" ...` gives the child
rem its own console, so the redirection binds to the wrong handle and the log
rem stays 0 bytes - that was measured, do not "simplify" this back.
rem OpenCV warnings are written by C code straight to fd 2, so Python-level
rem logging cannot capture them; only this redirection can.
start "" /B "%PY%" run.py --port %PORT% >> "data\server.log" 2>&1
timeout /t 5 >nul
start "" "http://localhost:%PORT%"
exit /b 0
