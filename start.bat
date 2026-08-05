@echo off
rem UAV_yolo ground station launcher.
rem Mode (SIM vs LIVE) is chosen in the web UI (Settings -> mode -> Restart engine),
rem NOT here - one launcher, the toggle is the single source of truth.
rem ASCII-only + CRLF on purpose; see start_guide.md for the Chinese notes.
title UAV_yolo Launcher
cd /d "%~dp0"

set "PORT=8610"

rem Pick a Python. A hardcoded path only works on the machine it was written on,
rem and picking "any Python" is not enough either: machines commonly have several
rem installs and only one of them has the dependencies. Measured on the dev box,
rem `py -3` resolves to a bare Python 3.12 with no cv2 while `python` resolves to
rem the conda install that has everything - so each candidate is TESTED for the
rem imports before being accepted, and the first one that passes wins.
rem PY_ANY remembers a Python that ran but lacked packages, so the error message
rem can name the exact interpreter to pip into.
set "PY="
set "PY_ANY="

call :try_path "%UAV_YOLO_PY%"
call :try_path "%~dp0.venv\Scripts\python.exe"
call :try_path "%~dp0venv\Scripts\python.exe"
call :try_cmd python
call :try_cmd py -3
call :try_path "%USERPROFILE%\miniconda3\python.exe"
call :try_path "%USERPROFILE%\anaconda3\python.exe"
call :try_path "%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
call :try_path "%LOCALAPPDATA%\Programs\Python\Python311\python.exe"

if not defined PY (
  if defined PY_ANY (
    echo [ERROR] Found Python but the dependencies are missing:
    echo     %PY_ANY%
    echo Install them with:
    echo     %PY_ANY% -m pip install -r requirements.txt
  ) else (
    echo [ERROR] No Python found.
    echo Install Python 3.10+ from python.org, then:
    echo     python -m pip install -r requirements.txt
    echo Or point the launcher at a specific interpreter:
    echo     set UAV_YOLO_PY=C:\path\to\python.exe
  )
  pause
  exit /b 1
)

echo Using Python: %PY%
echo Starting UAV_yolo ground station ...
echo URL: http://localhost:%PORT%
echo A server window will open. Close it to stop the server.
echo Switch SIM / LIVE inside the web UI (Settings tab), then Restart engine.

start "UAV_yolo Server" %PY% run.py --port %PORT%
timeout /t 5 >nul
start "" "http://localhost:%PORT%"
exit /b 0

rem ---- helpers -------------------------------------------------------------
rem Both set PY only when the interpreter can import everything run.py needs.

:try_path
if defined PY exit /b 0
if "%~1"=="" exit /b 0
if not exist "%~1" exit /b 0
if not defined PY_ANY set "PY_ANY=%~1"
"%~1" -c "import cv2, numpy, fastapi, uvicorn, yaml" >nul 2>&1
if not errorlevel 1 set "PY=%~1"
exit /b 0

:try_cmd
if defined PY exit /b 0
%* -c "import sys" >nul 2>&1
if errorlevel 1 exit /b 0
if not defined PY_ANY set "PY_ANY=%*"
%* -c "import cv2, numpy, fastapi, uvicorn, yaml" >nul 2>&1
if not errorlevel 1 set "PY=%*"
exit /b 0
