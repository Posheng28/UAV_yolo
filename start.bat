@echo off
rem UAV_yolo ground station launcher.
rem Mode (SIM vs LIVE) is chosen in the web UI (Settings -> mode -> Restart engine),
rem NOT here - one launcher, the toggle is the single source of truth.
rem ASCII-only + CRLF on purpose; see start_guide.md for the Chinese notes.
setlocal EnableExtensions
cd /d "%~dp0"

set "PORT=8610"

rem Re-invoked by ourselves to run the server in its own window (see :launch).
if /i "%~1"=="--serve" goto serve

title UAV_yolo Launcher

rem Pick a Python. Two things make this harder than it looks:
rem  1. A hardcoded path only works on the machine it was written on.
rem  2. "Any Python" is not enough - machines commonly carry several installs
rem     and only one has been pip'd into. Measured on the dev box, `py -3` is a
rem     bare 3.12 with no cv2 while `python` is the conda install that has
rem     everything. So each candidate is TESTED (tools\bootstrap.py --check
rem     looks for every package requirements.txt promises) and the first one
rem     that passes wins.
rem PY_ANY remembers a Python that runs but lacks packages - that one can still
rem build the environment we install into.
set "PY="
set "PY_ANY="

call :try_path "%UAV_YOLO_PY%"
call :try_path "%~dp0.venv\Scripts\python.exe"
call :try_path "%~dp0venv\Scripts\python.exe"
call :try_cmd python
call :try_cmd py -3
call :try_path "%USERPROFILE%\miniconda3\python.exe"
call :try_path "%USERPROFILE%\anaconda3\python.exe"
call :try_path "%LOCALAPPDATA%\Programs\Python\Python313\python.exe"
call :try_path "%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
call :try_path "%LOCALAPPDATA%\Programs\Python\Python311\python.exe"
call :try_path "%LOCALAPPDATA%\Programs\Python\Python310\python.exe"
call :try_path "%ProgramFiles%\Python313\python.exe"
call :try_path "%ProgramFiles%\Python312\python.exe"
call :try_path "%ProgramFiles%\Python311\python.exe"

if defined PY goto launch
if not defined PY_ANY goto no_python

rem Nothing here can run the app yet. Install the packages instead of just
rem reporting the problem - "No module named uvicorn" is not something the
rem person who double-clicked this is meant to have to solve.
echo.
echo No Python on this computer has the packages UAV_yolo needs.
echo Installing them into a private environment inside this folder:
echo     %~dp0.venv
echo Nothing outside this folder is touched. Delete .venv to undo it.
echo.
echo Press Ctrl+C to cancel. Starting in 5 seconds ...
call :sleep 5
echo.
"%PY_ANY%" "%~dp0tools\bootstrap.py"
if %ERRORLEVEL% NEQ 0 goto install_failed

rem Re-test: normally the new .venv, but bootstrap falls back to installing
rem into PY_ANY itself when a virtual environment cannot be created.
call :try_path "%~dp0.venv\Scripts\python.exe"
call :try_path "%PY_ANY%"
if not defined PY goto install_failed
echo.

:launch
echo Using Python: %PY%
echo Starting UAV_yolo ground station ...
echo URL: http://localhost:%PORT%
echo A server window will open. Close it to stop the server.
echo Switch SIM / LIVE inside the web UI (Settings tab), then Restart engine.
set "UAV_YOLO_RESOLVED_PY=%PY%"
start "UAV_yolo Server" "%~f0" --serve
call :wait_port
if %ERRORLEVEL% NEQ 0 goto not_listening
start "" "http://localhost:%PORT%"
exit /b 0

:not_listening
echo.
echo [ERROR] The server did not start listening on port %PORT%.
echo         The reason is in the "UAV_yolo Server" window that just opened
echo         (and in data\server.log). Not opening the browser.
echo.
pause
exit /b 1

rem ---- the server window ----------------------------------------------------
rem Run from this same file so the interpreter path never has to survive a
rem round of nested cmd quoting. It arrives in the environment instead.

:serve
title UAV_yolo Server
if not defined UAV_YOLO_RESOLVED_PY goto serve_no_python
"%UAV_YOLO_RESOLVED_PY%" run.py --port %PORT%
set "RC=%ERRORLEVEL%"
if "%RC%"=="0" exit /b 0
if "%RC%"=="-1073741510" exit /b 0
goto serve_failed

:serve_failed
echo.
echo The server stopped with an error (exit code %RC%).
rem run.py sends stderr to the log at the OS level (fd 2) before it imports
rem anything, so this window really has nothing in it - the reason is only in
rem the file. Saying "see the lines above" was simply false. Show the tail.
echo Last lines of data\server.log, where the reason actually is:
echo ------------------------------------------------------------------
if exist "%~dp0data\server.log" powershell -NoProfile -Command "Get-Content -LiteralPath '%~dp0data\server.log' -Tail 25"
echo ------------------------------------------------------------------
echo.
echo If it says "address already in use", a ground station is already running:
echo just open http://localhost:%PORT% instead of starting a second one.
echo.
pause
exit /b 1

:serve_no_python
echo [ERROR] Launched with --serve but no interpreter was passed in.
echo         Start this file normally (double-click it) instead.
pause
exit /b 1

rem ---- failure messages -----------------------------------------------------

:no_python
echo.
echo [ERROR] No Python 3.10 or newer was found on this computer.
echo.
echo Install one and run this file again:
echo     1. Open https://www.python.org/downloads/
echo     2. Get Python 3.12 for Windows
echo     3. IMPORTANT - tick "Add python.exe to PATH" in the installer
echo.
echo Already have Python somewhere unusual? Point the launcher at it with
echo setx - plain "set" only lasts until you close that console window, so it
echo would be gone by the time you double-click this file:
echo     setx UAV_YOLO_PY "C:\path\to\python.exe"
echo Then close that window and double-click this file again.
echo.
pause
exit /b 1

:install_failed
echo.
echo [ERROR] Setup did not finish, so the ground station was not started.
echo         pip log: %~dp0data\pip-install.log
echo.
echo Run this file again to retry, or install by hand with:
echo     "%PY_ANY%" -m pip install -r requirements.txt
echo.
pause
exit /b 1

rem ---- helpers --------------------------------------------------------------
rem Both leave PY set only when that interpreter can actually run the app.

:try_path
rem NOTE the "NEQ 0" / "EQU 0" tests below instead of "if errorlevel 1".
rem "if errorlevel 1" means "exit code >= 1" and cmd compares SIGNED, so every
rem NTSTATUS crash code is NEGATIVE and reads as success: an access violation
rem (0xC0000005 = -1073741819, e.g. a broken cv2/torch DLL) would pass both
rem gates here and be announced as "Using Python: ..." before the server window
rem blinked out. Verified with a stub .exe that returns -1073741515.
if defined PY exit /b 0
if "%~1"=="" exit /b 0
if not exist "%~1" exit /b 0
"%~1" -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if %ERRORLEVEL% NEQ 0 exit /b 0
if not defined PY_ANY set "PY_ANY=%~1"
"%~1" "%~dp0tools\bootstrap.py" --check >nul 2>&1
if %ERRORLEVEL% EQU 0 set "PY=%~1"
exit /b 0

:wait_port
rem Wait until the server is actually listening before opening the browser.
rem A fixed sleep is a guess: measured 2.5 s on a warm conda install, but the
rem very first start also pays for importing cv2 + torch + ultralytics, which
rem alone took 5.1 s here. Opening too early just shows "can't reach this page".
setlocal
set /a "LEFT=30"
:wait_port_loop
netstat -an | findstr /r /c:":%PORT% .*LISTENING" >nul 2>&1
if %ERRORLEVEL% EQU 0 endlocal & exit /b 0
set /a "LEFT-=1"
if %LEFT% LEQ 0 endlocal & exit /b 1
call :sleep 1
goto wait_port_loop

:sleep
rem Wait %1 seconds. NOT "timeout" - it needs a real console input handle and
rem dies with "ERROR: Input redirection is not supported" whenever stdin is
rem redirected, which then skips the wait and opens the browser before the
rem server is listening. Measured on this machine. ping always works.
rem -n counts packets and the first one is immediate, so N+1 packets = N seconds.
rem -w 1000 keeps each gap at a second even where ICMP loopback is filtered.
setlocal
set /a "TICKS=%~1+1"
ping -n %TICKS% -w 1000 127.0.0.1 >nul 2>&1
endlocal
exit /b 0

:try_cmd
rem Resolve a PATH command (python, py -3) to the real executable path, so
rem everything downstream can quote it - "C:\Program Files\..." has a space in
rem it, and an unquoted %PY% would be split there. A Microsoft Store stub
rem prints a message instead of a path, and is rejected by the exist check.
if defined PY exit /b 0
set "CAND="
for /f "usebackq delims=" %%p in (`%* -c "import sys; print(sys.executable)" 2^>nul`) do set "CAND=%%p"
if not defined CAND exit /b 0
call :try_path "%CAND%"
set "CAND="
exit /b 0
