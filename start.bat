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
rem Version banner. If someone reports a problem and their window does NOT show
rem this line, they are running an older copy of the file and no amount of
rem debugging their machine will help - they need to pull.
echo UAV_yolo launcher (auto-setup build)
call :log ---- launcher start ----

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
call :log no interpreter had the packages; bootstrapping with "%PY_ANY%"
"%PY_ANY%" "%~dp0tools\bootstrap.py"
if %ERRORLEVEL% NEQ 0 goto install_failed

rem Re-test: normally the new .venv, but bootstrap falls back to installing
rem into PY_ANY itself when a virtual environment cannot be created.
call :try_path "%~dp0.venv\Scripts\python.exe"
call :try_path "%PY_ANY%"
if not defined PY goto install_failed
echo.

:launch
rem Is one already running? Starting a second is pointless - it cannot bind the
rem port and dies - and it is actively misleading: :wait_port would see the OLD
rem server still LISTENING, call it success and open the browser onto it, so the
rem operator believes they are looking at the instance they just launched.
call :port_busy
if %ERRORLEVEL% EQU 0 goto already_running

echo Using Python: %PY%
rem Say which environment won, on screen. "Why is there no .venv folder?" is the
rem first thing people ask, and the answer is usually "because you did not need
rem one" - but nothing said so, which makes a success look like a failure.
if exist "%~dp0.venv\Scripts\python.exe" echo Environment: the project's own .venv folder
if not exist "%~dp0.venv\Scripts\python.exe" echo Environment: no .venv needed - that Python already has every package
call :log using "%PY%"
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

:already_running
call :log port %PORT% already in use; did not start a second server
echo.
echo A ground station is ALREADY running on port %PORT%.
echo This launcher did NOT start a second one - it could not take the port.
echo Opening the one that is already there: http://localhost:%PORT%
echo.
echo To restart it: close its "UAV_yolo Server" window first, then run this
echo file again. If something else is holding port %PORT%, close that instead.
echo.
start "" "http://localhost:%PORT%"
call :sleep 3
exit /b 0

:not_listening
call :log server did not reach LISTENING on port %PORT%
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
call :log server exited with %RC%
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
call :log no Python 3.10+ found on this computer
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
call :log SETUP FAILED
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

:log
rem Append one line to data\launcher.log. The launcher window is closed by the
rem time anyone asks what happened, so the decisions have to survive it - "send
rem me data\launcher.log" beats "what did it say?" every time.
rem %DATE% is localized and carries the weekday - "week3 2026/08/05" on this
rem machine - so writing it produces non-ASCII bytes and the log comes back as
rem mojibake in whatever tool the student pastes it into. The last 10 characters
rem are the numeric date, which is ASCII everywhere.
if not exist "%~dp0data" mkdir "%~dp0data" >nul 2>&1
>>"%~dp0data\launcher.log" echo [%DATE:~-10% %TIME%] %*
exit /b 0

:port_busy
rem Exit 0 when something is already listening on %PORT%.
netstat -an | findstr /r /c:":%PORT% .*LISTENING" >nul 2>&1
exit /b %ERRORLEVEL%

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
