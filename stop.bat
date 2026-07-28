@echo off
rem Stop the UAV_yolo ground station (frees port 8610, the capture card and the COM port).
rem ASCII-only + CRLF on purpose; see start_guide.md for the Chinese notes.
title UAV_yolo Stop

set "PORT=8610"
set "FOUND="

for /f "tokens=5" %%P in ('netstat -ano ^| findstr /r /c:"LISTENING" ^| findstr ":%PORT% "') do (
  echo Stopping ground station, PID %%P ...
  taskkill /F /PID %%P >nul 2>&1
  set "FOUND=1"
)

if defined FOUND (
  echo Stopped. Port %PORT% is free.
) else (
  echo Nothing was listening on port %PORT%.
)

rem Portable pause: timeout fails when stdin is redirected, ping does not.
ping -n 4 127.0.0.1 >nul 2>&1
exit /b 0
