@echo off
title SPARK ROBOT - AUTONOMOUS GRASP TEST
color 0B

echo ======================================================================
echo           AUTONOMOUS CUBE GRASP TEST (SPARK ROBOT)
echo ======================================================================
echo.

set ROBOT_IP=10.24.170.0
set ROBOT_USER=spark
set SRC_DIR=d:\NXROBO\auto\codes

echo [1/3] Checking connection to robot (%ROBOT_IP%)...
ping -n 1 -w 1500 %ROBOT_IP% >nul 2>&1
if errorlevel 1 goto :error_ping

ssh -o BatchMode=yes -o ConnectTimeout=5 %ROBOT_USER%@%ROBOT_IP% "echo OK" >nul 2>&1
if errorlevel 1 goto :error_ssh

echo       Connection established.
echo.
echo [2/3] Auto-syncing local code to robot...
scp -o StrictHostKeyChecking=accept-new "%SRC_DIR%\grasp_generic.py" "%SRC_DIR%\match_logic.py" "%SRC_DIR%\run_grasp_test.sh" %ROBOT_USER%@%ROBOT_IP%:/home/spark/codes/
if errorlevel 1 goto :error_scp
echo       Code synced successfully.
echo.
echo [3/3] Checking robot drivers and running grasp test...
echo ======================================================================
echo.

ssh -tt -o StrictHostKeyChecking=accept-new %ROBOT_USER%@%ROBOT_IP% "bash -l /home/spark/codes/run_grasp_test.sh %*"

echo.
echo ======================================================================
echo Test finished. Press any key to close...
pause >nul
exit /b 0

:error_ping
color 0C
echo.
echo ======================================================================
echo [NETWORK ERROR] Robot %ROBOT_IP% is not responding to ping!
echo.
echo Please check:
echo   1. Robot is powered on.
echo   2. PC is connected to robot network (Wi-Fi 10.24.170.x or cable).
echo ======================================================================
echo.
pause
exit /b 1

:error_ssh
color 0C
echo.
echo ======================================================================
echo [SSH ERROR] Ping ok, but SSH connection to %ROBOT_IP% failed!
echo Running verbose SSH to check the error:
ssh -v -o ConnectTimeout=5 %ROBOT_USER%@%ROBOT_IP% "echo OK"
echo ======================================================================
echo.
pause
exit /b 2

:error_scp
color 0C
echo.
echo ======================================================================
echo [SYNC ERROR] Failed to copy local files from %SRC_DIR% to robot!
echo ======================================================================
echo.
pause
exit /b 3
