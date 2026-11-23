@echo off
title SPARK ROBOT - CUBE VISIBILITY SCAN
color 0E

echo ======================================================================
echo       CUBE VISIBILITY CHECK [SCAN ONLY] (SPARK ROBOT)
echo ======================================================================
echo.

set ROBOT_IP=10.24.170.0
set ROBOT_USER=spark
set SRC_DIR=%~dp0auto\codes

echo [1/3] Checking connection to robot (%ROBOT_IP%)...
rem ICMP may be filtered on the arena Wi-Fi, so SSH is the connectivity probe.
ssh -o BatchMode=yes -o ConnectTimeout=5 %ROBOT_USER%@%ROBOT_IP% "echo OK" >nul 2>&1
if errorlevel 1 goto :error_ssh

echo       Connection established.
echo.
echo [2/3] Complete upload of clean codes and calibration to robot...
echo       - Creating workspace directories...
ssh -o StrictHostKeyChecking=accept-new %ROBOT_USER%@%ROBOT_IP% "mkdir -p /home/spark/codes /home/spark/logs /home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes"
if errorlevel 1 goto :error_scp

echo       - Uploading Python scripts to ~/codes/...
scp -o StrictHostKeyChecking=accept-new "%SRC_DIR%\grasp_generic.py" "%SRC_DIR%\match_logic.py" "%SRC_DIR%\runtask.py" "%SRC_DIR%\detect_ros.py" "%SRC_DIR%\pad_detect.py" "%SRC_DIR%\patch_swiftpro.py" "%SRC_DIR%\snap.py" "%SRC_DIR%\run_grasp_test.sh" %ROBOT_USER%@%ROBOT_IP%:/home/spark/codes/
if errorlevel 1 goto :error_scp

echo       - Uploading calibration files (thefile*.txt, transform_3d.dat) to ~/...
scp -o StrictHostKeyChecking=accept-new "%SRC_DIR%\thefile.txt" "%SRC_DIR%\thefile1.txt" "%SRC_DIR%\thefile2.txt" "%SRC_DIR%\transform_3d.dat" %ROBOT_USER%@%ROBOT_IP%:/home/spark/
if errorlevel 1 goto :error_scp

echo       - Syncing match_logic.py + detect_ros.py to ROS auto_match node...
scp -o StrictHostKeyChecking=accept-new "%SRC_DIR%\match_logic.py" %ROBOT_USER%@%ROBOT_IP%:/home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/match_logic.py
if errorlevel 1 goto :error_scp
scp -o StrictHostKeyChecking=accept-new "%SRC_DIR%\detect_ros.py" %ROBOT_USER%@%ROBOT_IP%:/home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/detect_ros.py
if errorlevel 1 goto :error_scp

echo       - Restarting detector so the new cake-confidence gate is applied...
ssh -o StrictHostKeyChecking=accept-new %ROBOT_USER%@%ROBOT_IP% "pkill -f detect_ros.py 2>/dev/null; sleep 1"

echo       - Setting execution permissions...
ssh -o StrictHostKeyChecking=accept-new %ROBOT_USER%@%ROBOT_IP% "chmod +x /home/spark/codes/run_grasp_test.sh /home/spark/codes/*.py 2>/dev/null"

echo       All codes and calibration files uploaded fresh and ready.
echo.
echo [3/3] Checking drivers and running visibility check...
echo ======================================================================
echo.

ssh -tt -o StrictHostKeyChecking=accept-new %ROBOT_USER%@%ROBOT_IP% "bash -l /home/spark/codes/run_grasp_test.sh --scan-only"

echo.
echo ======================================================================
echo Scan finished. Press any key to close...
pause >nul
exit /b 0

:error_ssh
color 0C
echo.
echo ======================================================================
echo [SSH ERROR] Cannot reach %ROBOT_USER%@%ROBOT_IP% over SSH!
echo.
echo Please check:
echo   1. Robot is powered on and still connected to the arena Wi-Fi.
echo   2. PC is on the robot network (Wi-Fi 10.24.170.x or cable).
echo   3. A stale IP in robot_ip.txt (edit ROBOT_IP at the top of this file).
echo.
echo Running verbose SSH to show the exact error:
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
