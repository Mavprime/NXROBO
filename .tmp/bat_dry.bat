@echo off
title SPARK ROBOT - AUTONOMOUS GRASP TEST
color 0B

rem ---------------------------------------------------------------------------
rem Resolve the repository root regardless of where this .bat is launched from.
rem The canonical copy lives in the repo root, but a copy/forwarder may sit in
rem auto\ ; without this fallback %~dp0auto\codes would resolve to
rem auto\auto\codes and every scp below would fail.
rem ---------------------------------------------------------------------------
set "PROJECT_ROOT=%~dp0"
if not exist "%PROJECT_ROOT%auto\codes\run_grasp_test.sh" (
    for %%I in ("%~dp0..") do set "PROJECT_ROOT=%%~fI\"
)
if not exist "%PROJECT_ROOT%auto\codes\run_grasp_test.sh" (
    color 0C
    echo.
    echo ======================================================================
    echo [LAYOUT ERROR] Cannot find auto\codes\run_grasp_test.sh
    echo.
    echo This launcher must live in the repository root
    echo ^(the folder that contains auto\^), or inside the auto\ folder.
    echo Current location: %~dp0
    echo ======================================================================
    echo.
    pause
    exit /b 4
)

echo ======================================================================
echo           AUTONOMOUS CUBE GRASP TEST (SPARK ROBOT)
echo ======================================================================
echo.

set ROBOT_IP=10.24.170.0
if exist "%PROJECT_ROOT%auto\launchers\robot_ip.txt" (
    set /p ROBOT_IP=<"%PROJECT_ROOT%auto\launchers\robot_ip.txt"
)
set ROBOT_USER=spark
set SRC_DIR=%PROJECT_ROOT%auto\codes

echo [1/3] Checking connection to robot (%ROBOT_IP%)...
rem ICMP may be filtered on the arena Wi-Fi, so SSH is the connectivity probe.
echo SSH -o BatchMode=yes -o ConnectTimeout=5 %ROBOT_USER%@%ROBOT_IP% "echo OK" >nul 2>&1
rem :error_ssh

echo       Connection established.
echo.
echo [2/3] Uploading updated code to robot (preserving robot's ~/transform_3d.dat)...
echo       - Creating workspace directories...
echo SSH -o StrictHostKeyChecking=accept-new %ROBOT_USER%@%ROBOT_IP% "mkdir -p /home/spark/codes /home/spark/logs /home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes"
rem :error_scp

echo       - Uploading Python scripts to ~/codes/...
echo SCP -o StrictHostKeyChecking=accept-new "%SRC_DIR%\grasp_generic.py" "%SRC_DIR%\robot_view.py" "%SRC_DIR%\match_logic.py" "%SRC_DIR%\runtask.py" "%SRC_DIR%\detect_ros.py" "%SRC_DIR%\pad_detect.py" "%SRC_DIR%\patch_swiftpro.py" "%SRC_DIR%\snap.py" "%SRC_DIR%\run_grasp_test.sh" %ROBOT_USER%@%ROBOT_IP%:/home/spark/codes/
rem :error_scp

echo       - Uploading tier-2 calibration (thefile2.txt) without touching ~/transform_3d.dat...
echo SCP -o StrictHostKeyChecking=accept-new "%SRC_DIR%\thefile2.txt" %ROBOT_USER%@%ROBOT_IP%:/home/spark/
rem :error_scp

echo       - Syncing match_logic.py + detect_ros.py + pad_detect.py to ROS auto_match node...
echo SCP -o StrictHostKeyChecking=accept-new "%SRC_DIR%\match_logic.py" "%SRC_DIR%\detect_ros.py" "%SRC_DIR%\pad_detect.py" %ROBOT_USER%@%ROBOT_IP%:/home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/
rem :error_scp

echo       - Restarting detector so the new cake-confidence gate is applied...
echo SSH -o StrictHostKeyChecking=accept-new %ROBOT_USER%@%ROBOT_IP% "pkill -f detect_ros.py 2>/dev/null; sleep 1"

echo       - Setting execution permissions...
echo SSH -o StrictHostKeyChecking=accept-new %ROBOT_USER%@%ROBOT_IP% "chmod +x /home/spark/codes/run_grasp_test.sh /home/spark/codes/*.py 2>/dev/null"

echo       All codes uploaded fresh and ready.
echo.

set "TEST_ARGS=%*"
if not "%~1"=="" goto :mode_chosen
echo       What should the robot do?
echo         1) FULL grasp test : lower -^> suction -^> lift -^> hold 5 s -^> put back -^> park
echo         2) VIEW ONLY       : annotated photo of what the robot sees (no grasp motion)
echo         3) SCAN ONLY       : text assessment of the target (no motion)
echo.
set "RUN_CHOICE=1"
set /p RUN_CHOICE=      Choose 1/2/3 [default 1]: 
if "%RUN_CHOICE%"=="2" set "TEST_ARGS=--view"
if "%RUN_CHOICE%"=="3" set "TEST_ARGS=--scan-only"
:mode_chosen

echo.
echo [3/3] Initializing drivers and running: %TEST_ARGS%
echo ======================================================================
echo.

echo SSH -tt -o StrictHostKeyChecking=accept-new %ROBOT_USER%@%ROBOT_IP% "bash -l /home/spark/codes/run_grasp_test.sh %TEST_ARGS%"
rem Keep the remote exit code (0 OK, 3 NO_TARGET, 6 OUT_OF_REACH, ...) so a
rem failed field test is visible to any caller and not masked by this script.
set "TEST_RC=%ERRORLEVEL%"

if /I "%TEST_ARGS%"=="--view" (
    echo.
    echo       Downloading the annotated view to the PC...
    if not exist "%PROJECT_ROOT%snapshots" mkdir "%PROJECT_ROOT%snapshots"
    scp -o StrictHostKeyChecking=accept-new "%ROBOT_USER%@%ROBOT_IP%:/home/spark/logs/latest_view.jpg" "%PROJECT_ROOT%snapshots\robot_view_latest.jpg"
    if exist "%PROJECT_ROOT%snapshots\robot_view_latest.jpg" (
        echo       Opening %PROJECT_ROOT%snapshots\robot_view_latest.jpg
        start "" "%PROJECT_ROOT%snapshots\robot_view_latest.jpg"
    ) else (
        echo       [WARN] Could not download the view image from the robot.
    )
)

echo.
echo ======================================================================
echo Test finished. Press any key to close...
pause >nul
exit /b %TEST_RC%

:error_ssh
color 0C
echo.
echo ======================================================================
echo [SSH ERROR] Cannot reach %ROBOT_USER%@%ROBOT_IP% over SSH!
echo.
echo Please check:
echo   1. Robot is powered on and still connected to the arena Wi-Fi.
echo   2. PC is on the robot network (Wi-Fi 10.24.170.x or cable).
echo   3. A stale IP in %PROJECT_ROOT%auto\launchers\robot_ip.txt.
echo.
echo Running verbose SSH to show the exact error:
echo SSH -v -o ConnectTimeout=5 %ROBOT_USER%@%ROBOT_IP% "echo OK"
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
