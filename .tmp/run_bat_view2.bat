@echo off
call "D:\NXROBO\run_grasp_test.bat" --view < nul > D:\NXROBO\.tmp\bat_view2.out 2>&1
echo BAT_EXIT=%errorlevel% >> D:\NXROBO\.tmp\bat_view2.out
