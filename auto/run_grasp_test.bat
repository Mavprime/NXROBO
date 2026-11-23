@echo off
rem Forwarder. The canonical launcher lives in the repository root so there is
rem exactly one copy of the upload/run logic (a full copy in auto\ used to
rem resolve %~dp0auto\codes as auto\auto\codes and failed every scp).
call "%~dp0..\run_grasp_test.bat" %*
