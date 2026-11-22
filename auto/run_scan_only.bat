@echo off
title SPARK ROBOT - CUBE VISIBILITY SCAN
rem Forwarder: see run_grasp_test.bat in the repository root.
call "%~dp0..\run_grasp_test.bat" --scan-only %*
