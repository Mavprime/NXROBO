@echo off
title SPARK ROBOT 1-CLICK LAUNCHER
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0connect.ps1"
if %errorlevel% neq 0 (
    echo.
    echo Connection error. Press any key to exit.
    pause >nul
)
