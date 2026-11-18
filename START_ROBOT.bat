@echo off
chcp 65001 >nul
title ПУЛЬТ УПРАВЛЕНИЯ РОБОТОМ SPARK

set "IP_FILE=%~dp0robot_ip.txt"
set "ROBOT_IP=10.24.170.0"

if exist "%IP_FILE%" (
    set /p ROBOT_IP=<"%IP_FILE%"
)

echo ==============================================================
echo           РОБОТ SPARK: БЫСТРЫЙ ЗАПУСК И УПРАВЛЕНИЕ           
echo ==============================================================
echo Текущий сохраненный IP робота: %ROBOT_IP%
echo.
set /p USER_INPUT="Нажмите [Enter] для подключения к %ROBOT_IP% или введите новый IP: "

if not "%USER_INPUT%"=="" (
    set "ROBOT_IP=%USER_INPUT%"
    echo %USER_INPUT%>"%IP_FILE%"
    echo [OK] Новый IP сохранен в robot_ip.txt: %USER_INPUT%
)

echo.
echo >> Подключение к роботу spark@%ROBOT_IP%...
echo >> Пароль робота (если спросит): simple2026
echo --------------------------------------------------------------

ssh -t spark@%ROBOT_IP% "bash -ic '~/codes/dashboard.sh'"

echo.
echo --------------------------------------------------------------
echo Сессия управления завершена.
pause
