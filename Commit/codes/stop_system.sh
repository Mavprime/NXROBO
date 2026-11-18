#!/usr/bin/env bash
# Остановка всех подсистем Spark

echo "Остановка всех процессов..."
fuser -k 8080/tcp 2>/dev/null
pkill -f "runtask.py" 2>/dev/null
pkill -f "detect_ros.py" 2>/dev/null
pkill -f "camera_stream.py" 2>/dev/null
pkill -f "pro_control_nomoveit" 2>/dev/null
pkill -f "spark_bringup" 2>/dev/null
pkill -f "spark_base_server" 2>/dev/null
rosnode kill -a 2>/dev/null
echo "Все процессы остановлены."
