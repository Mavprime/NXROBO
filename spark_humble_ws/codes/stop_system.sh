#!/usr/bin/env bash
# Stop all Spark subsystems

echo "Stopping all processes..."
fuser -k 8080/tcp 2>/dev/null
pkill -f "runtask.py" 2>/dev/null
pkill -f "detect_ros.py" 2>/dev/null
pkill -f "camera_stream.py" 2>/dev/null
pkill -f "pro_control_nomoveit" 2>/dev/null
pkill -f "spark_bringup" 2>/dev/null
pkill -f "spark_base_server" 2>/dev/null
rosnode kill -a 2>/dev/null
echo "All processes stopped."
