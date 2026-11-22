#!/usr/bin/env bash
# Wrapper for bridge_check.py on the robot (hardware ROS master, default 11311).
# Usage:
#   bash ~/codes/robot/run_bridge_check.sh _expected_class:=clock _expected_x:=1.2 _expected_y:=0.8

source /opt/ros/noetic/setup.bash
source "$HOME/spark_noetic/devel/setup.bash"
exec python3 -u "$HOME/codes/bridge_check.py" "$@"
