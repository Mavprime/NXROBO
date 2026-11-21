#!/usr/bin/env bash
# Source the robot ROS environment and exec the given command.
# Usage: bash ~/codes/run_ros.sh python3 -u ~/codes/cake_pick_place.py
source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/spark/spark_noetic/devel/setup.bash 2>/dev/null
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
exec "$@"
