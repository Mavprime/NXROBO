#!/usr/bin/env bash
# Stop the robot Gazebo simulation. Never touches the hardware ROS master (11311).

set -o pipefail

ROBOT_IP="${ROBOT_IP:-$(hostname -I | awk '{print $1}')}"
export ROS_MASTER_URI="http://${ROBOT_IP}:11312"
export ROS_IP="${ROBOT_IP}"
source /opt/ros/noetic/setup.bash 2>/dev/null || true

pkill -f "[s]park_arena.launch" 2>/dev/null || true
pkill -f "[n]av_arena.launch" 2>/dev/null || true
pkill -f "[g]zserver" 2>/dev/null || true
pkill -f "[g]azebo_truth_detections" 2>/dev/null || true
pkill -f "[a]rm_mock_node" 2>/dev/null || true
pkill -f "[m]ission_node.py" 2>/dev/null || true
pkill -f "[m]ission_viz.py" 2>/dev/null || true
pkill -f "[d]etections_viz.py" 2>/dev/null || true
pkill -f "[/]opt/ros/noetic/lib/rviz/rviz" 2>/dev/null || true
# the sim rosmaster's command line is just "rosmaster"; kill it by port
if command -v fuser >/dev/null 2>&1; then
    fuser -k 11312/tcp 2>/dev/null || true
fi
pkill -f "[r]osmaster.*11312" 2>/dev/null || true
echo "sim stopped (hardware master on 11311 untouched)"
