#!/usr/bin/env bash
# Start RViz marker publishers for the robot-side Gazebo sim (master 11312).
# The script's own command line does not contain the pkill patterns.

set -o pipefail
ROBOT_IP="${ROBOT_IP:-$(hostname -I | awk '{print $1}')}"
export ROS_MASTER_URI="http://${ROBOT_IP}:11312"
export ROS_IP="${ROBOT_IP}"
source /opt/ros/noetic/setup.bash
source "$HOME/spark_noetic/devel/setup.bash"

pkill -f "[m]ission_viz.py" 2>/dev/null || true
pkill -f "[d]etections_viz.py" 2>/dev/null || true
sleep 1

setsid nohup python3 -u "$HOME/codes/mission_viz.py" \
    _world:="$HOME/codes/config/arena_map.yaml" \
    >"$HOME/logs/sim/mission_viz.log" 2>&1 </dev/null &

setsid nohup python3 -u "$HOME/codes/detections_viz.py" \
    _frame:=map _topic:=/detections_world \
    >"$HOME/logs/sim/detections_viz.log" 2>&1 </dev/null &

echo "markers: /mission_markers and /detection_markers on master 11312"
