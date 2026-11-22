#!/usr/bin/env bash
# Start RViz on the robot desktop (:0) for the Gazebo sim on master 11312.
# Connect a VNC viewer to the robot (credentials as configured; not stored here).

set -o pipefail
ROBOT_IP="${ROBOT_IP:-$(hostname -I | awk '{print $1}')}"
export ROS_MASTER_URI="http://${ROBOT_IP}:11312"
export ROS_IP="${ROBOT_IP}"
source /opt/ros/noetic/setup.bash
source "$HOME/spark_noetic/devel/setup.bash"

pkill -f "[/]opt/ros/noetic/lib/rviz/rviz" 2>/dev/null || true
sleep 1

DISPLAY=:0 XAUTHORITY="$HOME/.Xauthority" \
    setsid nohup rosrun rviz rviz -d "$HOME/codes/ros/sim_view.rviz" \
    >"$HOME/logs/sim/rviz.log" 2>&1 </dev/null &

echo "RViz started on :0; connect VNC with the robot's configured credentials"
