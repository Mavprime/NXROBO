#!/usr/bin/env bash
# Restart only the nav stack on the isolated sim master (11312).
# Bracket patterns keep pkill from matching its own command line.

set -o pipefail

ROBOT_IP="${ROBOT_IP:-$(hostname -I | awk '{print $1}')}"
export ROS_MASTER_URI="http://${ROBOT_IP}:11312"
export ROS_IP="${ROBOT_IP}"
source /opt/ros/noetic/setup.bash
source "$HOME/spark_noetic/devel/setup.bash"

mkdir -p "$HOME/logs/sim"
pkill -f "[n]av_arena.launch" 2>/dev/null || true
sleep 1
nohup roslaunch "$HOME/codes/ros/nav_arena.launch" use_sim_time:=true \
    map:="$HOME/codes/maps/arena_map.yaml" \
    >"$HOME/logs/sim/nav.log" 2>&1 &
echo "nav restarted"
