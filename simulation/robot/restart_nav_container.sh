#!/usr/bin/env bash
# Restart nav inside the spark_gazebo container. Run with:
#   docker exec spark_gazebo bash /ws/robot/restart_nav_container.sh
# The script's own command line does not contain the pkill pattern, so pkill only
# matches the old roslaunch process.

set -o pipefail
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=/ws/assets:${ROS_PACKAGE_PATH:-}

pkill -f "[n]av_arena.launch" 2>/dev/null || true
sleep 1
setsid nohup roslaunch /ws/ros/nav_arena.launch use_sim_time:=true \
    map:=/ws/maps/arena_map.yaml \
    move_base_launch:=/ws/ros/move_base_arena.launch \
    >/tmp/nav.log 2>&1 </dev/null &
echo "nav restarted in container"
