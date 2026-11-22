#!/usr/bin/env bash
# Start Gazebo + navigation on the robot's NUC, isolated from the hardware stack.
# The real stack usually runs on ROS master 11311; this sim uses 11312.
#
# Deploy: copy simulation/robot/*.sh and the sim assets to the robot (see README).

set -o pipefail

ROBOT_IP="${ROBOT_IP:-$(hostname -I | awk '{print $1}')}"
export ROS_MASTER_URI="http://${ROBOT_IP}:11312"
export ROS_IP="${ROBOT_IP}"
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/noetic/setup.bash
source "$HOME/spark_noetic/devel/setup.bash"

LOG="$HOME/logs/sim"
mkdir -p "$LOG"
SIM="$HOME/codes/sim"

pkill -f "[s]park_arena.launch" 2>/dev/null || true
pkill -f "[n]av_arena.launch" 2>/dev/null || true
pkill -f "[r]osmaster.*11312" 2>/dev/null || true
pkill -f "[g]zserver.*arena.world" 2>/dev/null || true
sleep 1

nohup roscore -p 11312 >"$LOG/roscore.log" 2>&1 &
sleep 2

nohup roslaunch "$SIM/spark_arena.launch" gui:=false \
    world:="$SIM/arena.world" \
    model:="$SIM/spark1_description_nav.urdf" \
    x:=0.225 y:=2.775 z:=0.05 yaw:=-0.785398 \
    >"$LOG/gazebo.log" 2>&1 &
echo "gazebo launching..."

for _ in $(seq 1 20); do
    if timeout 2 bash -lc 'source /opt/ros/noetic/setup.bash; rostopic list 2>/dev/null | grep -q "^/clock$"' ; then
        break
    fi
    sleep 1
done

nohup roslaunch "$HOME/codes/ros/nav_arena.launch" use_sim_time:=true \
    map:="$HOME/codes/maps/arena_map.yaml" \
    >"$LOG/nav.log" 2>&1 &
echo "navigation launching..."

echo "logs: $LOG"
echo "check: ROS_MASTER_URI=$ROS_MASTER_URI rostopic list"
echo "stop:  $HOME/codes/robot/stop_sim.sh"
