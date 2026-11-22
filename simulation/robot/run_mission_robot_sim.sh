#!/usr/bin/env bash
# Run the full mission loop against the robot-side Gazebo sim (master 11312).
#
#   bash ~/codes/robot/start_sim.sh          # Gazebo + nav
#   bash ~/codes/robot/run_mission_robot_sim.sh
#
# Starts synthetic world-truth detections, the arm mock, and mission_node, then
# publishes /task_start_flag. Does not touch the hardware stack (11311).

set -o pipefail
ROBOT_IP="${ROBOT_IP:-$(hostname -I | awk '{print $1}')}"
export ROS_MASTER_URI="http://${ROBOT_IP}:11312"
export ROS_IP="${ROBOT_IP}"
source /opt/ros/noetic/setup.bash
source "$HOME/spark_noetic/devel/setup.bash"

LOG="$HOME/logs/sim"
mkdir -p "$LOG"

for _ in $(seq 1 40); do
    rosnode list 2>/dev/null | grep -q /move_base && break
    sleep 1
done

pkill -f "[g]azebo_truth_detections" 2>/dev/null || true
pkill -f "[a]rm_mock_node" 2>/dev/null || true
pkill -f "[m]ission_node.py" 2>/dev/null || true
sleep 1

setsid nohup python3 -u "$HOME/codes/gazebo_truth_detections.py" \
    _world:="$HOME/codes/config/arena_map.yaml" \
    >"$LOG/truth.log" 2>&1 </dev/null &

setsid nohup python3 -u "$HOME/codes/arm_mock_node.py" \
    _pick_s:=4.0 _place_s:=3.0 \
    >"$LOG/arm.log" 2>&1 </dev/null &

sleep 2

setsid nohup python3 -u "$HOME/codes/mission_node.py" \
    _world:="$HOME/codes/config/arena_map.yaml" \
    _mission:="$HOME/codes/config/mission_demo.yaml" \
    _gate:="$HOME/codes/config/gate.yaml" \
    >"$LOG/mission.log" 2>&1 </dev/null &

sleep 3
rostopic pub -1 /task_start_flag std_msgs/String "data: 'start'" >/dev/null 2>&1
echo "mission started on 11312; logs: $LOG/{truth,arm,mission}.log"
