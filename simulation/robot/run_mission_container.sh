#!/usr/bin/env bash
# Run the full mission loop inside the spark_gazebo container:
#   synthetic world-truth detections + arm mock + mission node, then start.
#
# Run with:
#   docker exec spark_gazebo bash /ws/robot/run_mission_container.sh
# The script's own command line does not contain the pkill patterns.

set -o pipefail
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=/ws/assets:${ROS_PACKAGE_PATH:-}
export ROS_MASTER_URI=http://127.0.0.1:11311

# wait for the nav stack started by run_gazebo.sh
for _ in $(seq 1 40); do
    rosnode list 2>/dev/null | grep -q /move_base && break
    sleep 1
done

pkill -f "[g]azebo_truth_detections" 2>/dev/null || true
pkill -f "[a]rm_mock_node" 2>/dev/null || true
pkill -f "[m]ission_node.py" 2>/dev/null || true
sleep 1

setsid nohup python3 -u /ws/codes/gazebo_truth_detections.py \
    _world:=/ws/config/arena_map.yaml \
    >/tmp/truth.log 2>&1 </dev/null &

setsid nohup python3 -u /ws/codes/arm_mock_node.py \
    _pick_s:=4.0 _place_s:=3.0 \
    >/tmp/arm.log 2>&1 </dev/null &

sleep 2

setsid nohup python3 -u /ws/codes/mission_node.py \
    _world:=/ws/config/arena_map.yaml \
    _mission:=/ws/config/mission_demo.yaml \
    _gate:=/ws/config/gate.yaml \
    >/tmp/mission.log 2>&1 </dev/null &

sleep 3
rostopic pub -1 /task_start_flag std_msgs/String "data: 'start'" >/dev/null 2>&1
echo "mission started; logs: /tmp/truth.log /tmp/arm.log /tmp/mission.log"
