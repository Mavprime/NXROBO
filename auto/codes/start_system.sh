#!/usr/bin/env bash
# Unified background startup for Spark robot subsystems on ROS Noetic
# All long-running processes are fully detached with setsid so they survive
# closing the SSH session / MobaXterm window (the previous cause of ROS master
# and all drivers being killed with "killing on exit").

mkdir -p ~/logs
touch ~/logs/runtask.log ~/logs/base.log ~/logs/arm.log ~/logs/stream.log ~/logs/detect.log

echo "=========================================="
echo "    Starting Spark Robot Subsystems       "
echo "=========================================="

source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/spark_noetic/devel/setup.bash 2>/dev/null
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}

DETECT_NODE=~/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/detect_ros.py
RUNTASK_NODE=~/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/runtask.py

master_up() { timeout 2 rosnode list >/dev/null 2>&1; }
node_up()   { rosnode list 2>/dev/null | grep -qx "$1"; }

# wait_for_node <node_name> <seconds>
wait_for_node() {
    local n="$1" t="${2:-25}" i
    for ((i=0; i<t; i++)); do
        node_up "$n" && return 0
        sleep 1
    done
    return 1
}

# 1. Base driver, RealSense camera, and Lidar
if ! master_up || ! node_up /spark_base_server; then
    echo "[1/5] Starting base, RealSense D435 and YDLidar..."
    pkill -f "driver_bringup.launch" 2>/dev/null
    pkill -f "spark_base_server" 2>/dev/null
    sleep 1
    setsid nohup roslaunch spark_bringup driver_bringup.launch \
        camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6 \
        > ~/logs/base.log 2>&1 < /dev/null &
    for i in $(seq 1 25); do master_up && break; sleep 1; done
    if wait_for_node /spark_base_server 25; then
        echo "      + ROS master and base started."
    else
        echo "      ! Base did not appear. See ~/logs/base.log"
    fi
else
    echo "[1/5] Base and camera are already running."
fi

# 2. uArm manipulator
if ! node_up /swiftpro_write_node; then
    echo "[2/5] Starting uArm manipulator driver..."
    pkill -f "pro_control_nomoveit" 2>/dev/null
    sleep 1
    setsid nohup roslaunch swiftpro pro_control_nomoveit.launch \
        > ~/logs/arm.log 2>&1 < /dev/null &
    if wait_for_node /swiftpro_write_node 20; then
        echo "      + uArm started."
    else
        echo "      ! uArm did not appear. See ~/logs/arm.log"
    fi
else
    echo "[2/5] uArm manipulator is already running."
fi

# 3. Web video stream (restarted so it binds to the current ROS master)
echo "[3/5] Starting camera web stream on port 8080..."
pkill -f "codes/camera_stream\.py" 2>/dev/null
fuser -k 8080/tcp 2>/dev/null
sleep 1
setsid nohup python3 -u ~/semi/camera_stream.py > ~/logs/stream.log 2>&1 < /dev/null &
sleep 1

# 4. Neural network detector
# FORCE_DETECT=1 restarts it even if already running, so freshly uploaded
# detect_ros.py code actually takes effect.
if [ "${FORCE_DETECT:-0}" = "1" ] || ! node_up /detector_node; then
    echo "[4/5] Starting YOLO object detector (detect_ros.py)..."
    pkill -f "auto_match_25/auto_match/nodes/detect_ros\.py" 2>/dev/null
    sleep 1
    setsid nohup python3 -u "$DETECT_NODE" > ~/logs/detect.log 2>&1 < /dev/null &
    # YOLO import + two model loads can take tens of seconds on the Spark CPU.
    # Wait until /objects actually produces a message (even an empty one).
    for i in $(seq 1 90); do
        timeout 2 rostopic echo -n 1 /objects >/dev/null 2>&1 && break
        sleep 1
    done
    if node_up /detector_node; then
        echo "      + Detector started and publishing /objects."
    else
        echo "      ! Detector did not start. See ~/logs/detect.log"
    fi
else
    echo "[4/5] Object detector is already running."
fi

# 5. runtask (always restarted so it picks up the current master and fresh code)
pkill -f "auto_match_25/auto_match/nodes/runtask\.py" 2>/dev/null
sleep 1
echo "[5/5] Starting fresh runtask.py decision node..."
setsid nohup python3 -u "$RUNTASK_NODE" > ~/logs/runtask.log 2>&1 < /dev/null &
sleep 2

ROBOT_IP=$(hostname -I | awk '{print $1}')
echo ""
echo "=========================================="
echo "  All systems started successfully in bg! "
echo "  Camera stream: http://${ROBOT_IP}:8080/ "
echo "  Logs location: ~/logs/                  "
echo "=========================================="
