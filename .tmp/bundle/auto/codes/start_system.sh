#!/usr/bin/env bash
# Unified background startup for Spark robot subsystems on ROS Noetic.
# All long-running processes are fully detached with setsid so they survive
# closing the SSH session / MobaXterm window (the previous cause of ROS master
# and all drivers being killed with "killing on exit").
#
# Environment switches:
#   FORCE_DETECT=1   restart detect_ros.py even if it is already running
#   ROSBAG=0         do not record a mission bag (default: record)
#   ROSBAG_CAMERA=1  also record /camera/color/image_raw (big: ~1.5 MB/frame)
#   LIDAR_LAUNCH=... override the lidar launch (default
#                    "lidar_driver_transfer ydlidar_g6.launch"); set to "none"
#                    to skip it.
#
# Why the lidar is started explicitly: spark_bringup/driver_bringup.launch
# starts the description, the chassis and the camera, but NOT any lidar driver
# (verified on 2026-11-20: the lidar_type_tel argument only reaches the URDF).
# Without /scan there is no costmap, so the mission can never navigate.
#
# The navigation stack (map_server / amcl / move_base / mark_nav) is still not
# started here: it arrives with the arena map (Phase 2). Provide the exact
# command once in ~/codes/nav_bringup.sh (executable) and this script will run
# it and wait for the move_base node.

mkdir -p ~/logs
touch ~/logs/runtask.log ~/logs/base.log ~/logs/arm.log ~/logs/stream.log \
      ~/logs/detect.log ~/logs/lidar.log ~/logs/nav.log ~/logs/rosbag.log

echo "=========================================="
echo "    Starting Spark Robot Subsystems       "
echo "=========================================="

source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/spark_noetic/devel/setup.bash 2>/dev/null
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}

DETECT_NODE=~/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/detect_ros.py
RUNTASK_NODE=~/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/runtask.py
LIDAR_LAUNCH=${LIDAR_LAUNCH:-"lidar_driver_transfer ydlidar_g6.launch"}

master_up() { timeout 2 rosnode list >/dev/null 2>&1; }
node_up()   { rosnode list 2>/dev/null | grep -qx "$1"; }
# A node, not a topic: rosbag subscribing to /move_base/status would otherwise
# make a missing navigation stack look healthy.
nav_ready() { node_up /move_base; }
lidar_up()  { rosnode list 2>/dev/null | grep -qE "ydlidar|lidar|delta_lidar"; }

# wait_for_node <node_name> <seconds>
wait_for_node() {
    local n="$1" t="${2:-25}" i
    for ((i=0; i<t; i++)); do
        node_up "$n" && return 0
        sleep 1
    done
    return 1
}

# 1. Base driver, RealSense camera
if ! master_up || ! node_up /spark_base_server; then
    echo "[1/8] Starting base and RealSense D435..."
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
    echo "[1/8] Base and camera are already running."
fi

# 2. uArm manipulator
if ! node_up /swiftpro_write_node; then
    echo "[2/8] Starting uArm manipulator driver..."
    rm -f /tmp/arm_calib_done
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
    echo "[2/8] uArm manipulator is already running."
fi

# 3. YDLidar G6 -> /scan (navigation depends on it)
if [ "$LIDAR_LAUNCH" = "none" ]; then
    echo "[3/8] LIDAR_LAUNCH=none: skipping the lidar."
elif lidar_up; then
    echo "[3/8] Lidar is already running."
else
    echo "[3/8] Starting lidar ($LIDAR_LAUNCH)..."
    pkill -f "ydlidar_ros_driver_node" 2>/dev/null
    pkill -f "delta_lidar_node" 2>/dev/null
    sleep 1
    # shellcheck disable=SC2086
    setsid nohup roslaunch $LIDAR_LAUNCH > ~/logs/lidar.log 2>&1 < /dev/null &
    for i in $(seq 1 20); do lidar_up && break; sleep 1; done
    if lidar_up; then
        echo "      + Lidar is publishing /scan."
    else
        echo "      ! Lidar did not start. See ~/logs/lidar.log"
    fi
fi

# 4. Web video stream (restarted so it binds to the current ROS master)
echo "[4/8] Starting camera web stream on port 8080..."
pkill -f "codes/camera_stream\.py" 2>/dev/null
fuser -k 8080/tcp >/dev/null 2>&1
sleep 1
setsid nohup python3 -u ~/codes/camera_stream.py > ~/logs/stream.log 2>&1 < /dev/null &
sleep 1

# 5. Neural network detector
# FORCE_DETECT=1 restarts it even if already running, so freshly uploaded
# detect_ros.py code actually takes effect.
if [ "${FORCE_DETECT:-0}" = "1" ] || ! node_up /detector_node; then
    echo "[5/8] Starting YOLO object detector (detect_ros.py)..."
    pkill -f "auto_match_25/auto_match/nodes/detect_ros\.py" 2>/dev/null
    sleep 1
    # ROS keeps private parameters from the previous node instance on the
    # parameter server, so a one-off `_cake_conf:=...` would silently survive
    # a restart.  Clear them so detect_ros.py file defaults are used.
    rosparam delete /detector_node/cake_conf 2>/dev/null
    rosparam delete /detector_node/cake_site_conf 2>/dev/null
    setsid nohup python3 -u "$DETECT_NODE" _augment:=true _conf:=0.25 > ~/logs/detect.log 2>&1 < /dev/null &
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
    echo "[5/8] Object detector is already running."
fi

# 6. Navigation stack (optional hook, see the header comment)
if [ -x ~/codes/nav_bringup.sh ]; then
    if nav_ready; then
        echo "[6/8] Navigation stack already running."
    else
        echo "[6/8] Starting the navigation stack from ~/codes/nav_bringup.sh..."
        pkill -f "nav_bringup.sh" 2>/dev/null
        sleep 1
        setsid nohup bash ~/codes/nav_bringup.sh > ~/logs/nav.log 2>&1 < /dev/null &
        for i in $(seq 1 40); do nav_ready && break; sleep 1; done
        if nav_ready; then
            echo "      + move_base is up."
        else
            echo "      ! move_base still missing. See ~/logs/nav.log"
        fi
    fi
else
    echo "[6/8] No ~/codes/nav_bringup.sh: navigation stack not started."
    if ! nav_ready; then
        echo "      ! move_base is NOT running. The autonomous mission can"
        echo "        select and grasp cubes, but it cannot drive between"
        echo "        waypoints until the arena navigation stack is launched."
    fi
fi

# 7. Mission recorder
if [ "${ROSBAG:-1}" = "1" ]; then
    if pgrep -f "rosbag record" >/dev/null; then
        echo "[7/8] A bag is already being recorded."
    else
        mkdir -p ~/logs/bags
        BAG_TOPICS="/objects /cmd_vel /scan /tf /tf_static /move_base/status \
/move_base/result /position_write_topic /pump_topic /grasp_status \
/task_start_flag /spark_base/sensor"
        if [ "${ROSBAG_CAMERA:-0}" = "1" ]; then
            BAG_TOPICS="$BAG_TOPICS /camera/color/image_raw"
            echo "[7/8] Recording a mission bag (with camera: expect GBs)..."
        else
            echo "[7/8] Recording a mission bag (topics only; ROSBAG_CAMERA=1 adds video)..."
        fi
        # shellcheck disable=SC2086
        setsid nohup rosbag record -o ~/logs/bags/match_ --lz4 -b 1024 \
            $BAG_TOPICS > ~/logs/rosbag.log 2>&1 < /dev/null &
        sleep 1
        if pgrep -f "rosbag record" >/dev/null; then
            echo "      + Bag recording into ~/logs/bags/"
        else
            echo "      ! rosbag did not start. See ~/logs/rosbag.log"
        fi
    fi
else
    echo "[7/8] ROSBAG=0: mission bag recording disabled."
fi

# 8. Mission node (always restarted so it picks up the current master and new code)
pkill -f "auto_match_25/auto_match/nodes/runtask\.py" 2>/dev/null
sleep 1
echo "[8/8] Starting fresh runtask.py decision node..."
setsid nohup python3 -u "$RUNTASK_NODE" > ~/logs/runtask.log 2>&1 < /dev/null &
sleep 2

ROBOT_IP=$(hostname -I | awk '{print $1}')
echo ""
echo "=========================================="
echo "  All systems started successfully in bg! "
echo "  Camera stream: http://${ROBOT_IP}:8080/ "
echo "  Logs location: ~/logs/                  "
echo "  Mission bags:  ~/logs/bags/             "
echo "=========================================="
