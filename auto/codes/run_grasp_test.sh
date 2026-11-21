#!/usr/bin/env bash
# =============================================================================
# Automated driver check & bringup for cube grasping test
# Robot: NXROBO Spark (Ubuntu 20.04, ROS Noetic)
# =============================================================================

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

echo -e "${CYAN}${BOLD}"
echo "=========================================================="
echo "      SPARK ROBOT - CUBE GRASP PREPARATION & TEST       "
echo "=========================================================="
echo -e "${NC}"

source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/spark_noetic/devel/setup.bash 2>/dev/null
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}

mkdir -p ~/logs

node_up() { rosnode ping -c 1 "$1" >/dev/null 2>&1; }
topic_has_msg() { timeout "$1" rostopic echo -n 1 "$2" >/dev/null 2>&1; }

# -----------------------------------------------------------------------------
# 1. Check / Start ROS Core & Base Driver (spark_bringup)
# -----------------------------------------------------------------------------
echo -e "[1/4] Checking base chassis and camera..."
if ! node_up /spark_base_server; then
    echo -e "      ${YELLOW}Launching base drivers (driver_bringup.launch)...${NC}"
    setsid nohup roslaunch spark_bringup driver_bringup.launch \
        camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6 \
        > ~/logs/base.log 2>&1 < /dev/null &
    
    for i in $(seq 1 25); do
        if node_up /spark_base_server; then
            break
        fi
        sleep 1
    done
fi

if node_up /spark_base_server; then
    echo -e "      ${GREEN}[OK] Base chassis and camera active.${NC}"
else
    echo -e "      ${RED}[ERROR] Base chassis failed to start! See ~/logs/base.log${NC}"
    exit 1
fi

# -----------------------------------------------------------------------------
# 2. Check / Start uArm Manipulator Driver (swiftpro)
# -----------------------------------------------------------------------------
echo -e "[2/4] Checking uArm manipulator driver..."
if ! node_up /swiftpro_write_node; then
    echo -e "      ${YELLOW}Launching manipulator driver (pro_control_nomoveit.launch)...${NC}"
    pkill -f pro_control_nomoveit 2>/dev/null
    pkill -f swiftpro 2>/dev/null
    echo y | rosnode cleanup >/dev/null 2>&1
    sleep 1
    setsid nohup roslaunch swiftpro pro_control_nomoveit.launch \
        > ~/logs/arm.log 2>&1 < /dev/null &
    
    for i in $(seq 1 20); do
        if node_up /swiftpro_write_node; then
            break
        fi
        sleep 1
    done
fi

if node_up /swiftpro_write_node; then
    echo -e "      ${GREEN}[OK] uArm manipulator active.${NC}"
else
    echo -e "      ${RED}[ERROR] uArm manipulator failed to start! See ~/logs/arm.log${NC}"
    exit 2
fi

# -----------------------------------------------------------------------------
# 3. Check / Start YOLO Object Detector (detect_ros.py)
# -----------------------------------------------------------------------------
echo -e "[3/4] Checking neural network cube detector (/detector_node)..."
DETECT_NODE=~/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/detect_ros.py
if ! node_up /detector_node; then
    echo -e "      ${YELLOW}Launching neural network cube detector...${NC}"
    setsid nohup python3 -u "$DETECT_NODE" _augment:=true _conf:=0.25 > ~/logs/detect.log 2>&1 < /dev/null &
    
    echo -e "      Waiting for neural network weights initialization..."
    for i in $(seq 1 60); do
        if node_up /detector_node && topic_has_msg 2 /objects; then
            break
        fi
        sleep 1
    done
fi

if node_up /detector_node; then
    echo -e "      ${GREEN}[OK] Cube detector active and publishing to /objects.${NC}"
else
    echo -e "      ${RED}[ERROR] Detector failed to start! See ~/logs/detect.log${NC}"
    exit 3
fi

# -----------------------------------------------------------------------------
# 4. Run Grasp Logic
# -----------------------------------------------------------------------------
echo -e "[4/4] ${CYAN}${BOLD}All drivers ready. Starting grasp logic...${NC}"
echo "=========================================================="
echo ""

python3 -u ~/codes/grasp_generic.py "$@"
EXIT_CODE=$?

echo ""
echo "=========================================================="
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}${BOLD}[OK] Test completed successfully.${NC}"
else
    echo -e "${RED}${BOLD}[ERROR] Test failed with exit code: $EXIT_CODE${NC}"
fi
echo "=========================================================="
exit $EXIT_CODE
