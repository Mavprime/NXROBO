#!/usr/bin/env bash
# Unified interactive control dashboard for Spark robot
# Auto-detects teleoperation scripts and guards against file renaming

source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/spark_noetic/devel/setup.bash 2>/dev/null
mkdir -p ~/logs
touch ~/logs/runtask.log ~/logs/base.log ~/logs/arm.log ~/logs/stream.log ~/logs/detect.log

# Auto-start everything on first open if the core services are down.
# This is what makes the dashboard a true one-click entry point.
if [ "${DASHBOARD_NO_AUTOSTART:-0}" != "1" ]; then
    need_start=0
    timeout 2 rosnode list >/dev/null 2>&1 || need_start=1
    rosnode list 2>/dev/null | grep -qx /spark_base_server || need_start=1
    rosnode list 2>/dev/null | grep -qx /detector_node || need_start=1
    if [ "$need_start" = "1" ]; then
        echo ">> Core services are down. Starting everything automatically..."
        bash ~/codes/start_system.sh
        sleep 1
    fi
fi

while true; do
    trap '' INT  # Block Ctrl+C inside dashboard to prevent accidental exits
    clear
    ROBOT_IP=$(hostname -I | awk '{print $1}')
    echo "======================================================"
    echo "       SPARK ROBOT: UNIFIED CONTROL DASHBOARD         "
    echo "       IP: ${ROBOT_IP} | Web Stream: :8080            "
    echo "======================================================"
    echo "  Background Services Status:"
    pgrep -f "spark_base_server" >/dev/null && echo "    [✓] Base & RealSense D435:  RUNNING" || echo "    [✗] Base: STOPPED"
    pgrep -f "pro_control_nomoveit" >/dev/null && echo "    [✓] Manipulator uArm:       RUNNING" || echo "    [✗] Arm: STOPPED"
    pgrep -f "camera_stream.py" >/dev/null && echo "    [✓] Camera Stream :8080:    RUNNING" || echo "    [✗] Stream: STOPPED"
    pgrep -f "detect_ros.py" >/dev/null && echo "    [✓] Neural Network Detector: RUNNING" || echo "    [✗] Detector: STOPPED"
    pgrep -f "runtask.py" >/dev/null && echo "    [✓] Brain runtask:          RUNNING" || echo "    [✗] runtask: STOPPED"
    rostopic list 2>/dev/null | grep -qx '/move_base/status' && echo "    [✓] Navigation move_base:   RUNNING" || echo "    [✗] Navigation: STOPPED (needs nav_bringup.sh)"
    pgrep -f "rosbag record" >/dev/null && echo "    [✓] Mission bag recorder:   RUNNING (~/logs/bags)" || echo "    [~] Mission bag recorder:   stopped"
    echo "------------------------------------------------------"
    echo "  1) START ALL SYSTEMS (start_system.sh)"
    echo "  2) TEST CUBE GRASP (arm grasps cube and lifts)"
    echo "  3) TEST CUBE RELEASE (pump releases cube)"
    echo "  4) SMOOTH KEYBOARD TELEOP (auto-detect key*.py/teleop*.py)"
    echo "  5) VIEW LIVE RUNTASK LOG (press 'q' or Ctrl+C to exit)"
    echo "  6) STOP ALL SYSTEMS (stop_system.sh)"
    echo "  7) DIAGNOSTICS & SYSTEM TEST (test_all_systems.sh)"
    echo "  8) VIEW DETECTOR LOG (~/logs/detect.log)"
    echo "  9) MISSION BAGS (~/logs/bags) + STOP RECORDER"
    echo "  0) Exit to bash"
    echo "======================================================"
    read -p "Select action [0-8]: " choice

    case $choice in
        1)
            bash ~/codes/start_system.sh
            echo ""
            read -p "Press Enter to return to menu..."
            ;;
        2)
            if ! pgrep -f "spark_base_server" > /dev/null || ! pgrep -f "detect_ros.py" > /dev/null || ! pgrep -f "pro_control_nomoveit" > /dev/null; then
                echo ">> [WARNING] Required subsystems are not running!"
                echo ">> Auto-starting base, camera, detector and arm via start_system.sh..."
                bash ~/codes/start_system.sh
                echo ">> Waiting 4s for neural network to warm up..."
                sleep 4
            fi
            if ! pgrep -f "runtask.py" > /dev/null; then
                echo ">> [INFO] runtask.py not running. Starting..."
                nohup python3 -u ~/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/runtask.py > ~/logs/runtask.log 2>&1 &
                sleep 2
            fi
            echo ">> Sending cube grasp command (/grasp 1)..."
            rostopic pub -1 /grasp std_msgs/String "data: '1'" >/dev/null 2>&1
            echo ">> Executing grasp and visual servoing alignment (up to 15s)..."
            for i in $(seq 1 15); do
                sleep 1
                if grep -E "Cube lifted successfully|sees no cubes|No cube detections|timeout exceeded" ~/logs/runtask.log | tail -n 1 | grep -q .; then
                    break
                fi
            done
            echo ">> Recent runtask entries:"
            tail -n 12 ~/logs/runtask.log
            echo ""
            read -p "Press Enter to return to menu..."
            ;;
        3)
            echo ">> Sending cube release command (/grasp 0)..."
            rostopic pub -1 /grasp std_msgs/String "data: '0'" >/dev/null 2>&1
            echo ">> Executing release and vertical retraction..."
            sleep 3
            echo ">> Recent runtask entries:"
            tail -n 8 ~/logs/runtask.log
            echo ""
            read -p "Press Enter to return to menu..."
            ;;
        4)
            trap - INT  # Restore Ctrl+C for teleoperation
            # Auto-detect newest teleoperation script:
            TELEOP_SCRIPT=$(ls -t ~/codes/key*.py ~/codes/*teleop*.py 2>/dev/null | head -n 1)
            if [ -n "$TELEOP_SCRIPT" ] && [ -f "$TELEOP_SCRIPT" ]; then
                echo "Launching teleoperation script: $TELEOP_SCRIPT"
                python3 "$TELEOP_SCRIPT"
            else
                echo "Teleoperation script not found in ~/codes/. Place key*.py or *teleop*.py"
                sleep 2
            fi
            ;;
        5)
            trap - INT
            if [ -f ~/codes/view_log.py ]; then
                python3 ~/codes/view_log.py
            else
                echo ">> Viewing live log. Press Ctrl+C to return to menu:"
                tail -n 25 -f ~/logs/runtask.log
            fi
            trap '' INT
            ;;
        6)
            bash ~/codes/stop_system.sh
            echo ""
            read -p "Press Enter to return to menu..."
            ;;
        7)
            bash ~/codes/test_all_systems.sh
            echo ""
            read -p "Press Enter to return to menu..."
            ;;
        8)
            trap - INT
            echo ">> Showing last 30 lines of detect.log (Ctrl+C to return):"
            tail -n 30 -f ~/logs/detect.log
            trap '' INT
            ;;
        9)
            echo ">> Mission bags in ~/logs/bags:"
            ls -lh ~/logs/bags 2>/dev/null || echo "   (no bags recorded yet)"
            echo ""
            if pgrep -f "rosbag record" >/dev/null; then
                read -p "Stop the recorder now (the bag is closed cleanly)? [y/N] " ans
                if [ "$ans" = "y" ] || [ "$ans" = "Y" ]; then
                    pkill -INT -f "rosbag record" 2>/dev/null
                    sleep 2
                    pkill -f "rosbag record" 2>/dev/null
                    echo ">> Recorder stopped."
                fi
            else
                echo ">> Recorder is not running (start it with option 1)."
            fi
            echo ""
            read -p "Press Enter to return to menu..."
            ;;
        0)
            trap - INT
            break
            ;;
        *)
            echo "Invalid selection."
            sleep 1
            ;;
    esac
done
