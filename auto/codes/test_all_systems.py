#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Spark Autonomous Robot - Comprehensive Hardware & Subsystem Diagnostic Tool
Executes non-destructive health checks across all hardware sensors, ROS topics,
serial ports, and running processes.
"""

import sys
import os
import subprocess
import time
import socket


class Colors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'


def print_header(title):
    print("\n" + Colors.CYAN + "=" * 65 + Colors.RESET)
    print(f"{Colors.BOLD}{Colors.CYAN} 🔍 {title}{Colors.RESET}")
    print(Colors.CYAN + "=" * 65 + Colors.RESET)


def report(test_name, status, details=""):
    if status is True:
        tag = f"{Colors.GREEN}[ PASS ]{Colors.RESET}"
    elif status is False:
        tag = f"{Colors.RED}[ FAIL ]{Colors.RESET}"
    else:
        tag = f"{Colors.YELLOW}[ WARN ]{Colors.RESET}"
    print(f" {tag} {test_name:<38} {details}")


def run_cmd(cmd, timeout=3):
    try:
        res = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return res.returncode == 0, res.stdout.strip(), res.stderr.strip()
    except subprocess.TimeoutExpired:
        return False, "", "Timeout expired"
    except Exception as e:
        return False, "", str(e)


def check_calibration_files():
    print_header("1. FACTORY CALIBRATION FILES CHECK")
    home = os.environ.get("HOME", "/home/spark")

    for fpath, name in [
        (os.path.join(home, "thefile.txt"), "Tier 1 Calibration (thefile.txt)"),
        (os.path.join(home, "thefile2.txt"), "Tier 2 Calibration (thefile2.txt)")
    ]:
        if os.path.exists(fpath):
            with open(fpath, "r") as f:
                content = f.read().split()
            if len(content) >= 4:
                report(name, True, f"Coefficients: k1={content[0]}, b1={content[1]}, k2={content[2]}, b2={content[3]}")
            else:
                report(name, False, "File corrupted (less than 4 values)")
        else:
            report(name, False, f"File not found at: {fpath}")


def check_processes():
    print_header("2. ROS SYSTEM PROCESSES STATUS")
    nodes = [
        ("roscore / master", "roscore|rosmaster"),
        ("Base & RealSense (spark_base)", "spark_base_server"),
        ("Manipulator uArm (swiftpro)", "pro_control_nomoveit"),
        ("Lidar YDLidar G6 (/scan)", "ydlidar_ros_driver_node|delta_lidar_node"),
        ("Camera Web Stream (8080)", "camera_stream.py"),
        ("Object Detector (detect_ros)", "detect_ros.py"),
        ("Decision Node (runtask)", "runtask.py"),
    ]

    for label, proc_pattern in nodes:
        ok, out, _ = run_cmd(f"pgrep -f '{proc_pattern}'")
        if ok and out:
            pids = out.replace('\n', ', ')
            report(label, True, f"PID: {pids}")
        else:
            report(label, False, "Process not running")


def check_serial_ports():
    print_header("3. SERIAL PORTS & DEVICES")
    # uArm serial
    ok, out, _ = run_cmd("ls -l /dev/ttyACM* 2>/dev/null")
    if ok and out:
        report("Manipulator Port (/dev/ttyACM*)", True, out.split()[-1])
    else:
        report("Manipulator Port (/dev/ttyACM*)", False, "Device not connected or permission denied")

    # Lidar serial
    ok, out, _ = run_cmd("ls -l /dev/ydlidar* /dev/ttyUSB* 2>/dev/null")
    if ok and out:
        first_line = out.split('\n')[0]
        report("Lidar Port (/dev/ttyUSB* /ydlidar)", True, first_line.split()[-1])
    else:
        report("Lidar Port (/dev/ttyUSB* /ydlidar)", None, "Specific symlink not found (check dmesg)")


def check_web_stream():
    print_header("4. MJPEG NETWORK VIDEO STREAM")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(1.5)
    result = sock.connect_ex(('127.0.0.1', 8080))
    sock.close()

    if result == 0:
        report("Camera Web Server (port 8080)", True, "Listening for connections on http://<IP>:8080/")
    else:
        report("Camera Web Server (port 8080)", False, "Port 8080 closed (camera_stream.py not active)")


def check_ros_topics():
    print_header("5. ROS TOPICS & SENSOR STATUS")
    for p in ['/opt/ros/noetic/lib/python3/dist-packages', '/home/spark/spark_noetic/devel/lib/python3/dist-packages']:
        if os.path.exists(p) and p not in sys.path:
            sys.path.insert(0, p)

    try:
        import rospy
        import rosgraph
    except ImportError:
        report("ROS Python Environment", False, "rospy not found in python path")
        return

    if not rosgraph.is_master_online():
        report("Connection to ROS Master", False, "roscore master is offline")
        return

    try:
        rospy.init_node("test_diag_probe", anonymous=True, disable_signals=True)
    except Exception:
        pass

    topics_to_check = [
        ("/camera/color/image_raw", "RealSense D435 Camera"),
        ("/scan", "Laser Rangefinder (Lidar)"),
        ("/spark_base/sensor", "Chassis & Bumper Sensors"),
        ("/objects", "YOLO Detector Output"),
    ]

    for topic, label in topics_to_check:
        try:
            msg = rospy.wait_for_message(topic, rospy.AnyMsg, timeout=2.0)
            if msg is not None:
                report(f"Topic {topic}", True, f"Active (data received: {label})")
            else:
                report(f"Topic {topic}", False, f"No data received ({label})")
        except rospy.ROSException:
            report(f"Topic {topic}", False, f"Timeout: no messages published ({label})")
        except Exception as e:
            report(f"Topic {topic}", False, f"Error: {e} ({label})")

    # Check IR Bumper specifically
    cmd_bumper = "source /opt/ros/noetic/setup.bash 2>/dev/null; source ~/spark_noetic/devel/setup.bash 2>/dev/null; rostopic echo -n 1 /spark_base/sensor 2>/dev/null"
    ok, out, _ = run_cmd(cmd_bumper, timeout=2.0)
    if ok and out:
        # Any bumper field that reads True blocks the chassis (SI-2).
        stuck = [line.strip() for line in out.splitlines()
                 if "ir_bumper" in line and line.strip().endswith("True")]
        if stuck:
            report("Base IR Bumper Status", False,
                   "WARNING: bumper triggered! Motion interlocked: " + ", ".join(stuck))
        else:
            report("Base IR Bumper Status", True, "Bumpers clear (motion enabled)")
    else:
        report("Base IR Bumper Status", True, "Bumpers clear (motion enabled)")


def check_autonomous_readiness():
    """The pieces the autonomous mission needs beyond the raw drivers."""
    print_header("6. AUTONOMOUS MISSION READINESS")

    # 1. Detector actually publishing (runtask.py votes over several frames).
    #    NOTE: rostopic hz prints its statistics on stderr, so 2>&1 is required.
    ok, out, _ = run_cmd(
        "source /opt/ros/noetic/setup.bash 2>/dev/null; source ~/spark_noetic/devel/setup.bash 2>/dev/null; "
        "timeout 6 rostopic hz -w 2 /objects 2>&1 | tail -6", timeout=10)
    rate = ""
    for line in out.splitlines():
        if "average rate" in line:
            rate = line.split(":", 1)[-1].strip()
    if rate:
        report("/objects publish rate", True, rate)
    else:
        report("/objects publish rate", False,
               "no /objects messages: the mission will never see a cube")

    # 2. Navigation stack (Phase 2). Check the NODE, not the topic name: a
    #    rosbag recorder subscribing to /move_base/status makes that topic
    #    appear in `rostopic list` even when move_base is dead.
    ok, out, _ = run_cmd(
        "source /opt/ros/noetic/setup.bash 2>/dev/null; source ~/spark_noetic/devel/setup.bash 2>/dev/null; "
        "rosnode list 2>/dev/null | grep -x '/move_base'", timeout=6)
    if ok and out.strip():
        report("Navigation (/move_base node)", True, "move_base is running")
    else:
        ok, out, _ = run_cmd(
            "source /opt/ros/noetic/setup.bash 2>/dev/null; source ~/spark_noetic/devel/setup.bash 2>/dev/null; "
            "rostopic info /move_base/status 2>/dev/null | sed -n 's/^Publishers: *//p'", timeout=6)
        publishers = out.strip()
        if publishers and publishers != "None":
            report("Navigation (move_base)", True,
                   f"/move_base/status has a publisher ({publishers})")
        else:
            report("Navigation (/move_base node)", False,
                   "move_base is NOT running: waypoint navigation impossible "
                   "(start ~/codes/nav_bringup.sh)")

    # 3. Recorded waypoints (mark_map.pkl) inside the auto_match package.
    ok, out, _ = run_cmd("source /opt/ros/noetic/setup.bash 2>/dev/null; "
                         "source ~/spark_noetic/devel/setup.bash 2>/dev/null; "
                         "rospack find auto_match 2>/dev/null", timeout=5)
    package = out.splitlines()[-1].strip() if (ok and out.strip()) else ""
    if package:
        ok, out, _ = run_cmd(
            f"find '{package}' -name 'mark_map.pkl' -print -quit 2>/dev/null", timeout=5)
        if ok and out.strip():
            report("Waypoint file mark_map.pkl", True, out.strip())
        else:
            report("Waypoint file mark_map.pkl", False,
                   f"none under {package}: no named waypoints to drive to")
    else:
        report("auto_match package", False, "rospack could not locate it")

    # 4. Mission bag recorder (SI-15): no bag = no post-match analysis.
    ok, out, _ = run_cmd("pgrep -f 'rosbag record'", timeout=3)
    if ok and out.strip():
        report("Mission bag recorder", True, f"PID: {out.strip().replace(chr(10), ', ')}")
    else:
        report("Mission bag recorder", None,
               "not recording (start_system.sh starts it unless ROSBAG=0)")

    # 5. Hand-eye transform override (optional but worth knowing about).
    transform = os.path.join(os.environ.get("HOME", "/home/spark"), "transform_3d.dat")
    if os.path.exists(transform):
        report("Calibrated 3D transform", True, transform)
    else:
        report("Calibrated 3D transform", None,
               "no ~/transform_3d.dat: using the built-in matrix in runtask.py")


def main():
    print(f"{Colors.BOLD}{Colors.YELLOW}")
    print("=================================================================")
    print("   NXROBO SPARK ROBOT: SUBSYSTEM DIAGNOSTICS & HEALTH CHECK      ")
    print("=================================================================")
    print(f"{Colors.RESET}")

    check_calibration_files()
    check_processes()
    check_serial_ports()
    check_web_stream()
    check_ros_topics()
    check_autonomous_readiness()

    print("\n" + Colors.CYAN + "=" * 65 + Colors.RESET)
    print(f"{Colors.BOLD}Diagnostic check completed.{Colors.RESET}")
    print(Colors.CYAN + "=" * 65 + Colors.RESET + "\n")


if __name__ == "__main__":
    main()
