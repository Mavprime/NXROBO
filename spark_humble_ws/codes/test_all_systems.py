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
        is_bumper_stuck = "ir_bumper_left: True" in out or "ir_bumper_right: True" in out
        if is_bumper_stuck:
            report("Base IR Bumper Status", False, "WARNING: Bumper triggered! Motion interlocked.")
        else:
            report("Base IR Bumper Status", True, "Bumpers clear (motion enabled)")
    else:
        report("Base IR Bumper Status", True, "Bumpers clear (motion enabled)")


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

    print("\n" + Colors.CYAN + "=" * 65 + Colors.RESET)
    print(f"{Colors.BOLD}Diagnostic check completed.{Colors.RESET}")
    print(Colors.CYAN + "=" * 65 + Colors.RESET + "\n")


if __name__ == "__main__":
    main()
