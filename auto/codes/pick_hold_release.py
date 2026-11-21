#!/usr/bin/env python3
"""
Detect the blue box, grab it with the vacuum pump, lift it for 3 seconds,
then release it and go home.

Sequence (timings from the Spark grasp examples):
    1. detect blue box (HSV) and average its pixel centre
    2. pixel -> arm coordinates using ~/thefile.txt
    3. reachability check (the firmware silently ignores unreachable poses)
    4. attach arm (status 0 -> 1)
    5. move to hover height, wait 2 s
    6. move down to touch height, wait 4 s
    7. vacuum ON  -> grip, wait 1.5 s
    8. lift straight up to LIFT_Z, hold for HOLD_SECONDS (default 3 s)
    9. vacuum OFF -> release
   10. go home

Prerequisites:
    camera: roslaunch spark_bringup driver_bringup.launch camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6
    arm:    roslaunch swiftpro pro_control_nomoveit.launch

Usage:
    python3 ~/codes/pick_hold_release.py
    python3 ~/codes/pick_hold_release.py --dry-run

Environment overrides (millimetres / seconds):
    HOVER_Z=20 TOUCH_Z=-50 LIFT_Z=120 HOLD_SECONDS=3

Note: release happens at LIFT_Z, so the box drops from that height. For a
gentle release, lower the lift height or ask for a controlled-release variant.

Keep hands clear. If this process is killed with SIGKILL while the pump is on,
the vacuum stays on - stop it with pump_test.py or power-cycle the arm.
"""

import os
import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from swiftpro.msg import position

try:
    from swiftpro.msg import status
except ImportError:
    status = None

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_ik import ik_reachable, pixel_to_arm

CALIBRATION_FILE = "/home/spark/thefile.txt"

LOWER = np.array([90, 50, 50])
UPPER = np.array([135, 255, 255])
MIN_AREA = 300
FRAMES = 10

HOVER_Z = float(os.environ.get("HOVER_Z", "20"))
TOUCH_Z = float(os.environ.get("TOUCH_Z", "-50"))
LIFT_Z = float(os.environ.get("LIFT_Z", "120"))
HOLD_SECONDS = float(os.environ.get("HOLD_SECONDS", "3"))
HOME = (120.0, 0.0, 35.0)

DRY_RUN = "--dry-run" in sys.argv
ROS_ARGV = [arg for arg in sys.argv if arg != "--dry-run"]


def read_calibration(path):
    with open(path) as handle:
        values = [float(v) for v in handle.read().split()]
    if len(values) < 4:
        raise RuntimeError("calibration file needs 4 values: %s" % path)
    return (values[0], values[1]), (values[2], values[3])


def detect(frame, kernel):
    mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), LOWER, UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.dilate(cv2.erode(mask, None, iterations=4), None, iterations=4)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) > MIN_AREA]
    if not contours:
        return None
    biggest = max(contours, key=cv2.contourArea)
    moment = cv2.moments(biggest)
    if moment["m00"] == 0:
        return None
    return moment["m10"] / moment["m00"], moment["m01"] / moment["m00"], cv2.contourArea(biggest)


def attach_arm(publisher):
    if publisher is None or status is None:
        return
    publisher.publish(0)
    rospy.sleep(1.0)
    publisher.publish(1)
    rospy.sleep(1.0)


def set_pump(publisher, on):
    if publisher is None or status is None:
        return
    message = status()
    message.status = 1 if on else 0
    publisher.publish(message)


def main():
    x_kb, y_kb = read_calibration(CALIBRATION_FILE)
    print("calibration: x=%.5f*row+%.2f  y=%.5f*col+%.2f"
          % (x_kb[0], x_kb[1], y_kb[0], y_kb[1]))

    rospy.init_node("pick_hold_release", anonymous=True, argv=ROS_ARGV)
    arm_pub = rospy.Publisher("/position_write_topic", position, queue_size=10)
    pump_pub = rospy.Publisher("/pump_topic", status, queue_size=1) if status else None
    status_pub = rospy.Publisher("/swiftpro_status_topic", status, queue_size=1) if status else None
    rospy.sleep(0.5)

    bridge = CvBridge()
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))

    samples = []
    for _ in range(FRAMES):
        msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5.0)
        result = detect(bridge.imgmsg_to_cv2(msg, "bgr8"), kernel)
        if result:
            samples.append(result)

    if not samples:
        print("no blue box")
        return 1

    pixel_col = sum(s[0] for s in samples) / len(samples)
    pixel_row = sum(s[1] for s in samples) / len(samples)
    area = int(max(s[2] for s in samples))

    arm_x, arm_y = pixel_to_arm(pixel_row, pixel_col, x_kb, y_kb)
    reachable = all(ik_reachable(arm_x, arm_y, z) for z in (HOVER_Z, TOUCH_Z, LIFT_Z))

    print("pixel: col=%.0f row=%.0f area=%d" % (pixel_col, pixel_row, area))
    print("arm:   x=%.1f y=%.1f  hover=%.0f touch=%.0f lift=%.0f  reachable=%s"
          % (arm_x, arm_y, HOVER_Z, TOUCH_Z, LIFT_Z, reachable))

    if not reachable:
        print("UNREACHABLE: the arm would ignore this pose. Move the box toward "
              "the lower-middle of the image (rows about 320-400) and retry.")
        return 2

    if DRY_RUN:
        print("dry run: not moving")
        return 0

    attach_arm(status_pub)
    pose = position()
    pose.x, pose.y = arm_x, arm_y

    try:
        # hover
        pose.z = HOVER_Z
        arm_pub.publish(pose)
        print("hover z=%.0f" % HOVER_Z)
        rospy.sleep(2.0)

        # down onto the box
        pose.z = TOUCH_Z
        arm_pub.publish(pose)
        print("down z=%.0f" % TOUCH_Z)
        rospy.sleep(4.0)

        # vacuum on, let it grip
        set_pump(pump_pub, True)
        print("pump ON (grip)")
        rospy.sleep(1.5)

        # lift and hold
        pose.z = LIFT_Z
        arm_pub.publish(pose)
        print("lift z=%.0f - holding %.1f s" % (LIFT_Z, HOLD_SECONDS))
        rospy.sleep(HOLD_SECONDS)

        # release
        set_pump(pump_pub, False)
        print("pump OFF (release)")
        rospy.sleep(0.5)
    finally:
        # never leave the pump running
        set_pump(pump_pub, False)
        pose.x, pose.y, pose.z = HOME
        arm_pub.publish(pose)
        print("home")
        rospy.sleep(1.0)

    return 0


if __name__ == "__main__":
    sys.exit(main())
