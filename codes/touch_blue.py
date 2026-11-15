#!/usr/bin/env python3
"""
Detect the blue box and move the uArm suction cup down to touch its top.

Implemented from the Spark documentation and the shipped grasp examples:
  1. HSV blue mask -> largest contour centroid (pixel_col, pixel_row), averaged
     over several frames.
  2. Calibration from ~/thefile.txt (k1 b1 k2 b2):
         arm_x = k1 * pixel_row + b1
         arm_y = k2 * pixel_col + b2
  3. Reachability check with the same IK math used by swiftpro_rviz_node.cpp.
     Unreachable poses are silently ignored by the arm firmware, so abort early.
  4. Attach the arm (status 0 -> 1), move to z_hover, wait, move to z_touch,
     wait, then go home.

Documented heights vary by object height:
    ROS auto-match doc:  hover z = -45, touch z = -55   (floor objects)
    M-Robots doc:        hover z =  20, touch z = -45
    Spark-H HM task:     hover z = 120, touch z =  95   (cargo on a table)
For a floor box the one-key demo uses 20 / -50, which is the default here.
Override with environment variables if needed:
    HOVER_Z=20 TOUCH_Z=-55 python3 ~/codes/touch_blue.py

Prerequisites:
    camera: roslaunch spark_bringup driver_bringup.launch camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6
    arm:    roslaunch swiftpro pro_control_nomoveit.launch

Usage:
    python3 ~/codes/touch_blue.py            # move and touch
    python3 ~/codes/touch_blue.py --dry-run  # print coordinates/reachability only

Keep hands clear: the arm moves directly to the computed point.
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
    """0 -> 1 on /swiftpro_status_topic, as in the M-Robots grasp code."""
    if publisher is None or status is None:
        return
    publisher.publish(0)
    rospy.sleep(1.0)
    publisher.publish(1)
    rospy.sleep(1.0)


def main():
    x_kb, y_kb = read_calibration(CALIBRATION_FILE)
    print("calibration: x=%.5f*row+%.2f  y=%.5f*col+%.2f"
          % (x_kb[0], x_kb[1], y_kb[0], y_kb[1]))

    rospy.init_node("touch_blue", anonymous=True, argv=ROS_ARGV)
    pub = rospy.Publisher("/position_write_topic", position, queue_size=10)
    status_pub = None
    if status is not None:
        status_pub = rospy.Publisher("/swiftpro_status_topic", status, queue_size=1)
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
    reachable = ik_reachable(arm_x, arm_y, HOVER_Z) and ik_reachable(arm_x, arm_y, TOUCH_Z)

    print("pixel: col=%.0f row=%.0f area=%d" % (pixel_col, pixel_row, area))
    print("arm:   x=%.1f y=%.1f  hover_z=%.0f touch_z=%.0f  reachable=%s"
          % (arm_x, arm_y, HOVER_Z, TOUCH_Z, reachable))

    if not reachable:
        print("UNREACHABLE: the arm firmware would ignore this pose. Move the box "
              "toward the lower-middle of the image (around rows 320-400) and retry.")
        return 2

    if DRY_RUN:
        print("dry run: not moving")
        return 0

    attach_arm(status_pub)

    pose = position()
    pose.x, pose.y, pose.z = arm_x, arm_y, HOVER_Z
    pub.publish(pose)
    rospy.sleep(2.0)

    pose.z = TOUCH_Z
    pub.publish(pose)
    rospy.sleep(4.0)
    print("touched - going home")

    pose.x, pose.y, pose.z = HOME
    pub.publish(pose)
    rospy.sleep(2.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
