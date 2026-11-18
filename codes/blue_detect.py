#!/usr/bin/env python3
"""
Live blue-box detector for the NXROBO Spark robot (ROS Noetic).

Subscribes to /camera/color/image_raw, applies the same HSV mask and
morphology used by spark_carry_object, and prints the largest contour's
area and centroid. Use it to check that the camera sees the box before
moving the arm.

Run on the robot (camera driver must already be running):

    source ~/spark_noetic/devel/setup.bash
    python3 ~/codes/blue_detect.py

Tune LOWER/UPPER if the box is not detected or other objects are.
The values below are the patched defaults used for the light-blue cube.
"""

import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

LOWER = np.array([90, 50, 50])      # H, S, V
UPPER = np.array([135, 255, 255])
MIN_AREA = 300
PRINT_HZ = 5.0


class BlueDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
        self.last_print = 0.0
        self.latest = None

    def mask(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        m = cv2.inRange(hsv, LOWER, UPPER)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self.kernel)
        m = cv2.erode(m, None, iterations=4)
        m = cv2.dilate(m, None, iterations=4)
        return m

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:  # CvBridgeError and friends
            rospy.logwarn("image conversion failed: %s", exc)
            return

        m = self.mask(frame)
        contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) > MIN_AREA]

        if contours:
            biggest = max(contours, key=cv2.contourArea)
            moment = cv2.moments(biggest)
            if moment["m00"] > 0:
                cx = moment["m10"] / moment["m00"]
                cy = moment["m01"] / moment["m00"]
                self.latest = (cv2.contourArea(biggest), cx, cy)
            else:
                self.latest = None
        else:
            self.latest = None

        now = time.time()
        if now - self.last_print >= 1.0 / PRINT_HZ:
            self.last_print = now
            if self.latest:
                area, cx, cy = self.latest
                print("BLUE area=%4d center=(%3d,%3d)" % (area, cx, cy))
            else:
                print("no blue box")

    def run(self):
        rospy.init_node("blue_detect", anonymous=True)
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)
        rospy.loginfo("blue_detect ready; waiting for /camera/color/image_raw")
        rospy.spin()


if __name__ == "__main__":
    BlueDetector().run()
