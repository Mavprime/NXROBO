#!/usr/bin/env python3
"""
Replay the grasp node's detection pipeline frame by frame.

Prints, for each frame: number of contours, largest area, centroid and how
many consecutive frames satisfied the stability test. Use this when the
grasp demo keeps printing "not found".

Run on the robot:

    source ~/spark_noetic/devel/setup.bash
    python3 ~/codes/detect_pipeline_debug.py

The grasp demo (s_carry_object_cv3.py) was patched to require 10 stable
frames within 6 px. Original values were 20 frames / 2 px.
"""

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

LOWER = np.array([90, 50, 50])
UPPER = np.array([135, 255, 255])
FRAMES = 20
STABLE_PX = 6.0


def pipeline(frame, kernel):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER, UPPER)
    masked_hsv = cv2.bitwise_and(hsv, hsv, mask=mask)
    hue = masked_hsv[:, :, 0]
    blurred = cv2.blur(hue, (9, 9))
    _, thresh = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    thresh = cv2.erode(thresh, None, iterations=4)
    thresh = cv2.dilate(thresh, None, iterations=4)
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours


def main():
    rospy.init_node("detect_pipeline_debug", anonymous=True)
    bridge = CvBridge()
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
    frames = []

    def callback(msg):
        frames.append(bridge.imgmsg_to_cv2(msg, "bgr8"))

    subscriber = rospy.Subscriber("/camera/color/image_raw", Image, callback, queue_size=30)
    rospy.sleep(1.0)
    subscriber.unregister()

    print("captured %d frames" % len(frames))
    previous = None
    stable = 0
    for index, frame in enumerate(frames[-FRAMES:]):
        contours = pipeline(frame, kernel)
        if not contours:
            print("%2d no contours" % index)
            previous = None
            stable = 0
            continue
        biggest = max(contours, key=cv2.contourArea)
        moment = cv2.moments(biggest)
        if moment["m00"] == 0:
            continue
        cx = moment["m10"] / moment["m00"]
        cy = moment["m01"] / moment["m00"]
        if previous is not None and abs(cx - previous[0]) <= STABLE_PX and abs(cy - previous[1]) <= STABLE_PX:
            stable += 1
        else:
            stable = 0
        print("%2d n=%d area=%d centroid=(%.1f,%.1f) stable=%d"
              % (index, len(contours), cv2.contourArea(biggest), cx, cy, stable))
        previous = (cx, cy)


if __name__ == "__main__":
    main()
