#!/usr/bin/env python3
"""Diagnose the pad-check failure: move to the check pose, save the frame,
report every orange blob (area + centroid).  Motion: one pose, then park."""
import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge

sys.path.insert(0, "/home/spark/codes")

import pad_detect  # noqa: E402
import runtask  # noqa: E402


def top_blobs(image, limit=5):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, pad_detect.LOWER, pad_detect.UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            np.ones((5, 5), np.uint8), iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    rows = []
    for blob in contours:
        area = float(cv2.contourArea(blob))
        if area < 20.0:
            continue
        moments = cv2.moments(blob)
        if moments["m00"] == 0:
            continue
        rows.append((area, moments["m10"] / moments["m00"],
                     moments["m01"] / moments["m00"]))
    rows.sort(reverse=True)
    return rows[:limit]


def main():
    rospy.init_node("pad_diag", anonymous=True, disable_signals=True)
    cam = runtask.CamAction()
    arm = runtask.ArmAction(cam=cam)
    pose = runtask.ArmAction.CALIB_CHECK_POSE

    arm.interface.set_pose(float(pose[0]), float(pose[1]), float(pose[2]),
                           delay=1.4)
    rospy.sleep(0.8)

    bridge = CvBridge()
    image = pad_detect.grab(bridge)
    frame = cv2.resize(image, (640, 480)) if image.shape[:2] != (480, 640) else image
    cv2.imwrite("/home/spark/logs/check_pose.jpg", frame)
    print("[DIAG] pose=%s image=%s saved=/home/spark/logs/check_pose.jpg"
          % (pose, image.shape))
    print("[DIAG] pad_pixel strict -> %s" % (pad_detect.pad_pixel(image, debug=True),))
    blobs = top_blobs(image)
    print("[DIAG] orange blobs (area, u, v): %s" % ["(%.0f, %.1f, %.1f)" % b for b in blobs])

    arm.arm_home(block=True)
    print("[DIAG] arm parked")
    return 0


if __name__ == "__main__":
    sys.exit(main())
