#!/usr/bin/env python3
"""Visual sanity probe: park the wrist above a target and photograph it.

Saves ~/logs/check_target.jpg, prints the orange-pad pixel, the projection of
the commanded wrist pose and the current object detections, then goes home.
Nothing is grasped and the vacuum never turns on.

Usage: python3 check_target.py X Y Z
"""
import sys

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge

import match_logic
from pad_detect import grab, pad_pixel
from runtask import ArmAction, CamAction

X = float(sys.argv[1])
Y = float(sys.argv[2])
Z = float(sys.argv[3])
OUT = "/home/spark/logs/check_target.jpg"


def project(M, x, y, z):
    cam = np.array([x, y, z, 1.0]) @ np.asarray(M, dtype=np.float64)
    if cam[2] <= 1e-6:
        return None
    return (match_logic.CAM_FX * cam[0] / cam[2] + match_logic.CAM_CX,
            match_logic.CAM_FY * cam[1] / cam[2] + match_logic.CAM_CY)


rospy.init_node("check_target", anonymous=True)
cam = CamAction()
arm = ArmAction(cam=cam)
bridge = CvBridge()

try:
    arm.interface.set_motor(True)
except Exception:
    pass
arm.arm_grasp_ready(block=True)
rospy.sleep(1.0)
arm.interface.set_pose(130.0, 0.0, 75.0, delay=0.6)
rospy.sleep(0.4)
arm.interface.set_pose(X, Y, Z, delay=0.9)
rospy.sleep(0.9)

image = grab(bridge)
cv2.imwrite(OUT, image)
pad = pad_pixel(image)
print("[PROBE] wrist=(%.1f, %.1f, %.1f)" % (X, Y, Z))
print("[PROBE] pad pixel: %s" % (pad,))
print("[PROBE] projected wrist pixel: %s" % (project(cam.transform3d, X, Y, Z),))
print("[PROBE] saved %s" % OUT)

dets = cam.detector(num_samples=6, timeout=6.0, conf_threshold=0.04) or []
print("[PROBE] detections:")
for det in dets:
    print("   class=%s pixel=(%.0f, %.0f)" % (det[0], det[1][0], det[1][1]))

arm.arm_home()
print("[DONE] home")
