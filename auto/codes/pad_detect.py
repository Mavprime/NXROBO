#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Orange suction-pad detection, shared by runtask.py and the calib tools.

WHY A SEPARATE MODULE
---------------------
The startup calibration check needs to see the pad, and so do calib_pad.py and
calib3d.py.  This code used to live in calib_pad.py, which imports runtask.py
(for ArmAction) at module level.  When runtask.py itself is the running program
it is the ``__main__`` module, so ``import calib_pad`` tried to import a SECOND
copy of runtask under a different name; that failed on the robot and the
self-check silently degraded to "pad not visible".  Keeping the detector free
of runtask breaks that cycle.

Deployment: this file must sit next to runtask.py in
``auto_match/nodes/`` and in ``~/codes/`` (upload.ps1 copies it to both).
"""

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image


# Orange suction pad. The bear/banana plush are beige/brown, so saturation and
# value stay low there; the printed cubes are muted too.
LOWER = np.array([4, 130, 110])
UPPER = np.array([24, 255, 255])
MIN_AREA = 200          # a partially visible pad gives a biased centroid
BORDER_PX = 6           # blobs touching the image edge are clipped -> reject


def grab(bridge, timeout=3.0):
    """One color frame from the RealSense as a BGR numpy array."""
    msg = rospy.wait_for_message("/camera/color/image_raw", Image,
                                 timeout=timeout)
    return bridge.imgmsg_to_cv2(msg, "bgr8")


def pad_pixel(image, debug=False):
    """Centroid of the largest orange blob (the pad), or None.

    Rejects blobs that touch the image border: a clipped pad still yields a
    centroid, but it is shifted towards the visible part and would silently
    bend the homography.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER, UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            np.ones((5, 5), np.uint8), iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    blob = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(blob)
    if area < MIN_AREA:
        return None
    bx, by, bw, bh = cv2.boundingRect(blob)
    height, width = image.shape[:2]
    if (bx <= BORDER_PX or by <= BORDER_PX
            or bx + bw >= width - BORDER_PX or by + bh >= height - BORDER_PX):
        if debug:
            print("      pad blob clipped at the border, rejected")
        return None
    moments = cv2.moments(blob)
    if moments["m00"] == 0:
        return None
    u = moments["m10"] / moments["m00"]
    v = moments["m01"] / moments["m00"]
    if debug:
        print("      pad blob area=%.0f at (%.1f, %.1f)" % (area, u, v))
    return u, v, area
