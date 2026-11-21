#!/usr/bin/env python3
"""Reconstruct a grid of scene points (arm frame) from aligned depth.

Fixes the hand-eye scale from the orange pad once, then prints depth and the
reconstructed arm (x, y, z) for a set of scene pixels plus a vertical/horizontal
scan.  This maps the table plane and the real heights of the cake tiers.
"""
import sys

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import match_logic
from pad_detect import pad_pixel
from runtask import ArmAction, CamAction

M = np.load("/home/spark/transform_3d.dat")
FX, FY, CX, CY = (match_logic.CAM_FX, match_logic.CAM_FY,
                  match_logic.CAM_CX, match_logic.CAM_CY)
PAD_SCALE = 0.390585
PAD_POSE = (304.7, -16.1, 70.0)
POINTS = [(361, 71, "cake top centre"), (365, 148, "cake front centre"),
          (340, 195, "banana front"), (300, 120, "cake top L"),
          (420, 100, "cake top R"), (348, 302, "bear front"),
          (350, 250, "bear top edge"), (320, 240, "table near tower"),
          (150, 200, "table left far"), (500, 250, "table right"),
          (320, 380, "table near"), (320, 20, "wall line")]

bridge = CvBridge()


def grab_depth():
    msg = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw",
                                  Image, timeout=15)
    raw = np.frombuffer(msg.data, dtype=np.uint16)
    return raw.reshape(msg.height, msg.width).astype(np.float64) / 1000.0


def depth_at(depth, u, v, half=2):
    u, v = int(round(u)), int(round(v))
    patch = depth[max(0, v - half):v + half + 1, max(0, u - half):u + half + 1]
    valid = patch[patch > 0]
    return float(np.median(valid)) if len(valid) else None


def solve_arm_point(cam_point, scale):
    a = np.array([[M[0][j], M[1][j], M[2][j]] for j in range(3)])
    b = np.array([cam_point[j] / scale - M[3][j] for j in range(3)])
    return np.linalg.solve(a, b)


def recon(depth, u, v):
    d = depth_at(depth, u, v)
    if d is None or d <= 0.05:
        return None, None
    cam = np.array([(u - CX) / FX * d, (v - CY) / FY * d, d])
    return d, solve_arm_point(cam, PAD_SCALE)


rospy.init_node("scene_3d", anonymous=True)
cam = CamAction()
arm = ArmAction(cam=cam)

try:
    arm.arm_home(block=True)
    rospy.sleep(1.0)
    depth = grab_depth()

    print("=== named points ===")
    for (u, v, name) in POINTS:
        d, xyz = recon(depth, u, v)
        if xyz is None:
            print("%-18s (%3d,%3d): no depth" % (name, u, v))
        else:
            print("%-18s (%3d,%3d): d=%.3f  arm=(%7.1f, %7.1f, %8.1f)"
                  % (name, u, v, d, xyz[0], xyz[1], xyz[2]))

    print("=== vertical scan u=320 ===")
    for v in range(20, 481, 20):
        d, xyz = recon(depth, 320, v)
        if xyz is None:
            print("  v=%3d: no depth" % v)
        else:
            print("  v=%3d: d=%.3f arm=(%7.1f, %7.1f, %8.1f)"
                  % (v, d, xyz[0], xyz[1], xyz[2]))

    print("=== horizontal scan v=240 ===")
    for u in range(40, 621, 40):
        d, xyz = recon(depth, u, 240)
        if xyz is None:
            print("  u=%3d: no depth" % u)
        else:
            print("  u=%3d: d=%.3f arm=(%7.1f, %7.1f, %8.1f)"
                  % (u, d, xyz[0], xyz[1], xyz[2]))
finally:
    try:
        arm.arm_home()
    except Exception:
        pass
    print("[DONE]")
