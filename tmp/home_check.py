#!/usr/bin/env python3
"""Validate depth reconstruction against the known wrist pose at home.

Reconstructs the orange pad and a grid over the front-row cube from aligned
depth.  The pad must come back at the arm_home pose, which also validates the
depth alignment near the bottom of the image where the front-row cube is.
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
GRID = [(u, v) for v in range(260, 366, 15) for u in range(300, 421, 20)]

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


rospy.init_node("home_check", anonymous=True)
cam = CamAction()
arm = ArmAction(cam=cam)

try:
    arm.arm_home(block=True)
    rospy.sleep(1.2)
    image = bridge.imgmsg_to_cv2(
        rospy.wait_for_message("/camera/color/image_raw", Image, timeout=10),
        "bgr8")
    pad = pad_pixel(image)
    depth = grab_depth()
    print("[HOME] pad pixel=%s" % (pad,))
    if pad:
        d, xyz = recon(depth, pad[0], pad[1])
        print("[HOME] pad d=%s arm=%s" % (
            d, "(%7.1f, %7.1f, %8.1f)" % (xyz[0], xyz[1], xyz[2]) if xyz is not None else "n/a"))
    print("[HOME] grid over the front-row cube:")
    for (u, v) in GRID:
        d, xyz = recon(depth, u, v)
        if xyz is None:
            print("  (%3d,%3d): no depth" % (u, v))
        else:
            print("  (%3d,%3d): d=%.3f arm=(%7.1f, %7.1f, %8.1f)"
                  % (u, v, d, xyz[0], xyz[1], xyz[2]))
finally:
    try:
        arm.arm_home()
    except Exception:
        pass
    print("[DONE]")
