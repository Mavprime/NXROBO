#!/usr/bin/env python3
"""Validate the depth+hand-eye reconstruction at several wrist poses.

Scale s is fixed from the first pose; every later pose is reconstructed from
its own pad pixel and depth only.  If the reconstructed (x, y, z) matches the
commanded pose across the workspace, the cake reconstruction can be trusted.
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
POSES = [(304.7, -16.1, 70.0),
         (250.0, 0.0, 10.0),
         (250.0, 80.0, 20.0),
         (250.0, -80.0, 20.0),
         (280.0, 0.0, 50.0)]

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


def project_pose(pose):
    return np.array([pose[0], pose[1], pose[2], 1.0]) @ M


rospy.init_node("depth_multi", anonymous=True)
cam = CamAction()
arm = ArmAction(cam=cam)

scale = None
try:
    arm.arm_home(block=True)
    rospy.sleep(1.0)
    for pose in POSES:
        arm.interface.set_pose(130.0, 0.0, 75.0, delay=0.6)
        rospy.sleep(0.3)
        arm.interface.set_pose(*pose, delay=0.9)
        rospy.sleep(0.9)
        image = bridge.imgmsg_to_cv2(
            rospy.wait_for_message("/camera/color/image_raw", Image, timeout=10),
            "bgr8")
        pad = pad_pixel(image)
        if not pad:
            print("[MISS] pad not visible at %s" % (pose,))
            continue
        depth = grab_depth()
        d = depth_at(depth, pad[0], pad[1])
        if d is None or d <= 0.05:
            print("[MISS] no depth at pad pixel %s for pose %s" % (pad, pose))
            continue
        cam_point = np.array([(pad[0] - CX) / FX * d,
                              (pad[1] - CY) / FY * d, d])
        model = project_pose(pose)
        if scale is None:
            scale = cam_point[2] / model[2]
            print("[SCALE] s = %.6f  (from pose %s)" % (scale, pose))
        xyz = solve_arm_point(cam_point, scale)
        err = np.array(xyz) - np.array(pose)
        print("[POSE] cmd=(%6.1f,%6.1f,%6.1f) rec=(%6.1f,%6.1f,%6.1f) "
              "err=(%5.1f,%5.1f,%5.1f) mm"
              % (pose[0], pose[1], pose[2], xyz[0], xyz[1], xyz[2],
                 err[0], err[1], err[2]))
finally:
    try:
        arm.arm_home()
    except Exception:
        pass
    print("[DONE]")
