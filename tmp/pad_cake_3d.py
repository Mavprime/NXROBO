#!/usr/bin/env python3
"""Measure the real 3D position of the cake top face with the D435 depth stream.

The DLT hand-eye matrix is projective, so it is only known up to a scale.  The
orange pad gives that scale: at a commanded wrist pose the pad pixel is known,
and the aligned depth gives its true camera-space Z.  With the scale fixed the
cake top pixel + its depth unproject into the arm frame, which settles the open
question: is the cake top plane really at z = +50 mm?

Nothing is grasped; the arm only rises to a safe pose above the cake.
"""
import sys

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import numpy as np
import rospy
from sensor_msgs.msg import Image

import match_logic
from pad_detect import pad_pixel
from runtask import ArmAction, CamAction

M = np.load("/home/spark/transform_3d.dat")
FX, FY, CX, CY = (match_logic.CAM_FX, match_logic.CAM_FY,
                  match_logic.CAM_CX, match_logic.CAM_CY)
PROBE_POSE = (304.7, -16.1, 70.0)


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


def unproject(u, v, d):
    """pixel + aligned depth (m) -> camera optical-frame point (m)."""
    return np.array([(u - CX) / FX * d, (v - CY) / FY * d, d])


def solve_arm_point(cam_point, scale):
    """Solve [x,y,z,1] @ M * scale = cam_point for the arm point (mm)."""
    a = np.array([[M[0][j], M[1][j], M[2][j]] for j in range(3)])
    b = np.array([cam_point[j] / scale - M[3][j] for j in range(3)])
    return np.linalg.solve(a, b)


def project_pose(pose):
    cam = np.array([pose[0], pose[1], pose[2], 1.0]) @ M
    return cam


rospy.init_node("pad_cake_3d", anonymous=True)
cam = CamAction()
arm = ArmAction(cam=cam)

try:
    arm.arm_home(block=True)
    rospy.sleep(1.2)

    dets = cam.detector(num_samples=6, timeout=6.0, conf_threshold=0.04) or []
    print("[DETS] %s" % dets)
    cake = match_logic.select_grasp_candidate(dets) if dets else None
    if not cake or not match_logic.is_cake(cake[0]):
        print("[FAIL] no cake in view")
        sys.exit(2)
    cake_u, cake_v = float(cake[1][0]), float(cake[1][1])
    bear = None
    for det in dets:
        if int(det[0]) == 1:
            bear = (float(det[1][0]), float(det[1][1]))

    depth = grab_depth()
    cake_d = depth_at(depth, cake_u, cake_v)
    print("[CAKE] pixel=(%.0f, %.0f) depth=%.4f m" % (cake_u, cake_v, cake_d or -1))
    if bear:
        bear_d = depth_at(depth, bear[0], bear[1])
        print("[BEAR] pixel=(%.0f, %.0f) depth=%.4f m" % (bear[0], bear[1], bear_d or -1))

    # --- reference: orange pad at a known wrist pose ------------------------
    arm.interface.set_pose(130.0, 0.0, 75.0, delay=0.6)
    rospy.sleep(0.3)
    arm.interface.set_pose(*PROBE_POSE, delay=0.9)
    rospy.sleep(0.9)
    color = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=10)
    from cv_bridge import CvBridge
    image = CvBridge().imgmsg_to_cv2(color, "bgr8")
    pad = pad_pixel(image)
    print("[PAD ] pixel=%s" % (pad,))
    if not pad:
        print("[FAIL] orange pad not visible at probe pose")
        sys.exit(3)
    pad_u, pad_v = pad[0], pad[1]
    depth = grab_depth()
    pad_d = depth_at(depth, pad_u, pad_v)
    print("[PAD ] depth=%.4f m at pixel=(%.1f, %.1f)" % (pad_d or -1, pad_u, pad_v))

    cam_pad = unproject(pad_u, pad_v, pad_d)
    projet = project_pose(PROBE_POSE)
    scale = cam_pad[2] / projet[2]
    print("[SCALE] cam/pad scale factor s = %.6f" % scale)
    cam_pad_model = projet * scale
    print("[SCALE] pad camera point measured = %s" % np.array2string(cam_pad, precision=4))
    print("[SCALE] pad camera point model*s  = %s" % np.array2string(cam_pad_model, precision=4))

    # --- cake top-face point in the arm frame ------------------------------
    cam_cake = unproject(cake_u, cake_v, cake_d)
    cake_xyz = solve_arm_point(cam_cake, scale)
    print("[CAKE] camera point = %s" % np.array2string(cam_cake, precision=4))
    print("[CAKE] arm point    = (%.1f, %.1f, %.1f) mm"
          % (cake_xyz[0], cake_xyz[1], cake_xyz[2]))

    # Also report the pad pose reconstructed from its own pixel+depth.
    pad_xyz = solve_arm_point(cam_pad, scale)
    print("[PAD ] arm point reconstruction = (%.1f, %.1f, %.1f) mm (commanded %.1f, %.1f, %.1f)"
          % (pad_xyz[0], pad_xyz[1], pad_xyz[2], PROBE_POSE[0], PROBE_POSE[1], PROBE_POSE[2]))

    if bear:
        cam_bear = unproject(bear[0], bear[1], bear_d)
        bear_xyz = solve_arm_point(cam_bear, scale)
        print("[BEAR] arm point = (%.1f, %.1f, %.1f) mm"
              % (bear_xyz[0], bear_xyz[1], bear_xyz[2]))
finally:
    try:
        arm.arm_home()
    except Exception:
        pass
    print("[DONE]")
