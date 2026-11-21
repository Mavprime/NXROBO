#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Calibrate the pixel <-> arm mapping from the orange suction pad itself.

Why: ``thefile.txt`` is a 4-coefficient linear fit (x = a*v + b, y = c*u + d)
and on this robot it lands the wrist ~60 mm away from the cube.  The suction
pad is bright orange, so the camera can see exactly where the wrist really is.
For a fixed wrist height the mapping pixel <-> (x, y) is a homography, which
is exact for a plane and needs 4 points instead of a hand-tuned fit.

The script moves the arm to N known poses at ONE height, finds the pad in each
frame, fits the homography, prints the residuals and stores it in
``~/grasp_homography.npz``.

Usage:
    python3 -u ~/codes/calib_pad.py                 # calibrate at z = -55 mm
    python3 -u ~/codes/calib_pad.py --z -55
    python3 -u ~/codes/calib_pad.py --pose 200,0 ...  # custom poses

Nothing is grasped and the vacuum never turns on.
"""

import argparse
import json
import os
import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# NOTE: ``from runtask import ArmAction`` lives inside main() on purpose.
# runtask.py imports this module for the orange-pad detector; a module-level
# import of runtask here would load a second copy of it (runtask is ``__main__``
# when the node runs) and can fail, which silently disabled the calibration
# self-check at startup.

OUT_PATH = os.path.expanduser("~/grasp_homography.npz")

# The orange-pad detector itself lives in pad_detect.py (no runtask import) so
# that runtask.py can use it at startup.  Re-exported here because calib3d.py
# and servo_grasp.py import it from this module.
from pad_detect import MIN_AREA, grab, pad_pixel  # noqa: E402,F401

DEFAULT_POSES = [(280, 0), (255, 0), (225, 0), (255, 50),
                 (255, -50), (225, -45), (280, 45), (205, 55)]


def collect(arm, bridge, poses, z, settle):
    samples = []
    for (x, y) in poses:
        print("[MOVE] wrist -> x=%d y=%d z=%.0f" % (x, y, z))
        arm.interface.set_pose(float(x), float(y), z, delay=settle)
        rospy.sleep(0.3)
        found = None
        for attempt in range(3):
            image = grab(bridge)
            found = pad_pixel(image)
            if found:
                break
            rospy.sleep(0.15)
        if not found:
            print("[MISS] pad not found at x=%d y=%d" % (x, y))
            continue
        u, v, area = found
        print("[SEEN] x=%d y=%d -> pixel (%.1f, %.1f) area=%.0f"
              % (x, y, u, v, area))
        samples.append((float(x), float(y), u, v))
    return samples


def fit(samples):
    """Homography arm->pixel plus honest error estimates.

    In-sample residuals are useless for 4 points (a homography interpolates
    them exactly), so a leave-one-out error is computed as well: fit on N-1
    points and predict the held-out one.
    """
    if len(samples) < 4:
        print("[FAIL] need at least 4 pad detections, got %d" % len(samples))
        return None, None, None
    src = np.array([[s[0], s[1]] for s in samples], dtype=np.float64)
    dst = np.array([[s[2], s[3]] for s in samples], dtype=np.float64)
    H, _ = cv2.findHomography(src, dst, method=0)
    if H is None:
        print("[FAIL] homography did not converge")
        return None, None, None

    inside = []
    for i in range(len(samples)):
        p = cv2.perspectiveTransform(src[i:i + 1].reshape(1, 1, 2), H)[0, 0]
        inside.append(float(np.hypot(p[0] - dst[i][0], p[1] - dst[i][1])))

    loo = []
    if len(samples) >= 6:
        for i in range(len(samples)):
            H_i, _ = cv2.findHomography(np.delete(src, i, axis=0),
                                        np.delete(dst, i, axis=0), method=0)
            if H_i is None:
                continue
            p = cv2.perspectiveTransform(src[i:i + 1].reshape(1, 1, 2), H_i)[0, 0]
            loo.append(float(np.hypot(p[0] - dst[i][0], p[1] - dst[i][1])))

    return H, inside, loo


def main():
    parser = argparse.ArgumentParser(description="pad-based visual calibration")
    parser.add_argument("--z", type=float, default=-55.0,
                        help="wrist height to calibrate at (default -55, the "
                             "height the operator confirmed is correct)")
    parser.add_argument("--settle", type=float, default=1.1,
                        help="seconds to wait after each move")
    parser.add_argument("--pose", action="append", default=None,
                        help="x,y pose (repeatable); defaults to a 6-point set")
    args = parser.parse_args(rospy.myargv()[1:])

    poses = DEFAULT_POSES
    if args.pose:
        poses = [tuple(int(v) for v in p.split(",")) for p in args.pose]

    rospy.init_node("calib_pad", anonymous=True)
    bridge = CvBridge()
    from runtask import ArmAction          # lazy: see the note at the imports
    arm = ArmAction()

    print("[CALIB] height z=%.0f mm, %d poses" % (args.z, len(poses)))
    samples = collect(arm, bridge, poses, args.z, args.settle)

    H, errors, loo = fit(samples)
    if H is None:
        print("[CALIB] aborted, arm goes home")
        arm.arm_home()
        return 2

    print("[FIT ] homography from %d points" % len(samples))
    print("[FIT ] H = %s" % np.array2string(H, precision=5, separator=", "))
    print("[FIT ] in-sample residual px: max=%.1f mean=%.1f"
          % (max(errors), sum(errors) / len(errors)))
    if loo:
        print("[FIT ] leave-one-out  residual px: max=%.1f mean=%.1f  <- real accuracy"
              % (max(loo), sum(loo) / len(loo)))
    else:
        print("[FIT ] leave-one-out skipped: need >= 6 detections")

    Hinv = np.linalg.inv(H)
    np.savez(OUT_PATH, H=H, Hinv=Hinv, z=args.z,
             poses=np.array(samples),
             residual_px=float(max(errors)),
             loo_px=float(max(loo)) if loo else -1.0)
    print("[SAVE] %s" % OUT_PATH)

    # Sanity check: where does the pad sit right now, and what does the inverse
    # mapping say for a cube seen at the image centre?
    image = grab(bridge)
    pad = pad_pixel(image, debug=True)
    if pad:
        u, v = pad[0], pad[1]
        back = cv2.perspectiveTransform(
            np.array([[[u, v]]], dtype=np.float64), Hinv)[0, 0]
        print("[CHECK] pad pixel (%.1f, %.1f) -> arm (%.1f, %.1f) mm "
              "(inverse mapping of its own pixel)" % (u, v, back[0], back[1]))

    for probe in [(320, 240), (320, 300), (255, 222)]:
        back = cv2.perspectiveTransform(
            np.array([[list(probe)]], dtype=np.float64), Hinv)[0, 0]
        print("[MAP ] pixel %s -> arm (%.1f, %.1f) mm at z=%.0f"
              % (probe, back[0], back[1], args.z))

    arm.arm_home()
    print("[DONE] arm home")
    return 0


if __name__ == "__main__":
    sys.exit(main())
