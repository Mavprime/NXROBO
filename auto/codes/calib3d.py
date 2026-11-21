#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Hand-eye calibration of the camera<-arm transform from the orange pad.

Fits the 4x3 projective matrix ``M`` used by runtask.py::

    [Xc, Yc, Zc] = [x, y, z, 1] @ M          (arm mm -> camera metres)
    u = CAM_FX * Xc/Zc + CAM_CX
    v = CAM_FY * Yc/Zc + CAM_CY

Only the projection is needed for grasping (pixel_to_arm_at_z works on ratios),
so the matrix is recovered up to scale with the Direct Linear Transform from
>= 6 correspondences (commanded wrist pose -> detected pad pixel).  The depth
camera is NOT used, so the broken depth stream (SI-33) cannot pollute the fit.

Output: ``~/transform_3d.dat`` (np.save of a (4,3) array), which runtask.py
already loads automatically and prefers over the hard-coded TRANSFORM_3D.
A sidecar ``~/transform_3d.meta.json`` records when the fit was made and how
good it was, so the startup log can say how old the calibration is.

Usage:
    python3 -u ~/codes/calib3d.py            # fit and save
    python3 -u ~/codes/calib3d.py --poses ...  # custom
    python3 -u ~/codes/calib3d.py --check    # verify the saved matrix only

The ``--check`` mode moves the arm to one pose above the cubes, finds the
orange suction pad and compares its pixel with the projection of the saved
matrix.  Run it after moving the camera: a stale matrix shows up as tens of
pixels of error, a good one as ~2 px.
"""

import argparse
import hashlib
import json
import os
import sys
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge

from calib_pad import pad_pixel, grab
from runtask import ArmAction, CAM_FX, CAM_FY, CAM_CX, CAM_CY

OUT_PATH = os.path.expanduser("~/transform_3d.dat")
META_PATH = os.path.expanduser("~/transform_3d.meta.json")

# Spread over the reachable + visible workspace (the pad is invisible for
# x < ~235 mm, SI-29).  Three z levels give vertical spread so the fit is
# well conditioned, unlike the single-height homography (SI-31).
DEFAULT_POSES = [
    (240, 0, -60), (240, 0, -10), (240, 0, 30),
    (270, 0, -60), (270, 0, -10),
    (300, 0, -60), (300, 0, -10),
    (270, 55, -35), (270, -55, -35),
    (300, 55, -35), (300, -55, -35),
    (270, 0, -35),
]

#: A suction pad that is partly cut off by the arm or the image border still
#: returns a centroid, but it is biased.  Reject those blobs: one weak point
#: used to drag a whole fit from 1.1 px to 3.1 px leave-one-out.
MIN_SAMPLE_AREA = 400.0


def collect(arm, bridge, poses, settle):
    samples = []
    for (x, y, z) in poses:
        arm.interface.set_pose(float(x), float(y), float(z), delay=settle)
        rospy.sleep(0.25)
        found = None
        for _ in range(3):
            found = pad_pixel(grab(bridge))
            if found:
                break
            rospy.sleep(0.12)
        if not found:
            print("[MISS] pad not visible at (%d, %d, %d)" % (x, y, z))
            continue
        u, v, area = found
        if area < MIN_SAMPLE_AREA:
            # A pad cut in half by the arm or the border still yields a
            # centroid, but it is biased; one such point dragged a whole fit
            # from 1.1 px to 3.1 px leave-one-out.
            print("[SKIP] pad at (%d, %d, %d) is tiny (area=%.0f, need %.0f): "
                  "centroid would bias the fit" % (x, y, z, area, MIN_SAMPLE_AREA))
            continue
        print("[SEEN] arm=(%d, %d, %d) -> pixel (%.1f, %.1f) area=%.0f"
              % (x, y, z, u, v, area))
        samples.append((float(x), float(y), float(z), u, v))
    return samples


def dlt(samples):
    """Solve the 4x3 projective matrix from pixel correspondences (DLT)."""
    fx, fy, cx, cy = CAM_FX, CAM_FY, CAM_CX, CAM_CY
    rows = []
    for x, y, z, u, v in samples:
        p = (x, y, z, 1.0)
        up = (u - cx) / fx
        vp = (v - cy) / fy
        rows.append([p[0], p[1], p[2], p[3], 0, 0, 0, 0,
                     -up * p[0], -up * p[1], -up * p[2], -up * p[3]])
        rows.append([0, 0, 0, 0, p[0], p[1], p[2], p[3],
                     -vp * p[0], -vp * p[1], -vp * p[2], -vp * p[3]])
    A = np.array(rows, dtype=np.float64)
    if A.shape[0] < 12:
        return None
    _, _, vh = np.linalg.svd(A)
    m = vh[-1]                 # null vector, 12 elements
    return m.reshape(3, 4).T   # -> (4, 3)


def project(M, x, y, z):
    cam = np.array([x, y, z, 1.0], dtype=np.float64) @ M
    if cam[2] <= 1e-6:
        return None
    return (CAM_FX * cam[0] / cam[2] + CAM_CX,
            CAM_FY * cam[1] / cam[2] + CAM_CY)


def evaluate(M, samples):
    errors = []
    for x, y, z, u, v in samples:
        proj = project(M, x, y, z)
        if proj is None:
            # Keep the list aligned with ``samples``: the robust trim zips them.
            errors.append(float("inf"))
            continue
        errors.append(float(np.hypot(proj[0] - u, proj[1] - v)))
    return errors


def leave_one_out(samples):
    loo = []
    for i in range(len(samples)):
        M = dlt(samples[:i] + samples[i + 1:])
        if M is None:
            continue
        x, y, z, u, v = samples[i]
        proj = project(M, x, y, z)
        if proj is not None:
            loo.append(float(np.hypot(proj[0] - u, proj[1] - v)))
    return loo


def fit(samples):
    """DLT plus the sign convention; returns (M, per-sample pixel errors)."""
    M = dlt(samples)
    if M is None:
        return None, []
    # The DLT null vector has an arbitrary sign.  A point in front of the
    # camera must have Zc > 0, so flip the whole matrix if the samples end up
    # behind it.
    zcs = [float((np.array([x, y, z, 1.0]) @ M)[2])
           for x, y, z, _, _ in samples]
    if np.median(zcs) < 0:
        M = -M
        print("[FIT ] sign flipped so samples are in front of the camera")
    return M, evaluate(M, samples)


def fit_robust(samples, keep_min=8):
    """Fit, drop gross outliers once, refit.  Returns (M, errors, samples).

    The DLT is a least-squares fit, so a single bad correspondence (pad half
    hidden behind the arm) tilts the whole matrix.  Trimming once and refitting
    keeps the honest ~1 px residual instead of the ~3 px one.
    """
    M, errors = fit(samples)
    if M is None or len(samples) < keep_min + 1:
        return M, errors, samples
    ordered = sorted(errors)
    median = ordered[len(ordered) // 2]
    limit = max(3.0, 3.0 * median)
    keep = [sample for sample, error in zip(samples, errors) if error <= limit]
    if len(keep) < keep_min or len(keep) == len(samples):
        return M, errors, samples
    M2, errors2 = fit(keep)
    if M2 is None:
        return M, errors, samples
    dropped = [sample for sample, error in zip(samples, errors) if error > limit]
    print("[FIT ] dropped %d outlier(s) above %.1f px: %s"
          % (len(dropped), limit, dropped))
    return M2, errors2, keep


def run_check(tol_px=6.0):
    """Verify the saved matrix against the real pad; 0 = pass, 2 = fail."""
    arm = ArmAction()
    ok, detail = arm.verify_calibration(tol_px=tol_px)
    print("[CHECK] %s: %s" % ("PASS" if ok else "FAIL", detail))
    if not ok:
        print("[HINT] the camera probably moved: run the full fit "
              "`python3 -u ~/codes/calib3d.py`")
    arm.arm_home(block=True)
    print("[DONE] arm home")
    return 0 if ok else 2


def main():
    parser = argparse.ArgumentParser(description="fit camera<-arm transform")
    parser.add_argument("--settle", type=float, default=1.2)
    parser.add_argument("--pose", action="append", default=None,
                        help="x,y,z pose (repeatable); overrides the default set")
    parser.add_argument("--check", action="store_true",
                        help="do not fit: verify the saved matrix against the "
                             "real suction pad")
    parser.add_argument("--tol-px", type=float, default=6.0,
                        help="pass/fail threshold of the --check pad error")
    args = parser.parse_args(rospy.myargv()[1:])

    poses = DEFAULT_POSES
    if args.pose:
        poses = [tuple(int(v) for v in p.split(",")) for p in args.pose]

    rospy.init_node("calib3d", anonymous=True)

    if args.check:
        return run_check(args.tol_px)

    bridge = CvBridge()
    arm = ArmAction()

    print("[CALIB] %d poses" % len(poses))
    samples = collect(arm, bridge, poses, args.settle)
    print("[CALIB] %d valid correspondences" % len(samples))

    if len(samples) < 6:
        print("[FAIL] need >= 6 correspondences for the 4x3 DLT")
        arm.arm_home()
        return 2

    M, inside, samples = fit_robust(samples)
    if M is None:
        print("[FAIL] DLT did not converge")
        arm.arm_home()
        return 3

    loo = leave_one_out(samples)
    print("[FIT ] M (arm mm -> camera m) =")
    print(np.array2string(M, precision=6, separator=", "))
    print("[FIT ] in-sample pixel error: max=%.1f mean=%.1f"
          % (max(inside), sum(inside) / len(inside)))
    if loo:
        print("[FIT ] leave-one-out pixel error: max=%.1f mean=%.1f  <- trust this"
              % (max(loo), sum(loo) / len(loo)))

    # A pixel error maps to mm differently with distance; state the local scale
    # at the touch workspace so the residual is interpretable.
    px_per_mm = 0.0
    p0 = project(M, 270, 0, -50)
    p1 = project(M, 280, 0, -50)
    if p0 and p1:
        px_per_mm = float(np.hypot(p1[0] - p0[0], p1[1] - p0[1]) / 10.0)
        print("[SCALE] ~%.2f px/mm at the touch workspace" % px_per_mm)

    # np.save(path) appends ".npy" whenever the name does not already end with
    # it, which silently wrote transform_3d.dat.npy while runtask.py kept
    # loading the old transform_3d.dat.  Write through a file object instead.
    with open(OUT_PATH, "wb") as handle:
        np.save(handle, M)
    print("[SAVE] %s" % OUT_PATH)

    # Sidecar metadata: the startup log reports age and residuals from here, so
    # the operator can tell a fresh calibration from a months-old one.
    meta = {
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "poses": len(samples),
        "in_sample_max_px": round(max(inside), 2),
        "in_sample_mean_px": round(sum(inside) / len(inside), 2),
        "loo_max_px": round(max(loo), 2) if loo else None,
        "loo_mean_px": round(sum(loo) / len(loo), 2) if loo else None,
        "px_per_mm": round(px_per_mm, 3),
        "intrinsics": [CAM_FX, CAM_FY, CAM_CX, CAM_CY],
        "file_md5": hashlib.md5(open(OUT_PATH, "rb").read()).hexdigest(),
    }

    # Reference frame for the startup self-check: the pad pixel at one fixed
    # pose, measured now.  Comparing a later measurement against this real
    # frame detects a moved camera far more sharply than comparing against the
    # model, whose own residual is ~2-3 px.
    check_pose = ArmAction.CALIB_CHECK_POSE
    pad_uv = arm.measure_pad(check_pose)
    if pad_uv:
        meta["check"] = {
            "pose": [float(value) for value in check_pose],
            "uv": [round(pad_uv[0], 2), round(pad_uv[1], 2)],
        }
        print("[CHECK] reference pad at (%.1f, %.1f) for pose %s"
              % (pad_uv[0], pad_uv[1], check_pose))
    else:
        print("[WARN] pad not visible at the check pose %s: the startup "
              "self-check will fall back to the projection" % (check_pose,))

    try:
        with open(META_PATH, "w", encoding="utf8") as handle:
            json.dump(meta, handle, indent=2, sort_keys=True)
        print("[META] %s" % META_PATH)
    except Exception as exc:
        print("[WARN] could not write %s: %s" % (META_PATH, exc))

    # Sanity anchors for the operator's measurements: floor ~ -150 mm, cube top
    # ~ -50 mm, pad ~ 180 mm above the floor at home z=35.
    for label, (px, py, pz) in (
            ("home(35)", (130.0, 0.0, 35.0)),
            ("touch(-50)", (250.0, 0.0, -50.0))):
        proj = project(M, px, py, pz)
        print("[MAP ] %s -> %s" % (label, proj if proj else "behind camera"))

    arm.arm_home()
    print("[DONE] arm home")
    return 0


if __name__ == "__main__":
    sys.exit(main())
