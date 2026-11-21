#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Closed-loop visual alignment with the suction pad, then a real grasp.

Open-loop calibration on this robot is only good to ~1-2 cm (the homography is
fitted inside a narrow image band, SI-29), which is not enough to hit a cube.
So this script does what a human does: put the wrist roughly above the cube,
look at the picture, measure the offset between the pad and the cube, correct,
and repeat until they line up.  Only then it descends and turns the pump on.

Sequence:
    1. tuck the arm, detect the cube (majority vote)
    2. first guess (x, y) from ~/grasp_homography.npz
    3. servo loop at hover height: pad pixel vs cube pixel -> Jacobian -> move
    4. converge, descend to the touch height, pump ON, lift
    5. report the alignment error that was reached

Nothing is saved into the production code; the pump is always released on exit.

Usage:
    python3 -u ~/codes/servo_grasp.py --z-touch -55 --z-hover -20
    python3 -u ~/codes/servo_grasp.py --dry-run     # align only, no pump
"""

import argparse
import os
import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import match_logic
from match_logic import pixel_to_xy_at_z
from calib_pad import pad_pixel, grab
from runtask import CamAction, ArmAction, CAM_FX, CAM_FY, CAM_CX, CAM_CY
try:                       # older runtask.py variants have no voting window
    from runtask import GRASP_FRAMES
except ImportError:
    GRASP_FRAMES = 4

#: Below this wrist extension the suction pad is not visible to the camera
#: (it sits under the camera / behind the arm), so visual servoing is blind.
PAD_VISIBLE_X = 235.0
#: Comfortable working extension the base backs off to.
TARGET_X = 250.0


def first_guess(cam, u, v, z, x0=250.0, y0=0.0):
    """Pixel -> arm (x, y) at height z, via the calibrated transform."""
    xy = pixel_to_xy_at_z(cam.transform3d, u, v, z)
    if xy is None:
        print("[WARN] inverse mapping is degenerate, using a neutral guess")
        return x0, y0
    return xy


def jacobian(cam, x, y, z, step=8.0):
    """d(pixel)/d(arm) at (x, y, z) from the calibrated projective transform."""
    p0 = cam.project_wrist(x, y, z)
    px = cam.project_wrist(x + step, y, z)
    py = cam.project_wrist(x, y + step, z)
    if p0 is None or px is None or py is None:
        return None
    return np.array([[(px[0] - p0[0]) / step, (py[0] - p0[0]) / step],
                     [(px[1] - p0[1]) / step, (py[1] - p0[1]) / step]],
                    dtype=np.float64)


def align(arm, bridge, cam, x, y, z, cu, cv, iters, tol, max_step=45.0):
    """Iteratively put the pad on the cube pixel AT HEIGHT z.

    Aligning once at hover height and then descending does not work: the
    camera is tilted, so the pad's projection slides across the image while
    the arm goes down (parallax).  Every height on the way down needs its own
    correction.
    """
    best = (x, y, 1e9, False)
    for iteration in range(1, iters + 1):
        arm.interface.set_pose(x, y, z, delay=1.0)
        rospy.sleep(0.30)
        pad = pad_pixel(grab(bridge))
        if pad is None:
            print("    [z=%.0f] pad not visible" % z)
            return x, y, best[2], False
        du, dv = cu - pad[0], cv - pad[1]
        err = float(np.hypot(du, dv))
        print("    [z=%.0f] iter %d wrist=(%.1f, %.1f) pad=(%.1f, %.1f) "
              "err=%.1f px" % (z, iteration, x, y, pad[0], pad[1], err))
        if err < best[2]:
            best = (x, y, err, True)
        if err <= tol:
            return x, y, err, True
        try:
            J = jacobian(cam, x, y, z)
            if J is None:
                return x, y, err, False
            delta = np.linalg.solve(J, np.array([du, dv]))
        except np.linalg.LinAlgError:
            return x, y, err, False
        delta = np.clip(delta, -max_step, max_step)
        x += float(delta[0])
        y += float(delta[1])
    return best[0], best[1], best[2], best[3]


def cube_pixel(cam):
    cubes = cam.detector(num_samples=GRASP_FRAMES, timeout=6.0)
    if not cubes:
        return None
    best = match_logic.select_grasp_candidate(cubes)
    if best is None:
        return None
    return best[0], float(best[1][0]), float(best[1][1])


def cube_still_there(cam, cube_id, u, v, radius=25.0):
    """Is a cube of the same class still lying at (almost) the same pixel?

    A radius of 25 px: a cube that was picked up and lifted leaves the floor
    and moves much further than that in the image, while the same cube lying
    untouched stays within a few pixels of its averaged centre.
    """
    cubes = cam.detector(num_samples=GRASP_FRAMES, timeout=6.0)
    for cube in cubes or []:
        try:
            cid = int(cube[0])
            cu = float(cube[1][0])
            cv_ = float(cube[1][1])
        except Exception:
            continue
        if cid == cube_id and np.hypot(cu - u, cv_ - v) < radius:
            return True, (cu, cv_)
    return False, None


def main():
    parser = argparse.ArgumentParser(description="pad-servo grasp")
    parser.add_argument("--z-touch", type=float, default=-50.0,
                        help="touch height: cube top for a 10 cm cube on the "
                             "floor is ~ -50 mm (operator-confirmed)")
    parser.add_argument("--z-hover", type=float, default=-20.0,
                        help="alignment height")
    parser.add_argument("--iters", type=int, default=5)
    parser.add_argument("--tol", type=float, default=8.0, help="pixels")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--hold", type=float, default=0.0,
                        help="seconds to hold the cube with the pump on after "
                             "lifting (time to photograph the proof)")
    parser.add_argument("--final-clearance", type=float, default=3.0,
                        help="align this many mm above the touch height, then "
                             "descend straight down (default 3)")
    parser.add_argument("--final-step", type=float, default=12.0,
                        help="max lateral correction on the last stage, mm")
    parser.add_argument("--probe-z", action="store_true",
                        help="after aligning, step down 5 mm at a time and let "
                             "the operator report where the pad touches")
    parser.add_argument("--step-z", type=float, default=5.0,
                        help="probe step, mm (default 5)")
    parser.add_argument("--probe-min", type=float, default=-75.0,
                        help="lowest probe height, mm (default -75)")
    parser.add_argument("--fix-dx", type=float, default=0.0, metavar="MM",
                        help="constant x correction after alignment: the orange "
                             "blob centres on the cube edge, not the cup centre")
    parser.add_argument("--fix-dy", type=float, default=0.0, metavar="MM",
                        help="constant y correction after alignment")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("servo_grasp", anonymous=True)
    bridge = CvBridge()
    cam = CamAction()
    arm = ArmAction(cam=cam)
    print("[INIT] using the calibrated ~/transform_3d.dat "
          "(leave-one-out ~1 px)")

    entry = {"ts": None, "cube": None, "iterations": [], "final_px": None,
             "aligned": False, "grasped": False}
    try:
        arm.arm_grasp_ready(block=True)
        rospy.sleep(1.2)

        found = cube_pixel(cam)
        if not found:
            print("[FAIL] no cube detected")
            return 3
        cube_id, cu, cv_ = found
        entry["cube"] = {"id": cube_id, "u": cu, "v": cv_}
        print("[CUBE] class=%d at pixel (%.0f, %.0f)" % (cube_id, cu, cv_))

        x, y = first_guess(cam, cu, cv_, args.z_touch)
        print("[START] first guess from calibrated transform: "
              "x=%.1f y=%.1f (z=%.0f)" % (x, y, args.z_touch))

        # The camera only sees the pad from x ~ 235 mm outwards (below that it
        # sits under the camera / behind the arm).  A cube closer than that
        # cannot be servoed, so back the base off instead - which is also what
        # the match FSM does when the target is too close.
        for attempt in range(2):
            if x >= PAD_VISIBLE_X:
                break
            back = min(0.10, (TARGET_X - x) / 1000.0)
            if back <= 0.005:
                break
            print("[BASE] x=%.0f is inside the pad's blind zone; "
                  "backing off %.1f cm" % (x, back * 100.0))
            arm.drive_base(-back)
            rospy.sleep(0.9)
            found = cube_pixel(cam)
            if not found:
                print("[BASE] cube lost after backing off")
                return 3
            cube_id, cu, cv_ = found
            x, y = first_guess(cam, cu, cv_, args.z_touch)
            print("[BASE] after backing off: cube at (%.0f, %.0f) -> "
                  "x=%.1f y=%.1f" % (cu, cv_, x, y))

        # Align at every height on the way down: matching pixels at hover
        # height means nothing at touch height when the camera is tilted.
        # The last alignment happens just ABOVE the cube so the pad never
        # sweeps sideways at cube height; the 3 mm descent adds ~2 px of
        # parallax, which is far below the cube size.
        z_last = args.z_touch + args.final_clearance
        stages = [(args.z_hover, 45.0),
                  ((args.z_hover + z_last) / 2.0, 25.0),
                  (z_last, args.final_step)]
        aligned = True
        last_err = None
        for stage_z, step_limit in stages:
            print("[STAGE] align at z=%.0f (max step %.0f mm)"
                  % (stage_z, step_limit))
            x, y, last_err, ok = align(arm, bridge, cam, x, y, stage_z,
                                       cu, cv_, args.iters, args.tol,
                                       max_step=step_limit)
            entry["iterations"].append({"z": stage_z, "x": x, "y": y,
                                        "err_px": last_err, "ok": ok})
            print("[STAGE] z=%.0f done: x=%.1f y=%.1f err=%.1f px ok=%s"
                  % (stage_z, x, y, last_err, ok))
            if not ok:
                aligned = False
                break

        entry["aligned"] = aligned
        print("[ALIGN] %s (last error %.1f px)"
              % ("OK" if aligned else "NOT reached", last_err if last_err else -1))

        if args.dry_run:
            print("[DRY] alignment only, no descent")
            return 0

        if args.probe_z:
            print("[PROBE] stepping down %.0f mm at a time from z=%.0f to %.0f"
                  % (args.step_z, z_last, args.probe_min))
            print("[PROBE] watch the pad and note the height where it touches")
            z = z_last
            while z > args.probe_min and not rospy.is_shutdown():
                z -= args.step_z
                arm.interface.set_pose(x, y, z, delay=1.6)
                print("[PROBE] z=%.0f" % z)
            print("[PROBE] retracting")
            arm.interface.set_pose(x, y, z_last + 30.0, delay=1.0)
            return 0

        if not aligned:
            print("[ABORT] refusing to pump while misaligned")
            return 4

        if args.fix_dx or args.fix_dy:
            # Matching the orange blob's centroid to the cube's pixel puts the
            # EDGE of the cup on the cube: the camera looks from behind/above,
            # so the visible blob is biased towards the camera and the cup
            # centre sits ~6 cm away from where the alignment believes it is.
            # This constant shift is applied AFTER alignment on purpose - a
            # further servo iteration would drag the pad straight back.
            x += args.fix_dx
            y += args.fix_dy
            print("[FIX] cup-centre correction dx=%+.0f dy=%+.0f -> (%.1f, %.1f)"
                  % (args.fix_dx, args.fix_dy, x, y))
            arm.interface.set_pose(x, y, z_last, delay=1.0)
            rospy.sleep(0.3)

        print("[GRASP] final descent %.0f mm, pump ON" % args.final_clearance)
        arm.interface.set_pose(x, y, args.z_touch, delay=0.8)
        arm.interface.set_pump(True)
        rospy.sleep(0.5)

        print("[GRASP] lifting (single smooth motion)")
        arm.interface.set_pose(x, y, 60.0, delay=0.8)
        entry["grasped"] = True
        print("[GRASP] lifted to z=60")

        if args.hold:
            print("[HOLD] pump stays on for %.1f s" % args.hold)
            rospy.sleep(args.hold)

        # The only grasp feedback available: look for the cube on the table.
        still_there, where = cube_still_there(cam, cube_id, cu, cv_)
        entry["cube_still_on_table"] = bool(still_there)
        if still_there:
            print("[VERIFY] FAILED: the cube is still on the table at %s - "
                  "nothing was picked up" % (where,))
        else:
            print("[VERIFY] cube is no longer where it was - it was picked up")
    finally:
        try:
            arm.interface.set_pump(False)
            rospy.sleep(0.3)
            arm.arm_home()
            print("[SAFE] pump off, arm home")
        except Exception as exc:
            print("[WARN] cleanup: %s" % exc)

    import json
    import time
    entry["ts"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    with open(os.path.expanduser("~/logs/servo_grasp.jsonl"), "a") as handle:
        handle.write(json.dumps(entry) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
