#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pick the cube: aim at the cube CENTRE, verify with the arm's own feedback.

Two lessons from the field session are baked in here:

1. The detector returns the centre of the cube's bounding box, i.e. the cube's
   CENTRE in height (~ -100 mm for a 10 cm cube whose top is at -50 mm), not
   its top face.  Inverting the pixel at z_touch therefore lands the wrist 'on
   the ray' but short of the cube - so the inverse is evaluated at the cube
   centre height instead.

2. Commanding the wrist past its physical reach silently does nothing: the old
   code believed it was at x=310 while the arm had stopped short.  The swiftpro
   driver publishes the REAL wrist position on /SwiftproState_topic (20 Hz), so
   every move is verified: if the arm falls short, the robot drives towards the
   cube and retries.

Order of motion (operator requirement): rise ABOVE the cube first, then descend
strictly vertically - no lateral motion at cube height, so the cube is not
pushed.

Usage:
    python3 -u ~/codes/pick_direct.py                     # full pick
    python3 -u ~/codes/pick_direct.py --dry-run           # compute only
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rospy
from swiftpro.msg import SwiftproState

import match_logic
from runtask import CamAction, ArmAction, CAM_FX, CAM_FY, CAM_CX, CAM_CY
try:                       # older runtask.py variants have no voting window
    from runtask import GRASP_FRAMES
except ImportError:
    GRASP_FRAMES = 4

STATE_TOPIC = "/SwiftproState_topic"
X_MIN, X_MAX = 120.0, 320.0
Y_LIMIT = 190.0


def pixel_to_xy_at_z(M, u, v, z,
                     fx=CAM_FX, fy=CAM_FY, cx=CAM_CX, cy=CAM_CY):
    """Closed-form inverse of the projective map at a fixed height."""
    up = (u - cx) / fx
    vp = (v - cy) / fy
    r0, r1, r2 = M[:, 0], M[:, 1], M[:, 2]
    C0 = r0[2] * z + r0[3]
    C1 = r1[2] * z + r1[3]
    C2 = r2[2] * z + r2[3]
    a11 = r0[0] - up * r2[0]
    a12 = r0[1] - up * r2[1]
    a21 = r1[0] - vp * r2[0]
    a22 = r1[1] - vp * r2[1]
    b1 = up * C2 - C0
    b2 = vp * C2 - C1
    det = a11 * a22 - a12 * a21
    if abs(det) < 1e-12:
        return None
    return ((b1 * a22 - a12 * b2) / det, (a11 * b2 - b1 * a21) / det)


def read_actual(timeout=3.0):
    """REAL wrist position from the arm driver, not the commanded one."""
    try:
        msg = rospy.wait_for_message(STATE_TOPIC, SwiftproState,
                                     timeout=timeout)
        return float(msg.x), float(msg.y), float(msg.z)
    except Exception as exc:
        print("[WARN] no arm feedback (%s)" % exc)
        return None


def move_checked(arm, x, y, z, settle=0.9, tol=6.0):
    """Move and report how far the real position is from the command."""
    arm.interface.set_pose(x, y, z, delay=0.25)
    rospy.sleep(settle)
    actual = read_actual()
    if actual is None:
        return None
    ax, ay, az = actual
    err = max(abs(ax - x), abs(ay - y))
    print("[MOVE] cmd=(%.1f, %.1f, %.1f) actual=(%.1f, %.1f, %.1f) err=%.1f mm"
          % (x, y, z, ax, ay, az, err))
    return ax, ay, az, err <= tol


def detect(cam):
    cubes = cam.detector(num_samples=GRASP_FRAMES, timeout=6.0)
    if not cubes:
        return None
    best = match_logic.select_grasp_candidate(cubes)
    if best is None:
        return None
    return int(best[0]), float(best[1][0]), float(best[1][1])


def cube_still_there(cam, cube_id, u, v, radius=25.0):
    cubes = cam.detector(num_samples=GRASP_FRAMES, timeout=6.0)
    for cube in cubes or []:
        try:
            cid = int(cube[0])
            cu, cv_ = float(cube[1][0]), float(cube[1][1])
        except Exception:
            continue
        if cid == cube_id and np.hypot(cu - u, cv_ - v) < radius:
            return True, (cu, cv_)
    return False, None


def ramp(arm, x, y, z_from, z_to, steps=1, delay=0.8):
    """Move vertically. With steps=1 (default), commands a single smooth G0 move."""
    if steps <= 1:
        arm.interface.set_pose(x, y, z_to, delay=delay)
        return
    for index in range(1, steps + 1):
        z = z_from + (z_to - z_from) * float(index) / steps
        arm.interface.set_pose(x, y, z, delay=delay)


def main():
    parser = argparse.ArgumentParser(description="verified direct pick")
    parser.add_argument("--z-touch", type=float, default=-50.0,
                        help="cup contact height on the cube top (mm)")
    parser.add_argument("--z-above", type=float, default=-15.0,
                        help="height of the lateral approach (mm)")
    parser.add_argument("--cube-height", type=float, default=100.0,
                        help="cube height in mm; the pixel is inverted at the "
                             "cube CENTRE, i.e. z_touch - height/2")
    parser.add_argument("--fix-dx", type=float, default=0.0,
                        help="extra cup-centre correction forward (mm)")
    parser.add_argument("--fix-dy", type=float, default=0.0,
                        help="extra cup-centre correction lateral (mm)")
    parser.add_argument("--reach-tol", type=float, default=6.0,
                        help="allowed commanded-vs-actual error (mm)")
    parser.add_argument("--max-approach", type=int, default=6,
                        help="how many times the base may drive towards the cube")
    parser.add_argument("--hold", type=float, default=0.0)
    parser.add_argument("--place", action="store_true",
                        help="after lifting, put the cube back down so another "
                             "cycle can run without the operator")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("pick_direct", anonymous=True)
    cam = CamAction()
    arm = ArmAction(cam=cam)

    z_center = args.z_touch - args.cube_height / 2.0
    print("[INIT] cube centre height z=%.0f (top %.0f, cube %.0f mm)"
          % (z_center, args.z_touch, args.cube_height))

    entry = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "cube": None,
             "pixel": None, "target": None, "approach_steps": 0,
             "result": None}
    try:
        arm.arm_grasp_ready(block=True)
        rospy.sleep(1.2)

        found = detect(cam)
        if not found:
            print("[FAIL] no cube detected")
            return 3
        cube_id, cu, cv_ = found
        entry["cube"] = cube_id
        entry["pixel"] = [cu, cv_]
        print("[CUBE] class=%d pixel=(%.0f, %.0f)" % (cube_id, cu, cv_))

        xy = pixel_to_xy_at_z(cam.transform3d, cu, cv_, z_center)
        if xy is None:
            print("[FAIL] inverse mapping is degenerate")
            return 4
        x = xy[0] + args.fix_dx
        y = xy[1] + args.fix_dy
        entry["target"] = [x, y]
        print("[TARGET] cube centre maps to x=%.1f y=%.1f" % (x, y))

        # --- Check reachability, approach if needed, then re-scan and grasp ---
        TARGET_X = 245.0
        approach_step = 0

        while approach_step < args.max_approach:
            dist_to_cube = math.hypot(x, y)
            # Comfortable sweet-spot envelope: arm is not strained, lands dead center on cube
            in_reach = (160.0 <= x <= 265.0) and (abs(y) <= 85.0) and (dist_to_cube <= 270.0)

            if in_reach:
                break

            approach_step += 1
            entry["approach_steps"] = approach_step
            print("[APPROACH #%d] cube at (%.1f, %.1f), dist=%.1f mm is outside comfortable sweet spot. Approaching..."
                  % (approach_step, x, y, dist_to_cube))

            if args.dry_run:
                angle_rad = math.atan2(y, x)
                drive_m = (dist_to_cube - TARGET_X) / 1000.0
                print("[DRY] would rotate %+.1f deg and drive %+.1f cm"
                      % (math.degrees(angle_rad), drive_m * 100.0))
                entry["result"] = "DRY"
                return 0

            # 1. Rotate base to align with cube angle (centering cube ahead of robot)
            angle_rad = math.atan2(y, x)
            if abs(angle_rad) > math.radians(2.0):
                print("[APPROACH] rotating base towards cube: %+.1f deg" % math.degrees(angle_rad))
                arm.turn_base(angle_rad)
                rospy.sleep(0.4)

            # 2. Drive base forward to place cube at target distance X = 245 mm
            drive_m = (dist_to_cube - TARGET_X) / 1000.0
            if abs(drive_m) >= 0.02:
                step_m = max(-0.25, min(0.25, drive_m))
                print("[APPROACH] driving %+.1f cm to place cube at %.1f cm" % (step_m * 100.0, TARGET_X / 10.0))
                arm.drive_base(step_m)
                rospy.sleep(0.5)

            # 3. ЗАТЕМ: запустить алгоритм сканирования позиции кубика еще раз!
            print("[RE-SCAN] fresh scan of cube position after approach step #%d..." % approach_step)
            arm.arm_grasp_ready(block=True)
            rospy.sleep(0.4)
            found = detect(cam)
            if not found:
                print("[FAIL] cube lost after approach")
                entry["result"] = "LOST_AFTER_APPROACH"
                return 3
            cube_id, cu, cv_ = found
            entry["cube"] = cube_id
            entry["pixel"] = [cu, cv_]
            xy = pixel_to_xy_at_z(cam.transform3d, cu, cv_, z_center)
            if xy is None:
                print("[FAIL] inverse mapping degenerate after approach")
                return 4
            x = xy[0] + args.fix_dx
            y = xy[1] + args.fix_dy
            entry["target"] = [x, y]
            print("[RE-SCAN] cube now at px=(%.0f, %.0f) -> x=%.1f, y=%.1f" % (cu, cv_, x, y))

        if args.dry_run:
            print("[DRY] in_reach=True, would approach above at z=%.0f and descend to z=%.0f"
                  % (args.z_above, args.z_touch))
            entry["result"] = "DRY"
            return 0

        # 4. Standard grasp sequence: position above, verify actual, descend, pump, lift
        result = move_checked(arm, x, y, args.z_above, tol=args.reach_tol)
        if result is None or not result[3]:
            # If slightly short on reach, small correction
            if result is not None:
                ax, ay, az, _ = result
                shortfall = x - ax
                if abs(shortfall) > 6.0:
                    arm.drive_base(max(-0.06, min(0.06, shortfall / 1000.0)))
                    rospy.sleep(0.4)
                    result = move_checked(arm, x, y, args.z_above, tol=args.reach_tol)
            if result is None or not result[3]:
                print("[ABORT] arm cannot reach pose above cube")
                entry["result"] = "UNREACHABLE"
                return 5

        print("[DESCEND] straight down to z=%.0f (single smooth motion)" % args.z_touch)
        arm.interface.set_pose(x, y, args.z_touch, delay=0.8)

        actual = read_actual()
        if actual is not None:
            ax, ay, az = actual
            print("[DESCEND] actual at touch: (%.1f, %.1f, %.1f)" % (ax, ay, az))
            if max(abs(ax - x), abs(ay - y)) > args.reach_tol:
                print("[ABORT] the wrist did not reach the touch pose - "
                      "retracting without pumping")
                entry["result"] = "TOUCH_UNREACHED"
                return 6

        print("[PUMP] on")
        arm.interface.set_pump(True)
        rospy.sleep(0.5)

        print("[LIFT] straight up to z=60 (single smooth motion)")
        arm.interface.set_pose(x, y, 60.0, delay=0.8)

        if args.hold:
            print("[HOLD] pump on for %.1f s" % args.hold)
            rospy.sleep(args.hold)

        still, where = cube_still_there(cam, cube_id, cu, cv_)
        entry["cube_still_on_table"] = bool(still)
        if still:
            print("[VERIFY] FAILED: cube still on the table at %s" % (where,))
            entry["result"] = "MISS"
        else:
            print("[VERIFY] cube left the table - picked up")
            entry["result"] = "OK"

        if args.place:
            print("[PLACE] lowering the cube back and releasing (single smooth motion)")
            arm.interface.set_pose(x, y, args.z_touch, delay=0.8)
            rospy.sleep(0.3)
            arm.interface.set_pump(False)
            rospy.sleep(0.5)
            arm.interface.set_pose(x, y, args.z_above, delay=0.8)
            print("[PLACE] released, retracted")
    finally:
        try:
            arm.interface.set_pump(False)
            rospy.sleep(0.3)
            arm.arm_home()
            print("[SAFE] pump off, arm home")
        except Exception as exc:
            print("[WARN] cleanup: %s" % exc)

    try:
        os.makedirs(os.path.expanduser("~/logs"), exist_ok=True)
        with open(os.path.expanduser("~/logs/pick_direct.jsonl"), "a") as handle:
            handle.write(json.dumps(entry) + "\n")
    except Exception as exc:
        print("[WARN] log: %s" % exc)
    return 0


if __name__ == "__main__":
    sys.exit(main())
