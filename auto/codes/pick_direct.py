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
from match_logic import pixel_to_xy_at_z
from runtask import CamAction, ArmAction, CAM_FX, CAM_FY, CAM_CX, CAM_CY
try:                       # older runtask.py variants have no voting window
    from runtask import GRASP_FRAMES
except ImportError:
    GRASP_FRAMES = 4

STATE_TOPIC = "/SwiftproState_topic"
X_MIN, X_MAX = 120.0, 320.0
Y_LIMIT = 190.0


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


def detect(cam, conf_threshold=0.04, num_samples=6):
    cubes = cam.detector(num_samples=num_samples, timeout=6.0, conf_threshold=conf_threshold)
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

        is_cake = match_logic.is_cake(cube_id)
        if is_cake:
            z_center = 0.0
            z_touch = 40.0    # Touch surface for 20 cm tier (Layer 2)
            z_above = 75.0    # Safe clearance over Tier 1 obstacles (-50 mm)
            z_lift = 85.0     # Lift cake +45mm off pedestal (safe joint angles theta2=9.5 deg)
            z_retract = 120.0 # Elevated retract clearance once pulled back
            x_retract = 270.0 # Retracted X pose reducing moment arm
            max_reach_x = 350.0   # Expanded reach limit specifically for Cake
            max_dist = 355.0
            cake_xy = match_logic.pixel_to_cake_xy(cam.transform3d, cu, cv_, cup_offset_x=30.0)
            if cake_xy is None:
                print("[FAIL] cake approximation is degenerate")
                return 4
            x = cake_xy[0] + args.fix_dx
            y = cake_xy[1] + args.fix_dy
            print(f"[TARGET] DEDICATED CAKE APPROXIMATION (thefile2.txt +3cm): maps to x={x:.1f}, y={y:.1f}, z_touch={z_touch:.1f}")
        else:
            z_center = args.z_touch - args.cube_height / 2.0
            z_touch = args.z_touch
            z_above = args.z_above
            z_lift = 60.0
            max_reach_x = 265.0
            max_dist = 270.0
            xy = pixel_to_xy_at_z(cam.transform3d, cu, cv_, z_center)
            if xy is None:
                print("[FAIL] inverse mapping is degenerate")
                return 4
            x = xy[0] + 20.0 + args.fix_dx
            y = xy[1] + args.fix_dy
            print("[TARGET] cube centre maps to x=%.1f y=%.1f" % (x, y))

        print("[CUBE] class=%d (cake=%s) pixel=(%.0f, %.0f)" % (cube_id, is_cake, cu, cv_))

        # --- Check reachability, approach if needed, then re-scan and grasp ---
        TARGET_X = 245.0
        approach_step = 0

        # CRITICAL: For Cake positioned behind front-row obstacles,
        # base motion is strictly inhibited to avoid colliding with front-row cubes.
        # The manipulator reaches over the obstacle row from the stationary position.
        while approach_step < args.max_approach:
            if is_cake:
                in_reach = True
                print("[OBSTACLE-GUARD] Cake grasp: base motion strictly inhibited to preserve front-row cubes.")
                break

            dist_to_cube = math.hypot(x, y)
            # Comfortable sweet-spot envelope: arm is not strained, lands dead center on cube
            in_reach = (160.0 <= x <= max_reach_x) and (abs(y) <= 85.0) and (dist_to_cube <= max_dist)

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

            # 3. THEN: re-run cube position scanning algorithm!
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
            if is_cake:
                cake_xy = match_logic.pixel_to_cake_xy(cam.transform3d, cu, cv_)
                if cake_xy is None:
                    print("[FAIL] cake approximation degenerate after approach")
                    return 4
                x = cake_xy[0] + args.fix_dx
                y = cake_xy[1] + args.fix_dy
            else:
                xy = pixel_to_xy_at_z(cam.transform3d, cu, cv_, z_center)
                if xy is None:
                    print("[FAIL] inverse mapping degenerate after approach")
                    return 4
                x = xy[0] + 20.0 + args.fix_dx
                y = xy[1] + args.fix_dy
            entry["target"] = [x, y]
            print("[RE-SCAN] cube now at px=(%.0f, %.0f) -> x=%.1f, y=%.1f" % (cu, cv_, x, y))

        if args.dry_run:
            print("[DRY] in_reach=True, would approach above at z=%.0f and descend to z=%.0f"
                  % (z_above, z_touch))
            entry["result"] = "DRY"
            return 0

        # Clamp target x within physical arm limit
        x_max = 350.0 if is_cake else 320.0
        x = max(120.0, min(x_max, x))

        # 4. Standard grasp sequence: position above, verify actual, descend, pump, lift
        # Rise above front obstacles first before moving forward in (X, Y)
        print("[CLEARANCE] moving arm up to z=%.0f clearance pose..." % z_above)
        arm.interface.set_pose(130.0, 0.0, z_above, delay=0.5)

        result = move_checked(arm, x, y, z_above, tol=args.reach_tol)
        if result is None or not result[3]:
            # If slightly short on reach, only drive base if not cake (never hit front obstacles)
            if result is not None and not is_cake:
                ax, ay, az, _ = result
                shortfall = x - ax
                if abs(shortfall) > 6.0:
                    arm.drive_base(max(-0.06, min(0.06, shortfall / 1000.0)))
                    rospy.sleep(0.4)
                    result = move_checked(arm, x, y, z_above, tol=args.reach_tol)
            if result is None or not result[3]:
                print("[ABORT] arm cannot reach pose above cube")
                entry["result"] = "UNREACHABLE"
                return 5

        # Turn pump ON while above the cube so suction is active upon contact
        print("[PUMP] turning on vacuum pump before descent...")
        arm.interface.set_pump(True)
        rospy.sleep(0.3)

        print("[DESCEND] straight down to z=%.0f (compressing suction cup)" % z_touch)
        arm.interface.set_pose(x, y, z_touch, delay=0.8)

        actual = read_actual()
        if actual is not None:
            ax, ay, az = actual
            print("[DESCEND] actual at touch: (%.1f, %.1f, %.1f)" % (ax, ay, az))

        # Dwell at contact to build solid vacuum
        rospy.sleep(0.6)

        if is_cake:
            print("[LIFT] stage 1: vertical separation to z=%.0f (lifting cake off pedestal)..." % z_lift)
            arm.interface.set_pose(x, y, z_lift, delay=0.8)
            rospy.sleep(0.6)

            print("[RETRACT-INWARD] stage 2: pulling cake inward to x=%.0f, z=%.0f..." % (x_retract, z_retract))
            arm.interface.set_pose(x_retract, y, z_retract, delay=0.8)
            rospy.sleep(1.0)
        else:
            print("[LIFT] straight up to z=%.0f (confident vertical lift)" % z_lift)
            arm.interface.set_pose(x, y, z_lift, delay=0.8)
            rospy.sleep(1.2)

        still, where = cube_still_there(cam, cube_id, cu, cv_)
        entry["cube_still_on_table"] = bool(still)
        if still:
            print("[VERIFY] FAILED: cube still on the table at %s" % (where,))
            entry["result"] = "MISS"
        else:
            print("[VERIFY] SUCCESS: cube lifted into the air!")
            entry["result"] = "OK"

        # Safely lower the cube back down and release
        print("[PLACE] returning to drop position...")
        if is_cake:
            arm.interface.set_pose(x, y, z_lift, delay=0.8)
            rospy.sleep(0.4)
        print("[PLACE] safely lowering cube back to z=%.0f..." % z_touch)
        arm.interface.set_pose(x, y, z_touch, delay=0.8)
        rospy.sleep(0.4)
        print("[PLACE] turning off vacuum pump, releasing cube...")
        arm.interface.set_pump(False)
        rospy.sleep(0.6)
        print("[RETRACT] rising back to clearance z=%.0f..." % z_above)
        arm.interface.set_pose(x, y, z_above, delay=0.8)
        rospy.sleep(0.4)
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
