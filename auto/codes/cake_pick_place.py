#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pick the Tier-2 cake, verify it is in the air, put it back on its pedestal.

Why this exists next to ``pick_direct.py``: the cake sits on a pedestal behind
the front row of cubes, so the base must not move, and the old 2-D
``thefile2.txt`` approximation maps the cake ~55 mm too far forward (it was
fitted at a lower plane).  Here the target is the top-face pixel inverted at
the *top plane* with the calibrated DLT (``match_logic.pixel_to_xy_at_z``),
which is the same mapping that already places ground cubes dead centre.

The sequence is deliberately conservative:

1. detect the cake (class 2 is preferred by ``select_grasp_candidate``);
2. rise in front of the tower, then travel at z_above over the front row;
3. descend straight down onto the cake top, vacuum on;
4. lift, retract sideways, then park the arm out of the camera view
   (``arm_grasp_ready``) and re-detect: if the cake is still on the pedestal
   the grasp failed and the run aborts without dropping anything;
5. put the cake back on the same pixel, release, rise, go home.

Usage:
    python3 -u ~/codes/cake_pick_place.py            # full cycle
    python3 -u ~/codes/cake_pick_place.py --dry-run  # detect + print plan only
    python3 -u ~/codes/cake_pick_place.py --x 305 --y -15   # override target
"""

import argparse
import json
import math
import os
import sys
import time

import rospy

import match_logic
from runtask import ArmAction, CamAction

#: Cake top plane in arm coordinates (mm).  The cake stands on a 100 mm cube,
#: whose top is at -50 mm, so the cake top is at +50 mm.
Z_TOP = 50.0
#: Wrist height at contact: a few mm below the top plane so the rubber cup
#: compresses instead of stopping in the air.
Z_TOUCH = 40.0
#: Clearance used to fly over the front row of cubes.
Z_ABOVE = 75.0
#: First vertical lift after the vacuum seals.
Z_LIFT = 100.0
#: Arm pose after the lift, so the camera can see the pedestal again.
VERIFY_POSE = None  # filled in main(): ArmAction.arm_grasp_ready()

X_MIN, X_MAX = 150.0, 340.0
Y_LIMIT = 150.0
VERIFY_RADIUS_PX = 45.0


def log(message):
    print(message)
    sys.stdout.flush()


def actual_pose(arm, timeout=3.0):
    """REAL wrist position reported by the swiftpro driver."""
    try:
        from swiftpro.msg import SwiftproState
        msg = rospy.wait_for_message("/SwiftproState_topic", SwiftproState,
                                     timeout=timeout)
        return float(msg.x), float(msg.y), float(msg.z)
    except Exception as exc:
        log("[WARN] no arm feedback (%s)" % exc)
        return None


def move(arm, x, y, z, delay=0.8, settle=0.6, tol=8.0):
    """Move and report the actual pose; returns True when the arm got there."""
    arm.interface.set_pose(x, y, z, delay=delay)
    rospy.sleep(settle)
    pose = actual_pose(arm)
    if pose is None:
        return False
    err = max(abs(pose[0] - x), abs(pose[1] - y))
    log("[MOVE] cmd=(%.1f, %.1f, %.1f) actual=(%.1f, %.1f, %.1f) err=%.1f mm"
        % (x, y, z, pose[0], pose[1], pose[2], err))
    return err <= tol


def detect_cake(cam, samples=6, threshold=0.04):
    """Highest-priority graspable object; class 2 (cake) wins when visible."""
    cubes = cam.detector(num_samples=samples, timeout=6.0,
                         conf_threshold=threshold)
    if not cubes:
        return None
    best = match_logic.select_grasp_candidate(cubes)
    if best is None:
        return None
    try:
        return int(best[0]), float(best[1][0]), float(best[1][1])
    except Exception:
        return None


def cake_still_on_pedestal(cam, u0, v0, radius=VERIFY_RADIUS_PX):
    """True when a cake-class object is still where the cake was."""
    cubes = cam.detector(num_samples=6, timeout=6.0, conf_threshold=0.04) or []
    for cube in cubes:
        try:
            cid = int(cube[0])
            cu, cv = float(cube[1][0]), float(cube[1][1])
        except Exception:
            continue
        if match_logic.is_cake(cid) and math.hypot(cu - u0, cv - v0) <= radius:
            log("[VERIFY] cake class %d still at (%.0f, %.0f)" % (cid, cu, cv))
            return True
    return False


def main():
    parser = argparse.ArgumentParser(description="Tier-2 cake pick & put-back")
    parser.add_argument("--x", type=float, default=None,
                        help="override wrist x (mm) instead of the DLT target")
    parser.add_argument("--y", type=float, default=None,
                        help="override wrist y (mm) instead of the DLT target")
    parser.add_argument("--z-top", type=float, default=Z_TOP,
                        help="cake top plane used for the pixel inversion (mm)")
    parser.add_argument("--z-touch", type=float, default=Z_TOUCH)
    parser.add_argument("--z-above", type=float, default=Z_ABOVE)
    parser.add_argument("--no-place", action="store_true",
                        help="lift only: keep the cake on the cup (operator "
                             "takes over)")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("cake_pick_place", anonymous=True)
    cam = CamAction()
    arm = ArmAction(cam=cam)
    verify_pose = (50.0, 180.0, 160.0)   # arm_grasp_ready: out of camera view

    entry = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "pixel": None,
             "target": None, "result": None, "cake_on_pedestal": None}

    try:
        arm.arm_grasp_ready(block=True)
        rospy.sleep(1.2)

        found = detect_cake(cam)
        if not found:
            log("[FAIL] no graspable object detected")
            entry["result"] = "NO_TARGET"
            return 3
        cube_id, u0, v0 = found
        entry["pixel"] = [u0, v0]
        log("[DETECT] class=%d (cake=%s) pixel=(%.0f, %.0f)"
            % (cube_id, match_logic.is_cake(cube_id), u0, v0))
        if not match_logic.is_cake(cube_id):
            log("[FAIL] the best candidate is not the cake - aborting instead "
                "of grabbing a front-row cube")
            entry["result"] = "NOT_CAKE"
            return 4

        xy = match_logic.pixel_to_xy_at_z(cam.transform3d, u0, v0, args.z_top)
        if xy is None:
            log("[FAIL] pixel inversion is degenerate")
            entry["result"] = "DEGENERATE"
            return 5
        x = args.x if args.x is not None else xy[0]
        y = args.y if args.y is not None else xy[1]
        x = max(X_MIN, min(X_MAX, x))
        y = max(-Y_LIMIT, min(Y_LIMIT, y))
        entry["target"] = [x, y]
        log("[TARGET] top-face pixel inverted at z=%.0f -> x=%.1f y=%.1f"
            % (args.z_top, x, y))

        if args.dry_run:
            log("[DRY] clearance z=%.0f, descend to z=%.0f, lift to z=%.0f"
                % (args.z_above, args.z_touch, Z_LIFT))
            entry["result"] = "DRY"
            return 0

        # --- approach: up in front, then along above the front row ----------
        if not move(arm, 130.0, 0.0, args.z_above, delay=0.5):
            log("[FAIL] could not reach the clearance pose")
            entry["result"] = "CLEARANCE_UNREACHED"
            return 6
        if not move(arm, x, y, args.z_above):
            log("[FAIL] could not reach the pose above the cake")
            entry["result"] = "ABOVE_UNREACHED"
            return 6

        log("[PUMP] on")
        arm.interface.set_pump(True)
        rospy.sleep(0.35)

        log("[DESCEND] straight down to z=%.0f" % args.z_touch)
        arm.interface.set_pose(x, y, args.z_touch, delay=0.9)
        rospy.sleep(0.7)
        touch = actual_pose(arm)
        if touch is not None:
            log("[DESCEND] actual at touch: (%.1f, %.1f, %.1f)" % touch)

        log("[LIFT] vertical separation to z=%.0f" % Z_LIFT)
        arm.interface.set_pose(x, y, Z_LIFT, delay=0.9)
        rospy.sleep(0.6)

        log("[RETRACT] pulling the cake inward to x=240")
        arm.interface.set_pose(240.0, y, 125.0, delay=0.9)
        rospy.sleep(0.5)

        # --- verify: park the arm, then look at the pedestal ----------------
        log("[VERIFY] parking the arm out of view (50, 180, 160)")
        arm.interface.set_pose(*verify_pose, delay=0.9)
        rospy.sleep(1.0)
        still = cake_still_on_pedestal(cam, u0, v0)
        entry["cake_on_pedestal"] = bool(still)
        if still:
            log("[VERIFY] FAILED: cake is still on the pedestal")
            entry["result"] = "MISS"
            return 7
        log("[VERIFY] SUCCESS: no cake on the pedestal any more")
        entry["result"] = "PICKED"

        if args.no_place:
            log("[HOLD] --no-place: cake stays on the vacuum cup")
            entry["result"] = "PICKED_HELD"
            return 0

        # --- put it back on the same spot --------------------------------
        log("[PLACE] returning above the pedestal")
        if not move(arm, x, y, args.z_above):
            log("[FAIL] could not return above the pedestal")
            entry["result"] = "PLACE_UNREACHED"
            return 8
        log("[PLACE] lowering to z=%.0f" % args.z_touch)
        arm.interface.set_pose(x, y, args.z_touch, delay=0.9)
        rospy.sleep(0.6)
        log("[PLACE] pump off")
        arm.interface.set_pump(False)
        rospy.sleep(0.8)
        log("[PLACE] rising to z=%.0f" % args.z_above)
        arm.interface.set_pose(x, y, args.z_above, delay=0.8)
        rospy.sleep(0.4)
        entry["result"] = "PLACED"
    finally:
        try:
            arm.interface.set_pump(False)
            rospy.sleep(0.3)
            arm.arm_home()
            log("[SAFE] pump off, arm home")
        except Exception as exc:
            log("[WARN] cleanup: %s" % exc)

    try:
        os.makedirs(os.path.expanduser("~/logs"), exist_ok=True)
        with open(os.path.expanduser("~/logs/cake_pick_place.jsonl"), "a") as fh:
            fh.write(json.dumps(entry) + "\n")
    except Exception as exc:
        log("[WARN] log: %s" % exc)

    log("[DONE] result=%s" % entry["result"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
