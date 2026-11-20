#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Drive closer to the cube until the arm can actually reach it.

Why: on this robot the arm's reach is ~320 mm, while a cube standing half a
metre away looks perfectly graspable in the image.  The old code estimated the
distance from a broken camera->arm calibration, so it believed the cube was at
25 cm, extended fully, and stopped ~20 cm short.

The depth camera does *not* depend on that calibration: it reports the metric
range to the point under the cube's pixel.  That range is what we drive on.

Loop:
    detect cube -> measure depth range -> if range > target, drive forward
    small step -> repeat (bounded number of steps, bounded step size)

Usage:
    python3 -u ~/codes/approach.py                 # drive to 0.33 m
    python3 -u ~/codes/approach.py --target 0.30 --max-step 0.06
    python3 -u ~/codes/approach.py --dry-run       # measure only, no motion
"""

import argparse
import math
import sys
import time

import rospy

import match_logic
from runtask import CamAction, ArmAction
try:                       # older runtask.py variants have no voting window
    from runtask import GRASP_FRAMES
except ImportError:
    GRASP_FRAMES = 4


def measure(cam):
    """Detected cube (class, u, v) and the metric range from the depth camera."""
    cubes = cam.detector(num_samples=GRASP_FRAMES, timeout=6.0)
    if not cubes:
        return None
    best = match_logic.select_grasp_candidate(cubes)
    if best is None:
        return None
    cube_id = int(best[0])
    u = float(best[1][0])
    v = float(best[1][1])
    point = cam.point3d(u, v)
    if point is None:
        return cube_id, u, v, None
    rng = math.sqrt(point[0] ** 2 + point[1] ** 2 + point[2] ** 2)
    return cube_id, u, v, rng


def main():
    parser = argparse.ArgumentParser(description="approach the cube")
    parser.add_argument("--target", type=float, default=0.33,
                        help="stop when the cube is closer than this, metres")
    parser.add_argument("--max-step", type=float, default=0.08,
                        help="maximum forward step per iteration, metres")
    parser.add_argument("--max-steps", type=int, default=6)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("approach", anonymous=True)
    cam = CamAction()
    arm = ArmAction(cam=cam)

    print("[APPROACH] target range %.2f m, max step %.2f m" % (args.target, args.max_step))
    arm.arm_grasp_ready(block=True)
    rospy.sleep(1.2)

    try:
        for step in range(1, args.max_steps + 1):
            found = measure(cam)
            if not found:
                print("[APPROACH] cube lost from view, stopping")
                return 3
            cube_id, u, v, rng = found
            if rng is None:
                print("[APPROACH] no depth at the cube pixel, stopping "
                      "(cannot judge distance safely)")
                return 4
            print("[APPROACH %d] class=%d pixel=(%.0f, %.0f) range=%.2f m"
                  % (step, cube_id, u, v, rng))
            if rng <= args.target:
                print("[APPROACH] close enough (%.2f m <= %.2f m)"
                      % (rng, args.target))
                return 0
            if args.dry_run:
                print("[DRY] would drive %.2f m forward"
                      % min(args.max_step, rng - args.target))
                return 0
            distance = min(args.max_step, rng - args.target)
            print("[APPROACH] driving forward %.1f cm" % (distance * 100.0))
            arm.drive_base(distance)
            rospy.sleep(0.9)
        print("[APPROACH] step budget exhausted")
        return 5
    finally:
        try:
            arm.arm_home()
        except Exception as exc:
            print("[WARN] cleanup: %s" % exc)


if __name__ == "__main__":
    sys.exit(main())
