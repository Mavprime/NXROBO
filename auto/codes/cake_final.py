#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Scan, pick and put back the Tier-2 cake, verified with the depth camera.

Why this script exists
----------------------
The previous cake attempts trusted a fixed-height 2-D/3-D hand-eye inversion and
a hard-coded touch height.  The D435 depth stream (validated against the orange
suction pad at several wrist poses, error < 8 mm) shows the cake top plane is
much lower than the old constants assumed.  This script therefore:

1. re-derives the hand-eye scale ``s`` from the orange pad at a known pose;
2. detects the cake and unprojects its top-face pixel + depth into the arm
   frame, so the approach target is measured, not guessed;
3. descends in small steps while watching the orange pad through the fixed
   camera: if the pad stops following the commanded motion, the cup has met the
   cake top - that is the real touch height (also the safety stop if the
   estimate is wrong);
4. turns the vacuum on, lifts, parks the arm and *looks at the pedestal* to
   confirm the cake left it;
5. puts the cake back on the same spot, parks and confirms it is back.

Usage:
    python3 -u ~/codes/cake_final.py --probe     # only approach + photo
    python3 -u ~/codes/cake_final.py             # pick and put back
    python3 -u ~/codes/cake_final.py --no-place  # lift only, keep on the cup
"""

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import match_logic
from pad_detect import grab, pad_pixel
from runtask import ArmAction, CamAction

#: Distance from the wrist pose to the suction-cup sealing rim (mm).
PAD_THICKNESS_MM = 8.0
#: Small extra push below the rim-contact height, so the cup compresses a bit.
PRESS_MM = 3.0
#: Commanded descent step while looking for contact (mm).
STEP_MM = 4.0
#: Pause after each descent step so the arm settles and the camera sees it.
SETTLE_S = 0.45
#: Stop if the arm has not met the cake this far below the estimated top.
EXTRA_DEPTH_MM = 20.0
#: Hard mechanical floor for this routine (well above the table).
Z_MIN_HARD = -60.0

OUT_DIR = os.path.expanduser("~/logs/cake_final")
CAL_POSE = (270.0, 0.0, 30.0)     # inside the fitted z range, pad visible
APPROACH_Z = 90.0
VERIFY_POSE = (50.0, 180.0, 160.0)

#: Hand-eye matrix, loaded once.  ``cam.transform3d`` re-assigns it in main().
M = np.load(os.path.expanduser("~/transform_3d.dat"))


def log(msg):
    print(msg)
    sys.stdout.flush()


def project_arm(M, x, y, z):
    return match_logic.project_arm(M, x, y, z)


def grab_depth():
    msg = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw",
                                  Image, timeout=8)
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


def unproject(u, v, d):
    return np.array([(u - match_logic.CAM_CX) / match_logic.CAM_FX * d,
                     (v - match_logic.CAM_CY) / match_logic.CAM_FY * d, d])


def cake_pixels(cubes):
    """All cake detections, top face first (smallest v)."""
    out = []
    for cube in cubes or []:
        try:
            cid = int(cube[0])
            u, v = float(cube[1][0]), float(cube[1][1])
        except Exception:
            continue
        if match_logic.is_cake(cid):
            out.append((u, v))
    return sorted(out, key=lambda uv: uv[1])


def cake_still_near(cam, u0, v0, radius):
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


def snapshot(bridge, name):
    image = grab(bridge)
    path = os.path.join(OUT_DIR, name)
    cv2.imwrite(path, image)
    log("[PHOTO] %s" % path)
    return image


def approach(arm, x, y, z_start):
    """Rise in front of the row, then travel at a safe height over the cubes."""
    arm.interface.set_pose(130.0, 0.0, z_start, delay=0.6)
    rospy.sleep(0.4)
    arm.interface.set_pose(x, y, z_start, delay=0.9)
    rospy.sleep(0.4)


def descend_until_contact(arm, bridge, M, x, y, z_from, z_limit):
    """Lower the wrist step by step; return (z_touch, trace).

    Contact is detected from the orange pad seen by the fixed camera: while the
    arm is free, the pad pixel tracks the commanded pose.  When the cup meets
    the cake the arm stops, so the observed pad pixel and its depth stop
    following.  This is both the touch-height measurement and the safety stop.
    """
    trace = []
    z = z_from
    prev = None
    z_touch = None
    while z >= z_limit - 1e-6:
        arm.interface.set_pose(x, y, z, delay=0.5)
        rospy.sleep(SETTLE_S)
        image = grab(bridge)
        pad = pad_pixel(image)
        pred = project_arm(M, x, y, z)
        depth = grab_depth()
        if pad is not None:
            entry = {"z": round(z, 1), "pad": [round(pad[0], 1), round(pad[1], 1)],
                     "area": round(pad[2], 0),
                     "pred": [round(pred[0], 1), round(pred[1], 1)],
                     "d": round(depth_at(depth, pad[0], pad[1]) or -1, 4)}
        else:
            entry = {"z": round(z, 1), "pad": None, "pred": [round(pred[0], 1),
                                                             round(pred[1], 1)]}
        trace.append(entry)
        log("[DESCEND] z=%6.1f pad=%s pred=%s %s"
            % (z,
               "None" if pad is None else "(%.1f, %.1f) a=%.0f" % (pad[0], pad[1], pad[2]),
               "(%.1f, %.1f)" % (pred[0], pred[1]),
               "" if pad is None else "d=%.3f" % entry["d"]))

        if prev is not None and pad is not None and prev.get("pad") is not None:
            dv_obs = pad[1] - prev["pad"][1]
            dd_obs = entry["d"] - prev["d"] if (entry.get("d", -1) > 0 and
                                                prev.get("d", -1) > 0) else None
            area_ratio = pad[2] / max(prev["area"], 1.0)
            # Free descent: the pad pixel moves ~4-5 px and the depth drops
            # ~4 mm per step.  Contact means both stop while the command keeps
            # going down - and the blob is still a full pad, not an occlusion.
            stalled = (abs(dv_obs) < 2.0
                       and dd_obs is not None and dd_obs > -0.0015
                       and 0.7 < area_ratio < 1.4)
            if stalled:
                z_touch = prev["z"]
                log("[CONTACT] pad stopped following: dv=%.1f px dd=%.0f mm at "
                    "z=%.1f (previous free step)" % (dv_obs, dd_obs * 1000.0, z_touch))
                break
        prev = entry
        z -= STEP_MM
    return z_touch, trace


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--probe", action="store_true",
                        help="approach and photograph only, no descent")
    parser.add_argument("--no-place", action="store_true",
                        help="pick and hold the cake (operator takes over)")
    args = parser.parse_args(rospy.myargv()[1:])

    os.makedirs(OUT_DIR, exist_ok=True)
    rospy.init_node("cake_final", anonymous=True)
    bridge = CvBridge()
    cam = CamAction()
    arm = ArmAction(cam=cam)
    global M
    M = cam.transform3d
    entry = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "result": None,
             "cake_pixel": None, "cake_top": None, "target": None,
             "contact_z": None, "scale": None}

    try:
        arm.interface.set_status(True)
        rospy.sleep(0.3)

        # --- 1. depth/hand-eye scale from the orange pad -------------------
        arm.interface.set_pose(*CAL_POSE, delay=0.9)
        rospy.sleep(1.0)
        image = grab(bridge)
        pad = pad_pixel(image)
        if not pad:
            log("[FAIL] orange pad not visible at the calibration pose")
            entry["result"] = "NO_PAD"
            return 3
        depth = grab_depth()
        d_pad = depth_at(depth, pad[0], pad[1])
        if not d_pad:
            log("[FAIL] no depth on the pad")
            entry["result"] = "NO_PAD_DEPTH"
            return 4
        model = np.array([CAL_POSE[0], CAL_POSE[1], CAL_POSE[2], 1.0]) @ M
        scale = d_pad / model[2]
        entry["scale"] = round(float(scale), 6)
        log("[SCALE] pad pixel=(%.1f, %.1f) depth=%.4f m -> s=%.6f"
            % (pad[0], pad[1], d_pad, scale))

        # --- 2. detect the cake while the arm is out of view ---------------
        arm.arm_grasp_ready(block=True)
        rospy.sleep(1.2)
        cubes = cam.detector(num_samples=6, timeout=6.0, conf_threshold=0.04) or []
        log("[DETS] %s" % cubes)
        cakes = cake_pixels(cubes)
        if not cakes:
            log("[FAIL] no cake detected")
            entry["result"] = "NO_CAKE"
            return 5
        u0, v0 = cakes[0]
        entry["cake_pixel"] = [round(u0, 1), round(v0, 1)]
        log("[CAKE] top-face pixel=(%.1f, %.1f)" % (u0, v0))

        depth = grab_depth()
        d_cake = depth_at(depth, u0, v0)
        if not d_cake:
            log("[FAIL] no depth on the cake pixel")
            entry["result"] = "NO_CAKE_DEPTH"
            return 6
        cake = solve_arm_point(unproject(u0, v0, d_cake), scale)
        entry["cake_top"] = [round(float(cake[0]), 1), round(float(cake[1]), 1),
                             round(float(cake[2]), 1)]
        log("[CAKE] depth=%.4f m -> top-face centre x=%.1f y=%.1f z=%.1f mm"
            % (d_cake, cake[0], cake[1], cake[2]))

        x = min(max(float(cake[0]), 140.0), 348.0)
        y = max(min(float(cake[1]), 140.0), -140.0)
        z_contact_est = float(cake[2]) + PAD_THICKNESS_MM
        z_press = max(z_contact_est - PRESS_MM, Z_MIN_HARD)
        entry["target"] = [round(x, 1), round(y, 1)]
        log("[TARGET] wrist x=%.1f y=%.1f  contact z~%.1f  press z~%.1f"
            % (x, y, z_contact_est, z_press))

        # --- 3. approach and (optionally) stop ------------------------------
        approach(arm, x, y, APPROACH_Z)
        snapshot(bridge, "01_above.jpg")
        if args.probe:
            log("[PROBE] stopping above the cake")
            entry["result"] = "PROBE"
            return 0

        # --- 4. descend until the cup meets the cake ------------------------
        skip = max(APPROACH_Z - 30.0, z_contact_est + 25.0)
        z_touch, trace = descend_until_contact(
            arm, bridge, M, x, y, skip,
            max(z_contact_est - EXTRA_DEPTH_MM, Z_MIN_HARD))
        snapshot(bridge, "02_descent.jpg")
        if z_touch is None:
            log("[FAIL] no contact detected down to the safety limit")
            entry["result"] = "NO_CONTACT"
            return 7
        entry["contact_z"] = z_touch
        jump = z_touch - 2.0
        log("[TOUCH] lowering %.1f -> %.1f mm" % (z_touch, jump))
        arm.interface.set_pose(x, y, jump, delay=0.9)
        rospy.sleep(0.8)

        # --- 5. vacuum on, lift, park, verify ------------------------------
        log("[PUMP] on")
        arm.interface.set_pump(True)
        rospy.sleep(1.0)
        z_lift = jump + 25.0
        arm.interface.set_pose(x, y, z_lift, delay=0.9)
        rospy.sleep(0.8)
        snapshot(bridge, "03_lifted.jpg")

        arm.interface.set_pose(x, y, z_lift + 40.0, delay=0.7)
        rospy.sleep(0.4)
        arm.interface.set_pose(250.0, y, z_lift + 55.0, delay=0.8)
        rospy.sleep(0.4)
        arm.interface.set_pose(*VERIFY_POSE, delay=0.9)
        rospy.sleep(1.2)
        still = cake_still_near(cam, u0, v0, 45.0)
        snapshot(bridge, "04_verify_pick.jpg")
        if still:
            log("[VERIFY] FAILED: the cake is still on the pedestal")
            entry["result"] = "MISS"
            return 8
        log("[VERIFY] SUCCESS: the cake left the pedestal")
        entry["result"] = "PICKED"

        if args.no_place:
            log("[HOLD] --no-place: cake stays on the vacuum cup")
            entry["result"] = "PICKED_HELD"
            return 0

        # --- 6. put it back on the same spot -------------------------------
        log("[PLACE] returning above the pedestal")
        arm.interface.set_pose(250.0, y, z_lift + 40.0, delay=0.8)
        rospy.sleep(0.4)
        arm.interface.set_pose(x, y, APPROACH_Z, delay=0.9)
        rospy.sleep(0.5)
        snapshot(bridge, "05_above_place.jpg")
        arm.interface.set_pose(x, y, jump + 6.0, delay=0.9)
        rospy.sleep(0.8)
        arm.interface.set_pose(x, y, jump + 1.0, delay=0.9)
        rospy.sleep(0.8)
        snapshot(bridge, "06_place.jpg")
        log("[PLACE] pump off")
        arm.interface.set_pump(False)
        rospy.sleep(1.0)
        arm.interface.set_pose(x, y, APPROACH_Z, delay=0.9)
        rospy.sleep(0.5)
        arm.interface.set_pose(*VERIFY_POSE, delay=0.9)
        rospy.sleep(1.2)
        back = cake_still_near(cam, u0, v0, 60.0)
        snapshot(bridge, "07_verify_place.jpg")
        if back:
            log("[VERIFY] SUCCESS: the cake is back on the pedestal")
            entry["result"] = "PLACED"
        else:
            log("[VERIFY] WARNING: the cake is not detected on the pedestal")
            entry["result"] = "PLACED_UNVERIFIED"

        with open(os.path.join(OUT_DIR, "trace.json"), "w") as handle:
            json.dump({"entry": entry, "trace": trace}, handle, indent=1)
        return 0
    finally:
        try:
            arm.interface.set_pump(False)
            rospy.sleep(0.3)
            arm.arm_home()
            rospy.sleep(0.3)
            log("[SAFE] pump off, arm home")
        except Exception as exc:
            log("[WARN] cleanup: %s" % exc)
        try:
            with open(os.path.expanduser("~/logs/cake_final.jsonl"), "a") as fh:
                fh.write(json.dumps(entry) + "\n")
        except Exception:
            pass
        if entry["result"]:
            log("[DONE] result=%s" % entry["result"])


if __name__ == "__main__":
    sys.exit(main())
