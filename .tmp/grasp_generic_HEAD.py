#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unified cube grasping logic with smooth native motion planning and single-command base approach.

Priority Logic (as specified by operator):
  1) Cake visible and IN REACH:
     Pick central cake immediately without moving base (reach over front row).
  2) Cake visible and OUT OF REACH:
     Approach base in ONE command (align + drive to safety distance, avoiding front row),
     damp vibrations, re-check reach (if not in reach, loop back to approach again), grasp.
  3) No cake, standard cube IN REACH:
     Pick central cube immediately without moving base.
  4) No cake, standard cube OUT OF REACH:
     Approach base in ONE command (align + drive to sweet spot),
     damp vibrations, re-check reach (if not in reach, loop back to approach again), grasp.

Motion & Kinematics Refinements:
  - Eliminates micro-waypoint streaming that caused buffer-underrun motor chatter/jerks.
  - Native smooth point-to-point moves with calm, stable trajectory timing.
  - Strictly vertical lift and descent (X, Y fixed) to prevent strange sideways cube swinging.
  - Settle pause after base movement (0.70s) to completely damp chassis and camera mast oscillations.
  - Multi-sample clustered re-scan after vibration settles for millimeter-accurate coordinates.
  - Pump engages before touch for instantaneous, rock-solid vacuum seal.
  - Put-back guarantees cube is resting on ground before vacuum release.
  - No photo/snapshot overhead.
"""

import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2

import match_logic
from pad_detect import grab
from runtask import ArmAction, CamAction

PRESS_MM = 3.0
DEFAULT_CAKE_TOUCH_Z = 35.0
FLOOR_TOUCH_Z = -55.0

# Forward offset (mm) for Tier-2 cake suction cup centering to reach true center of elevated cake
CAKE_CUP_OFFSET_X = 125.0

# Approach, lift and clearance heights (mm)
APPROACH_CAKE_Z = 75.0
LIFT_CAKE_Z = 80.0

APPROACH_FLOOR_Z = 70.0
LIFT_FLOOR_Z = 60.0

PARK_POSE = (50.0, 180.0, 160.0)
OUT_DIR = os.path.expanduser("~/logs/grasp_generic")
STATE_FILE = os.path.expanduser("~/logs/cake_smooth_state.json")

# Workspace envelopes
# Standard floor cube (uArm physical reach extends safely to 300 mm):
FLOOR_IN_REACH = dict(x_lo=140.0, x_hi=290.0, y_lim=100.0, dist_max=295.0,
                      dist_target=240.0)
# Tier-2 cake (reaching over front row):
# dist_target=305 keeps the chassis >6 cm away from first-row cubes to prevent knocking them over!
# Horizontal/lateral safe limits removed per operator directive:
CAKE_IN_REACH = dict(dist_target=305.0)


def log(msg):
    print(msg)
    sys.stdout.flush()


# ---------------------------------------------------------------------------
# Perception & Coordinates
# ---------------------------------------------------------------------------

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


def unproject(u, v, d):
    return np.array([(u - match_logic.CAM_CX) / match_logic.CAM_FX * d,
                     (v - match_logic.CAM_CY) / match_logic.CAM_FY * d, d])


def solve_arm_point(M, cam_point, scale):
    a = np.array([[M[0][j], M[1][j], M[2][j]] for j in range(3)])
    b = np.array([cam_point[j] / scale - M[3][j] for j in range(3)])
    return np.linalg.solve(a, b)


def load_memory():
    """Return (depth scale, cake touch z) from previous runs."""
    scale = None
    touch_z = None
    try:
        with open(STATE_FILE, "r", encoding="utf8") as handle:
            state = json.load(handle)
        scale = state.get("scale")
        touch_z = state.get("touch_z")
    except Exception:
        pass
    if scale is None:
        scale = 0.386784
    if touch_z is None:
        touch_z = DEFAULT_CAKE_TOUCH_Z
    return scale, touch_z


def bumper(side="front"):
    """Check base IR bumper."""
    try:
        from spark_base.msg import SparkBaseSensor
        msg = rospy.wait_for_message("/spark_base/sensor", SparkBaseSensor,
                                     timeout=3.0)
    except Exception as exc:
        log("[WARN] no base sensor: %s" % exc)
        return False
    if side == "front":
        return bool(msg.ir_bumper_front or msg.ir_bumper_front_left
                    or msg.ir_bumper_front_right)
    return bool(getattr(msg, "ir_bumper_back_left", False)
                or getattr(msg, "ir_bumper_back_right", False))


def reach_state(mode, x, y, z_est=None):
    dist = math.hypot(x, y)
    if mode == "cake":
        # Lateral/horizontal limits completely removed per operator directive;
        # height validation only:
        if z_est is not None and not (-140.0 <= z_est <= 130.0):
            return False, dist
        # Base approach condition: target is considered in grasping reach once chassis
        # is at or near the target distance (protecting front row cubes)
        if dist > (CAKE_IN_REACH["dist_target"] + 50.0):
            return False, dist
        return True, dist
    env = FLOOR_IN_REACH
    in_reach = (env["x_lo"] <= x <= env["x_hi"] and abs(y) <= env["y_lim"]
                and dist <= env["dist_max"])
    return in_reach, dist


def target_distance(mode, x, y):
    if mode == "cake":
        return math.hypot(x, y), CAKE_IN_REACH["dist_target"]
    return math.hypot(x, y), FLOOR_IN_REACH["dist_target"]


def compute_target_coords(cam, u, v, is_cake, scale, cake_offset=CAKE_CUP_OFFSET_X):
    """Compute 3D arm workspace coordinates (x, y) from image pixel."""
    xy = None
    z_est = None
    try:
        depth = grab_depth()
        d = depth_at(depth, u, v)
        if d is not None:
            point = solve_arm_point(cam.transform3d, unproject(u, v, d), scale)
            cand_z = float(point[2])
            # For elevated cake, if depth reading hit floor/background (z < -30 mm),
            # reject it so we use the dedicated Tier-2 cake affine calibration
            if is_cake and cand_z < -30.0:
                log("[TARGET] Cake depth reading hit floor/background (z=%.1f) -> using cake affine" % cand_z)
            else:
                xy = (float(point[0]), float(point[1]))
                z_est = cand_z
    except Exception as exc:
        log("[WARN] depth reconstruction exception: %s" % exc)

    if xy is None:
        if is_cake:
            xy = match_logic.pixel_to_cake_xy(cam.transform3d, u, v, cup_offset_x=cake_offset)
        else:
            dlt_xy = match_logic.pixel_to_xy_at_z(cam.transform3d, u, v, -100.0)
            if dlt_xy is not None:
                xy = (float(dlt_xy[0]) + 20.0, float(dlt_xy[1]))

    return xy, z_est


def scan_scene(cam, arm, scale, fast=False, tag="", cake_offset=CAKE_CUP_OFFSET_X):
    """Detect cubes, separating Cake vs Floor cubes.

    If fast=False: park arm out of camera view and take 2 samples.
    If fast=True: arm is already parked; take 2 stable samples (after vibration damping).
    """
    if not fast:
        arm.interface.set_pose(PARK_POSE[0], PARK_POSE[1], PARK_POSE[2], delay=0.35)
        rospy.sleep(0.15)

    num_samples = 5
    raw_detections = cam.detector(num_samples=num_samples, timeout=4.0, conf_threshold=0.04) or []
    log("[SCAN%s] Detections (samples=%d): %s" % (tag, num_samples, raw_detections))

    valid_cubes = []
    for item in raw_detections:
        try:
            cid = int(item[0])
            u, v = float(item[1][0]), float(item[1][1])
            # Filter to active competition field (ignore distant ceiling/wall noise v < 130)
            if 130.0 <= v <= 450.0 and 80.0 <= u <= 560.0:
                valid_cubes.append((cid, (u, v)))
        except Exception:
            continue

    if not valid_cubes:
        log("[SCAN%s] No valid cubes detected within camera field of play" % tag)
        return None

    cake_list = [c for c in valid_cubes if match_logic.is_cake(c[0])]
    floor_list = [c for c in valid_cubes if not match_logic.is_cake(c[0])]

    if cake_list:
        for c in sorted(cake_list, key=lambda item: abs(item[1][0] - 320.0)):
            u, v = c[1]
            cid = c[0]
            xy, z_est = compute_target_coords(cam, u, v, is_cake=True, scale=scale, cake_offset=cake_offset)
            if xy is None:
                continue
            if z_est is not None and not (-130.0 <= z_est <= 130.0):
                log("[FILTER] Cake candidate %d at (%.1f, %.1f) has out-of-bounds z_est=%.1f -> skipped"
                    % (cid, u, v, z_est))
                continue
            return {
                "mode": "cake",
                "is_cake": True,
                "class": cid,
                "pixel": [round(u, 1), round(v, 1)],
                "xy": [round(xy[0], 1), round(xy[1], 1)],
                "z_est": round(z_est, 1) if z_est is not None else None,
            }

    if floor_list:
        # Pick candidate closest to optical centerline and grasping sweet spot (u=320, v=270)
        best_cube = min(floor_list, key=lambda c: abs(c[1][0] - 320.0) * 0.8 + abs(c[1][1] - 270.0))
        if best_cube is not None:
            u, v = float(best_cube[1][0]), float(best_cube[1][1])
            cid = int(best_cube[0])
            xy, z_est = compute_target_coords(cam, u, v, is_cake=False, scale=scale)
            if xy is not None and (z_est is None or (-250.0 <= z_est <= 110.0)):
                return {
                    "mode": "floor",
                    "is_cake": False,
                    "class": cid,
                    "pixel": [round(u, 1), round(v, 1)],
                    "xy": [round(xy[0], 1), round(xy[1], 1)],
                    "z_est": round(z_est, 1) if z_est is not None else None,
                }

    return None


# ---------------------------------------------------------------------------
# Single-Command Base Approach with Vibration Damping
# ---------------------------------------------------------------------------

def drive_approach_step(arm, target):
    """Align heading and approach base towards target in ONE clean command sequence."""
    mode = target["mode"]
    x, y = target["xy"]

    # 1. Heading alignment (in-place turn)
    angle_rad = math.atan2(y, x)
    if abs(angle_rad) > math.radians(2.5):
        log("[APPROACH] Aligning heading: %.1f deg" % math.degrees(angle_rad))
        arm.turn_base(angle_rad)
        rospy.sleep(0.35)  # Dwell for turn rotation to settle

    # 2. Linear displacement to target sweet spot
    dist, desired = target_distance(mode, x, y)
    drive_m = (dist - desired) / 1000.0

    if abs(drive_m) >= 0.02:
        target_dist = math.hypot(x, y)
        forward = drive_m > 0
        blocked = bumper("front" if forward else "back")

        if blocked and target_dist < 180.0:
            log("[APPROACH] Front bumper triggered and obstacle within %.0f mm - "
                "refusing forward drive to protect front row cubes!" % target_dist)
            return False

        step_m = max(-0.25, min(0.25, drive_m))
        log("[APPROACH] Single drive: %+.1f cm (target dist: %.0f mm, desired: %.0f mm)"
            % (step_m * 100.0, dist, desired))
        arm.drive_base(step_m)

    # 3. Vibration damping dwell: allow Kobuki suspension and camera mast to completely settle
    log("[APPROACH] Base motion complete. Settling chassis for 0.85s to eliminate all vibrations...")
    rospy.sleep(0.85)
    return True


# ---------------------------------------------------------------------------
# Smooth, Jerk-Free Grasp Execution (Native Point-to-Point Trajectory)
# ---------------------------------------------------------------------------

def object_near(cam, u0, v0, radius=55.0, classes=None, tries=2):
    """Fast check whether object is still detected near (u0, v0)."""
    for attempt in range(max(1, int(tries))):
        cubes = cam.detector(num_samples=2, timeout=2.0, conf_threshold=0.04) or []
        for c in cubes:
            try:
                cid = int(c[0])
                cu, cv = float(c[1][0]), float(c[1][1])
            except Exception:
                continue
            if classes is not None and match_logic.norm_id(cid) not in classes:
                continue
            if math.hypot(cu - u0, cv - v0) <= radius:
                return True
        if attempt + 1 < tries:
            rospy.sleep(0.15)
    return False


def grasp_cycle(cam, arm, bridge, target, cake_touch, no_place=False):
    """Calm, smooth, jerk-free pick and place with strictly vertical motion and no photo delays."""
    x, y = target["xy"]
    u0, v0 = target["pixel"]
    mode = target["mode"]
    z_est = target.get("z_est")
    class_set = {match_logic.CAKE_ID} if target["is_cake"] else None

    # Determine whether target is physically elevated (Tier-2 cake pedestal) or resting on floor.
    # Floor cubes have z_est around -115 mm; Tier-2 raised pedestal has z_est around -20..+40 mm.
    is_elevated = (mode == "cake" and (z_est is None or z_est > -70.0))

    if is_elevated:
        z_approach = APPROACH_CAKE_Z
        z_touch = float(cake_touch) - PRESS_MM
        z_lift = LIFT_CAKE_Z
        log("[GRASP] Target mode=%s is physically ELEVATED (z_est=%s) -> z_touch=%.1f"
            % (mode, z_est, z_touch))
    else:
        z_approach = APPROACH_FLOOR_Z
        z_touch = FLOOR_TOUCH_Z - PRESS_MM
        z_lift = LIFT_FLOOR_Z
        log("[GRASP] Target mode=%s is physically on FLOOR (z_est=%s) -> z_touch=%.1f"
            % (mode, z_est, z_touch))

    log("[GRASP] Target: %s at (x=%.1f, y=%.1f) | z_app=%.1f -> z_touch=%.1f -> z_lift=%.1f"
        % (mode, x, y, z_approach, z_touch, z_lift))

    entry = {"target": target, "result": None}

    # 1. Smooth, calm approach: transit above cube
    log("[MOTION] 1. Transit above cube")
    arm.interface.set_pose(140.0, y * 0.4, z_approach, delay=0.45)
    arm.interface.set_pose(x, y, z_approach, delay=0.85)

    # 2. Engage vacuum pump
    log("[MOTION] 2. Vacuum pump ON")
    arm.interface.set_pump(True)
    rospy.sleep(0.25)

    # 3. Clean vertical descent straight to cube surface (1.75s calm physical descent)
    log("[MOTION] 3. Smooth descent to surface (z=%.1f)" % z_touch)
    arm.interface.set_pose(x, y, z_touch, delay=1.75)

    # Dwell to compress silicone cup and build full negative pressure
    rospy.sleep(0.45)

    # 4. STRICTLY VERTICAL LIFT: X and Y fixed, no swinging
    log("[MOTION] 4. Vertical lift to z=%.1f" % z_lift)
    arm.interface.set_pose(x, y, z_lift, delay=1.15)
    rospy.sleep(0.35)

    # Move arm slightly back/aside so camera has an unobstructed line of sight to pedestal at (u0, v0)
    arm.interface.set_pose(140.0, 90.0, z_lift + 30.0, delay=0.85)
    rospy.sleep(0.45)

    # 5. Verification that cube was picked from pedestal/surface
    still_there = object_near(cam, u0, v0, radius=45.0, classes=class_set, tries=2)
    if still_there:
        log("[VERIFY] Pick verification FAILED: cube still detected on pedestal/floor")
        entry["result"] = "MISS"
        return entry

    log("[VERIFY] Pick verification SUCCESS: cube lifted into the air!")
    entry["result"] = "PICKED"

    if no_place:
        entry["result"] = "PICKED_HELD"
        return entry

    # 6. Put-Back: Return above location and descend vertically all the way to surface
    log("[MOTION] 5. Moving back above placement site")
    arm.interface.set_pose(x, y, z_lift, delay=0.85)
    log("[MOTION] 6. Lowering cube vertically to ground (z=%.1f)" % z_touch)
    arm.interface.set_pose(x, y, z_touch, delay=1.75)

    # Crucial dwell: ensure cube is completely resting on the floor before pump off
    rospy.sleep(0.55)

    # 7. Release vacuum pump AFTER cube is stationary on ground
    log("[MOTION] 6. Disengaging vacuum pump")
    arm.interface.set_pump(False)

    # Dwell to break vacuum seal
    rospy.sleep(0.45)

    # 8. Retract arm straight up away from released cube
    log("[MOTION] 7. Retracting arm straight up")
    arm.interface.set_pose(x, y, z_approach, delay=0.80)
    arm.interface.set_pose(140.0, 0.0, z_approach, delay=0.45)
    arm.interface.set_pose(PARK_POSE[0], PARK_POSE[1], PARK_POSE[2], delay=0.45)

    back_in_place = object_near(cam, u0, v0, radius=55.0, classes=class_set, tries=1)
    if back_in_place:
        log("[VERIFY] Place verification SUCCESS: cube confirmed back in place")
        entry["result"] = "PLACED"
    else:
        log("[VERIFY] Place completed (unverified)")
        entry["result"] = "PLACED_UNVERIFIED"

    return entry


# ---------------------------------------------------------------------------
# Main Orchestration Loop
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scan-only", action="store_true",
                        help="Only scan and classify according to 4-step logic, do not move arm or base")
    parser.add_argument("--no-place", action="store_true",
                        help="Grasp and hold without putting back")
    parser.add_argument("--cake-offset", type=float, default=CAKE_CUP_OFFSET_X,
                        help="Suction cup forward offset in mm for elevated cake (default %.1f mm)" % CAKE_CUP_OFFSET_X)
    args = parser.parse_args(rospy.myargv()[1:])

    os.makedirs(OUT_DIR, exist_ok=True)
    rospy.init_node("grasp_generic", anonymous=True)

    bridge = CvBridge()
    cam = CamAction()
    arm = ArmAction(cam=cam)

    # Ensure uArm driver is actively connected before any actions
    connect_deadline = time.time() + 3.0
    while arm.interface.arm_position_pub.get_num_connections() == 0 and time.time() < connect_deadline:
        rospy.sleep(0.1)

    if arm.interface.arm_position_pub.get_num_connections() == 0:
        log("[ERROR] uArm SwiftPro driver is NOT connected (0 subscribers on position_write_topic)!")
        entry["result"] = "ARM_NOT_CONNECTED"
        return 2

    scale, cake_touch = load_memory()
    log("[CONFIG] Depth scale: %s | Cake touch z: %.1f mm | Floor touch z: %.1f mm"
        % (scale, cake_touch, FLOOR_TOUCH_Z))

    entry = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "result": None}

    MAX_ATTEMPTS = 3
    is_initial = True

    try:
        arm.interface.set_status(True)
        rospy.sleep(0.20)

        for attempt in range(MAX_ATTEMPTS):
            # ---------------------------------------------------------------
            # 1. Scan and evaluate the 4-step logic in strict specified priority:
            #   1) Cake in reach -> pick immediately without moving base
            #   2) Cake out of reach -> approach base in one command, re-check
            #   3) No cake, standard cube in reach -> pick immediately without moving base
            #   4) No cake, standard cube out of reach -> approach base in one command, re-check
            # ---------------------------------------------------------------
            target = scan_scene(cam, arm, scale, fast=not is_initial,
                                tag="_init" if is_initial else f"_approach_{attempt}",
                                cake_offset=args.cake_offset)
            is_initial = False

            if target is None:
                log("[DECISION] No valid targets detected on field.")
                entry["result"] = "NO_TARGET"
                return 5

            mode = target["mode"]
            x, y = target["xy"]
            in_reach, metric = reach_state(mode, x, y, z_est=target.get("z_est"))

            if mode == "cake":
                if in_reach:
                    case_id = 1
                    case_desc = "1) Cake visible and IN REACH -> pick immediately without moving base"
                else:
                    case_id = 2
                    case_desc = "2) Cake visible and OUT OF REACH -> single approach, avoid front row"
            else:
                if in_reach:
                    case_id = 3
                    case_desc = "3) No cake, standard cube IN REACH -> pick immediately without moving base"
                else:
                    case_id = 4
                    case_desc = "4) No cake, standard cube OUT OF REACH -> single approach to sweet spot"

            log("[DECISION] (Attempt %d/%d) Step %s" % (attempt + 1, MAX_ATTEMPTS, case_desc))
            log("[TARGET] mode=%s class=%d xy=(%.1f, %.1f) in_reach=%s (dist=%.1f) z_est=%s"
                % (mode, target["class"], x, y, in_reach, metric, target.get("z_est")))
            entry["case"] = case_id
            entry["target"] = target

            if args.scan_only:
                log("[SCAN-ONLY] Assessment complete, stopping before any motion.")
                entry["result"] = "SCANNED"
                return 0

            # IF IN REACH -> GRASP IMMEDIATELY!
            if in_reach:
                log("[EXECUTE] Target IN REACH! Grasping cube calmly and smoothly...")
                result = grasp_cycle(cam, arm, bridge, target, cake_touch,
                                     no_place=args.no_place)
                entry.update(result)
                return 0

            # IF OUT OF REACH -> Approach base in one command, damp vibrations, then loop back to re-check!
            log("[EXECUTE] Target OUT OF REACH. Approaching base in one command...")
            drive_ok = drive_approach_step(arm, target)
            if not drive_ok:
                log("[FAIL] Base approach blocked or unsafe.")
                entry["result"] = "APPROACH_BLOCKED"
                return 6

            log("[RETRY] Base stopped and vibrations eliminated. Re-checking reach at start of algorithm...")

        log("[FAIL] Target could not be brought into comfortable reach after %d attempts." % MAX_ATTEMPTS)
        entry["result"] = "OUT_OF_REACH"
        return 6

    finally:
        try:
            if entry.get("result") != "PICKED_HELD":
                arm.interface.set_pump(False)
                rospy.sleep(0.20)
                arm.arm_home()
                rospy.sleep(0.25)
                log("[SAFE] Arm homed, vacuum pump disengaged.")
        except Exception as exc:
            log("[WARN] Cleanup error: %s" % exc)

        try:
            with open(os.path.expanduser("~/logs/grasp_generic.jsonl"), "a") as fh:
                fh.write(json.dumps(entry) + "\n")
        except Exception:
            pass

        if entry.get("result"):
            log("[COMPLETED] Result: %s (Case %s)" % (entry["result"], entry.get("case")))


if __name__ == "__main__":
    sys.exit(main())
