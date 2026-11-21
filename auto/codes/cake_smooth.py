#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Smooth cake pick & place: one continuous trajectory instead of step pauses.

Why this exists next to ``cake_final.py``: that version descended in 4 mm steps
with a camera check and a settle pause after every step - safe, but the arm
visibly stopped and started ~7 times on the way down.  The uArm firmware
follows a *stream* of closely spaced waypoints perfectly well (verified with
``smooth_probe.py``: the orange pad tracks the commanded line with a constant
2-3 px lag and no stutter).  So this script:

1. streams every transition as a time-parameterised polyline at 25 Hz, so the
   arm moves continuously through the waypoints and never stops between them
   (approach, descent, lift+retract+park, return, place and retract are six
   continuous motions instead of ~25 separate moves);
2. keeps the empirical touch height: the last successful contact z is read from
   the previous run's log, so the single descent ends 2 mm past the rim contact
   and the arm itself stalls on the cake, exactly as in the validated runs;
3. verifies "picked" and "placed" by parking the arm and looking at the
   pedestal, retrying the detector once or twice because the top-face class
   flickers on the rotated cube.

Usage:
    python3 -u ~/codes/cake_smooth.py --probe      # approach + photo only
    python3 -u ~/codes/cake_smooth.py              # full pick & put-back
    python3 -u ~/codes/cake_smooth.py --no-place   # pick and hold
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

import match_logic
from pad_detect import grab, pad_pixel
from runtask import ArmAction, CamAction

#: Waypoint publish period (s).  The firmware follows ~25 Hz waypoints smoothly.
DT = 1.0 / 25.0
#: Distance from the wrist pose to the suction-cup sealing rim (mm).
PAD_THICKNESS_MM = 8.0
#: Default touch height used when no previous run is on record (mm).  Matches the
#: long-standing ``grasp1`` constant (z_touch = 40) that this arena validates.
DEFAULT_TOUCH_Z = 40.0
#: How far past the rim contact the wrist is pushed (cup compression), mm.
PRESS_MM = 2.0
#: Speeds (mm/s) for the individual phases.
TRAVEL_MM_S = 90.0
DESCENT_MM_S = 22.0
LIFT_MM_S = 90.0

VERIFY_POSE = (50.0, 180.0, 160.0)
OUT_DIR = os.path.expanduser("~/logs/cake_smooth")
HISTORY_FILES = [os.path.expanduser("~/logs/cake_smooth.jsonl"),
                 os.path.expanduser("~/logs/cake_final.jsonl")]
STATE_FILE = os.path.expanduser("~/logs/cake_smooth_state.json")


def log(msg):
    print(msg)
    sys.stdout.flush()


# ---------------------------------------------------------------------------
# Continuous waypoint streaming
# ---------------------------------------------------------------------------

class Path:
    """Time-parameterised polyline through Cartesian waypoints (mm)."""

    def __init__(self, points):
        self.points = [np.array(p, dtype=float) for p in points]
        self.cum = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.cum.append(self.cum[-1] + float(np.linalg.norm(b - a)))
        self.total = self.cum[-1]

    def at(self, distance):
        if distance <= 0.0:
            return self.points[0]
        if distance >= self.total:
            return self.points[-1]
        for i in range(1, len(self.cum)):
            if distance <= self.cum[i]:
                a, b = self.points[i - 1], self.points[i]
                span = self.cum[i] - self.cum[i - 1]
                fraction = 0.0 if span < 1e-9 else (distance - self.cum[i - 1]) / span
                return a + (b - a) * fraction
        return self.points[-1]


class ArmStream:
    """Publishes interpolated ``position_write_topic`` targets at a fixed rate."""

    def __init__(self, arm, bridge=None):
        self.arm = arm
        self.bridge = bridge

    def _publish(self, pose):
        self.arm.interface.set_pose(pose[0], pose[1], pose[2], delay=0)

    def run(self, waypoints, speed, label="", monitor=None,
            monitor_period=0.12, settle=0.35):
        """Stream the polyline; optionally call ``monitor(pose, pad)``.

        ``monitor`` returns ``"stop"`` to abort the stream immediately (used
        for early contact detection).  Returns (elapsed, samples, stopped).
        """
        path = Path(waypoints)
        t0 = time.monotonic()
        last_monitor = -1e9
        samples = []
        stopped = False
        while True:
            t = time.monotonic() - t0
            distance = min(path.total, speed * t)
            pose = path.at(distance)
            self._publish(pose)
            if monitor is not None and (t - last_monitor) >= monitor_period:
                last_monitor = t
                pad = None
                if self.bridge is not None:
                    pad = pad_pixel(grab(self.bridge))
                samples.append((round(t, 3),
                                [round(float(v), 1) for v in pose],
                                None if pad is None else [round(pad[0], 1),
                                                          round(pad[1], 1),
                                                          round(pad[2], 0)]))
                if monitor(pose, pad) == "stop":
                    stopped = True
                    log("[STREAM] %s: stopped by monitor at z=%.1f after %.2f s"
                        % (label, pose[2], t))
                    break
            if distance >= path.total - 1e-9:
                break
            rospy.sleep(DT)
        elapsed = time.monotonic() - t0
        self._publish(path.points[-1])
        rospy.sleep(settle)
        log("[STREAM] %s: %.0f mm in %.2f s (%.0f mm/s)%s"
            % (label, path.total, elapsed, path.total / max(elapsed, 1e-3),
               " [early stop]" if stopped else ""))
        return elapsed, samples, stopped


def observe_only(pose, pad):
    """Passive observer: records pad samples, never aborts the stream."""
    return None


# ---------------------------------------------------------------------------
# Perception helpers (same validated pipeline as cake_final.py)
# ---------------------------------------------------------------------------

def load_memory():
    """Return (scale, touch_z) from previous runs, or (None, None)."""
    scale = None
    touch_z = None
    try:
        with open(STATE_FILE, "r", encoding="utf8") as handle:
            state = json.load(handle)
        scale = state.get("scale")
        touch_z = state.get("touch_z")
    except Exception:
        pass
    for path in HISTORY_FILES:
        if not os.path.exists(path):
            continue
        try:
            with open(path, "r", encoding="utf8") as handle:
                for line in handle:
                    try:
                        entry = json.loads(line)
                    except Exception:
                        continue
                    if entry.get("scale"):
                        scale = entry["scale"]
                    if entry.get("contact_z"):
                        touch_z = entry["contact_z"]
        except Exception:
            continue
    return scale, touch_z


def save_memory(scale, touch_z):
    try:
        with open(STATE_FILE, "w", encoding="utf8") as handle:
            json.dump({"scale": scale, "touch_z": touch_z,
                       "updated": time.strftime("%Y-%m-%dT%H:%M:%S")},
                      handle, indent=1)
    except Exception as exc:
        log("[WARN] could not save state: %s" % exc)


def solve_arm_point(M, cam_point, scale):
    a = np.array([[M[0][j], M[1][j], M[2][j]] for j in range(3)])
    b = np.array([cam_point[j] / scale - M[3][j] for j in range(3)])
    return np.linalg.solve(a, b)


def unproject(u, v, d):
    return np.array([(u - match_logic.CAM_CX) / match_logic.CAM_FX * d,
                     (v - match_logic.CAM_CY) / match_logic.CAM_FY * d, d])


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


def cake_pixels(cubes):
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


def cake_near(cam, u0, v0, radius, tries=1):
    """True when a cake-class detection sits within ``radius`` px of (u0, v0).

    The detector occasionally misses the top face of the rotated cake cube for
    a cycle, so a negative answer is confirmed with ``tries`` scans before it is
    trusted.
    """
    for attempt in range(max(1, int(tries))):
        cubes = cam.detector(num_samples=6, timeout=6.0, conf_threshold=0.04) or []
        for cube in cubes:
            try:
                cid = int(cube[0])
                cu, cv = float(cube[1][0]), float(cube[1][1])
            except Exception:
                continue
            if match_logic.is_cake(cid) and math.hypot(cu - u0, cv - v0) <= radius:
                log("[VERIFY] cake class %d at (%.0f, %.0f)" % (cid, cu, cv))
                return True
        if attempt + 1 < tries:
            log("[VERIFY] no cake near the pedestal, retrying scan")
            rospy.sleep(0.4)
    return False


def snapshot(bridge, name):
    import cv2
    image = grab(bridge)
    path = os.path.join(OUT_DIR, name)
    cv2.imwrite(path, image)
    log("[PHOTO] %s" % path)
    return image


def measure_scale(arm, bridge):
    """Derive the depth<->hand-eye scale from the orange pad at a known pose."""
    pose = (270.0, 0.0, 30.0)
    arm.interface.set_pose(*pose, delay=0.9)
    rospy.sleep(1.0)
    pad = pad_pixel(grab(bridge))
    if not pad:
        return None
    depth = grab_depth()
    d = depth_at(depth, pad[0], pad[1])
    if not d:
        return None
    model = np.array([pose[0], pose[1], pose[2], 1.0]) @ arm.cam.transform3d
    scale = d / model[2]
    log("[SCALE] pad pixel=(%.1f, %.1f) depth=%.4f m -> s=%.6f"
        % (pad[0], pad[1], d, scale))
    return float(scale)


# ---------------------------------------------------------------------------
# Main sequence
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--probe", action="store_true",
                        help="approach and photograph only, no descent")
    parser.add_argument("--no-place", action="store_true",
                        help="pick and hold the cake (operator takes over)")
    args = parser.parse_args(rospy.myargv()[1:])

    os.makedirs(OUT_DIR, exist_ok=True)
    rospy.init_node("cake_smooth", anonymous=True)
    bridge = CvBridge()
    cam = CamAction()
    arm = ArmAction(cam=cam)
    M = cam.transform3d
    streamer = ArmStream(arm, bridge)
    entry = {"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "result": None,
             "cake_pixel": None, "target": None, "contact_z": None,
             "scale": None, "timings": {}}
    keep_pump = False

    try:
        arm.interface.set_status(True)
        rospy.sleep(0.3)

        scale, touch_z = load_memory()
        if not scale:
            log("[SCALE] no previous scale on record, measuring the pad once")
            scale = measure_scale(arm, bridge)
            if not scale:
                log("[FAIL] orange pad not visible, cannot calibrate")
                entry["result"] = "NO_PAD"
                return 3
            arm.arm_home(block=True)
            rospy.sleep(0.6)
        entry["scale"] = round(float(scale), 6)
        if touch_z:
            log("[TOUCH] using last measured contact z=%.1f mm" % touch_z)
        else:
            touch_z = DEFAULT_TOUCH_Z
            log("[TOUCH] no contact history, using z=%.1f mm" % touch_z)

        # --- 1. scan -------------------------------------------------------
        arm.arm_home(block=True)
        rospy.sleep(0.8)
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
        if d_cake:
            cake = solve_arm_point(M, unproject(u0, v0, d_cake), scale)
            x = min(max(float(cake[0]), 140.0), 348.0)
            y = max(min(float(cake[1]), 140.0), -140.0)
            log("[CAKE] depth %.3f m -> lateral target x=%.1f y=%.1f "
                "(depth z estimate %.0f is only diagnostics)"
                % (d_cake, x, y, cake[2]))
        else:
            log("[FAIL] no depth on the cake pixel")
            entry["result"] = "NO_CAKE_DEPTH"
            return 6
        entry["target"] = [round(x, 1), round(y, 1)]

        z_press = max(touch_z - PRESS_MM, -60.0)
        z_approach = 90.0
        log("[TARGET] x=%.1f y=%.1f; smooth descent 90 -> %.1f mm" % (x, y, z_press))

        # --- 2. one smooth approach from home ------------------------------
        t0 = time.monotonic()
        streamer.run([(130.0, 0.0, z_approach), (x, y, z_approach)],
                     TRAVEL_MM_S, label="approach")
        snapshot(bridge, "01_above.jpg")
        entry["timings"]["approach_s"] = round(time.monotonic() - t0, 2)
        if args.probe:
            log("[PROBE] stopping above the cake")
            entry["result"] = "PROBE"
            return 0

        # --- 3. one smooth descent -----------------------------------------
        t0 = time.monotonic()
        _, descent_samples, _ = streamer.run(
            [(x, y, z_approach), (x, y, z_press)],
            DESCENT_MM_S, label="descent", monitor=observe_only)
        entry["timings"]["descent_s"] = round(time.monotonic() - t0, 2)
        entry["descent"] = descent_samples
        snapshot(bridge, "02_touch.jpg")

        # --- 4. vacuum + one smooth lift/retract/park ----------------------
        log("[PUMP] on")
        arm.interface.set_pump(True)
        rospy.sleep(0.8)
        t0 = time.monotonic()
        streamer.run([(x, y, z_press + 40.0), (x, y, 85.0),
                      (255.0, y, 110.0), (90.0, 175.0, 150.0),
                      list(VERIFY_POSE)],
                     LIFT_MM_S, label="lift+retract+park")
        entry["timings"]["lift_retract_s"] = round(time.monotonic() - t0, 2)
        rospy.sleep(0.6)
        snapshot(bridge, "03_hold.jpg")
        still = cake_near(cam, u0, v0, 45.0, tries=2)
        snapshot(bridge, "04_verify_pick.jpg")
        if still:
            log("[VERIFY] FAILED: the cake is still on the pedestal")
            entry["result"] = "MISS"
            return 8
        log("[VERIFY] SUCCESS: the cake left the pedestal")
        entry["result"] = "PICKED"
        entry["contact_z"] = float(touch_z)
        save_memory(entry["scale"], entry["contact_z"])

        if args.no_place:
            log("[HOLD] --no-place: cake stays on the vacuum cup")
            entry["result"] = "PICKED_HELD"
            keep_pump = True
            return 0

        # --- 5. back above the pedestal in one motion ----------------------
        t0 = time.monotonic()
        streamer.run([(90.0, 175.0, 150.0), (255.0, y, 110.0),
                      (x, y, z_approach)],
                     TRAVEL_MM_S, label="return above pedestal")
        entry["timings"]["return_s"] = round(time.monotonic() - t0, 2)
        snapshot(bridge, "05_above_place.jpg")

        # one smooth descent to the same wrist height, then release
        t0 = time.monotonic()
        streamer.run([(x, y, z_approach), (x, y, z_press)],
                     DESCENT_MM_S, label="place descent")
        entry["timings"]["place_s"] = round(time.monotonic() - t0, 2)
        snapshot(bridge, "06_place.jpg")
        log("[PLACE] pump off")
        arm.interface.set_pump(False)
        rospy.sleep(0.8)

        # --- 6. one smooth retract + verify --------------------------------
        streamer.run([(x, y, 85.0), (255.0, y, 110.0),
                      (90.0, 175.0, 150.0), list(VERIFY_POSE)],
                     LIFT_MM_S, label="retract away")
        rospy.sleep(0.6)
        back = cake_near(cam, u0, v0, 60.0, tries=3)
        snapshot(bridge, "07_verify_place.jpg")
        if back:
            log("[VERIFY] SUCCESS: the cake is back on the pedestal")
            entry["result"] = "PLACED"
        else:
            log("[VERIFY] WARNING: the cake is not detected on the pedestal")
            entry["result"] = "PLACED_UNVERIFIED"
        return 0
    finally:
        try:
            if keep_pump:
                # The operator takes over while the vacuum keeps holding the cake.
                log("[SAFE] pump left ON for operator takeover")
            else:
                arm.interface.set_pump(False)
                rospy.sleep(0.3)
                arm.arm_home()
                rospy.sleep(0.3)
                log("[SAFE] pump off, arm home")
        except Exception as exc:
            log("[WARN] cleanup: %s" % exc)
        try:
            with open(os.path.expanduser("~/logs/cake_smooth.jsonl"), "a") as fh:
                fh.write(json.dumps(entry) + "\n")
        except Exception:
            pass
        if entry["result"]:
            log("[DONE] result=%s timings=%s" % (entry["result"], entry["timings"]))

if __name__ == "__main__":
    sys.exit(main())
