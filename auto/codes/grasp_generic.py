#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cube grasp -> raise -> 5 s hold -> put back -> park, field test (NXROBO Spark).

FIELD-TEST SEQUENCE (exactly the operator-requested algorithm)
---------------------------------------------------------------
1. Lower the manipulator: safe transit above the detected item at the
   clearance height.
2. Suction: vacuum ON, then a strictly vertical descent to the item surface
   and a dwell at the contact height to build vacuum.
3. Raise the cube STRICTLY VERTICALLY (X and Y never change while holding it).
4. Hold the cube in the raised position for ``hold_seconds`` (5 s by default).
5. Put the cube back at the SAME X/Y (strictly vertical descent) and release
   the pump only after the cube is resting on the ground.
6. Retract the arm vertically and park it in its ORIGINAL pose.

There is deliberately NO camera snapshot after the lift and NO success check
after the put-back.

TARGET ACQUISITION: A CUBE OUT OF THE ARM'S REACH
-------------------------------------------------
A cube that is visible but outside the manipulator's workspace is NOT an
error.  The scan reports it as a far-field target (``far=True``), the base
aligns its heading and drives one bounded step towards it, and the next scan -
now taken from the calibrated near field - produces the exact grasp
coordinates.  ``MAX_APPROACH_ATTEMPTS`` bounds the loop; a target that still
cannot be reached ends with ``OUT_OF_REACH`` (exit code 6).  Before this was
implemented, any cube beyond ~35 cm was filtered out of the scan, so the
robot answered ``NO_TARGET`` (exit code 3) without moving.

The script installs its own SIGINT/SIGTERM handler (``disable_signals=True``):
rospy's default handler calls ``signal_shutdown()`` first (after that every
publish is silently dropped) and then swallows ``KeyboardInterrupt``, so the
abort path below would never run.  With the custom handler Ctrl-C raises
inside the sequence while the node is still alive: the abort path lowers the
cube back to the pickup point, releases the pump only at contact and then
parks the arm.  That is what prevents the "robot dropped the cube instead of
returning it" failures (a false camera check after the lift used to abort the
put-back and switch the pump off in mid-air).

INTEGRATION INTO THE MAIN PROGRAM
---------------------------------
The reusable API is free of camera/field-test scaffolding:

    plan = plan_grasp(target, cake_touch_z)      # wrist heights from a detection
    execute_sequence(arm, plan,
                     hold_seconds=5.0,
                     return_cube=False)           # steps 1-4 only
    # returns with the cube attached, vacuum still ON, wrist at plan.z_lift;
    # the main program now carries the cube to its destination zone.

Steps 5-6 (put the cube back at the pickup point + park the arm) are marked
with a banner in ``execute_sequence`` and are deleted for the final
integration (or skipped with ``return_cube=False``).

Run on the robot through ``run_grasp_test.sh``:

    python3 -u ~/codes/grasp_generic.py               # full test incl. put-back
    python3 -u ~/codes/grasp_generic.py --no-place    # steps 1-4 only (preview)
    python3 -u ~/codes/grasp_generic.py --scan-only   # perception/assessment only
"""

import argparse
import json
import math
import os
import signal
import sys
import time
from dataclasses import asdict, dataclass
from typing import Tuple

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import numpy as np
import rospy
from sensor_msgs.msg import Image

import match_logic
from match_logic import (
    DEFAULT_CAKE_TOUCH_Z,
    FLOOR_TOUCH_Z,
    approach_target_distance as target_distance,
    grasp_heights as _grasp_heights,
    grasp_reach_state as reach_state,
)
from runtask import ArmAction, CamAction

# ---------------------------------------------------------------------------
# Grasp geometry (uArm wrist coordinates, millimetres)
# ---------------------------------------------------------------------------
# All heights, reach envelopes and the reach decision live in match_logic so
# the unit tests exercise exactly what runs on the robot.  Re-exported above:
# ``_grasp_heights``      -> match_logic.grasp_heights
# ``reach_state``         -> match_logic.grasp_reach_state (understands "far")
# ``target_distance``     -> match_logic.approach_target_distance
CAKE_CUP_OFFSET_X = match_logic.CAKE_CUP_OFFSET_X  # single source of truth (125 mm)

TRANSIT_X = 140.0                    # intermediate X for the safe lateral transit
TRANSIT_Y_FACTOR = 0.4               # intermediate Y = plan.y * TRANSIT_Y_FACTOR

#: "Initial position" of the manipulator: the pose it starts the flow from and
#: returns to.  Out of the camera view; the same pose as ``arm_grasp_ready()``.
PARK_POSE = (50.0, 180.0, 160.0)

# ---------------------------------------------------------------------------
# Timing (seconds)
# ---------------------------------------------------------------------------
HOLD_SECONDS = 5.0                   # operator requirement: hold in the air, 5 s
PUMP_SETTLE_TIME = 0.25
DESCENT_TIME = 1.75
SEAL_DWELL_TIME = 0.45
LIFT_TIME = 1.15
LIFT_SETTLE_TIME = 0.35              # cube stops swinging before the hold starts
PLACE_REST_DWELL_TIME = 0.55         # cube fully resting before the pump goes OFF
RELEASE_DWELL_TIME = 0.45            # vacuum seal breaks after the pump goes OFF
RETRACT_TIME = 0.80
TRANSIT_TIME = 0.45
APPROACH_TIME = 0.85
PARK_TIME = 0.45

# ---------------------------------------------------------------------------
# Target selection / base approach
# ---------------------------------------------------------------------------
#: A cube that is out of reach is approached by repeating
#: "drive one step -> re-scan -> re-check reach".  4 steps cover ~0.4 m of
#: approach even though every single step is clamped for safety.
MAX_APPROACH_ATTEMPTS = 4
#: Per-step drive clamp (metres).  This used to be 0.25 m: one open-loop step
#: was enough to drive the chassis into the first cube row (live field failure
#: of 2026-11-22, the operator asked for a shorter base travel).  Short steps
#: cost one extra scan each, a knocked-over row costs the match.
MAX_DRIVE_STEP_M = 0.10
FAR_DRIVE_STEP_M = 0.08
#: Distance kept between the chassis and a detected cube that is NOT the
#: target while driving forward (mm, from the arm base).  The same protection
#: also stops the drive completely when such a cube is already closer.
APPROACH_CLEARANCE_MM = match_logic.APPROACH_CLEARANCE_MM

#: A pressed (or stuck) chassis IR bumper only shortens the step to this many
#: metres; the geometric corridor guard above is the real obstacle protection.
#: The sensors on this robot are noisy (centre permanently True, right reads
#: True on an empty floor), so a hard veto would block every approach.
BUMPER_CRAWL_STEP_M = 0.04

#: Operator calibration: aim every grasp this much along the robot's forward
#: axis (negative = closer to the robot, see match_logic.GRASP_AIM_SHIFT_MM).
#: The operator compared both variants on the field and confirmed the raw
#: projection (0.0), so the temporary -40 mm offset is reverted.  Tunable with
#: ``--aim-shift``.
AIM_SHIFT_MM = match_logic.GRASP_AIM_SHIFT_MM

STATE_FILE = os.path.expanduser("~/logs/cake_smooth_state.json")

#: Confidence floor for ordinary (non-cake) detections used by the field test.
#: The detector node already drops non-cake boxes below 0.25; raising this to
#: ~0.35 (match_logic.CONF_THRESHOLD) filters the remaining one-frame glare.
DEFAULT_DETECT_CONF = match_logic.CONF_THRESHOLD
#: Anti-glare gate for the raw Cake class (id 2).  Same default as
#: match_logic.CAKE_CONF_THRESHOLD; override with ``--cake-conf``.
DEFAULT_CAKE_CONF = match_logic.CAKE_CONF_THRESHOLD
#: Anti-glare gate for the ``cake_site`` alias (id 20) - this is the class the
#: detector reports for the real Cake cube on the field.  Measured: real cake
#: 0.44..0.873, false cake_site <= 0.527; default 0.60.  Override with
#: ``--cake-site-conf``.
DEFAULT_CAKE_SITE_CONF = match_logic.CAKE_SITE_CONF_THRESHOLD

#: Phases in which the suction pump may already be holding the cube.
_HOLDING_PHASES = frozenset({"pump_on", "descent", "lift", "hold",
                             "return_descent", "release"})
#: Phases in which the cube is already on the ground and the arm is at/was
#: at the contact height (a vertical retract is enough).
_RETRACT_PHASES = frozenset({"released", "retract"})


def log(message):
    print(message)
    sys.stdout.flush()


class GraspError(RuntimeError):
    """Expected, user-facing failure (no driver, no target, blocked approach)."""

    def __init__(self, message, result="FAILED", code=1):
        super(GraspError, self).__init__(message)
        self.result = result
        self.code = code


@dataclass(frozen=True)
class GraspPlan:
    """Everything the motion sequence needs, in uArm wrist coordinates (mm)."""

    mode: str
    x: float
    y: float
    z_approach: float
    z_touch: float
    z_lift: float
    class_id: int = 0
    pixel: Tuple[float, float] = (0.0, 0.0)


# ---------------------------------------------------------------------------
# Low-level motion helpers
# ---------------------------------------------------------------------------

def _sleep(seconds, raw=False):
    """Wait ``seconds``: ROS time normally, wall time in the abort path.

    ``raw=True`` uses plain ``time.sleep`` so the abort path keeps working if
    rospy is already shutting down (``rospy.sleep`` raises immediately then).
    """
    if seconds is None or seconds <= 0:
        return
    if raw:
        time.sleep(seconds)
        return
    rospy.sleep(seconds)
    if rospy.is_shutdown():
        # Nobody should shut rospy down behind our back, but if it happens the
        # publishers are already dead - fail now so the caller can react.
        raise KeyboardInterrupt("ROS shutdown requested")


def _move(arm, x, y, z, settle, raw=False):
    """Command the wrist to (x, y, z) mm and wait ``settle`` seconds."""
    arm.interface.set_pose(float(x), float(y), float(z), delay=0.0)
    _sleep(settle, raw=raw)


# ---------------------------------------------------------------------------
# Perception & coordinates (field-test scaffolding; the main program has its own)
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
        # Hardware note: ir_bumper_front is permanently stuck True on this
        # chassis, so only left/right front IR sensors are checked.
        return bool(msg.ir_bumper_front_left or msg.ir_bumper_front_right)
    return bool(getattr(msg, "ir_bumper_back_left", False)
                or getattr(msg, "ir_bumper_back_right", False))


def compute_target_coords(cam, u, v, is_cake, scale, cake_offset=CAKE_CUP_OFFSET_X):
    """Compute arm workspace (x, y) deterministically without unreliable depth.

    RealSense depth is non-linearly biased under 0.5 m (SI-33), so near-field
    targeting uses the calibrated projective/affine models directly:
      * tier-2 Cake -> dedicated affine ``pixel_to_cake_xy`` (+125 mm offset);
      * floor cube  -> closed-form DLT inversion at z=-100 mm (+20 mm offset).
    """
    xy = None
    z_est = None
    if is_cake:
        xy = match_logic.pixel_to_cake_xy(cam.transform3d, u, v,
                                          cup_offset_x=cake_offset)
    else:
        dlt_xy = match_logic.pixel_to_xy_at_z(cam.transform3d, u, v, -100.0)
        if dlt_xy is not None:
            xy = (float(dlt_xy[0]) + 20.0, float(dlt_xy[1]))
    # Operator aim calibration: the cup lands ~4 cm further than the pure
    # projection predicts, so the aim point is moved back towards the robot.
    # Applied AFTER the Cake clamp: otherwise the 348 mm clamp would swallow
    # the correction and the arm would still extend to the maximum.
    xy = match_logic.apply_grasp_aim_shift(xy, shift_mm=AIM_SHIFT_MM)
    return xy, z_est


def _make_scan_target(cid, u, v, xy, z_est, is_cake, far):
    """Build the scan-target dict that ``_prepare_target`` consumes."""
    return {
        "mode": "cake" if is_cake else "floor",
        "is_cake": bool(is_cake),
        "class": int(cid),
        "pixel": [round(u, 1), round(v, 1)],
        "xy": [round(xy[0], 1), round(xy[1], 1)],
        "z_est": round(z_est, 1) if z_est is not None else None,
        "far": bool(far),
    }


def _scan_target_coords(cam, u, v, is_cake, scale, cake_offset, far):
    """``(x, y)`` and the height estimate of one detection, near or far field."""
    if far:
        # Outside the calibrated band the depth reconstruction extrapolates
        # badly; the grasp-plane inversion is the only usable estimate.  The
        # next scan, taken after the approach step, refines it.
        xy = match_logic.far_target_xy(cam.transform3d, u, v, is_cake,
                                       cup_offset_x=cake_offset)
        if xy is None:
            log("[SCAN] far-field reconstruction failed at pixel (%.0f, %.0f)"
                % (u, v))
        return xy, None
    return compute_target_coords(cam, u, v, is_cake, scale, cake_offset)


def _scan_candidates(cam, cubes, far, scale, cake_offset):
    """Turn raw ``(class_id, u, v)`` detections into target dicts."""
    candidates = []
    for cid, u, v in cubes:
        is_cake = match_logic.is_cake(cid)
        xy, z_est = _scan_target_coords(cam, u, v, is_cake, scale,
                                        cake_offset, far)
        if xy is None:
            continue
        if not far:
            if is_cake and z_est is not None and not (-130.0 <= z_est <= 130.0):
                log("[FILTER] Cake candidate %d at (%.1f, %.1f) has "
                    "out-of-bounds z_est=%.1f -> skipped" % (cid, u, v, z_est))
                continue
            if not is_cake and z_est is not None and not (-250.0 <= z_est <= 110.0):
                log("[FILTER] Floor candidate %d at (%.1f, %.1f) has "
                    "out-of-bounds z_est=%.1f -> skipped" % (cid, u, v, z_est))
                continue
        candidates.append(_make_scan_target(cid, u, v, xy, z_est, is_cake, far))
    return candidates


def scan_scene(cam, arm, scale, fast=False, tag="", cake_offset=CAKE_CUP_OFFSET_X,
               conf_threshold=DEFAULT_DETECT_CONF,
               cake_conf_threshold=DEFAULT_CAKE_CONF,
               cake_site_conf_threshold=DEFAULT_CAKE_SITE_CONF):
    """Detect cubes, separating Cake vs Floor and near-field vs far-field ones.

    If fast=False: park the arm out of the camera view and take the samples.
    If fast=True: the arm is already parked; take stable samples only.

    Two anti-glare gates protect the priority Cake class:
    ``cake_conf_threshold`` for raw id 2 and ``cake_site_conf_threshold`` for
    raw id 20 (the printed cake cube, which is what the detector reports for
    the real Cake on the field).  Boxes below the gate are ignored completely.

    A detection ABOVE the calibrated near field (``v < 130 px``) is a cube that
    is still too far away for the arm.  It used to be discarded as "far
    wall/ceiling noise", which turned every out-of-reach cube into a NO_TARGET
    failure (exit code 3) with no base motion.  It is now returned as a target
    with ``far=True``: ``grasp_reach_state`` reports it out of reach and the
    caller drives the base one step, re-scans from the calibrated band and
    only then plans the grasp.

    Which candidate wins is decided by ``match_logic.choose_scan_target``:
    Cake priority first (operator rule), then the target that is already
    graspable or cheapest to reach.  A low-confidence detection at the frame
    corner therefore no longer outranks the cube standing right ahead.
    """
    if not fast:
        arm.interface.set_pose(PARK_POSE[0], PARK_POSE[1], PARK_POSE[2], delay=0.35)
        rospy.sleep(0.15)

    num_samples = 5
    raw_detections = cam.detector(num_samples=num_samples, timeout=4.0,
                                  conf_threshold=conf_threshold,
                                  cake_conf_threshold=cake_conf_threshold,
                                  cake_site_conf_threshold=cake_site_conf_threshold) or []
    log("[SCAN%s] Detections (samples=%d, conf>=%.2f, cake_conf>=%.2f, "
        "cake_site_conf>=%.2f): %s"
        % (tag, num_samples, conf_threshold, cake_conf_threshold,
           cake_site_conf_threshold, raw_detections))

    near_cubes = []
    far_cubes = []
    dismissed = []
    for item in raw_detections:
        try:
            cid = int(item[0])
            u, v = float(item[1][0]), float(item[1][1])
        except Exception:
            continue
        if match_logic.in_near_field(u, v):
            near_cubes.append((cid, u, v))
        elif match_logic.in_scan_window(u, v):
            far_cubes.append((cid, u, v))
        else:
            dismissed.append((cid, round(u), round(v)))

    if dismissed:
        log("[SCAN%s] %d detection(s) outside the scan window ignored: %s"
            % (tag, len(dismissed), dismissed))
    if far_cubes:
        log("[SCAN%s] %d target(s) beyond the calibrated near field "
            "(out of arm reach): %s -> base approach required"
            % (tag, len(far_cubes), far_cubes))

    candidates = (_scan_candidates(cam, near_cubes, False, scale, cake_offset)
                  + _scan_candidates(cam, far_cubes, True, scale, cake_offset))
    target = match_logic.choose_scan_target(candidates)
    if target is not None:
        # First-row protection: the base drive must not run into any OTHER
        # detected cube that lies in its corridor (the chassis is wider than
        # the arm).  Stored on the target so drive_approach_step can limit or
        # refuse the forward step.
        other_xys = [cand["xy"] for cand in candidates
                     if cand is not target and cand.get("xy")]
        obstacle_mm = match_logic.nearest_corridor_obstacle_mm(
            target["xy"], other_xys)
        target["obstacle_mm"] = obstacle_mm
        log("[SCAN%s] Chosen target: mode=%s class=%d pixel=%s far=%s "
            "obstacle_ahead=%s"
            % (tag, target["mode"], target["class"], target["pixel"],
               target["far"],
               "none" if obstacle_mm is None else "%.0f mm" % obstacle_mm))
        return target

    if not near_cubes and not far_cubes:
        log("[SCAN%s] No valid cubes detected in the camera window" % tag)
    return None


# ---------------------------------------------------------------------------
# Single-command base approach with vibration damping
# ---------------------------------------------------------------------------

def drive_approach_step(arm, target):
    """Align the heading and approach the target in one clean command sequence."""
    mode = target["mode"]
    x, y = target["xy"]
    far = bool(target.get("far"))

    # 1. Heading alignment (in-place turn)
    angle_rad = math.atan2(y, x)
    if abs(angle_rad) > math.radians(2.5):
        log("[APPROACH] Aligning heading: %.1f deg" % math.degrees(angle_rad))
        arm.turn_base(angle_rad)
        rospy.sleep(0.35)

    # 2. Linear displacement to the target sweet spot
    dist, desired = target_distance(mode, x, y)
    drive_m = (dist - desired) / 1000.0

    if abs(drive_m) >= 0.02:
        forward = drive_m > 0
        blocked = bumper("front" if forward else "back")

        if forward and blocked:
            # The chassis IR bumper is UNRELIABLE on this robot: the centre
            # sensor is permanently stuck True and the right one reads True on
            # an empty floor (measured 2026-11-22, three consecutive samples).
            # It must not veto the drive - the base is allowed to approach
            # whenever the arm cannot reach - it only shortens the step so the
            # geometric corridor guard can react on the next re-scan.
            log("[APPROACH] Front bumper reports an obstacle (sensor is noisy) "
                "- step limited to %.0f cm" % (BUMPER_CRAWL_STEP_M * 100.0))
            drive_m = min(drive_m, BUMPER_CRAWL_STEP_M)

        if forward:
            obstacle_mm = target.get("obstacle_mm")
            if obstacle_mm is not None:
                allowed_mm = float(obstacle_mm) - APPROACH_CLEARANCE_MM
                if allowed_mm <= 0.0:
                    log("[APPROACH] A cube that is NOT the target stands "
                        "%.0f mm ahead (safety distance %.0f mm) - refusing "
                        "to drive into the first row"
                        % (float(obstacle_mm), APPROACH_CLEARANCE_MM))
                    return False
                if allowed_mm / 1000.0 < drive_m:
                    drive_m = allowed_mm / 1000.0
                    log("[APPROACH] Cube %.0f mm ahead: forward drive limited "
                        "to %.1f cm to keep the first row intact"
                        % (float(obstacle_mm), drive_m * 100.0))

        # A far-field target is only an estimate: take a shorter step so the
        # re-scan can correct the remaining distance instead of overshooting.
        step_limit = FAR_DRIVE_STEP_M if far else MAX_DRIVE_STEP_M
        step_m = max(-step_limit, min(step_limit, drive_m))
        log("[APPROACH] Single drive: %+.1f cm (target dist: %.0f mm, desired: "
            "%.0f mm%s)" % (step_m * 100.0, dist, desired,
                            ", far-field estimate" if far else ""))
        arm.drive_base(step_m)

    # 3. Vibration damping dwell: let the suspension and the camera mast settle
    log("[APPROACH] Base motion complete. Settling the chassis for 0.85 s ...")
    rospy.sleep(0.85)
    return True


# ---------------------------------------------------------------------------
# The reusable grasp sequence
# ---------------------------------------------------------------------------

def plan_grasp(target, cake_touch_z=DEFAULT_CAKE_TOUCH_Z):
    """Turn a scan target into the wrist heights of one grasp.

    Raises :class:`GraspError` (``result="UNREACHABLE"``) when the touch or
    lift pose is outside the firmware IK workspace: the arm would silently
    refuse to move and the sequence would hold/return nothing.
    """
    mode = str(target.get("mode", "floor"))
    x = float(target["xy"][0])
    y = float(target["xy"][1])
    z_est = target.get("z_est")
    z_approach, z_touch, z_lift = _grasp_heights(mode, z_est, cake_touch_z)

    for pose_name, z in (("touch", z_touch), ("lift", z_lift)):
        if not match_logic.ik_reachable(x, y, z):
            raise GraspError(
                "grasp plan is outside the arm IK workspace (%s pose: x=%.1f, "
                "y=%.1f, z=%.1f mm)" % (pose_name, x, y, z),
                result="UNREACHABLE", code=7)

    pixel = target.get("pixel") or [0.0, 0.0]
    return GraspPlan(
        mode=mode,
        x=x,
        y=y,
        z_approach=z_approach,
        z_touch=z_touch,
        z_lift=z_lift,
        class_id=int(target.get("class", 0)),
        pixel=(float(pixel[0]), float(pixel[1])),
    )


def execute_sequence(arm, plan, hold_seconds=HOLD_SECONDS, return_cube=True,
                     park_pose=PARK_POSE,
                     transit_xy=(TRANSIT_X, TRANSIT_Y_FACTOR)):
    """Run the operator-requested grasp sequence (see the module docstring).

    ``return_cube=False`` stops after the hold: the cube is attached, the
    pump stays ON and the wrist stays at ``plan.z_lift``.  This is the mode
    the main program uses before carrying the cube to its destination zone.

    ``park_pose`` and ``transit_xy`` are the field-test landmarks (initial arm
    pose and safe lateral transit point); the main program may pass its own
    carry/park pose here.

    The function is exception-safe: it never releases the cube in the air.
    """
    phase = "transit"
    try:
        # -------- Step 1: lower the manipulator -------------------------
        log("[1/6] Lowering the manipulator above the cube (z=%.1f mm)"
            % plan.z_approach)
        _move(arm, transit_xy[0], plan.y * transit_xy[1], plan.z_approach,
              TRANSIT_TIME)
        _move(arm, plan.x, plan.y, plan.z_approach, APPROACH_TIME)
        phase = "above"

        # -------- Step 2: suction --------------------------------------
        phase = "pump_on"
        log("[2/6] Vacuum ON; descending strictly vertically to the cube "
            "(z=%.1f mm)" % plan.z_touch)
        arm.interface.set_pump(True)
        _sleep(PUMP_SETTLE_TIME)
        phase = "descent"
        _move(arm, plan.x, plan.y, plan.z_touch, DESCENT_TIME)
        _sleep(SEAL_DWELL_TIME)

        # -------- Step 3: strictly vertical lift ------------------------
        phase = "lift"
        log("[3/6] Raising the cube strictly vertically (z=%.1f mm)"
            % plan.z_lift)
        _move(arm, plan.x, plan.y, plan.z_lift, LIFT_TIME)
        _sleep(LIFT_SETTLE_TIME)

        # -------- Step 4: hold in the raised position -------------------
        phase = "hold"
        log("[4/6] Holding the cube in the raised position for %.1f s"
            % hold_seconds)
        _sleep(max(0.0, float(hold_seconds)))
        log("      hold finished")

        if not return_cube:
            log("[INTEGRATION] Cube stays attached (pump ON, wrist z=%.1f); "
                "the caller now carries it to the destination zone." % plan.z_lift)
            return

        # ==================================================================
        # STEPS 5-6: PUT THE CUBE BACK AND PARK THE ARM.
        # For the final integration this block is deleted (or skipped with
        # return_cube=False): the main program carries the cube to the zone.
        # ШАГИ 5-6: ВЕРНУТЬ КУБ НА МЕСТО И УБРАТЬ МАНИПУЛЯТОР - при интеграции
        # этот блок просто вырезается.
        # ==================================================================
        phase = "return_descent"
        log("[5/6] Lowering the cube strictly vertically back to the pickup point")
        _move(arm, plan.x, plan.y, plan.z_touch, DESCENT_TIME)
        _sleep(PLACE_REST_DWELL_TIME)
        phase = "release"
        log("      cube is resting on the ground: vacuum OFF")
        arm.interface.set_pump(False)
        phase = "released"
        _sleep(RELEASE_DWELL_TIME)

        phase = "retract"
        _move(arm, plan.x, plan.y, plan.z_approach, RETRACT_TIME)
        phase = "transit_back"
        _move(arm, transit_xy[0], 0.0, plan.z_approach, PARK_TIME)

        # -------- Step 6: park in the initial position ------------------
        phase = "park"
        log("[6/6] Returning the manipulator to its initial position %s"
            % (tuple(park_pose),))
        _move(arm, park_pose[0], park_pose[1], park_pose[2], PARK_TIME)
        phase = "parked"
        log("[DONE] Cube returned to its original position; arm parked.")
    except BaseException as exc:
        log("[ABORT] Sequence interrupted in phase '%s': %s: %s"
            % (phase, type(exc).__name__, exc))
        _recover(arm, plan, phase, park_pose=park_pose)
        raise


def _emergency_put_back(arm, plan):
    """Best-effort abort path: lower the cube to the pickup point and release.

    Returns ``(released, retracted)``: ``released`` means the vacuum was
    switched off while the cup was at the contact height (cube is on the
    ground), ``retracted`` means the arm was then raised to the clearance
    height.  Raw ``time.sleep`` waits are used so the recovery still works
    while rospy is shutting down after Ctrl-C.
    """
    released = False
    retracted = False
    try:
        log("[SAFE] Cube may be attached - lowering it back to the pickup point")
        _move(arm, plan.x, plan.y, plan.z_touch, DESCENT_TIME, raw=True)
        _sleep(PLACE_REST_DWELL_TIME, raw=True)
        arm.interface.set_pump(False)
        released = True
        log("[SAFE] Vacuum OFF: the cube is resting on the ground")
        _sleep(RELEASE_DWELL_TIME, raw=True)
        _move(arm, plan.x, plan.y, plan.z_approach, RETRACT_TIME, raw=True)
        retracted = True
    except BaseException as exc:
        log("[SAFE] emergency return incomplete: %s: %s"
            % (type(exc).__name__, exc))
    return released, retracted


def _recover(arm, plan, phase, park_pose=PARK_POSE):
    """Abort handling: never release the cube in the air, then park the arm."""
    released = False
    retracted = False

    if phase in _HOLDING_PHASES:
        released, retracted = _emergency_put_back(arm, plan)
    elif phase in _RETRACT_PHASES:
        # The cube was already released on the ground; only retract vertically.
        released = True
        try:
            # Repeat the release BEFORE moving away: if the original pump-off
            # command had been lost, retracting would carry the cube into the
            # air and the repeated release would then drop it.
            arm.interface.set_pump(False)
        except BaseException:
            pass
        try:
            _move(arm, plan.x, plan.y, plan.z_approach, RETRACT_TIME, raw=True)
            retracted = True
        except BaseException as exc:
            log("[SAFE] vertical retract failed: %s" % exc)

    if phase in _HOLDING_PHASES and not retracted:
        if released:
            log("[SAFE] Cube is on the floor, but the arm could not retract; "
                "leaving the arm in place.")
        else:
            log("[SAFE] Pump LEFT ON and the arm LEFT IN PLACE: the cube may "
                "still be attached. Release it manually or retry the script.")
        return
    if phase in _RETRACT_PHASES and not retracted:
        log("[SAFE] Cube is on the floor, but the arm could not retract; "
            "leaving the arm in place.")
        return

    try:
        log("[SAFE] Parking the manipulator ...")
        _move(arm, park_pose[0], park_pose[1], park_pose[2], PARK_TIME, raw=True)
    except BaseException as exc:
        log("[SAFE] could not park the manipulator: %s" % exc)


# ---------------------------------------------------------------------------
# Field-test harness: scan the scene, approach the base, run the sequence
# ---------------------------------------------------------------------------

def _wait_for_arm_driver(arm, timeout=3.0):
    """Wait until the uArm driver subscribes to the position topic."""
    deadline = time.time() + timeout
    while (arm.interface.arm_position_pub.get_num_connections() == 0
           and time.time() < deadline):
        rospy.sleep(0.1)
    if arm.interface.arm_position_pub.get_num_connections() == 0:
        raise GraspError(
            "uArm SwiftPro driver is NOT connected "
            "(0 subscribers on position_write_topic)",
            result="ARM_NOT_CONNECTED", code=2)


def _prepare_target(cam, arm, scale, cake_offset, scan_only, entry,
                    conf_threshold, cake_conf_threshold,
                    cake_site_conf_threshold):
    """Scan the field and, if needed, drive the base until the target is in reach."""
    is_initial = True
    for attempt in range(MAX_APPROACH_ATTEMPTS):
        target = scan_scene(cam, arm, scale, fast=not is_initial,
                            tag="_init" if is_initial else "_approach_%d" % attempt,
                            cake_offset=cake_offset,
                            conf_threshold=conf_threshold,
                            cake_conf_threshold=cake_conf_threshold,
                            cake_site_conf_threshold=cake_site_conf_threshold)
        is_initial = False
        if target is None:
            if attempt == 0:
                raise GraspError(
                    "no valid target detected: the scan window is empty - put "
                    "the cube in front of the camera",
                    result="NO_TARGET", code=3)
            raise GraspError(
                "target lost after base-approach step %d (the re-scan is "
                "empty); the base may have turned away from the cube" % attempt,
                result="TARGET_LOST", code=6)

        mode = target["mode"]
        x, y = target["xy"]
        far = bool(target.get("far"))
        in_reach, metric = reach_state(mode, x, y, z_est=target.get("z_est"),
                                       far=far)
        if far:
            desc = ("5) Target beyond the near field (OUT OF ARM REACH) -> "
                    "base approach step, then re-scan")
        elif mode == "cake":
            desc = ("1) Cake IN REACH -> pick immediately without base motion"
                    if in_reach else
                    "2) Cake OUT OF REACH -> one base approach, then re-scan")
        else:
            desc = ("3) Standard cube IN REACH -> pick immediately without "
                    "base motion" if in_reach else
                    "4) Standard cube OUT OF REACH -> one base approach, "
                    "then re-scan")
        log("[DECISION] (Attempt %d/%d) %s"
            % (attempt + 1, MAX_APPROACH_ATTEMPTS, desc))
        log("[TARGET] mode=%s class=%d xy=(%.1f, %.1f) far=%s in_reach=%s "
            "dist=%.1f z_est=%s"
            % (mode, target["class"], x, y, far, in_reach, metric,
               target.get("z_est")))
        entry["target"] = target

        if in_reach or scan_only:
            return target

        if not drive_approach_step(arm, target):
            raise GraspError("base approach blocked or unsafe",
                             result="APPROACH_BLOCKED", code=6)
        log("[RETRY] Base stopped; re-scanning and re-checking the reach ...")

    last = entry.get("target") or {}
    raise GraspError(
        "target still out of reach after %d base-approach step(s) "
        "(last: mode=%s xy=%s far=%s); the cube is too far away - move it "
        "closer or check that the base can drive"
        % (MAX_APPROACH_ATTEMPTS, last.get("mode"), last.get("xy"),
           last.get("far")),
        result="OUT_OF_REACH", code=6)


def _write_result(entry):
    """Append one JSON line with the outcome (field diagnostics only)."""
    path = os.path.expanduser("~/logs/grasp_generic.jsonl")
    try:
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "a", encoding="utf8") as handle:
            handle.write(json.dumps(entry) + "\n")
    except Exception as exc:
        log("[WARN] cannot write the run log: %s" % exc)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Grasp a cube, raise it, hold it 5 s, put it back, park the arm.")
    parser.add_argument(
        "--scan-only", action="store_true",
        help="only detect/assess the target: the base never moves and the "
             "grasp sequence is not executed (the arm is parked for the camera)")
    parser.add_argument(
        "--no-place", action="store_true",
        help="stop after the 5 s hold and keep the cube on the suction cup "
             "(pump ON) - preview of the main-program integration")
    parser.add_argument(
        "--hold", type=float, default=HOLD_SECONDS,
        help="hold time in the raised position, seconds (default %.1f)"
             % HOLD_SECONDS)
    parser.add_argument(
        "--cake-offset", type=float, default=CAKE_CUP_OFFSET_X,
        help="suction-cup forward offset for the raised cake, mm (default %.1f)"
             % CAKE_CUP_OFFSET_X)
    parser.add_argument(
        "--aim-shift", type=float, default=AIM_SHIFT_MM,
        help="shift the grasp point along the robot's forward axis, mm "
             "(default %.1f = the raw pixel->arm projection; negative aims "
             "closer to the robot)" % AIM_SHIFT_MM)
    parser.add_argument(
        "--conf", type=float, default=DEFAULT_DETECT_CONF,
        help="confidence floor for ordinary cubes (default %.2f; the detector "
             "node itself already drops non-cake boxes below 0.25)"
             % DEFAULT_DETECT_CONF)
    parser.add_argument(
        "--cake-conf", type=float, default=DEFAULT_CAKE_CONF,
        help="anti-glare confidence gate for raw cake id 2 (default %.2f): "
             "false cake hypotheses on banana cubes reach 0.85, so keep it high"
             % DEFAULT_CAKE_CONF)
    parser.add_argument(
        "--cake-site-conf", type=float, default=DEFAULT_CAKE_SITE_CONF,
        help="anti-glare confidence gate for the cake_site id 20, i.e. the "
             "real Cake cube on the field (default %.2f; measured real cake "
             "0.44..0.87, false cake_site <= 0.53)" % DEFAULT_CAKE_SITE_CONF)
    return parser


def _install_signal_handlers():
    """Turn Ctrl-C/SIGTERM into a KeyboardInterrupt inside the sequence.

    rospy's default handler calls ``signal_shutdown()`` first - all topics are
    closed and later publishes are silently dropped - and then swallows the
    KeyboardInterrupt raised by the previous handler.  The script must instead
    catch the interruption while its publishers are still alive, so it uses
    ``init_node(disable_signals=True)`` and installs this handler.
    """

    def _raise_interrupt(signum, _frame):
        raise KeyboardInterrupt("signal %d" % signum)

    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)


def main():
    global AIM_SHIFT_MM
    args = build_parser().parse_args(rospy.myargv()[1:])
    AIM_SHIFT_MM = float(args.aim_shift)
    rospy.init_node("grasp_generic", anonymous=True, disable_signals=True)
    _install_signal_handlers()

    entry = {
        "ts": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "result": None,
        "target": None,
        "plan": None,
        "hold_seconds": float(args.hold),
    }
    result = "ERROR"
    exit_code = 1
    arm = None

    try:
        cam = CamAction()
        arm = ArmAction(cam=cam)
        _wait_for_arm_driver(arm)

        # NEVER blindly switch the pump off here: a previous interrupted run
        # may still hold a cube at lift height, and releasing it would drop
        # the cube.  The pump is controlled by steps 2 and 5 only.
        arm.interface.set_status(True)
        rospy.sleep(0.20)

        scale, cake_touch = load_memory()
        log("[CONFIG] depth scale=%s | cake touch z=%.1f mm | floor touch z=%.1f mm"
            % (scale, cake_touch, FLOOR_TOUCH_Z))

        target = _prepare_target(cam, arm, scale, args.cake_offset,
                                 args.scan_only, entry,
                                 conf_threshold=args.conf,
                                 cake_conf_threshold=args.cake_conf,
                                 cake_site_conf_threshold=args.cake_site_conf)
        if args.scan_only:
            log("[SCAN-ONLY] Assessment complete; no motion command was sent.")
            result, exit_code = "SCANNED", 0
        else:
            plan = plan_grasp(target, cake_touch)
            entry["plan"] = asdict(plan)
            log("=" * 70)
            log("GRASP -> LIFT -> HOLD %.1f s -> RETURN -> PARK" % args.hold)
            log("plan: mode=%s class=%d xy=(%.1f, %.1f) approach=%.1f "
                "touch=%.1f lift=%.1f"
                % (plan.mode, plan.class_id, plan.x, plan.y,
                   plan.z_approach, plan.z_touch, plan.z_lift))
            log("=" * 70)

            execute_sequence(arm, plan, hold_seconds=args.hold,
                             return_cube=not args.no_place)

            if args.no_place:
                log("[HOLD-ONLY] The cube stays attached to the suction cup "
                    "(pump ON); this mode is the main-program integration preview.")
                result = "HELD"
            else:
                result = "DONE"
            exit_code = 0
    except GraspError as exc:
        log("[FAIL] %s" % exc)
        result, exit_code = exc.result, exc.code
    except BaseException as exc:
        # execute_sequence() has already run its safe recovery (the cube was
        # lowered to the pickup point before the pump release).  Do NOT touch
        # the pump here - an attached cube would be dropped.
        log("[ABORT] %s: %s" % (type(exc).__name__, exc))
        result, exit_code = "ABORTED", 4
    finally:
        entry["result"] = result
        _write_result(entry)
        log("[COMPLETED] result=%s" % result)

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
