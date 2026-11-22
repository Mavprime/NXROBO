#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pure match logic for the Spark autonomous competition (ROS-free).

WHY THIS MODULE EXISTS
----------------------
Everything that decides *what the robot does* (which cube to pick, which zone
it belongs to, which tier to stack on, when to give up) lives here, with no
ROS, no hardware and no global state.  ``runtask.py`` only wires these
functions to topics and actuators, and ``codes/tests/`` imports THIS module.

That is the whole point: the unit tests must exercise the code that actually
runs on the robot.  Before this module existed, the tests re-implemented the
logic inside the test files, so they passed while production code was broken.

Deployment: this file must sit next to ``runtask.py`` in
``auto_match/nodes/`` (``upload.ps1`` copies it there) and next to
``arm_ik.py`` in ``~/codes/`` (the compatibility shim imports it).

Conventions used everywhere below
---------------------------------
* Item classes: 1 = Teddy bear, 2 = Cake/Wine, 3 = Clock, 4 = Banana.
  YOLO also emits "<item>_site" aliases (10/20/30/40); ``norm_id`` folds them.
* Detections are tuples ``(raw_id, score, cx, cy)`` in image pixels.
* Cube lists are the historical format ``[[canonical_id, [x, y, 0]], ...]``.
* Zones: Collection_A (red square, cake only) and Collection_B/C/D.
* Tiers: 1 = floor, 2 = on a cube, 3 = on two cubes (``MAX_TIER``).
"""

import math
import os
import time
from collections import Counter

# --------------------------------------------------------------------------
# Class identifiers and normalization
# --------------------------------------------------------------------------

TEDDY_ID = 1
CAKE_ID = 2
CLOCK_ID = 3
BANANA_ID = 4

CANONICAL_IDS = (TEDDY_ID, CAKE_ID, CLOCK_ID, BANANA_ID)
# Items that appear on the classification board (the cake never does).
BOARD_ITEMS = (TEDDY_ID, CLOCK_ID, BANANA_ID)

ID_MAP = {
    1: 1, 10: 1,
    2: 2, 20: 2,
    3: 3, 30: 3,
    4: 4, 40: 4,
}


def norm_id(raw_id):
    """Map a raw YOLO class (including ``*_site`` aliases) to a canonical id."""
    try:
        return ID_MAP.get(int(raw_id), int(raw_id))
    except Exception:
        return raw_id


def is_cake(item_id):
    """True when the item is the competition cake (rule-critical item)."""
    return norm_id(item_id) == CAKE_ID


# --------------------------------------------------------------------------
# Zones, tiers and scoring
# --------------------------------------------------------------------------

ZONES_BCD = ("Collection_B", "Collection_C", "Collection_D")
# Collection_A is the red 0.45 x 0.45 m acceptance square reserved for the cake.
RED_ZONES = ("Collection_A",)
MAX_TIER = 3
TOTAL_CUBES = 10
# Drop routine per tier index (0 -> floor, 1 -> 2nd tier, 2 -> 3rd tier).
DROP_ROUTINES = ("drop", "drop1", "drop2")

# Used when the classification board cannot be read (SI-14 fallback).
DEFAULT_MAPPING = {
    CAKE_ID: "Collection_A",
    TEDDY_ID: "Collection_B",
    CLOCK_ID: "Collection_C",
    BANANA_ID: "Collection_D",
}


def score_cube(is_red_zone, tier):
    """Competition points for one cube placed at ``tier`` (1..3)."""
    if tier < 1 or tier > MAX_TIER:
        raise ValueError(f"Invalid tier {tier}, must be 1, 2 or 3")
    if is_red_zone:
        return tier + (2 ** (tier - 1))
    return 2 ** (tier - 1)


def match_total(deliveries, started_from_base=True):
    """Score a full delivery sequence; 0 when the cake-first rule is violated.

    ``deliveries``: iterable of dicts with keys cube_type, zone, is_red, tier.
    """
    deliveries = list(deliveries)
    if not deliveries:
        return 1 if started_from_base else 0
    first = deliveries[0]
    if first.get("cube_type") != "cake" or first.get("zone") != "Collection_A":
        return 0
    total = 1 if started_from_base else 0
    for delivery in deliveries:
        total += score_cube(delivery["is_red"], delivery["tier"])
    return total


def estimate_score(drop_times, red_zones=RED_ZONES, started_from_base=True):
    """Estimate the match score from per-zone drop counts only.

    Used for the end-of-run log line.  It assumes the cake was delivered first
    and that each zone was filled bottom-up, which is exactly how the FSM
    drops cubes, so it is an upper bound when the cake rule was violated.
    """
    total = 1 if started_from_base else 0
    for zone, count in drop_times.items():
        for tier in range(1, min(int(count), MAX_TIER) + 1):
            total += score_cube(zone in red_zones, tier)
    return total


def plan_drop(zone_drop_count):
    """Name of the arm routine for the next cube in a zone, or None if full."""
    if zone_drop_count >= MAX_TIER:
        return None
    return DROP_ROUTINES[zone_drop_count]


def choose_drop_zone(item_id, mapping, drop_times, zones=ZONES_BCD):
    """Zone for ``item_id``: board mapping first, then the first non-full zone.

    Returns None only when every zone already holds ``MAX_TIER`` cubes, which
    must never happen with 10 cubes and 4 zones but is handled instead of
    dropping a cube into thin air.
    """
    cid = norm_id(item_id)
    primary = mapping.get(cid) or mapping.get(item_id)
    candidates = []
    if primary:
        candidates.append(primary)
    candidates.extend(zone for zone in zones if zone not in candidates)
    for zone in candidates:
        if drop_times.get(zone, 0) < MAX_TIER:
            return zone
    return None


# --------------------------------------------------------------------------
# Classification board -> zone mapping
# --------------------------------------------------------------------------

def expand_aliases(mapping):
    """Return a copy of ``mapping`` that also answers alias ids (10/20/30/40)."""
    out = dict(mapping)
    for alias, cid in ID_MAP.items():
        if cid in out and alias not in out:
            out[alias] = out[cid]
    return out


def normalize_mapping(mapping=None):
    """Guarantee that every canonical item has a zone and the cake is in A."""
    out = dict(DEFAULT_MAPPING if mapping is None else mapping)
    for cid, zone in DEFAULT_MAPPING.items():
        out.setdefault(cid, zone)
    out[CAKE_ID] = "Collection_A"
    return expand_aliases(out)


def board_items(slot_items):
    """Unique canonical board items in left-to-right order, cake excluded."""
    items = []
    for raw in slot_items:
        if raw is None:
            continue
        cid = norm_id(raw)
        if cid in BOARD_ITEMS and cid not in items:
            items.append(cid)
    return items


def board_is_confident(slot_items):
    """True when the board yielded three distinct, usable items."""
    return len(board_items(slot_items)) >= len(ZONES_BCD)


def decode_classification_board(slot_items, fallback=None):
    """Build ``{item_id: zone}`` from the board slots (left to right).

    On a partial/duplicated/unreadable board the fallback mapping is returned
    unchanged, so the mission can keep scoring instead of aborting (SI-14).
    """
    mapping = normalize_mapping(fallback)
    items = board_items(slot_items)
    if len(items) < len(ZONES_BCD):
        return mapping
    mapping[CAKE_ID] = "Collection_A"
    for cid, zone in zip(items[:len(ZONES_BCD)], ZONES_BCD):
        mapping[cid] = zone
    return expand_aliases(mapping)


# --------------------------------------------------------------------------
# uArm Swift Pro kinematics (the only implementation in the project)
# --------------------------------------------------------------------------
# The maths mirrors swiftpro_rviz_node.cpp / the vendor examples.  If the
# firmware's IK cannot reach a pose it silently refuses the command and the arm
# simply does not move, so every target is checked before it is published.
# Coordinate convention (from every Spark grasp example):
#     arm_x = k1 * pixel_row + b1
#     arm_y = k2 * pixel_col + b2
# i.e. the image row drives x and the image column drives y.  Do not "fix" this
# by swapping: it is how the calibration files were generated.

MATH_TRANS = 57.2958
MATH_L1 = 106.6
MATH_L2 = 13.2
MATH_LOWER_ARM = 142.07
MATH_UPPER_ARM = 158.81
MATH_UPPER_LOWER = MATH_UPPER_ARM / MATH_LOWER_ARM
MATH_Z_OFFSET = 74.55
MATH_X_OFFSET = 56.55


# Camera intrinsics constants for D435 RealSense
CAM_FX = 605.6510620117188
CAM_FY = 605.8469848632812
CAM_CX = 315.03875732421875
CAM_CY = 249.8477020263672


def matrix_columns(M):
    """Columns of a 4x3 matrix: works for numpy arrays and plain lists.

    ``pixel_to_xy_at_z`` is called both with the np.load()-ed calibration and
    with hand-written lists from the unit tests, so no slicing syntax that only
    numpy understands.
    """
    return ([M[i][0] for i in range(4)],
            [M[i][1] for i in range(4)],
            [M[i][2] for i in range(4)])


def pixel_to_xy_at_z(M, u, v, z,
                     fx=CAM_FX, fy=CAM_FY, cx=CAM_CX, cy=CAM_CY):
    """Closed-form inverse of the 3D projective map at a fixed height z (mm).

    Given the 4x3 DLT camera-to-arm matrix M, pixel (u, v) and plane height z,
    returns the exact (x, y) coordinates in millimetres in the arm frame.
    """
    up = (u - cx) / fx
    vp = (v - cy) / fy
    r0, r1, r2 = matrix_columns(M)
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
    return float((b1 * a22 - a12 * b2) / det), float((a11 * b2 - b1 * a21) / det)


def load_cake_calibration(path=None):
    """Load the dedicated Layer 2 (Cake) calibration parameters (k1, b1, k2, b2)."""
    if path is None:
        candidates = [
            os.path.expanduser("~/thefile2.txt"),
            os.path.join(os.path.dirname(__file__), "thefile2.txt"),
        ]
        for c in candidates:
            if os.path.exists(c):
                path = c
                break
    if path and os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                vals = [float(v) for v in f.read().split()]
                if len(vals) >= 4:
                    return (vals[0], vals[1]), (vals[2], vals[3])
        except Exception:
            pass
    # Built-in fallback parameters from shipped thefile2.txt:
    # -0.8030661953564442 395.53156305089396 -0.6955960479077558 247.96237774624151
    return (-0.8030661953564442, 395.53156305089396), (-0.6955960479077558, 247.96237774624151)


#: Verified suction-cup forward offset for the elevated tier-2 Cake (mm).
#: Single source of truth shared by match_logic, runtask.grasp1 and grasp_generic.
CAKE_CUP_OFFSET_X = 125.0


def pixel_to_cake_xy(M, u, v, fx=CAM_FX, fy=CAM_FY, cx=CAM_CX, cy=CAM_CY,
                     calib_file=None, cup_offset_x=CAKE_CUP_OFFSET_X, clamp=True):
    """Dedicated approximation for the competition Cake (Tier 2 / 20 cm height).

    Uses the dedicated Layer 2 calibration (thefile2.txt) created specifically
    for the elevated Cake.
    cup_offset_x = +125.0 mm places the suction cup dead-center on the Cake cube
    (340-348 mm from arm base, compensating for detector finding vertical face).

    ``clamp=True`` (the default) keeps the result inside the arm's physical
    workspace (120..348 mm) and is what the final grasp plan needs: a Cake seen
    on its top face looks "too far" to the affine mapping, so the clamp stops a
    perfectly reachable cake from being declared out of reach.

    ``clamp=False`` returns the raw affine estimate.  That is required by the
    base-approach step: a Cake that is really standing further away must keep
    its large ``x`` so the robot drives to it instead of trying to pick air.
    """
    x_kb, y_kb = load_cake_calibration(calib_file)
    cake_x = float(x_kb[0] * v + x_kb[1] + cup_offset_x)
    cake_y = float(y_kb[0] * u + y_kb[1])
    if not clamp:
        return cake_x, cake_y
    # Clamped at hardware limit (348.0 mm) so IK never enters singularity
    cake_x = max(120.0, min(348.0, cake_x))
    cake_y = max(-220.0, min(220.0, cake_y))
    return cake_x, cake_y


def pixel_to_arm(pixel_row, pixel_col, x_kb, y_kb):
    """Return (x, y) in millimetres from an image pixel.

    ``x_kb = (k1, b1)`` and ``y_kb = (k2, b2)`` as stored in ``~/thefile.txt``.
    """
    return x_kb[0] * pixel_row + x_kb[1], y_kb[0] * pixel_col + y_kb[1]


def ik_reachable(x, y, z):
    """True if the arm can reach (x, y, z) in millimetres."""
    try:
        z = z + MATH_Z_OFFSET
        z_in = (z - MATH_L1) / MATH_LOWER_ARM
        if x < 0.1:
            x = 0.1
        if y == 0:
            rot = 90.0
        elif y < 0:
            rot = -math.atan(x / y) * MATH_TRANS
        else:
            rot = 180.0 - math.atan(x / y) * MATH_TRANS

        sin_rot = math.sin(rot / MATH_TRANS)
        if sin_rot == 0:
            return False

        x_in = (x / sin_rot - MATH_L2 - MATH_X_OFFSET) / MATH_LOWER_ARM
        if x_in == 0:
            return False

        sqrt_zx = math.sqrt(z_in * z_in + x_in * x_in)
        if sqrt_zx == 0:
            return False

        upper = MATH_UPPER_LOWER
        right_all = (sqrt_zx * sqrt_zx + upper * upper - 1) / (2 * upper * sqrt_zx)
        left_all = (sqrt_zx * sqrt_zx + 1 - upper * upper) / (2 * sqrt_zx)
        if abs(right_all) > 1 or abs(left_all) > 1:
            return False

        phi = math.atan2(z_in, x_in) * MATH_TRANS
        angle_right = math.acos(right_all) * MATH_TRANS - phi
        angle_left = math.acos(left_all) * MATH_TRANS + phi

        # Firmware/hardware mechanical limits (uArm Swift Pro):
        # theta1 (lower arm): [0.0, 135.6] deg
        # theta2 (upper arm / elbow): [0.0, 100.7] deg (cannot be negative)
        # combined span: [10.0, 151.0] deg
        return (0.0 <= angle_left <= 135.6 and
                0.0 <= angle_right <= 100.7 and
                10.0 <= (angle_left + angle_right) <= 151.0)
    except Exception:
        return False


def pixel_reachable(pixel_row, pixel_col, z_list, x_kb, y_kb):
    """True if the pixel is reachable at every height in ``z_list``."""
    arm_x, arm_y = pixel_to_arm(pixel_row, pixel_col, x_kb, y_kb)
    return all(ik_reachable(arm_x, arm_y, z) for z in z_list)


# --------------------------------------------------------------------------
# Grasp geometry and reach envelopes (field test + FSM wiring)
# --------------------------------------------------------------------------
# All heights are uArm wrist coordinates in millimetres.
PRESS_MM = 3.0                      # suction-cup compression on contact
FLOOR_TOUCH_Z = -55.0               # wrist height at contact with a floor cube
DEFAULT_CAKE_TOUCH_Z = 35.0         # contact height on the raised tier-2 cake
APPROACH_FLOOR_Z = 70.0             # safe clearance height above the floor
LIFT_FLOOR_Z = 60.0                 # wrist height of the raised cube
APPROACH_CAKE_Z = 75.0              # safe clearance height above the tier
LIFT_CAKE_Z = 80.0                  # wrist height of the raised cake

#: Base-approach sweet spot: the robot drives until the target is this far
#: away (mm).  Reachability itself is decided by the IK model with a comfort
#: margin (see :func:`grasp_reach_state`), not by this number.
#: ``dist_max`` documents the arm's physical limit (the IK test is stricter).
FLOOR_IN_REACH = dict(dist_target=240.0, dist_max=345.0)
# Tier-2 cake (reaching over the front row).  dist_target=305 keeps the
# chassis >6 cm away from first-row cubes so they are not knocked over.
CAKE_IN_REACH = dict(dist_target=305.0)

#: The arm must be IK-reachable not only exactly at the planned pose but also a
#: few millimetres around it: the firmware silently ignores a pose on the very
#: edge of the workspace (no error, no motion), and the run would then "grasp"
#: thin air.  12 mm ring around the contact, the lift and the approach pose.
REACH_MARGIN_MM = 12.0

#: A floor cube closer to the arm base than this (mm) cannot be pressed: the
#: arm folds onto the robot itself.  It also keeps an ungraspable near-field
#: projection (a frame-corner phantom maps to ~x=103) from counting as
#: reachable, which used to win the target choice and turn the robot away.
FLOOR_MIN_X_MM = 140.0

#: Operator calibration of the field test: shift every grasp point along the
#: robot's forward axis (mm, negative = closer to the robot).  0.0 = use the
#: pixel->arm projection unchanged; the operator confirmed on 2026-11-22 that
#: the raw projection aims correctly, so the temporary -40 mm test offset was
#: reverted.  Tunable with ``--aim-shift``.
GRASP_AIM_SHIFT_MM = 0.0

#: A detected cube that is NOT the target must never be closer than this to
#: the chassis while the base approaches (mm, radial distance from the arm
#: base).  The chassis front edge is ~120 mm ahead of the arm base (the uArm is
#: mounted 44 mm in front of the robot centre), so 180 mm leaves a ~60 mm gap.
#: The live field failure: driving to a target behind the first row ran the
#: chassis into that row and knocked the cubes over.
APPROACH_CLEARANCE_MM = 180.0

#: Half-width of the "in the way" corridor around the drive heading (mm).  The
#: chassis is wider than the arm, so a cube up to this far to the side is still
#: hit while driving forward.
APPROACH_CORRIDOR_MM = 160.0


def grasp_heights(mode, z_est=None, cake_touch_z=DEFAULT_CAKE_TOUCH_Z):
    """Return the (approach, touch, lift) wrist heights for one target."""
    elevated = (mode == "cake" and (z_est is None or z_est > -70.0))
    if elevated:
        return (APPROACH_CAKE_Z, float(cake_touch_z) - PRESS_MM, LIFT_CAKE_Z)
    return (APPROACH_FLOOR_Z, FLOOR_TOUCH_Z - PRESS_MM, LIFT_FLOOR_Z)


def apply_grasp_aim_shift(xy, shift_mm=GRASP_AIM_SHIFT_MM):
    """Shift a grasp point along the robot's forward axis (operator calibration).

    Positive ``shift_mm`` aims further from the robot, negative aims closer.
    ``None`` passes through unchanged.  See :data:`GRASP_AIM_SHIFT_MM`.
    """
    if xy is None:
        return None
    try:
        x, y = float(xy[0]), float(xy[1])
    except (TypeError, ValueError, IndexError):
        return None
    return x + float(shift_mm), y


def nearest_corridor_obstacle_mm(target_xy, obstacle_xys,
                                 corridor_mm=APPROACH_CORRIDOR_MM):
    """Distance to the closest detected cube lying in the base-drive corridor.

    ``target_xy`` is the chosen target and ``obstacle_xys`` the (x, y) points of
    the OTHER detected cubes, all in the arm frame (mm).  A cube counts as "in
    the way" when it is ahead of the robot (positive projection on the drive
    heading) and closer than the target, and its lateral offset is below
    ``corridor_mm`` - i.e. the chassis would drive into it.

    Returns the radial distance (mm) from the arm base to the closest such cube,
    or ``None`` when the corridor is clear.
    """
    try:
        tx, ty = float(target_xy[0]), float(target_xy[1])
    except (TypeError, ValueError, IndexError):
        return None
    target_dist = math.hypot(tx, ty)
    if target_dist < 1e-3:
        return None
    heading_x, heading_y = tx / target_dist, ty / target_dist

    nearest = None
    for point in obstacle_xys or ():
        try:
            ox, oy = float(point[0]), float(point[1])
        except (TypeError, ValueError, IndexError):
            continue
        along = ox * heading_x + oy * heading_y
        if along <= 0.0 or along >= target_dist:
            continue
        lateral = abs(-ox * heading_y + oy * heading_x)
        if lateral > float(corridor_mm):
            continue
        distance = math.hypot(ox, oy)
        if nearest is None or distance < nearest:
            nearest = distance
    return nearest


def _ik_reachable_with_margin(x, y, z, margin=REACH_MARGIN_MM):
    """True when ``(x, y, z)`` and a small ring around it are IK-reachable.

    A pose exactly on the workspace edge is accepted by the maths but silently
    refused by the firmware, and the grasp sequence would then hold and return
    thin air.  Requiring the four ring points to be reachable as well keeps a
    real margin between the plan and that edge.
    """
    if not ik_reachable(x, y, z):
        return False
    margin = float(margin)
    for dx, dy in ((margin, 0.0), (-margin, 0.0), (0.0, margin), (0.0, -margin)):
        if not ik_reachable(x + dx, y + dy, z):
            return False
    return True


def grasp_reach_state(mode, x, y, z_est=None, far=False):
    """Can the arm REALLY reach this (x, y) at the grasp and lift heights?

    The criterion is the firmware's own IK model, checked at the contact, the
    lift AND the approach height, each with a :data:`REACH_MARGIN_MM` comfort
    ring.  It replaces the old hard rectangle (``x_hi=290, |y|<=100,
    dist<=295``) for floor cubes, which was wrong in both directions:

    * live detections of the side cubes of the first row (``|y| = 130..147``)
      are perfectly reachable, but the box declared them "out of reach" and
      the base drove for nothing;
    * a cube at ``x = 340`` was inside the box while the arm cannot even touch
      the floor at that distance, so the grasp silently missed.

    Now a target behind the arm's IK envelope (e.g. the middle field row at
    ~390 mm) reports ``in_reach=False`` and the base approach runs - the base
    is allowed and expected to drive whenever the manipulator cannot reach.

    ``far=True`` marks a detection taken outside the calibrated near field (the
    cube is beyond the arm's workspace).  Its (x, y) is only a coarse approach
    estimate, so it is NEVER in reach: the base has to drive first and the next
    scan - taken from the calibrated band - refines the target.  This is what
    makes the robot approach a cube instead of failing with NO_TARGET.
    """
    dist = math.hypot(x, y)
    if far:
        return False, dist
    z_approach, z_touch, z_lift = grasp_heights(mode, z_est,
                                                DEFAULT_CAKE_TOUCH_Z)
    ik_ok = (_ik_reachable_with_margin(x, y, z_touch)
             and _ik_reachable_with_margin(x, y, z_lift)
             and _ik_reachable_with_margin(x, y, z_approach))
    if mode == "cake":
        # Elevated tier-2 item: the height sanity check stays; the dedicated
        # cake affine already pins the near-field plan into the workspace.
        if z_est is not None and not (-140.0 <= z_est <= 130.0):
            return False, dist
    elif x < FLOOR_MIN_X_MM:
        # Too close to press a floor cube (see FLOOR_MIN_X_MM).
        return False, dist
    return ik_ok, dist


def approach_target_distance(mode, x, y):
    """Return ``(current distance, desired distance)`` for a base approach."""
    if mode == "cake":
        return math.hypot(x, y), CAKE_IN_REACH["dist_target"]
    return math.hypot(x, y), FLOOR_IN_REACH["dist_target"]


def approach_cost(target):
    """How expensive it is to bring ``target`` into reach (lower is better).

    A target that is already graspable wins outright (negative cost).  For the
    rest, the cost mixes the heading error (30 deg ~ 1.0) with the distance
    error (300 mm ~ 1.0).  Without the heading term a low-confidence detection
    at the frame corner could make the robot swing ~60 deg away from the cube
    that is standing right ahead - and with the camera then pointing at the
    empty field the next scan found nothing at all.

    ``target`` is a scan dict as built by the field test:
    ``{mode, class, pixel, xy, z_est, far, ...}``.
    """
    mode = str(target.get("mode", "floor"))
    x = float(target["xy"][0])
    y = float(target["xy"][1])
    pixel = target.get("pixel") or [320.0, 0.0]
    centre = abs(float(pixel[0]) - 320.0) / 6400.0     # tie-break: keep it central
    in_reach, dist = grasp_reach_state(mode, x, y, z_est=target.get("z_est"),
                                       far=target.get("far", False))
    if in_reach:
        return -1.0 + centre
    heading = abs(math.atan2(y, x)) / math.radians(30.0)
    desired = approach_target_distance(mode, x, y)[1]
    return heading + abs(dist - desired) / 300.0 + centre


def choose_scan_target(candidates):
    """Pick one scan target: Cake priority first, then the cheapest approach.

    The Cake is the only rule-critical item, so a Cake candidate always beats a
    standard cube (operator rule).  Within the winning class a graspable target
    is taken immediately; otherwise the target with the lowest
    :func:`approach_cost` drives the base.
    """
    pool = [candidate for candidate in candidates if candidate]
    if not pool:
        return None
    cakes = [candidate for candidate in pool
             if candidate.get("is_cake") or is_cake(candidate.get("class", 0))]
    return min(cakes if cakes else pool, key=approach_cost)




# --------------------------------------------------------------------------
# Hand-eye calibration guards
# --------------------------------------------------------------------------
# A stale hand-eye matrix is the most expensive silent failure in this
# project: a cube pixel is converted to an arm target that is 5-7 cm off, the
# arm descends into empty space, and nothing reports an error.  These helpers
# catch the cheap cases at startup (missing/corrupted/degenerate matrix, wrong
# units) and give the operator one number to look at after the camera moves.

CALIB_IMAGE_SIZE = (640, 480)      # D435 color stream
CALIB_SCALE_RANGE = (0.3, 3.0)     # px per mm at the grasp workspace
CALIB_MAX_ROUNDTRIP_MM = 25.0      # project -> inverse must come back
CALIB_WORKSPACE_PROBE = (270.0, 0.0, -50.0)   # centre of the grasp workspace
CALIB_ROUNDTRIP_Z = (-100.0, -50.0, 0.0)


def project_arm(M, x, y, z, fx=CAM_FX, fy=CAM_FY, cx=CAM_CX, cy=CAM_CY):
    """Project an arm-frame wrist point (mm) into the color image.

    Returns ``(u, v)`` in pixels, or ``None`` when the point is behind the
    camera.  This is the forward direction of the same 4x3 matrix that
    ``pixel_to_xy_at_z`` inverts.
    """
    try:
        cam = [x * M[0][j] + y * M[1][j] + z * M[2][j] + M[3][j]
               for j in range(3)]
        if cam[2] <= 1e-9:
            return None
        return (fx * cam[0] / cam[2] + cx, fy * cam[1] / cam[2] + cy)
    except Exception:
        return None


def pad_projection_error(M, pose, observed_uv):
    """Pixel distance between the projected wrist pose and the seen pad.

    ``pose`` is the commanded (x, y, z) in mm; ``observed_uv`` is the orange
    pad centroid detected in the frame.  Returns ``None`` if the pose is not
    projectable.
    """
    projected = project_arm(M, pose[0], pose[1], pose[2])
    if projected is None:
        return None
    return math.hypot(projected[0] - observed_uv[0],
                      projected[1] - observed_uv[1])


def check_transform(M, fx=CAM_FX, fy=CAM_FY, cx=CAM_CX, cy=CAM_CY):
    """Numeric sanity check of a 4x3 camera<-arm matrix.

    Returns ``(ok, problems, stats)``.  ``problems`` is a list of human
    readable strings (empty when ok); ``stats`` carries the numbers worth
    logging: px/mm at the workspace and the project->inverse round-trip error.

    This cannot detect a camera that was physically moved after calibration -
    only the active pad check can (``calib3d.py --check`` / the startup check).
    It does catch the other ways a calibration file goes bad: truncated or
    non-numeric file, NaN/inf, a degenerate fit whose inverse returns garbage,
    and a workspace that projects outside the image.
    """
    problems = []
    stats = {}
    try:
        if len(M) != 4 or any(len(row) != 3 for row in M):
            return False, ["matrix is not 4x3"], {}
        rows = [[float(value) for value in row] for row in M]
    except Exception as exc:
        return False, ["matrix is not a 4x3 array of numbers (%s)" % exc], {}

    if not all(math.isfinite(value) for row in rows for value in row):
        return False, ["matrix contains NaN/inf"], {}

    # Volume of the linear part: a collapsed fit (all points on one plane)
    # passes every other check but inverts to garbage.
    a, b, c = rows[0][:3]
    d, e, f = rows[1][:3]
    g, h, i = rows[2][:3]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-15:
        problems.append("linear part is degenerate (det=%.3g)" % det)

    centre = project_arm(rows, *CALIB_WORKSPACE_PROBE, fx=fx, fy=fy, cx=cx, cy=cy)
    if centre is None:
        problems.append("workspace centre projects behind the camera")
    else:
        width, height = CALIB_IMAGE_SIZE
        stats["workspace_uv"] = (round(centre[0], 1), round(centre[1], 1))
        if not (-0.5 * width <= centre[0] <= 1.5 * width
                and -0.5 * height <= centre[1] <= 1.5 * height):
            problems.append(
                "workspace centre projects far outside the image at (%.0f, %.0f)"
                % centre)

    nearby = project_arm(rows, CALIB_WORKSPACE_PROBE[0] + 10.0,
                         CALIB_WORKSPACE_PROBE[1], CALIB_WORKSPACE_PROBE[2],
                         fx=fx, fy=fy, cx=cx, cy=cy)
    if centre is not None and nearby is not None:
        scale = math.hypot(nearby[0] - centre[0], nearby[1] - centre[1]) / 10.0
        stats["px_per_mm"] = round(scale, 3)
        low, high = CALIB_SCALE_RANGE
        if not low <= scale <= high:
            problems.append(
                "scale is %.3f px/mm, outside %.1f-%.1f (wrong units?)"
                % (scale, low, high))

    worst = 0.0
    for z in CALIB_ROUNDTRIP_Z:
        projected = project_arm(rows, CALIB_WORKSPACE_PROBE[0],
                                CALIB_WORKSPACE_PROBE[1], z,
                                fx=fx, fy=fy, cx=cx, cy=cy)
        if projected is None:
            problems.append("z=%.0f projects behind the camera" % z)
            continue
        back = pixel_to_xy_at_z(rows, projected[0], projected[1], z,
                                fx=fx, fy=fy, cx=cx, cy=cy)
        if back is None:
            problems.append("z=%.0f inverse is degenerate" % z)
            continue
        error = math.hypot(back[0] - CALIB_WORKSPACE_PROBE[0],
                           back[1] - CALIB_WORKSPACE_PROBE[1])
        worst = max(worst, error)
    stats["roundtrip_mm"] = round(worst, 2)
    if worst > CALIB_MAX_ROUNDTRIP_MM:
        problems.append(
            "project->inverse round-trip error is %.0f mm (limit %.0f)"
            % (worst, CALIB_MAX_ROUNDTRIP_MM))

    return (not problems), problems, stats


# --------------------------------------------------------------------------
# Vision: temporal clustering and cross-model de-duplication
# --------------------------------------------------------------------------

CONF_THRESHOLD = 0.35      # grasp-grade confidence (lowered from 0.45 to support Banana cubes)
CLUSTER_DIST_PX = 45.0     # boxes closer than this are the same cube
DEDUPE_DIST_PX = 80.0      # two models / *_site aliases reporting the same cube (SI-26)
MIN_HIT_RATIO = 0.5        # share of frames a cluster must appear in

#: Anti-glare gate for the raw Cake class (id 2 / plain "cake").
#: Measured on the real field: false cake hypotheses on banana cubes reached
#: score 0.852, so this class stays strict.  The real Cake item on this field
#: is a printed "cake" cube and the detector reports it as ``cake_site``
#: (id 20) instead - see CAKE_SITE_CONF_THRESHOLD.
CAKE_CONF_THRESHOLD = 0.90

#: Anti-glare gate for the ``cake_site`` alias (raw id 20), i.e. what the
#: detector actually reports for the Cake cube on this field.
#: Measured over 394 cake detections / 20 s in real lighting:
#:   * real cake site:  score 0.440 .. 0.873 (median ~0.75, majority > 0.60)
#:   * false cake_site: score 0.030 .. 0.527 (clock faces, banana cubes)
#: A later sample showed a banana cube's white front face reaching 0.761, i.e.
#: INSIDE the real-cake range - a confidence gate alone cannot reject it, see
#: CAKE_OWNER_PRINT_IDS below for the structural check that does.  Tunable at
#: runtime with ``--cake-site-conf`` (field test) or ``~cake_site_conf``.
CAKE_SITE_CONF_THRESHOLD = 0.60

#: Raw ids of the ordinary-cube prints used as "this white face is a cube face"
#: evidence.  The Cake classes (2, 20) are deliberately excluded.
CAKE_OWNER_PRINT_IDS = frozenset((TEDDY_ID, CLOCK_ID, BANANA_ID))

#: Confidence floor for that owner print (the detector publishes ordinary
#: boxes from 0.25 up).
CAKE_OWNER_PRINT_CONF = 0.30

#: Geometry of the owner relation, measured live on the field:
#: an ordinary cube's print and its plain white front face are 68 px apart
#: vertically (banana cube: print (186, 272) -> face (194, 340)), so a print
#: whose centre is 45..110 px ABOVE a cake hypothesis and within 55 px of it
#: horizontally owns that hypothesis.  The real Cake site has no recognised
#: cube print above it (the cake picture itself is not one of the cube
#: classes), so it is not affected.
CAKE_OWNER_DY_PX = (45.0, 110.0)
CAKE_OWNER_DU_PX = 55.0


# --------------------------------------------------------------------------
# Scan window: calibrated near field vs. far field (base approach)
# --------------------------------------------------------------------------
# The hand-eye matrix is fitted around the grasp workspace only, so a pixel in
# the NEAR field yields millimetre coordinates.  A cube that stands further
# away appears ABOVE that band, near the horizon.  Such a pixel is still a
# perfectly valid target - the base just has to drive to it.  Dropping those
# detections (as the field test used to) made every out-of-reach cube end in
# NO_TARGET, i.e. exit code 3, with the robot never moving.
NEAR_FIELD_U = (80.0, 560.0)
NEAR_FIELD_V = (130.0, 450.0)
#: Loose sanity window for the far field: only the image edge / off-camera
#: noise is rejected, so a real cube can still drive the base.
SCAN_WINDOW_U = (20.0, 620.0)
SCAN_WINDOW_V = (25.0, 470.0)
#: Plane used to invert a far floor-cube pixel.  The depth-based reconstruction
#: extrapolates badly outside the calibrated volume (measured +35 mm at 0.65 m),
#: while the grasp-plane inversion degrades gracefully; the next scan after the
#: approach step refines it.  Same convention as the competition grasp
#: (z=-100 plane + 20 mm cup correction).
FAR_FLOOR_PLANE_Z = -100.0
FAR_FLOOR_CUP_OFFSET_X = 20.0


def in_near_field(u, v):
    """True when the pixel lies in the calibrated, grasp-ready near field."""
    return (NEAR_FIELD_U[0] <= u <= NEAR_FIELD_U[1]
            and NEAR_FIELD_V[0] <= v <= NEAR_FIELD_V[1])


def in_scan_window(u, v):
    """True when the pixel is a plausible on-field detection worth chasing."""
    return (SCAN_WINDOW_U[0] <= u <= SCAN_WINDOW_U[1]
            and SCAN_WINDOW_V[0] <= v <= SCAN_WINDOW_V[1])


def far_target_xy(M, u, v, is_cake, cup_offset_x=CAKE_CUP_OFFSET_X,
                  plane_z=FAR_FLOOR_PLANE_Z,
                  cup_correction_x=FAR_FLOOR_CUP_OFFSET_X):
    """Approximate (x, y) in millimetres for a target beyond the near field.

    A far target only needs enough geometry for ONE base-approach step: the
    heading and the distance.  The near-field depth reconstruction and the
    clamped cake affine are both invalid at that range, so:

    * a floor cube is inverted onto the known grasp plane (``plane_z``) and
      shifted by the cup correction, exactly like the competition grasp;
    * a Cake uses the dedicated affine WITHOUT the reach clamp, so a really
      distant cake keeps a large ``x`` and drives the base instead of being
      reported as already in reach.

    Returns ``None`` when the inverse mapping is degenerate.
    """
    if is_cake:
        return pixel_to_cake_xy(M, u, v, cup_offset_x=cup_offset_x, clamp=False)
    xy = pixel_to_xy_at_z(M, u, v, plane_z)
    if xy is None:
        return None
    return float(xy[0]) + cup_correction_x, float(xy[1])


def cake_face_owner(detections, cx, cy, owner_conf=CAKE_OWNER_PRINT_CONF,
                    dy_range=CAKE_OWNER_DY_PX, du_limit=CAKE_OWNER_DU_PX):
    """Return the ordinary-cube print that owns a white cube face, or ``None``.

    The detector's ``wine_site``/``cake_site`` (raw id 20) class fires on the
    plain WHITE FRONT FACE of any cube, not only on the Cake site, and can score
    as high as the real Cake (measured 0.761 on a banana cube vs 0.76..0.87 for
    the real site).  What separates the two is context: an ordinary cube's white
    face sits directly BELOW its own print, 45..110 px down and within 55 px
    horizontally (measured on the banana cube: print centre (186, 272), face
    centre (194, 340)).  The real Cake site has no recognised cube print above
    it, so this check does not reject it.

    ``detections`` is one raw frame of ``(raw_id, score, cx, cy)`` tuples (the
    format ``CamAction.detector`` feeds to :func:`cluster_detections`).

    Returns ``(raw_id, score, u, v)`` of the owning print, or ``None``.
    """
    for detection in detections:
        try:
            raw_id, score, u, v = detection[:4]
            raw_id = int(raw_id)
        except (TypeError, ValueError, IndexError):
            continue
        if raw_id not in CAKE_OWNER_PRINT_IDS:
            continue
        if score is None or float(score) < float(owner_conf):
            continue
        dy = float(cy) - float(v)
        if not (dy_range[0] <= dy <= dy_range[1]):
            continue
        if abs(float(cx) - float(u)) > float(du_limit):
            continue
        return (raw_id, float(score), int(round(float(u))), int(round(float(v))))
    return None


def cluster_detections(frames, conf_threshold=CONF_THRESHOLD,
                       cluster_dist_px=CLUSTER_DIST_PX, min_hits=None,
                       cake_conf_threshold=CAKE_CONF_THRESHOLD,
                       cake_site_conf_threshold=CAKE_SITE_CONF_THRESHOLD,
                       cake_owner_conf=CAKE_OWNER_PRINT_CONF):
    """Temporal voting + spatial clustering over a window of detection frames.

    ``frames``: sequence of frames, each an iterable of ``(raw_id, score, cx, cy)``.
    Returns ``[[canonical_id, [x, y, 0]], ...]`` — one entry per physical cube,
    with the majority class and the averaged centre.

    Cake detections additionally have to pass a per-class anti-glare gate:
      * raw id 2  (``cake``)      -> ``cake_conf_threshold``
      * raw id 20 (``cake_site``) -> ``cake_site_conf_threshold``
    The Cake class has priority in the FSM, so a cheap false positive would
    otherwise send the robot after a non-existent cake.  The two raw classes
    have different false-positive profiles, hence two separate thresholds.

    Confidence is not enough: the detector also reports ordinary cubes' white
    FRONT FACES as ``cake_site`` with real-cake scores.  Such a hypothesis is
    discarded by :func:`cake_face_owner` (a cube print sits directly above the
    face), so the robot does not go after a "cake" that is only the side of a
    banana/teddy/clock cube.

    Because detections from *all* frames are poured into the same spatial
    clusters, this also collapses a cube reported twice in one frame (the
    detector runs two YOLO models), which single-frame code cannot do.
    """
    frames = [list(frame) for frame in frames if frame is not None]
    if not frames:
        return []

    if min_hits is None:
        if len(frames) < 2:
            min_hits = 1
        else:
            min_hits = max(2, int(round(len(frames) * MIN_HIT_RATIO)))

    cake_conf_threshold = float(cake_conf_threshold)
    cake_site_conf_threshold = float(cake_site_conf_threshold)
    clusters = []
    for frame in frames:
        for detection in frame:
            try:
                raw_id, score, cx, cy = detection[:4]
            except Exception:
                continue
            cid = norm_id(raw_id)
            if cid not in CANONICAL_IDS:
                continue
            if score is None:
                continue
            if cid == CAKE_ID:
                try:
                    is_site = int(raw_id) != CAKE_ID
                except (TypeError, ValueError):
                    is_site = False
                cake_min = (cake_site_conf_threshold if is_site
                            else cake_conf_threshold)
                min_score = max(float(conf_threshold), cake_min)
            else:
                min_score = float(conf_threshold)
            if float(score) < min_score:
                continue
            if cid == CAKE_ID and cake_face_owner(
                    frame, cx, cy, owner_conf=cake_owner_conf) is not None:
                # This "cake" is the plain white front face of an ordinary
                # cube: its own print sits right above it.  Never a target.
                continue

            match = None
            for cluster in clusters:
                mean_x = sum(cluster["xs"]) / len(cluster["xs"])
                mean_y = sum(cluster["ys"]) / len(cluster["ys"])
                if math.hypot(cx - mean_x, cy - mean_y) < cluster_dist_px:
                    match = cluster
                    break
            if match is None:
                match = {"ids": [], "xs": [], "ys": []}
                clusters.append(match)
            match["ids"].append(cid)
            match["xs"].append(float(cx))
            match["ys"].append(float(cy))

    min_cake_hits = 1 if len(frames) < 2 else 2
    cubes = []
    for cluster in clusters:
        cake_hits = sum(1 for c in cluster["ids"] if is_cake(c))
        if len(cluster["ids"]) < min_hits:
            # Operator requirement: Cake priority rule. A Cake cluster may
            # survive with fewer hits than other cubes, but still requires at
            # least 2 frames in a multi-frame window so a 1-frame glare cannot
            # hijack the mission.
            if cake_hits >= min_cake_hits:
                pass
            else:
                continue
        # Cake priority rule: only inside the near-field elevated pedestal band
        # (NEAR_FIELD_V[0] <= mean_y < 215 px) does a Cake hit override majority
        # voting.  Far-field floor cubes (v < 130) and ground clusters (v >= 215)
        # use strict majority voting.
        mean_y = sum(cluster["ys"]) / len(cluster["ys"])
        if (NEAR_FIELD_V[0] <= mean_y < 215.0
                and cake_hits >= min_cake_hits):
            cid = CAKE_ID
        else:
            cid = Counter(cluster["ids"]).most_common(1)[0][0]
        x = int(round(sum(cluster["xs"]) / len(cluster["xs"])))
        y = int(round(sum(cluster["ys"]) / len(cluster["ys"])))
        cubes.append([cid, [x, y, 0]])
    return cubes


def _canonical_box_class(box):
    """Canonical class key for de-duplication (merges ``<item>_site`` aliases)."""
    if "class_id" in box and box["class_id"] is not None:
        return str(norm_id(box["class_id"]))
    name = str(box.get("name", "") or "").strip().lower()
    if name.endswith("_site"):
        name = name[:-5]
    return name


def dedupe_boxes(boxes, dist_px=DEDUPE_DIST_PX):
    """Drop cross-model duplicates: same canonical class, closer than ``dist_px``.

    ``boxes`` are dicts with ``name``, ``score``, ``cx``, ``cy`` (extra keys are
    preserved).  ``<item>_site`` aliases (e.g. ``banana`` and ``banana_site``)
    are treated as the same canonical class so one physical cube is not emitted
    twice (SI-26).  The highest-confidence box of each coincident group survives.
    """
    kept = []
    for box in sorted(boxes, key=lambda b: -float(b.get("score", 0.0))):
        box_cls = _canonical_box_class(box)
        duplicate = False
        for other in kept:
            if _canonical_box_class(other) != box_cls:
                continue
            if math.hypot(box["cx"] - other["cx"], box["cy"] - other["cy"]) < dist_px:
                duplicate = True
                break
        if not duplicate:
            kept.append(box)
    return kept


def cube_still_present(cubes, orig_uv, radius_px=40.0):
    """True when a cube is still visible near ``orig_uv`` after a lift (SI-28).

    Used as a cheap software grasp-verification check when no vacuum pressure
    sensor is installed: a lifted cube leaves its pickup pixel empty.
    """
    if not cubes or orig_uv is None:
        return False
    ou, ov = float(orig_uv[0]), float(orig_uv[1])
    for cube in cubes:
        try:
            u, v = float(cube[1][0]), float(cube[1][1])
        except Exception:
            continue
        if math.hypot(u - ou, v - ov) <= float(radius_px):
            return True
    return False


def select_grasp_candidate(cubes, preferred_id=None, center_x=320.0, y_ref=260.0):
    """Pick the cube closest to the optical centre line (ahead of the robot).

    ``preferred_id`` keeps the originally targeted class after a re-scan, so a
    YOLO class flip cannot send a cube to the wrong zone mid-approach.
    """
    if not cubes:
        return None
    pool = [c for c in cubes if len(c) >= 2]
    if not pool:
        return None
    if preferred_id is not None:
        same_class = [c for c in pool if c[0] == preferred_id]
        if same_class:
            pool = same_class
    else:
        cake_candidates = [c for c in pool if is_cake(c[0])]
        if cake_candidates:
            pool = cake_candidates

    # If any two candidates are vertical duplicates of the same physical cube
    # (top face vs front/bottom face, separated by <= 90 px vertically and <= 45 px horizontally),
    # eliminate the bottom/front face (larger v) in favor of the top face (smaller v).
    filtered = []
    for c in pool:
        u, v = c[1][0], c[1][1]
        has_top_face = any(
            other != c and
            abs(other[1][0] - u) <= 45.0 and
            0 < (v - other[1][1]) <= 90.0
            for other in pool
        )
        if not has_top_face:
            filtered.append(c)

    candidates = filtered if filtered else pool
    return min(candidates, key=lambda c: abs(c[1][0] - center_x) + (y_ref - c[1][1]) * 1.2)


# --------------------------------------------------------------------------
# FSM routing: stations, approach waypoints, exit manoeuvres
# --------------------------------------------------------------------------

STATION_ORDER = ("Sorting_W", "Sorting_S", "Sorting_E", "Sorting_N", "Sorting_M")
# From a station, prefer the one on the side we are about to deliver to.
STATION_PREFERENCE = {
    "Collection_B": ("Sorting_E", "Sorting_N"),
    "Collection_C": ("Sorting_S", "Sorting_E"),
    "Collection_D": ("Sorting_W", "Sorting_S"),
}
# Waypoint used to reach each station, depending on the previous drop zone.
APPROACH_WAYPOINTS = {
    "Collection_B": {"Sorting_W": "Step_NW", "Sorting_S": "Step_SE"},
    "Collection_C": {"Sorting_W": "Step_SW", "Sorting_N": "Step_NE", "Sorting_M": "Step_NE"},
    "Collection_D": {"Sorting_N": "Step_NW", "Sorting_M": "Step_NW", "Sorting_E": "Step_SE"},
}
# Manoeuvre executed after grasping, before driving to the drop zone.
EXIT_BACK_M = 0.4
EXIT_MANEUVERS = {
    "Collection_B": {"Sorting_S": ("back", EXIT_BACK_M), "Sorting_W": ("arrive", "Step_NW")},
    "Collection_C": {"Sorting_N": ("back", EXIT_BACK_M), "Sorting_W": ("arrive", "Step_SW")},
    "Collection_D": {"Sorting_N": ("back", EXIT_BACK_M), "Sorting_E": ("back", EXIT_BACK_M)},
}


def choose_station(drop_zone, stations, preference=STATION_PREFERENCE):
    """Next station to harvest: nearest to ``drop_zone``, else first with picks."""
    for name in preference.get(drop_zone, ()):
        if stations.get(name, 0) > 0:
            return name
    for name in STATION_ORDER:
        if stations.get(name, 0) > 0:
            return name
    for name, left in stations.items():
        if left > 0:
            return name
    return None


def approach_waypoint(station, drop_zone):
    """Intermediate waypoint needed to reach ``station`` from ``drop_zone``."""
    return APPROACH_WAYPOINTS.get(drop_zone, {}).get(station)


def exit_maneuver(station, drop_zone):
    """``("back", metres)`` / ``("arrive", name)`` before driving to the zone."""
    return EXIT_MANEUVERS.get(drop_zone, {}).get(station)


def retire_station(stations, name):
    """Remove ``name`` from the work plan; True when it was still scheduled."""
    return stations.pop(name, None) is not None


# --------------------------------------------------------------------------
# Operator commands and match timing
# --------------------------------------------------------------------------

START_COMMANDS = frozenset({"start", "1", "go", "on", "true", "begin", "run"})
STOP_COMMANDS = frozenset({"stop", "0", "halt", "off", "false", "abort", "kill"})
MATCH_DURATION_S = 600.0
DEFAULT_TIME_BUDGET_S = 540.0   # keep 60 s of the match for shutdown / scoring
TIME_WARN_S = 60.0


def parse_task_command(data):
    """Map a ``/task_start_flag`` payload to ``"start"`` / ``"stop"`` / ``None``.

    An unknown or empty payload returns None, which the caller must treat as
    "ignore".  This is deliberate: the previous code started the mission on ANY
    message, so the documented emergency-abort command could launch a run.
    """
    command = (data or "").strip().lower()
    if command in START_COMMANDS:
        return "start"
    if command in STOP_COMMANDS:
        return "stop"
    return None


class MatchClock(object):
    """Monotonic match timer with an injectable clock, so it is testable."""

    def __init__(self, limit_s=DEFAULT_TIME_BUDGET_S, warn_s=TIME_WARN_S,
                 time_fn=None):
        self.limit_s = float(limit_s)
        self.warn_s = float(warn_s)
        self._time_fn = time_fn or time.monotonic
        self._start = None

    def start(self):
        self._start = self._time_fn()
        return self

    def elapsed(self):
        if self._start is None:
            return 0.0
        return max(0.0, self._time_fn() - self._start)

    def remaining(self):
        return max(0.0, self.limit_s - self.elapsed())

    def expired(self):
        return self.remaining() <= 0.0

    def warn(self):
        return (not self.expired()) and self.remaining() <= self.warn_s
