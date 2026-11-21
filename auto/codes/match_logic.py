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


def pixel_to_cake_xy(M, u, v, fx=CAM_FX, fy=CAM_FY, cx=CAM_CX, cy=CAM_CY,
                     calib_file=None, cup_offset_x=30.0):
    """Dedicated approximation for the competition Cake (Tier 2 / 20 cm height).

    Uses the dedicated Layer 2 calibration (thefile2.txt) created specifically
    for the elevated Cake.
    cup_offset_x = +30.0 mm (+3 cm) places the suction cup dead-center on the
    Cake rather than on the front edge.
    Safe reach limits are expanded specifically for Cake up to X = 350.0 mm
    (supported by uArm Swift Pro firmware).
    """
    x_kb, y_kb = load_cake_calibration(calib_file)
    cake_x = float(x_kb[0] * v + x_kb[1] + cup_offset_x)
    cake_y = float(y_kb[0] * u + y_kb[1])
    # Expanded safe limits specifically for Cake (firmware supports up to 350 mm)
    cake_x = max(120.0, min(350.0, cake_x))
    cake_y = max(-190.0, min(190.0, cake_y))
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
DEDUPE_DIST_PX = 60.0      # two models reporting the same cube
MIN_HIT_RATIO = 0.5        # share of frames a cluster must appear in


def cluster_detections(frames, conf_threshold=CONF_THRESHOLD,
                       cluster_dist_px=CLUSTER_DIST_PX, min_hits=None):
    """Temporal voting + spatial clustering over a window of detection frames.

    ``frames``: sequence of frames, each an iterable of ``(raw_id, score, cx, cy)``.
    Returns ``[[canonical_id, [x, y, 0]], ...]`` — one entry per physical cube,
    with the majority class and the averaged centre.

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

    clusters = []
    for frame in frames:
        for detection in frame:
            try:
                raw_id, score, cx, cy = detection[:4]
            except Exception:
                continue
            if score is None or float(score) < conf_threshold:
                continue
            cid = norm_id(raw_id)
            if cid not in CANONICAL_IDS:
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

    cubes = []
    for cluster in clusters:
        if len(cluster["ids"]) < min_hits:
            # Operator requirement: Cake priority rule. If a cluster contains Cake,
            # it must not be discarded even if detected in fewer frames than other cubes.
            if any(is_cake(c) for c in cluster["ids"]):
                pass
            else:
                continue
        cid = Counter(cluster["ids"]).most_common(1)[0][0]
        x = int(round(sum(cluster["xs"]) / len(cluster["xs"])))
        y = int(round(sum(cluster["ys"]) / len(cluster["ys"])))
        cubes.append([cid, [x, y, 0]])
    return cubes


def dedupe_boxes(boxes, dist_px=DEDUPE_DIST_PX):
    """Drop cross-model duplicates: same class, centres closer than ``dist_px``.

    ``boxes`` are dicts with ``name``, ``score``, ``cx``, ``cy`` (extra keys are
    preserved).  The highest-confidence box of each coincident group survives.
    """
    kept = []
    for box in sorted(boxes, key=lambda b: -float(b.get("score", 0.0))):
        duplicate = False
        for other in kept:
            if other.get("name") != box.get("name"):
                continue
            if math.hypot(box["cx"] - other["cx"], box["cy"] - other["cy"]) < dist_px:
                duplicate = True
                break
        if not duplicate:
            kept.append(box)
    return kept


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
