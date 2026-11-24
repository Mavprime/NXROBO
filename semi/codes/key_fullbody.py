#!/usr/bin/env python3
"""
Keyboard teleop for the Spark-H base + the 4-DOF uArm (base, shoulder, elbow).

Base (browser bridge - exact hold duration):
    Run keyboard_web.py on the robot, forward port 8766 over SSH, then open
    http://127.0.0.1:8766/. Hold W/S to drive and A/D to turn; a key-up sends
    an immediate zero state, so the base moves for exactly as long as the key
    was held. A 0.25 s input watchdog and the page's focus-loss disarm stop the
    base if the browser goes away.

Base (terminal fallback - used only when no bridge client is attached):
    w  forward          s  backward
    a  turn left        d  turn right
    SSH terminals report no key-up, so this path approximates a hold from key
    repeats and a single w/s tap still coasts for TERM_FIRST_REPEAT. Use the
    browser bridge for true hold-to-move.

Arm (ramped joint jogging at a tip speed up to ARM_JOG_SPEED, plus combos):
    4  base rotate left          6  base rotate right
    7  raise upper arm           9  lower upper arm
    8  extend forearm            2  fold forearm
    q  7 + 8    e  7 + 2    z  9 + 8    c  9 + 2

Automatic pickup:
    1      pick up the first block: approach above it, descend vertically at
           the pickup X/Y, vacuum ON, wait SUCTION_DELAY_S, lift vertically,
           then move to the carry pose. Non-blocking state machine driven by
           the same 20 Hz loop. Any arm/base key, Space, 0, or Ctrl+C cancels
           it. A second 1 while it runs is ignored; a plain terminal must
           release 1 (or pause TERM_ACTION_GAP) before 1 can start it again.
           PICKUP_X_MM / PICKUP_Y_MM are required: the block position belongs
           to the field, not to the arm, so it cannot be guessed here.

           Heights use the driver's wrist frame, not the floor. The field
           calibration in auto/codes (calib3d.py operator anchors,
           pick_direct.py) puts the floor at FLOOR_Z_MM ~= -150 mm, and
           auto/codes/runtask.py contact_heights() documents that the suction
           cup's sealing rim is SUCTION_PAD_MM ~= 8 mm below the wrist pose.
           A block of height H on the floor is therefore contacted at
               wrist z = FLOOR_Z + H + SUCTION_PAD - SUCTION_PRESS
           The pump has no pressure/suction feedback on this robot, so the
           routine can only confirm the command sequence, never that the
           block is actually attached.

Other:
    Enter  toggle the vacuum pump and lift straight up when switching it on
    Space  cancel the lift with the pump still on
    m      re-attach the arm motors
    0      release the pump, stop the base, and send the arm to its home pose
    Ctrl+C stop everything (base stopped, pump off)

How the arm works
-----------------
The driver only accepts a Cartesian pose on /position_write_topic
(swiftpro/position x,y,z in mm). There is no joint-angle topic. So this script
keeps the three motor angles locally and converts them to x,y,z with the same
forward kinematics as the robot's swiftpro_moveit_node.cpp:

    stretch = LOWER*cos(a1) + UPPER*cos(a2) + L2 + 56.55
    height  = LOWER*sin(a1) - UPPER*sin(a2) + L1
    x = stretch*sin(a0),  y = -stretch*cos(a0),  z = height - 74.55

where a0 = base yaw, a1 = shoulder, a2 = elbow, all in degrees. At the home
pose (120, 0, 35) mm the inverse gives about (90, 103.4, 58.4) degrees.

Motion
------
A pressed arm key is held as a direction, and the control loop advances the tip
every tick (20 Hz) at a ramped speed up to ARM_JOG_SPEED, easing near the
relevant joint limit. Motion therefore starts immediately instead of waiting
for the terminal's key-repeat delay, and a quick tap still moves for
TAP_MIN_DURATION. Kitty terminals report the exact release; plain terminals
renew the hold for PLAIN_HOLD_TIMEOUT on every repeat byte. Queued repeats are
discarded rather than replayed, and an idle loop never sends a jog.
The base is ramped the same way by LIN_SPEED/ANG_SPEED and the MAX_* limits.

Base keys use the same tracking, so w/a/s/d can stay held while 4/6/7/9/8/2
are held: the base target and the arm jog are recomputed together every tick.
w/s use the long forward pulse (TERM_FIRST_REPEAT), while a/d use a short pulse
(TURN_TAP_MIN) and a lower ANG_SPEED, so one a/d tap only rotates a few degrees
instead of a large angle.

Terminals that support Kitty key events report each press and release, so held
keys stay independent. The Q/E/Z/C shortcuts work in ordinary SSH terminals
too. A plain SSH terminal repeats only the most recently pressed key, so a
fresh key coasts for its tap window and a repeating one for
TERM_REPEAT_TIMEOUT; re-tap the base key if you want to turn for longer while
another key is repeating.

Turning the pump on lifts as high as the arm can go at its current X and Y.
A movement key, Space, or turning the pump off cancels the remaining lift.
The re-attach key here is `m` because `a` is base turn-left; arm.py uses `a`.

Limits
------
A pose is accepted only if ALL of these hold (see check_pose()):
  * each joint is inside its own range (A0/A1/A2_LIMITS)
  * shoulder + elbow <= 180 - MIN_ELBOW_ANGLE. Past 180 the driver's inverse
    kinematics picks the OTHER elbow configuration for the same x,y,z, so the
    real arm would flip while this script believed it was moving smoothly.
  * radial reach <= MAX_REACH (a circle around the base, so the base can swing
    freely at any extension) and z inside Z_LIMITS
  * the driver's IK returns exactly the joint angles we think we are at
  * (optional) arm_ik.ik_reachable() agrees, if arm_ik.py is next to this file

Arm state / first run
---------------------
At power-on the arm motors are DETACHED: the driver accepts poses and reports
them on /SwiftproState_topic, but nothing physically moves until the attach
sequence runs (as in the M-Robots grasp code):

    /swiftpro_status_topic:  0  -> M2019 (detach)   1 -> M17 (attach)

This script sends that 0 -> 1 pair on startup and again whenever you press m.
If the arm "accepts" keys but never moves, press m to re-attach.

The script saves its last commanded pose in .arm_last_pose (next to this file)
and resumes from it on the next run. On a first run without a valid starting
pose, press 0 once to home the arm and create the saved pose.

Prerequisites
-------------
    base: roslaunch spark_bringup driver_bringup.launch camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6
    arm:  roslaunch swiftpro pro_control_nomoveit.launch
         (or: roslaunch ~/semi/server.launch)

Usage
-----
    ssh spark@10.25.141.8
    source ~/spark_noetic/devel/setup.bash
    python3 ~/semi/key_fullbody.py
    python3 ~/semi/key_fullbody.py --check-kinematics   # no ROS needed

Environment overrides:
    LIN_SPEED=0.30             m/s
    ANG_SPEED=0.5              rad/s
    MAX_LIN_ACCEL=2.0          m/s^2
    MAX_LIN_DECEL=4.0          m/s^2
    MAX_ANG_ACCEL=6.0          rad/s^2
    MAX_ANG_DECEL=10.0         rad/s^2
    BODY_PORT=8765             loopback port keyboard_web.py streams key states to
    BODY_WATCHDOG=0.25         s without browser input before the base is stopped
    PICKUP_X_MM / PICKUP_Y_MM  block suction point in arm coordinates (required)
    BLOCK_HEIGHT_MM=150        measured block height above the floor
    FLOOR_Z_MM=-150            measured model z of the floor
    SUCTION_PAD_MM=8           cup rim distance below the wrist pose
    SUCTION_PRESS_MM=3         extra descent to compress the cup
    APPROACH_HEIGHT_MM=100     approach height above the contact point
    LIFT_HEIGHT_MM=80          vertical lift above the contact point
    CARRY_X_MM/CARRY_Y_MM/CARRY_Z_MM
                               carry pose (default: pickup X/Y and lift height)
    SUCTION_DELAY_S=0.8        pump dwell at contact before lifting
    PICKUP_SPEED_MM_S=120      tip speed for the pickup moves
    PICKUP_TOL_MM=1.0          arrival tolerance for the pickup stages
    PICKUP_STAGE_TIMEOUT_S=15  abort a pickup stage that takes longer
    TERM_FIRST_REPEAT=0.95     s a fresh w/s key is held before release
    TERM_REPEAT_TIMEOUT=0.32   s between repeats before the base is released
    TURN_TAP_MIN=0.20          s a quick a/d tap rotates (keeps taps small)
    TURN_HOLD_TIMEOUT=0.35     s a plain a/d byte keeps turning (no repeats)
    TERM_REARM_PAUSE=0.90      s quiet time before movement re-arms after 0
    TERM_ACTION_GAP=0.90       s minimum gap between repeated Enter/0/m presses
    JOINT_STEP=10              upper bound on an arm joint step in degrees
    BASE_YAW_STEP=6            additional upper bound on a yaw step in degrees
    BASE_YAW_SPEED=120         upper bound on commanded yaw speed in deg/s
    ARM_JOG_SPEED=120          maximum commanded tip speed for jogs in mm/s
    PUMP_LIFT_DELAY=0.35       seconds to wait after turning the pump on
    BASE_YAW_MIN=5             set higher if this arm stops earlier at the low end
    BASE_YAW_MAX=175           set lower if this arm stops earlier at the high end
    MIN_ELBOW_ANGLE=10         keep the elbow at least this far from fully folded
    MAX_REACH=350              max radial distance of the tip from the base axis, mm
"""

import json
import math
import os
import re
import select
import socket
import sys
import termios
import threading
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from arm_ik import ik_reachable as _arm_ik_reachable
except Exception:
    _arm_ik_reachable = None

# ---------------------------------------------------------------------------
# Arm model - constants from swiftpro_moveit_node.cpp / swiftpro_rviz_node.cpp
# ---------------------------------------------------------------------------
MATH_L1 = 106.6
MATH_L2 = 13.2
MATH_LOWER_ARM = 142.07
MATH_UPPER_ARM = 158.81
MATH_Z_OFFSET = 74.55
MATH_BASE_OFFSET = MATH_L2 + 56.55

HOME = (120.0, 0.0, 35.0)

A0_LIMITS = (
    float(os.environ.get("BASE_YAW_MIN", "5")),
    float(os.environ.get("BASE_YAW_MAX", "175")),
)
if not (5.0 <= A0_LIMITS[0] <= 90.0 <= A0_LIMITS[1] <= 175.0 and A0_LIMITS[0] < A0_LIMITS[1]):
    sys.exit("Base yaw limits must be within 5..175 degrees and include home (90 degrees).")

A1_LIMITS = (0.0, 135.6)
A2_LIMITS = (0.0, 100.7)
JOINT_LIMITS = (A0_LIMITS, A1_LIMITS, A2_LIMITS)
JOINT_NAMES = ("base", "shoulder", "elbow")

MAX_REACH = float(os.environ.get("MAX_REACH", "350"))
Z_LIMITS = (-60.0, 170.0)
MIN_ELBOW_ANGLE = float(os.environ.get("MIN_ELBOW_ANGLE", "10"))
FOLD_MAX = 180.0 - MIN_ELBOW_ANGLE
POSE_EPSILON = 1e-9

# Base ramping.
LIN_SPEED = float(os.environ.get("LIN_SPEED", "0.30"))
ANG_SPEED = float(os.environ.get("ANG_SPEED", "0.5"))
MAX_LIN_ACCEL = float(os.environ.get("MAX_LIN_ACCEL", "2.0"))
MAX_LIN_DECEL = float(os.environ.get("MAX_LIN_DECEL", "4.0"))
MAX_ANG_ACCEL = float(os.environ.get("MAX_ANG_ACCEL", "6.0"))
MAX_ANG_DECEL = float(os.environ.get("MAX_ANG_DECEL", "10.0"))

# Browser base bridge: keyboard_web.py (running on the robot) forwards real
# key-down/key-up state here as newline-delimited JSON, so the base moves for
# exactly as long as a key is held. The terminal pulse path stays as fallback.
BASE_HOST = "127.0.0.1"
BASE_PORT = int(os.environ.get("BODY_PORT", "8765"))
BODY_WATCHDOG = float(os.environ.get("BODY_WATCHDOG", "0.25"))

TERM_FIRST_REPEAT = float(os.environ.get("TERM_FIRST_REPEAT", "0.95"))
TERM_REPEAT_TIMEOUT = float(os.environ.get("TERM_REPEAT_TIMEOUT", "0.32"))
# a/d turn in short pulses so one tap is only a few degrees.
TURN_TAP_MIN = float(os.environ.get("TURN_TAP_MIN", "0.20"))
TURN_HOLD_TIMEOUT = float(os.environ.get("TURN_HOLD_TIMEOUT", "0.35"))
TERM_REARM_PAUSE = float(os.environ.get("TERM_REARM_PAUSE", "0.90"))
TERM_ACTION_GAP = float(os.environ.get("TERM_ACTION_GAP", "0.90"))

# Arm jogging - same tuning as arm.py.
JOINT_STEP = float(os.environ.get("JOINT_STEP", "10"))
if not (math.isfinite(JOINT_STEP) and JOINT_STEP > 0.0):
    sys.exit("Joint step must be a finite positive number of degrees.")

BASE_YAW_STEP = float(os.environ.get("BASE_YAW_STEP", "6"))
if not (math.isfinite(BASE_YAW_STEP) and BASE_YAW_STEP > 0.0):
    sys.exit("Base yaw step must be a finite positive number of degrees.")
BASE_YAW_STEP = min(BASE_YAW_STEP, JOINT_STEP)

BASE_YAW_SPEED = float(os.environ.get("BASE_YAW_SPEED", "120"))
if not (math.isfinite(BASE_YAW_SPEED) and BASE_YAW_SPEED > 0.0):
    sys.exit("Base yaw speed must be a finite positive number of degrees per second.")

ARM_JOG_SPEED = float(os.environ.get("ARM_JOG_SPEED", "120"))
if not (math.isfinite(ARM_JOG_SPEED) and ARM_JOG_SPEED > 0.0):
    sys.exit("Arm jog speed must be a finite positive number of mm/s.")


# ---------------------------------------------------------------------------
# Automatic block pickup (key 1)
# ---------------------------------------------------------------------------
# Coordinate reference: fk()/ik() z is the uArm wrist pose in the driver's
# frame (mm), not a floor-relative height. Two measured offsets turn a floor
# height into a wrist target:
#   * FLOOR_Z_MM - model z of the physical floor. Field calibration in
#     auto/codes (calib3d.py operator anchors, pick_direct.py) measured about
#     -150 mm for this robot.
#   * SUCTION_PAD_MM - the rubber cup's sealing rim sits this far below the
#     wrist pose. auto/codes/runtask.py contact_heights() states that the wrist
#     must stop PAD_THICKNESS_MM (8) above a surface for the rim to meet it.
# A block standing on the floor therefore needs the wrist at
#     FLOOR_Z + BLOCK_HEIGHT + SUCTION_PAD
# and a little lower (SUCTION_PRESS_MM) so the cup compresses and seals.
# The block's X/Y are a property of the field, not the arm, so they have no
# default: measure them and pass PICKUP_X_MM / PICKUP_Y_MM.
def _env_optional_float(name):
    raw = os.environ.get(name, "").strip()
    if not raw:
        return None
    try:
        return float(raw)
    except ValueError:
        sys.exit("%s must be a number of millimetres." % name)


def _env_positive_float(name, default):
    value = float(os.environ.get(name, default))
    if not (math.isfinite(value) and value > 0.0):
        sys.exit("%s must be a finite positive number." % name)
    return value


def _env_nonnegative_float(name, default):
    value = float(os.environ.get(name, default))
    if not (math.isfinite(value) and value >= 0.0):
        sys.exit("%s must be a finite, nonnegative number." % name)
    return value


BLOCK_HEIGHT_MM = _env_positive_float("BLOCK_HEIGHT_MM", "150")
FLOOR_Z_MM = float(os.environ.get("FLOOR_Z_MM", "-150"))
if not math.isfinite(FLOOR_Z_MM):
    sys.exit("FLOOR_Z_MM must be a finite number.")
SUCTION_PAD_MM = _env_positive_float("SUCTION_PAD_MM", "8")
SUCTION_PRESS_MM = _env_nonnegative_float("SUCTION_PRESS_MM", "3")
APPROACH_HEIGHT_MM = _env_positive_float("APPROACH_HEIGHT_MM", "100")
LIFT_HEIGHT_MM = _env_positive_float("LIFT_HEIGHT_MM", "80")
SUCTION_DELAY_S = _env_nonnegative_float("SUCTION_DELAY_S", "0.8")
PICKUP_SPEED_MM_S = _env_positive_float("PICKUP_SPEED_MM_S", str(ARM_JOG_SPEED))
PICKUP_TOL_MM = _env_positive_float("PICKUP_TOL_MM", "1.0")
PICKUP_STAGE_TIMEOUT_S = _env_positive_float("PICKUP_STAGE_TIMEOUT_S", "15")

PICKUP_X_MM = _env_optional_float("PICKUP_X_MM")
PICKUP_Y_MM = _env_optional_float("PICKUP_Y_MM")
CARRY_X_MM = _env_optional_float("CARRY_X_MM")
CARRY_Y_MM = _env_optional_float("CARRY_Y_MM")
CARRY_Z_MM = _env_optional_float("CARRY_Z_MM")

#: Wrist z where the cup rim meets the top of a floor block (mm).
PICKUP_CONTACT_Z_MM = (
    FLOOR_Z_MM + BLOCK_HEIGHT_MM + SUCTION_PAD_MM - SUCTION_PRESS_MM
)

PUMP_LIFT_DELAY = float(os.environ.get("PUMP_LIFT_DELAY", "0.35"))
if not (math.isfinite(PUMP_LIFT_DELAY) and PUMP_LIFT_DELAY >= 0.0):
    sys.exit("Pump lift delay must be a finite, nonnegative number of seconds.")

JOG_ACCEL = 1200.0              # mm/s^2
JOG_PERIOD = 0.05               # SwiftPro processes targets at 20 Hz
JOG_MAX_INTERVAL = 0.07         # cap stride when repeats arrive slowly
JOG_RESTART_GAP = 0.14          # reset acceleration between separate presses
BLOCK_RETRY = 0.35              # s before a blocked jog retries
TAP_MIN_DURATION = 0.18         # s a quick tap keeps moving (never a dead press)
PLAIN_HOLD_TIMEOUT = 0.22       # s a byte keeps its key held (no release events)
HELD_MAX_DURATION = 5.0         # s safety cap if a release event never arrives
MIN_MOVE = 0.2                  # deg, ignore smaller partial moves

JOG_KEYS = {
    "4": (0, +1),  # base left
    "6": (0, -1),  # base right
    "7": (1, -1),  # shoulder raise
    "9": (1, +1),  # shoulder lower
    "8": (2, -1),  # elbow extend
    "2": (2, +1),  # elbow fold
}
REVERSE_KEY = {"4": "6", "6": "4", "7": "9", "9": "7", "8": "2", "2": "8"}
COMBO_KEYS = {"q": "78", "e": "72", "z": "98", "c": "92"}
BASE_KEYS = "wasd"
ATTACH_KEY = "m"
# Ctrl+C and the action keys, plus the keys forwarded to the base and arm.
CONTROL_KEYS = "\x03" + ATTACH_KEY + "0\r\n " + BASE_KEYS + "1"
ACCEPTED_KEYS = set(JOG_KEYS) | set(COMBO_KEYS) | set(CONTROL_KEYS)
KITTY_KEY = re.compile(rb"(\d+)(?::[\d:]*)?(?:;(\d+)(?::([123]))?)?(?:;[\d:]*)?u")
CSI_END = re.compile(rb"[\x40-\x7e]")

STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".arm_last_pose")


def fk(a0, a1, a2):
    """Motor angles (degrees) -> Cartesian pose (mm)."""
    r0, r1, r2 = math.radians(a0), math.radians(a1), math.radians(a2)
    stretch = (
        MATH_LOWER_ARM * math.cos(r1)
        + MATH_UPPER_ARM * math.cos(r2)
        + MATH_BASE_OFFSET
    )
    height = (
        MATH_LOWER_ARM * math.sin(r1)
        - MATH_UPPER_ARM * math.sin(r2)
        + MATH_L1
    )
    return (
        stretch * math.sin(r0),
        -stretch * math.cos(r0),
        height - MATH_Z_OFFSET,
    )


def ik(x, y, z):
    """Cartesian pose (mm) -> motor angles (degrees), or None if unreachable."""
    a0 = math.degrees(math.atan2(x, -y))
    stretch = math.hypot(x, y)
    x_in = stretch - MATH_BASE_OFFSET
    z_in = z + MATH_Z_OFFSET - MATH_L1
    d = math.hypot(x_in, z_in)
    if d < 1e-6:
        return None

    cos_delta = (
        (d * d - MATH_LOWER_ARM ** 2 - MATH_UPPER_ARM ** 2)
        / (2.0 * MATH_LOWER_ARM * MATH_UPPER_ARM)
    )
    if cos_delta < -1.0 - 1e-9 or cos_delta > 1.0 + 1e-9:
        return None

    delta = -math.acos(max(-1.0, min(1.0, cos_delta)))
    phi = math.atan2(z_in, x_in)
    a1 = phi - math.atan2(
        MATH_UPPER_ARM * math.sin(delta),
        MATH_LOWER_ARM + MATH_UPPER_ARM * math.cos(delta),
    )
    return (a0, math.degrees(a1), math.degrees(-(a1 + delta)))


HOME_JOINTS = ik(*HOME) or (90.0, 103.4, 58.4)


def distance(a, b):
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(a, b)))


def furthest_valid(lo, hi, allowed):
    """Binary search boundary between an allowed value and a rejected value."""
    for _ in range(14):
        mid = (lo + hi) * 0.5
        if allowed(mid):
            lo = mid
        else:
            hi = mid
    return lo


def check_pose(joints):
    """Return reason string if joints violate safety/kinematic limits, else None."""
    for value, (lo, hi), name in zip(joints, JOINT_LIMITS, JOINT_NAMES):
        if value < lo - POSE_EPSILON or value > hi + POSE_EPSILON:
            return "%s joint limit (%.1f..%.1f deg)" % (name, lo, hi)

    if joints[1] + joints[2] > FOLD_MAX + 1e-9:
        return (
            "elbow folded too far (shoulder+elbow %.0f > %.0f deg)"
            % (joints[1] + joints[2], FOLD_MAX)
        )

    x, y, z = fk(*joints)
    reach = math.hypot(x, y)
    if reach > MAX_REACH + POSE_EPSILON:
        return "reach %.0f mm > %.0f mm" % (reach, MAX_REACH)
    if not (Z_LIMITS[0] - POSE_EPSILON <= z <= Z_LIMITS[1] + POSE_EPSILON):
        return "height z=%.0f mm outside %.0f..%.0f" % (z, Z_LIMITS[0], Z_LIMITS[1])

    back = ik(x, y, z)
    if back is None or max(abs(p - q) for p, q in zip(joints, back)) > 0.05:
        return "driver IK would pick a different arm configuration"

    if _arm_ik_reachable is not None:
        try:
            ok = _arm_ik_reachable(x, y, z)
        except Exception:
            ok = False
        if not ok:
            return "arm_ik.ik_reachable() rejected the pose"
    return None


def validated_pose_joints(x, y, z):
    """Validate a saved or echoed Cartesian pose into joint angles."""
    joints = ik(x, y, z)
    if joints is None:
        return None
    bounded = tuple(
        max(lo, min(hi, value))
        for value, (lo, hi) in zip(joints, JOINT_LIMITS)
    )
    if joints != bounded and distance((x, y, z), fk(*bounded)) <= 0.001:
        joints = bounded
    return joints if check_pose(joints) is None else None


def save_pose(x, y, z):
    """Persist the last commanded Cartesian pose."""
    try:
        tmp = STATE_FILE + ".tmp"
        with open(tmp, "w") as handle:
            handle.write("%.3f %.3f %.3f\n" % (x, y, z))
        os.replace(tmp, STATE_FILE)
    except OSError:
        pass


def load_saved_pose():
    """Load joints from the last saved pose if valid."""
    try:
        with open(STATE_FILE) as handle:
            x, y, z = [float(value) for value in handle.read().split()]
    except (OSError, ValueError):
        return None
    return validated_pose_joints(x, y, z)


def _check_kinematics():
    """Self-check FK/IK round trips and pose validation without ROS."""
    ok = True
    cart_points = [HOME, (200.0, 0.0, 20.0), (250.0, 100.0, 50.0), (180.0, -80.0, 80.0)]
    for p in cart_points:
        joints = ik(*p)
        if joints is None:
            print("IK failed for %s" % (p,))
            ok = False
            continue
        q = fk(*joints)
        err = distance(p, q)
        print(
            "IK/FK %s -> (%0.2f, %0.2f, %0.2f) -> err %.4f mm"
            % (p, joints[0], joints[1], joints[2], err)
        )
        if err > 0.01:
            ok = False

    joint_points = [HOME_JOINTS, (90.0, 80.0, 40.0), (90.0, 120.0, 20.0), (45.0, 60.0, 30.0)]
    for a in joint_points:
        p = fk(*a)
        b = ik(*p)
        if b is None:
            print("FK/IK failed for %s" % (a,))
            ok = False
            continue
        err = max(abs(x - y) for x, y in zip(a, b))
        print(
            "FK/IK %s -> (%0.2f, %0.2f, %0.2f) -> err %.4f deg"
            % (a, b[0], b[1], b[2], err)
        )
        if err > 0.01:
            ok = False

    reason = check_pose(HOME_JOINTS)
    print("home pose accepted by check_pose: %s" % ("yes" if reason is None else "NO - " + reason))
    if reason is not None:
        ok = False

    accepted = mismatched = 0
    for i in range(35):
        a0 = A0_LIMITS[0] + (A0_LIMITS[1] - A0_LIMITS[0]) * i / 34.0
        for j in range(60):
            a1 = A1_LIMITS[0] + (A1_LIMITS[1] - A1_LIMITS[0]) * j / 59.0
            for k in range(45):
                a2 = A2_LIMITS[0] + (A2_LIMITS[1] - A2_LIMITS[0]) * k / 44.0
                joints = (a0, a1, a2)
                if check_pose(joints) is not None:
                    continue
                accepted += 1
                back = ik(*fk(*joints))
                if back is None or max(abs(p - q) for p, q in zip(joints, back)) > 0.05:
                    mismatched += 1

    print("accepted poses: %d, with IK mismatch: %d" % (accepted, mismatched))
    if mismatched:
        ok = False
    print("kinematics OK" if ok else "kinematics FAILED")
    return ok


if "--check-kinematics" in sys.argv:
    sys.exit(0 if _check_kinematics() else 1)

import rospy
from geometry_msgs.msg import Twist
from swiftpro.msg import SwiftproState, position, status

HELP = """
  Base:  browser bridge (keyboard_web.py), OR terminal w/a/s/d
         bridge: run keyboard_web.py on the robot; ssh -L 8766:127.0.0.1:8766;
                 open http://127.0.0.1:8766/ and hold W/S/A/D (real key-up)
  Base:  w forward   s backward   a turn left   d turn right   (terminal)
  Arm:   4 base left   6 base right
         7 upper arm up   9 upper arm down
         8 extend forearm  2 fold forearm        (jog tip speed up to %g mm/s)
  Combo: q = 7+8   e = 7+2   z = 9+8   c = 9+2
  Auto:  1 automatic block pickup (any movement key / Space / 0 cancels)
  Other: Enter pump ON + lift / OFF   Space cancel lift
         m re-attach arm   0 release + home   Ctrl+C stop all
"""


def safe_print(*args):
    message = " ".join(str(a) for a in args)
    sys.stdout.write("\r\033[K" + message + "\r\n")
    sys.stdout.flush()


def toward(current, target, accel, decel, dt):
    """Rate-limit starts and stops so base commands ramp smoothly."""
    braking = target == 0.0 or current * target < 0.0 or abs(target) < abs(current)
    max_change = (decel if braking else accel) * dt
    diff = target - current
    if abs(diff) <= max_change:
        return target
    return current + math.copysign(max_change, diff)


def parse_base_state(raw):
    """Decode one keyboard_web.py state packet into (linear, turn, stop)."""
    if len(raw) > 1024:
        return None
    try:
        msg = json.loads(raw.decode("ascii"))
    except (ValueError, UnicodeError):
        return None
    if not isinstance(msg, dict) or msg.get("kind") != "state":
        return None
    linear, turn, stop = msg.get("linear"), msg.get("turn"), msg.get("stop")
    if (type(linear) is not int or linear not in (-1, 0, 1) or
            type(turn) is not int or turn not in (-1, 0, 1) or
            type(stop) is not bool):
        return None
    return linear, turn, stop


def jog_directions(keys):
    """Per-axis -1/0/+1 from the held or pulsed key set."""
    positive = set()
    negative = set()
    for key in keys:
        for digit in COMBO_KEYS.get(key, key):
            axis, direction = JOG_KEYS[digit]
            (positive if direction > 0 else negative).add(axis)
    return tuple(int(axis in positive) - int(axis in negative) for axis in range(3))


class Arm:
    """Local joint state advanced by checked joint moves."""

    def __init__(self):
        self.current = list(HOME_JOINTS)
        self.synced = False

    def sync_to(self, joints):
        self.current = list(joints)
        self.synced = True

    def home(self):
        self.sync_to(HOME_JOINTS)

    def jog(self, axis, direction, step=None):
        if step is None:
            step = BASE_YAW_STEP if axis == 0 else JOINT_STEP
        deltas = [0.0] * len(JOINT_LIMITS)
        deltas[axis] = direction * step
        return self.jog_axes(deltas)

    def jog_axes(self, deltas, tip_step=None):
        """Advance the requested joints together, staying within the arm limits."""
        if not self.synced:
            return None
        start = self.current
        goal = []
        limit_reason = None
        for value, delta, (lo, hi), name in zip(start, deltas, JOINT_LIMITS, JOINT_NAMES):
            requested = value + delta
            bounded = max(lo, min(hi, requested))
            goal.append(bounded)
            if bounded != requested:
                limit_reason = "%s joint limit (%.1f..%.1f deg)" % (name, lo, hi)
        if goal == start:
            return False, limit_reason

        def joints_at(fraction):
            return [a + (b - a) * fraction for a, b in zip(start, goal)]

        fraction = 1.0
        if tip_step is not None:
            origin = fk(*start)
            if distance(origin, fk(*goal)) > tip_step:
                fraction = furthest_valid(
                    0.0, 1.0, lambda f: distance(origin, fk(*joints_at(f))) <= tip_step
                )
        trial = joints_at(fraction)
        reason = check_pose(trial)
        if reason:
            allowed = furthest_valid(0.0, fraction, lambda f: check_pose(joints_at(f)) is None)
            trial = joints_at(allowed)
            if max(abs(a - b) for a, b in zip(start, trial)) < MIN_MOVE:
                return False, reason
        if trial == start:
            return False, limit_reason
        self.current = trial
        return True, reason or limit_reason


class JogController:
    """Issue one joint target from fresh keyboard input."""

    def __init__(self, arm):
        self.arm = arm
        self.last_step = -math.inf
        self.stop()

    def stop(self):
        self.directions = (0, 0, 0)
        self.last_input = -math.inf
        self.speed = 0.0
        self.blocked = False
        self.blocked_at = -math.inf
        self.last_limit = None

    def press(self, keys, now):
        directions = jog_directions(keys)
        changed = directions != self.directions
        if changed:
            # Only a sign reversal needs a fresh ramp. Adding or dropping a
            # key must keep the current speed, otherwise quick multi-key
            # presses reset the ramp every tick and the arm crawls or stalls.
            reversal = any(old and new and old != new
                           for old, new in zip(self.directions, directions))
            if reversal:
                self.stop()
            else:
                self.blocked = False
                self.blocked_at = -math.inf
            self.directions = directions
        if not any(directions):
            return None
        if self.blocked:
            # A held key retries after a short pause instead of latching
            # forever when one combined pose was rejected.
            if now - self.blocked_at < BLOCK_RETRY:
                return None
            self.blocked = False

        restarted = now - self.last_input >= JOG_RESTART_GAP
        self.last_input = now
        elapsed = now - self.last_step
        # The control loop calls press() every tick, so allow a little jitter
        # below JOG_PERIOD instead of dropping every other step.
        if not changed and elapsed < JOG_PERIOD * 0.9:
            return None
        if restarted:
            self.speed = 0.0
        interval = JOG_PERIOD if restarted else min(elapsed, JOG_MAX_INTERVAL)
        return self._move(now, interval)

    def _move(self, now, interval):
        joints = self.arm.current
        x, y, _ = fk(*joints)
        radii = (max(1.0, math.hypot(x, y)), MATH_LOWER_ARM, MATH_UPPER_ARM)
        yaw_step = min(BASE_YAW_STEP, BASE_YAW_SPEED * interval)
        max_steps = (yaw_step, JOINT_STEP, JOINT_STEP)
        self.speed = min(ARM_JOG_SPEED, self.speed + JOG_ACCEL * interval)
        deltas = []
        for axis, (direction, radius, max_step) in enumerate(zip(self.directions, radii, max_steps)):
            end = JOINT_LIMITS[axis][int(direction > 0)]
            remaining = radius * math.radians(abs(end - joints[axis]))
            speed = min(self.speed, math.sqrt(2 * JOG_ACCEL * remaining)) if remaining else self.speed
            deltas.append(direction * min(max_step, math.degrees(speed * interval / radius)))

        result = self.arm.jog_axes(deltas, tip_step=self.speed * interval)
        if result is None:
            return None
        moved, reason = result
        if moved:
            self.last_step = now
        else:
            self.blocked = True
            self.blocked_at = now
        repeated_limit = reason == self.last_limit
        self.last_limit = reason
        return moved, None if repeated_limit else reason


def vertical_joints(x, y, z):
    joints = ik(x, y, z)
    return joints if joints is not None and check_pose(joints) is None else None


class VerticalLift:
    """Raise the tip through a checked vertical path at its pickup X and Y."""

    def __init__(self, arm):
        self.arm = arm
        self.stop()

    def stop(self):
        self.next_step = None

    def start(self, now):
        self.x, self.y, self.height = fk(*self.arm.current)
        self.goal = self.height
        self.speed = 0.0
        while self.goal < Z_LIMITS[1]:
            candidate = min(self.goal + 1.0, Z_LIMITS[1])
            if vertical_joints(self.x, self.y, candidate) is None:
                self.goal = furthest_valid(
                    self.goal, candidate, lambda z: vertical_joints(self.x, self.y, z) is not None
                )
                break
            self.goal = candidate
        if self.goal - self.height <= 0.05:
            return False
        self.next_step = now + PUMP_LIFT_DELAY
        return True

    def step(self, now):
        remaining = self.goal - self.height
        self.speed = min(ARM_JOG_SPEED, self.speed + JOG_ACCEL * JOG_PERIOD,
                         math.sqrt(2 * JOG_ACCEL * remaining))
        height = min(self.goal, self.height + self.speed * JOG_PERIOD)
        joints = vertical_joints(self.x, self.y, height)
        if joints is None:
            self.stop()
            return None
        self.arm.sync_to(joints)
        self.height = height
        if height >= self.goal:
            self.stop()
        else:
            self.next_step = now + JOG_PERIOD
        return fk(*joints)


class PickupAbort(Exception):
    """A pickup stage hit an invalid pose or stopped making progress."""


class PickupRoutine:
    """Non-blocking automatic block pickup, driven from the 20 Hz loop.

    Stages: APPROACH (joint move above the block) -> DESCEND (vertical to the
    contact height) -> SUCTION (pump command + dwell) -> LIFT (vertical clear
    of the block) -> CARRY (joint move to the carry pose). ``step()`` returns
    the pose to publish when the arm moved; it never sleeps.

    Before the first motion every waypoint goes through ik() and check_pose();
    if any is unusable the routine refuses to start. While moving,
    Arm.jog_axes()/vertical_joints() re-check every intermediate pose and a
    rejected step aborts the routine at its last valid pose.

    There is no suction feedback on this robot: /pump_topic commands the
    valve, and SwiftproState.pump only echoes that command. "COMPLETE" means
    the sequence finished with the pump commanded ON - not that the block is
    attached.
    """

    APPROACH, DESCEND, SUCTION, LIFT, CARRY, DONE = range(6)
    NAMES = ("approach", "descend", "suction", "lift", "carry", "done")

    def __init__(self, arm, pump_command):
        self.arm = arm
        self.pump_command = pump_command
        self.active = False
        self.stage = None
        self.stage_at = 0.0
        self.speed = 0.0
        self.plan = {}
        self.x = self.y = 0.0
        self.contact_z = 0.0
        self.lift_z = 0.0

    @staticmethod
    def _checked_joints(pose):
        """ik() + check_pose() for one Cartesian waypoint."""
        joints = ik(*pose)
        if joints is None:
            return None, "IK has no solution"
        reason = check_pose(joints)
        if reason is not None:
            return None, reason
        return tuple(joints), None

    def start(self, now):
        if self.active:
            return False
        if PICKUP_X_MM is None or PICKUP_Y_MM is None:
            safe_print(
                "pickup needs PICKUP_X_MM and PICKUP_Y_MM (the block position "
                "in arm coordinates). Jog over the block, read "
                "/SwiftproState_topic, restart with those values. Nothing was "
                "commanded."
            )
            return False
        if not self.arm.synced:
            safe_print("pickup needs a known arm pose - press 0 once, then 1.")
            return False

        x, y = PICKUP_X_MM, PICKUP_Y_MM
        contact_z = PICKUP_CONTACT_Z_MM
        lift_z = contact_z + LIFT_HEIGHT_MM
        carry = (
            x if CARRY_X_MM is None else CARRY_X_MM,
            y if CARRY_Y_MM is None else CARRY_Y_MM,
            lift_z if CARRY_Z_MM is None else CARRY_Z_MM,
        )
        waypoints = (
            ("approach", (x, y, contact_z + APPROACH_HEIGHT_MM)),
            ("contact", (x, y, contact_z)),
            ("lift", (x, y, lift_z)),
            ("carry", carry),
        )
        plan = {}
        for name, pose in waypoints:
            joints, reason = self._checked_joints(pose)
            if joints is None:
                safe_print(
                    "pickup refused: %s pose (%.1f, %.1f, %.1f) rejected: %s"
                    % ((name,) + pose + (reason,))
                )
                return False
            plan[name] = joints

        self.plan = plan
        self.x, self.y = x, y
        self.contact_z, self.lift_z = contact_z, lift_z
        self.speed = 0.0
        self.stage = self.APPROACH
        self.stage_at = now
        self.active = True
        safe_print(
            "PICKUP START  block %.0f mm  contact wrist z=%.1f  approach z=%.1f"
            % (BLOCK_HEIGHT_MM, contact_z, contact_z + APPROACH_HEIGHT_MM)
        )
        return True

    def cancel(self, reason):
        """Stop issuing motion; the pump is left exactly as it is."""
        if not self.active:
            return False
        self.active = False
        self.stage = None
        safe_print(
            "pickup canceled (%s) - arm holds its last pose, pump unchanged"
            % reason
        )
        return True

    def _abort(self, reason):
        self.active = False
        self.stage = None
        safe_print(
            "PICKUP ABORTED: %s - arm holds its last valid pose, pump unchanged"
            % reason
        )

    def _enter(self, stage, now):
        self.stage = stage
        self.stage_at = now
        self.speed = 0.0

    def step(self, now, dt):
        """Advance one tick; return the pose to publish, or None."""
        if not self.active:
            return None
        if (
            self.stage != self.SUCTION
            and now - self.stage_at > PICKUP_STAGE_TIMEOUT_S
        ):
            self._abort("stage '%s' timed out" % self.NAMES[self.stage])
            return None
        try:
            if self.stage == self.APPROACH:
                if self._joint_move(self.plan["approach"], dt):
                    return fk(*self.arm.current)
                self._enter(self.DESCEND, now)
                safe_print("pickup: above the block, descending")
            elif self.stage == self.DESCEND:
                if self._vertical_move(self.contact_z, dt):
                    return fk(*self.arm.current)
                self._enter(self.SUCTION, now)
                self.pump_command(True)
                safe_print(
                    "pickup: at contact z=%.1f - PUMP ON, waiting %.2f s "
                    "(no suction feedback exists)" % (self.contact_z, SUCTION_DELAY_S)
                )
            elif self.stage == self.SUCTION:
                if now - self.stage_at < SUCTION_DELAY_S:
                    return None
                self._enter(self.LIFT, now)
                safe_print("pickup: lifting")
            elif self.stage == self.LIFT:
                if self._vertical_move(self.lift_z, dt):
                    return fk(*self.arm.current)
                self._enter(self.CARRY, now)
                safe_print("pickup: clear of the block, moving to the carry pose")
            elif self.stage == self.CARRY:
                if self._joint_move(self.plan["carry"], dt):
                    return fk(*self.arm.current)
                self.active = False
                self.stage = self.DONE
                safe_print(
                    "PICKUP COMPLETE - pump stays ON; suction is NOT verified "
                    "(no pressure sensor on this robot)."
                )
        except PickupAbort as exc:
            self._abort(str(exc))
        return None

    def _joint_move(self, target, dt):
        """Move all three joints toward target; True while still moving."""
        error = max(abs(a - b) for a, b in zip(self.arm.current, target))
        if error <= 0.1:
            return False
        remaining = distance(fk(*self.arm.current), fk(*target))
        self.speed = min(
            PICKUP_SPEED_MM_S,
            self.speed + JOG_ACCEL * dt,
            math.sqrt(2.0 * JOG_ACCEL * max(remaining, 0.0)),
        )
        deltas = [b - a for a, b in zip(self.arm.current, target)]
        result = self.arm.jog_axes(deltas, tip_step=max(self.speed * dt, 0.5))
        if result is None:
            raise PickupAbort("arm is not synced")
        moved, reason = result
        if reason:
            raise PickupAbort(reason)
        if not moved and max(
            abs(a - b) for a, b in zip(self.arm.current, target)
        ) > 0.1:
            raise PickupAbort("no progress toward the target joints")
        return moved

    def _vertical_move(self, goal_z, dt):
        """Vertical Cartesian path at the pickup X/Y; True while moving."""
        current_z = fk(*self.arm.current)[2]
        remaining = abs(goal_z - current_z)
        if remaining <= PICKUP_TOL_MM:
            return False
        direction = 1.0 if goal_z > current_z else -1.0
        self.speed = min(
            PICKUP_SPEED_MM_S,
            self.speed + JOG_ACCEL * dt,
            math.sqrt(2.0 * JOG_ACCEL * remaining),
        )
        step = min(max(self.speed * dt, 0.5), remaining)
        next_z = current_z + direction * step
        joints = vertical_joints(self.x, self.y, next_z)
        if joints is None:
            raise PickupAbort(
                "vertical path blocked at z=%.1f (joint/kinematic limit)" % next_z
            )
        self.arm.sync_to(joints)
        return True


class TerminalKeys:
    """Decode plain keys and Kitty press, repeat and release messages."""

    def __init__(self):
        self.pending = b""

    def feed(self, data):
        data = self.pending + data
        self.pending = b""
        events = []
        index = 0
        while index < len(data):
            byte = data[index]
            index += 1
            event = 0
            if byte != 27:
                key = chr(byte).lower()
            elif index == len(data):
                self.pending = data[index - 1:]
                break
            elif data[index] == ord("["):
                end = CSI_END.search(data, index + 1)
                if end is None:
                    self.pending = data[index - 1:] if len(data) - index < 64 else b""
                    break
                sequence = data[index + 1:end.end()]
                index = end.end()
                if sequence == b"O":  # terminal lost focus
                    events.append((None, 3))
                    continue
                match = KITTY_KEY.fullmatch(sequence)
                if match is None:
                    continue
                code, mods, event = (int(value or b"1") for value in match.groups())
                if 57399 <= code <= 57408:
                    code = ord("0") + code - 57399
                if code == 57414:
                    code = 13
                key = chr(code).lower() if code < 128 else ""
                if key == "c" and (mods - 1) & 4:
                    key = "\x03"
                elif (mods - 1) & 6:
                    continue
            elif data[index] == ord("O"):
                if index + 1 == len(data):
                    self.pending = data[index - 1:]
                    break
                keypad = data[index + 1]
                index += 2
                key = chr(ord("0") + keypad - ord("p")) if ord("p") <= keypad <= ord("y") else ""
                if keypad == ord("M"):
                    key = "\r"
            else:
                index += 1
                continue
            if key and key in ACCEPTED_KEYS:
                events.append((key, event))
        return events


class HeldKeys:
    """Track which keys count as held, from Kitty events or plain bytes.

    Motion is driven from the control loop while a key is live, so a press
    starts moving on the next tick instead of waiting for the terminal's
    key-repeat delay. Kitty press/release events give exact edges, which is
    what lets an arm key and a base key stay held together; a plain terminal
    only repeats the most recently pressed key, so a byte renews a hold that
    is long enough to bridge the gaps between repeats.
    """

    def __init__(self, plain_timeout=PLAIN_HOLD_TIMEOUT, repeat_timeout=None,
                 tap_min=TAP_MIN_DURATION):
        self.plain_timeout = plain_timeout
        self.repeat_timeout = repeat_timeout
        self.tap_min = tap_min
        self.until = {}
        self.started = {}
        self.last_plain = {}

    def __bool__(self):
        return bool(self.until)

    def clear(self):
        self.until.clear()
        self.started.clear()
        self.last_plain.clear()

    def update(self, key, event, now):
        if event == 3:  # Kitty release
            started = self.started.pop(key, now)
            # A quick press+release in one read still moves briefly.
            deadline = max(now, started + self.tap_min)
            if deadline > now:
                self.until[key] = deadline
            else:
                self.until.pop(key, None)
        elif event == 1:  # Kitty press
            self.started[key] = now
            self.until[key] = now + HELD_MAX_DURATION
        elif event == 2:  # Kitty repeat refreshes the safety deadline
            if key in self.started:
                self.until[key] = now + HELD_MAX_DURATION
        else:  # plain byte: no release information
            if self.repeat_timeout is None:
                self.until[key] = now + self.plain_timeout
            else:
                prior = self.last_plain.get(key)
                self.last_plain[key] = now
                fresh = prior is None or now - prior > self.plain_timeout
                self.until[key] = now + (self.plain_timeout if fresh
                                         else self.repeat_timeout)

    def live(self, now):
        """Return the keys still held and drop the expired ones."""
        for key in [k for k, until in self.until.items() if until <= now]:
            del self.until[key]
            self.started.pop(key, None)
            self.last_plain.pop(key, None)
        return set(self.until)


class ArmStateReader:
    """Initialize local joint state from /SwiftproState_topic if not yet synced."""

    def __init__(self, arm):
        self.arm = arm
        self.done = False
        self.lock = threading.Lock()

    def finish(self):
        with self.lock:
            self.done = True

    def callback(self, msg):
        if self.done or self.arm.synced:
            return
        if abs(msg.x) <= 1.0 and abs(msg.y) <= 1.0 and abs(msg.z) <= 1.0:
            return
        joints = validated_pose_joints(msg.x, msg.y, msg.z)
        if joints is None:
            return
        with self.lock:
            if self.done or self.arm.synced:
                return
            self.arm.sync_to(joints)
            self.done = True
        safe_print(
            "arm synced from /SwiftproState_topic: a0=%.1f a1=%.1f a2=%.1f"
            % tuple(joints)
        )


def main():
    rospy.init_node("key_fullbody", anonymous=True, argv=[sys.argv[0]])

    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    pos_pub = rospy.Publisher("/position_write_topic", position, queue_size=1, tcp_nodelay=True)
    pump_pub = rospy.Publisher("/pump_topic", status, queue_size=1)
    status_pub = rospy.Publisher("/swiftpro_status_topic", status, queue_size=1)

    # Optional browser bridge for exact base key-down/key-up. If the port is
    # already in use (another teleop), keep running with the terminal fallback.
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((BASE_HOST, BASE_PORT))
        listener.listen(1)
        listener.setblocking(False)
    except OSError as exc:
        listener.close()
        listener = None
        safe_print(
            "WARNING: base bridge port %d unavailable (%s); terminal fallback only."
            % (BASE_PORT, exc)
        )

    arm = Arm()
    saved = load_saved_pose()
    if saved is not None:
        arm.sync_to(saved)
        print(
            "resumed from saved pose: x=%.1f y=%.1f z=%.1f (a0=%.1f a1=%.1f a2=%.1f)"
            % (fk(*saved) + tuple(saved))
        )

    state_reader = ArmStateReader(arm)
    rospy.Subscriber("/SwiftproState_topic", SwiftproState, state_reader.callback, queue_size=1)
    rospy.sleep(0.5)

    def wait_for(publisher, topic, hint, timeout=2.0):
        deadline = time.monotonic() + timeout
        while (
            publisher.get_num_connections() == 0
            and time.monotonic() < deadline
            and not rospy.is_shutdown()
        ):
            rospy.sleep(0.1)
        if publisher.get_num_connections() == 0:
            print("WARNING: no subscriber on %s." % topic)
            print("         %s" % hint)

    wait_for(
        cmd_pub,
        "/cmd_vel",
        "Start the base driver: roslaunch spark_bringup driver_bringup.launch "
        "camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6",
    )
    wait_for(
        pos_pub,
        "/position_write_topic",
        "Start the arm driver: roslaunch swiftpro pro_control_nomoveit.launch\n"
        "         Arm keys and the pump will not work until it is running.",
    )
    wait_for(pump_pub, "/pump_topic", "The arm driver (swiftpro) also provides the vacuum pump.")
    wait_for(
        status_pub,
        "/swiftpro_status_topic",
        "The arm driver (swiftpro) also handles motor attach.",
    )

    def attach_arm():
        """Send 0 -> 1 on /swiftpro_status_topic to detach then attach arm motors."""
        if status_pub.get_num_connections() == 0:
            safe_print("no subscriber on /swiftpro_status_topic - cannot attach arm")
            return
        safe_print("attaching arm motors (0 -> 1 on /swiftpro_status_topic)...")
        m0 = status()
        m0.status = 0
        status_pub.publish(m0)
        rospy.sleep(1.0)
        m1 = status()
        m1.status = 1
        status_pub.publish(m1)
        rospy.sleep(1.0)
        safe_print("  arm attached - motors enabled")

    attach_arm()

    if not arm.synced:
        print("")
        print("  ARM NOT SYNCED - arm keys are ignored until the pose is known.")
        print("  Establish the starting pose with one home command:")
        print("      press 0 once  ->  release the pump and send the arm to home")
        print("  After that the pose is saved and later runs resume automatically.")
        print("")

    def set_pump(on):
        msg = status()
        msg.status = 1 if on else 0
        pump_pub.publish(msg)

    def set_base(linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        cmd_pub.publish(twist)

    def publish_pose(x, y, z):
        msg = position()
        msg.x, msg.y, msg.z = x, y, z
        pos_pub.publish(msg)

    print(HELP % ARM_JOG_SPEED)
    print(
        "  base: %.2f m/s / %.2f rad/s" % (LIN_SPEED, ANG_SPEED)
    )
    if listener is not None:
        print(
            "  base bridge: keyboard_web.py -> 127.0.0.1:%d "
            "(hold = move for exactly that long)" % BASE_PORT
        )
    else:
        print("  base bridge: disabled (port busy); terminal repeat fallback only")
    print("  arm and pump keys remain terminal-only; the bridge drives the base.")
    if PICKUP_X_MM is None or PICKUP_Y_MM is None:
        print(
            "  pickup (1): NOT configured - measure the block and set "
            "PICKUP_X_MM and PICKUP_Y_MM"
        )
    else:
        print(
            "  pickup (1): block x=%.0f y=%.0f  contact wrist z=%.1f "
            "(floor %.0f + block %.0f + pad %.0f - press %.0f)"
            % (
                PICKUP_X_MM,
                PICKUP_Y_MM,
                PICKUP_CONTACT_Z_MM,
                FLOOR_Z_MM,
                BLOCK_HEIGHT_MM,
                SUCTION_PAD_MM,
                SUCTION_PRESS_MM,
            )
        )
    print(
        "  arm limits: base %.1f..%.1f  shoulder %.0f..%.0f  elbow %.0f..%.0f"
        % (A0_LIMITS + A1_LIMITS + A2_LIMITS)
    )
    print(
        "              shoulder+elbow <= %.0f   reach <= %.0f mm   z %.0f..%.0f mm"
        % (FOLD_MAX, MAX_REACH, Z_LIMITS[0], Z_LIMITS[1])
    )
    if _arm_ik_reachable is not None:
        print("              arm_ik.ik_reachable() is also checked")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    target_linear = target_turn = 0.0
    actual_linear = actual_turn = 0.0
    term_last_action = {}
    term_last_movement = 0.0
    term_interlock = False
    pump_on = False
    sync_warned = False
    dirty_pose = False
    last_target = 0.0
    jog = JogController(arm)
    lift = VerticalLift(arm)
    keyboard = TerminalKeys()
    held_keys = HeldKeys()
    # Base keys use the same press/release tracking, so a/d/w/s can be held
    # while arm keys are held. w/s keep the long forward pulse; a/d use a
    # short pulse so one tap only rotates a few degrees.
    move_keys = HeldKeys(
        plain_timeout=TERM_FIRST_REPEAT,
        repeat_timeout=TERM_REPEAT_TIMEOUT,
        tap_min=TERM_FIRST_REPEAT,
    )
    turn_keys = HeldKeys(
        plain_timeout=TURN_HOLD_TIMEOUT,
        repeat_timeout=TERM_REPEAT_TIMEOUT,
        tap_min=TURN_TAP_MIN,
    )

    # Browser bridge connection state.
    client = None
    buffer = b""
    net_linear = net_turn = 0
    net_stop = False
    net_last = 0.0

    def stop_base_now():
        """Zero every base target, including the browser's, and publish stop."""
        nonlocal target_linear, target_turn, actual_linear, actual_turn
        nonlocal net_linear, net_turn
        target_linear = target_turn = 0.0
        actual_linear = actual_turn = 0.0
        net_linear = net_turn = 0
        set_base(0.0, 0.0)

    def close_base_client(message):
        """Drop the browser stream, stop the base, re-interlock the terminal."""
        nonlocal client, buffer, net_linear, net_turn, net_stop, net_last
        nonlocal term_interlock, term_last_movement
        if client is not None:
            try:
                client.close()
            except OSError:
                pass
            client = None
        buffer = b""
        net_linear = net_turn = 0
        net_stop = False
        net_last = 0.0
        stop_base_now()
        # Terminal base keys must go quiet before they can move the base again.
        term_interlock = True
        term_last_movement = time.monotonic()
        if message:
            safe_print(message)

    def pump_command(on):
        """Pump command that also keeps the terminal Enter/0 state in sync."""
        nonlocal pump_on
        pump_on = on
        set_pump(on)

    pickup = PickupRoutine(arm, pump_command)
    # Plain terminals have no key-up: a held 1 that repeats must not restart
    # the routine after it finishes. The latch clears after TERM_ACTION_GAP of
    # silence (or immediately on a Kitty release).
    pickup_key_seen = -100.0
    pickup_wait_release = False

    last_loop = time.monotonic()
    rate = rospy.Rate(1.0 / JOG_PERIOD)

    try:
        tty.setraw(fd)
        # Throw away keys typed during startup/attach instead of replaying
        # them all as one burst when the control loop begins.
        termios.tcflush(fd, termios.TCIFLUSH)
        if sys.stdout.isatty():
            sys.stdout.write("\x1b[>11u\x1b[?1004h\r\n")
            sys.stdout.flush()
        set_base(0.0, 0.0)

        while not rospy.is_shutdown():
            now = time.monotonic()
            dt = min(max(now - last_loop, 0.0), 0.1)
            last_loop = now

            # Re-arm pickup key 1 once the terminal has been quiet, so the
            # repeats of a held key cannot start a second routine.
            if pickup_wait_release and now - pickup_key_seen > TERM_ACTION_GAP:
                pickup_wait_release = False

            watched = [sys.stdin]
            if listener is not None:
                watched.append(listener)
            if client is not None:
                watched.append(client)
            ready, _, _ = select.select(watched, [], [], 0)
            if sys.stdin in ready:
                data = os.read(fd, 4096)
                if not data:
                    break
                events = keyboard.feed(data)
                if b"\x03" in data or any(
                    key == "\x03" and event != 3 for key, event in events
                ):
                    break

                for key, event in events:
                    if key is None:  # terminal lost focus
                        held_keys.clear()
                        move_keys.clear()
                        turn_keys.clear()
                        jog.stop()
                        continue

                    if key in JOG_KEYS or key in COMBO_KEYS:
                        lift.stop()
                        pickup.cancel("manual arm input")
                        term_last_movement = now
                        held_keys.update(key, event, now)
                        continue

                    if key in BASE_KEYS:
                        term_last_movement = now
                        pickup.cancel("base input")
                        if client is not None:
                            continue  # the browser bridge owns the base
                        if key in "ws":
                            move_keys.update(key, event, now)
                        else:
                            turn_keys.update(key, event, now)
                        continue

                    if key == "1":
                        pickup_key_seen = now
                        if event == 3:  # Kitty release
                            pickup_wait_release = False
                        elif not pickup_wait_release:
                            pickup_wait_release = True
                            if not pickup.active:
                                lift.stop()
                                jog.stop()
                                held_keys.clear()
                                state_reader.finish()
                                if (pos_pub.get_num_connections() == 0
                                        or pump_pub.get_num_connections() == 0):
                                    safe_print(
                                        "pickup needs the arm driver "
                                        "(/position_write_topic and /pump_topic); "
                                        "nothing was commanded."
                                    )
                                else:
                                    pickup.start(now)
                        continue

                    if event not in (0, 1):
                        continue

                    was_lifting = lift.next_step is not None
                    lift.stop()
                    jog.stop()
                    held_keys.clear()

                    if key == ATTACH_KEY:
                        prior_action = term_last_action.get(ATTACH_KEY, -100.0)
                        term_last_action[ATTACH_KEY] = now
                        if now - prior_action >= TERM_ACTION_GAP:
                            pickup.cancel("attach key")
                            attach_arm()
                    elif key == " ":
                        pickup.cancel("space")
                        if was_lifting:
                            safe_print(
                                "Lift canceled. Pump stays %s."
                                % ("ON" if pump_on else "OFF")
                            )
                    elif key in ("\r", "\n"):
                        prior_action = term_last_action.get("enter", -100.0)
                        term_last_action["enter"] = now
                        if now - prior_action < TERM_ACTION_GAP:
                            continue
                        pickup.cancel("pump key")
                        if not pump_on and pump_pub.get_num_connections() == 0:
                            safe_print("Pump driver unavailable.")
                            continue
                        pump_on = not pump_on
                        set_pump(pump_on)
                        safe_print("PUMP ON" if pump_on else "PUMP OFF")
                        if pump_on:
                            if not arm.synced or pos_pub.get_num_connections() == 0:
                                safe_print(
                                    "Lift needs a known arm pose and a connected driver."
                                )
                            else:
                                state_reader.finish()
                                if lift.start(now):
                                    safe_print(
                                        "Lifting straight up to z=%.1f mm." % lift.goal
                                    )
                                else:
                                    safe_print(
                                        "Already at the highest reachable point here."
                                    )
                    elif key == "0":
                        prior_action = term_last_action.get("0", -100.0)
                        term_last_action["0"] = now
                        if now - prior_action < TERM_ACTION_GAP:
                            continue
                        pickup.cancel("0 key")
                        state_reader.finish()
                        pump_on = False
                        set_pump(False)
                        target_linear = target_turn = 0.0
                        actual_linear = actual_turn = 0.0
                        set_base(0.0, 0.0)
                        move_keys.clear()
                        turn_keys.clear()
                        term_interlock = True
                        term_last_movement = now
                        if pos_pub.get_num_connections() == 0:
                            safe_print("RELEASE - arm driver not running, home not sent")
                        else:
                            arm.home()
                            x, y, z = fk(*arm.current)
                            publish_pose(x, y, z)
                            save_pose(x, y, z)
                            dirty_pose = False
                            safe_print(
                                "RELEASE + HOME  (a0=%.1f a1=%.1f a2=%.1f)"
                                % tuple(arm.current)
                            )

            # Browser bridge: keyboard_web.py supplies real key-down/key-up
            # states, so the base moves for exactly as long as a key is held.
            if listener is not None and listener in ready:
                try:
                    new_client, _ = listener.accept()
                except OSError:
                    new_client = None
                if new_client is not None:
                    if client is not None:
                        new_client.close()
                    else:
                        client = new_client
                        client.setblocking(False)
                        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                        buffer = b""
                        net_last = 0.0
                        term_interlock = True
                        term_last_movement = now
                        move_keys.clear()
                        turn_keys.clear()
                        stop_base_now()
                        safe_print(
                            "browser base keyboard connected on %s:%d"
                            % (BASE_HOST, BASE_PORT)
                        )

            if client is not None and client in ready:
                disconnected = False
                try:
                    data = client.recv(8192)
                    if not data:
                        disconnected = True
                    else:
                        buffer += data
                        if len(buffer) > 8192:
                            disconnected = True
                        else:
                            *packets, buffer = buffer.split(b"\n")
                            for raw in packets:
                                state = parse_base_state(raw)
                                if state is not None:
                                    net_linear, net_turn, net_stop = state
                                    net_last = time.monotonic()
                                    if pickup.active and (net_linear or net_turn):
                                        pickup.cancel("base input from the browser")
                except OSError:
                    disconnected = True
                if disconnected:
                    close_base_client("base bridge disconnected - base stopped")

            # Drive the arm every tick while a key is logically held, not only
            # when a key event arrives. Terminal key-repeat starts after a
            # delay (and Kitty taps can press+release in one read), so
            # event-driven jogging leaves dead presses and late bursts.
            jog_time = time.monotonic()
            active = held_keys.live(jog_time)
            # Releasing every arm key ends the jog cleanly, so a later press
            # of the same direction is not left latched in `blocked`.
            if not active and jog.directions != (0, 0, 0):
                jog.stop()
            if active and not pickup.active:
                lift.stop()
                if not term_interlock:
                    if arm.synced and pos_pub.get_num_connections() > 0:
                        state_reader.finish()
                        result = jog.press(active, jog_time)
                        if result is not None:
                            moved, reason = result
                            pose = fk(*arm.current)
                            if moved:
                                publish_pose(*pose)
                                dirty_pose = True
                                last_target = jog_time
                            if reason:
                                if len(active) == 1 and next(iter(active)) in REVERSE_KEY:
                                    key = next(iter(active))
                                    safe_print(
                                        ("arm limit on %s: %s. Try %s to move back. "
                                         "[x=%.0f y=%.0f z=%.0f]")
                                        % ((key, reason, REVERSE_KEY[key]) + pose)
                                    )
                                else:
                                    safe_print(
                                        ("arm limit: %s  [x=%.0f y=%.0f z=%.0f]")
                                        % ((reason,) + pose)
                                    )
                    elif not sync_warned:
                        safe_print("Arm motion needs a known pose and a connected driver.")
                        sync_warned = True

            # Automatic pickup state machine: one tick per loop, no sleeps.
            if pickup.active:
                pose = pickup.step(now, dt)
                if pose is not None:
                    publish_pose(*pose)
                    dirty_pose = True
                    last_target = now

            # Base target: the browser bridge drives the base from exact
            # key-up/key-down state. Only fall back to the terminal's
            # key-repeat timing when no bridge client is attached.
            if client is not None:
                if net_stop or now - net_last > BODY_WATCHDOG:
                    net_linear = net_turn = 0
                target_linear = net_linear * LIN_SPEED
                target_turn = net_turn * ANG_SPEED
            elif term_interlock:
                target_linear = target_turn = 0.0
            else:
                move_live = move_keys.live(jog_time)
                turn_live = turn_keys.live(jog_time)
                target_linear = (
                    (LIN_SPEED if "w" in move_live else 0.0)
                    - (LIN_SPEED if "s" in move_live else 0.0)
                )
                target_turn = (
                    (ANG_SPEED if "a" in turn_live else 0.0)
                    - (ANG_SPEED if "d" in turn_live else 0.0)
                )
            actual_linear = toward(
                actual_linear, target_linear, MAX_LIN_ACCEL, MAX_LIN_DECEL, dt
            )
            actual_turn = toward(
                actual_turn, target_turn, MAX_ANG_ACCEL, MAX_ANG_DECEL, dt
            )
            set_base(actual_linear, actual_turn)

            # Re-arm after 0 (or a bridge disconnect) once keys are quiet
            if (term_interlock and client is None
                    and now - term_last_movement > TERM_REARM_PAUSE):
                term_interlock = False
                safe_print("  keyboard rearmed - movement enabled")

            # Ramped vertical lift step
            if lift.next_step is not None and now >= lift.next_step:
                if pos_pub.get_num_connections() == 0:
                    lift.stop()
                    safe_print("Lift canceled. Arm driver disconnected.")
                else:
                    pose = lift.step(now)
                    if pose is None:
                        safe_print("Lift stopped at the last valid pose.")
                    else:
                        publish_pose(*pose)
                        dirty_pose = True
                        last_target = now
                        if lift.next_step is None:
                            safe_print("Lift complete at z=%.1f mm." % pose[2])

            if dirty_pose and now - last_target > 0.3:
                save_pose(*fk(*arm.current))
                dirty_pose = False

            rate.sleep()

    finally:
        if sys.stdout.isatty():
            sys.stdout.write("\x1b[<u\x1b[?1004l")
            sys.stdout.flush()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        if client is not None:
            try:
                client.close()
            except OSError:
                pass
            client = None
        if listener is not None:
            listener.close()
        for _ in range(5):
            set_base(0.0, 0.0)
            time.sleep(0.02)
        set_pump(False)
        if arm.synced:
            save_pose(*fk(*arm.current))
        rospy.sleep(0.1)
        set_pump(False)
        print("\nstopped - base stopped, pump off")


if __name__ == "__main__":
    main()
