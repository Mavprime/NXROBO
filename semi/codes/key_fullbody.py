#!/usr/bin/env python3
"""
Keyboard teleop for the Spark-H base + the 4-DOF uArm (base, shoulder, elbow).

Base (hold for continuous motion - terminal key repeat):
    w  forward          s  backward
    a  turn left        d  turn right

Arm (ramped joint jogging at a tip speed up to ARM_JOG_SPEED, plus combos):
    4  base rotate left          6  base rotate right
    7  raise upper arm           9  lower upper arm
    8  extend forearm            2  fold forearm
    q  7 + 8    e  7 + 2    z  9 + 8    c  9 + 2

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

Terminals that support Kitty key events let you hold a shoulder and elbow key
together, or an arm key and a base key, and the script sees the release. The
Q/E/Z/C shortcuts work in ordinary SSH terminals too. An SSH terminal sends
repeats without a release event, so base motion also stops after
TERM_REPEAT_TIMEOUT of silence.

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
    LIN_SPEED=0.35             m/s
    ANG_SPEED=1.2              rad/s
    MAX_LIN_ACCEL=2.0          m/s^2
    MAX_LIN_DECEL=4.0          m/s^2
    MAX_ANG_ACCEL=6.0          rad/s^2
    MAX_ANG_DECEL=10.0         rad/s^2
    TERM_FIRST_REPEAT=0.95     s a fresh key is held before release
    TERM_REPEAT_TIMEOUT=0.32   s between repeats before the base is released
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

import math
import os
import re
import select
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
LIN_SPEED = float(os.environ.get("LIN_SPEED", "0.35"))
ANG_SPEED = float(os.environ.get("ANG_SPEED", "1.2"))
MAX_LIN_ACCEL = float(os.environ.get("MAX_LIN_ACCEL", "2.0"))
MAX_LIN_DECEL = float(os.environ.get("MAX_LIN_DECEL", "4.0"))
MAX_ANG_ACCEL = float(os.environ.get("MAX_ANG_ACCEL", "6.0"))
MAX_ANG_DECEL = float(os.environ.get("MAX_ANG_DECEL", "10.0"))

TERM_FIRST_REPEAT = float(os.environ.get("TERM_FIRST_REPEAT", "0.95"))
TERM_REPEAT_TIMEOUT = float(os.environ.get("TERM_REPEAT_TIMEOUT", "0.32"))
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

PUMP_LIFT_DELAY = float(os.environ.get("PUMP_LIFT_DELAY", "0.35"))
if not (math.isfinite(PUMP_LIFT_DELAY) and PUMP_LIFT_DELAY >= 0.0):
    sys.exit("Pump lift delay must be a finite, nonnegative number of seconds.")

JOG_ACCEL = 1200.0              # mm/s^2
JOG_PERIOD = 0.05               # SwiftPro processes targets at 20 Hz
JOG_MAX_INTERVAL = 0.07         # cap stride when repeats arrive slowly
JOG_RESTART_GAP = 0.14          # reset acceleration between separate presses
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
CONTROL_KEYS = "\x03" + ATTACH_KEY + "0\r\n " + BASE_KEYS
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
  Base:  w forward   s backward   a turn left   d turn right   (hold)
  Arm:   4 base left   6 base right
         7 upper arm up   9 upper arm down
         8 extend forearm  2 fold forearm        (jog tip speed up to %g mm/s)
  Combo: q = 7+8   e = 7+2   z = 9+8   c = 9+2
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
        self.last_limit = None

    def press(self, keys, now):
        directions = jog_directions(keys)
        changed = directions != self.directions
        if changed:
            self.stop()
            self.directions = directions
        if not any(directions) or self.blocked:
            return None

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
    # while arm keys are held. A fresh plain byte coasts for TERM_FIRST_REPEAT
    # (as before) because the OS only repeats the last key pressed.
    base_keys = HeldKeys(
        plain_timeout=TERM_FIRST_REPEAT,
        repeat_timeout=TERM_REPEAT_TIMEOUT,
        tap_min=TERM_FIRST_REPEAT,
    )
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

            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready:
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
                        base_keys.clear()
                        jog.stop()
                        continue

                    if key in JOG_KEYS or key in COMBO_KEYS:
                        lift.stop()
                        term_last_movement = now
                        held_keys.update(key, event, now)
                        continue

                    if key in BASE_KEYS:
                        term_last_movement = now
                        base_keys.update(key, event, now)
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
                            attach_arm()
                    elif key == " ":
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
                        state_reader.finish()
                        pump_on = False
                        set_pump(False)
                        target_linear = target_turn = 0.0
                        actual_linear = actual_turn = 0.0
                        set_base(0.0, 0.0)
                        base_keys.clear()
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

            # Drive the arm every tick while a key is logically held, not only
            # when a key event arrives. Terminal key-repeat starts after a
            # delay (and Kitty taps can press+release in one read), so
            # event-driven jogging leaves dead presses and late bursts.
            jog_time = time.monotonic()
            active = held_keys.live(jog_time)
            if active:
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

            # Base target from every base key still held (computed each tick,
            # so arm and base keys can run together).
            base_live = base_keys.live(jog_time)
            if term_interlock:
                target_linear = target_turn = 0.0
            else:
                target_linear = (
                    (LIN_SPEED if "w" in base_live else 0.0)
                    - (LIN_SPEED if "s" in base_live else 0.0)
                )
                target_turn = (
                    (ANG_SPEED if "a" in base_live else 0.0)
                    - (ANG_SPEED if "d" in base_live else 0.0)
                )
            actual_linear = toward(
                actual_linear, target_linear, MAX_LIN_ACCEL, MAX_LIN_DECEL, dt
            )
            actual_turn = toward(
                actual_turn, target_turn, MAX_ANG_ACCEL, MAX_ANG_DECEL, dt
            )
            set_base(actual_linear, actual_turn)

            # Re-arm after 0 once movement keys have been quiet
            if term_interlock and now - term_last_movement > TERM_REARM_PAUSE:
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
