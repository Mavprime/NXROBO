
#!/usr/bin/env python3
"""
Arm-only keyboard teleop for the Spark-H uArm (base yaw, shoulder, elbow).

Arm keys:
    4  base rotate left          6  base rotate right
    7  raise upper arm           9  lower upper arm
    8  extend forearm            2  fold forearm
    Q  7 + 8    E  7 + 2    Z  9 + 8    C  9 + 2

Other:
    Enter  toggle the vacuum pump and lift straight up when switching it on
    Space  cancel the lift with the pump still on
    0      release the pump and send the arm to its home pose
    A      re-attach the arm motors
    Ctrl+C exit and switch the pump off

Run this in its own terminal alongside body.py; both connect to the same ROS
master. body.py handles w/a/s/d for the mobile base.

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
A pressed motion key is held as a direction, and the control loop advances the
tip every tick (20 Hz) at a ramped speed up to ARM_JOG_SPEED, easing near the
relevant joint limit. Motion starts immediately instead of waiting for the
terminal's key-repeat delay, and a quick tap still moves for TAP_MIN_DURATION.
Kitty terminals report the exact release; plain SSH terminals renew the hold
for PLAIN_HOLD_TIMEOUT on every repeat byte. Queued repeats are discarded
rather than replayed, and an idle loop never sends a jog.
Targets already accepted by the driver or firmware can finish moving. ROS
keeps the latest unsent target.

Terminals that support Kitty key events let you hold a shoulder and elbow
key together. The Q/E/Z/C shortcuts work in ordinary SSH terminals too.
Turning the pump on lifts as high as the arm can go at its current X and Y.
A movement key, Space, or turning the pump off cancels the remaining lift.

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

This script sends that 0 -> 1 pair on startup and again whenever you press A.
If the arm "accepts" keys but never moves, press A to re-attach.

The stock driver echoes commanded positions on /SwiftproState_topic. The
script saves its last commanded pose in .arm_last_pose (next to this file) and
resumes from it on the next run. On a first run without a valid starting pose,
press 0 once to home the arm and create the saved pose.

    python3 arm.py --status     # show connections + starting pose, no motion

Prerequisites:
    arm: roslaunch swiftpro pro_control_nomoveit.launch
         (or: roslaunch ~/semi/server.launch)

Usage:
    ssh spark@10.25.141.8
    source ~/spark_noetic/devel/setup.bash
    python3 ~/semi/arm.py
    python3 ~/semi/arm.py --check-kinematics   # no ROS needed
    python3 ~/semi/arm.py --status             # connections + pose, no motion

Environment overrides:
    ARM_JOG_SPEED=120    maximum commanded tip speed for jogs in mm/s
    PUMP_LIFT_DELAY=0.35 seconds to wait after turning the pump on
    JOINT_STEP=10        upper bound on a joint step in degrees
    BASE_YAW_STEP=6      additional upper bound on a yaw step in degrees
    BASE_YAW_SPEED=120   upper bound on commanded yaw speed in degrees/second
    BASE_YAW_MIN=5       set higher if this arm stops earlier at the low end
    BASE_YAW_MAX=175     set lower if this arm stops earlier at the high end
    MIN_ELBOW_ANGLE=10   keep the elbow at least this far from fully folded
    MAX_REACH=350        max radial distance of the tip from the base axis, mm

Topics:
    /position_write_topic swiftpro/position         arm x,y,z (mm)
    /pump_topic           swiftpro/status           vacuum 1/0
    /SwiftproState_topic  swiftpro/SwiftproState    last commanded pose

Safety:
The arm moves immediately when a command is published. Keep hands, cables and
the field clear. Press 0 before the first arm move so the script's joint state
is synced with the physical arm. If this process is SIGKILLed while the pump is
on, the vacuum stays on; use Ctrl+C or press 0.
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

# Arm model constants from the SwiftPro ROS driver.
MATH_L1 = 106.6
MATH_L2 = 13.2
MATH_LOWER_ARM = 142.07
MATH_UPPER_ARM = 158.81
MATH_Z_OFFSET = 74.55
MATH_BASE_OFFSET = MATH_L2 + 56.55

HOME = (120.0, 0.0, 35.0)

A0_LIMITS = (float(os.environ.get("BASE_YAW_MIN", "5")),
             float(os.environ.get("BASE_YAW_MAX", "175")))
if not (5.0 <= A0_LIMITS[0] <= 90.0 <= A0_LIMITS[1] <= 175.0
        and A0_LIMITS[0] < A0_LIMITS[1]):
    sys.exit("Base yaw limits must be within 5..175 degrees and include home (90 degrees).")

A1_LIMITS = (0.0, 135.6)
A2_LIMITS = (0.0, 100.7)
JOINT_LIMITS = (A0_LIMITS, A1_LIMITS, A2_LIMITS)
JOINT_NAMES = ("base", "shoulder", "elbow")

MAX_REACH = float(os.environ.get("MAX_REACH", "350"))
Z_LIMITS = (-60.0, 170.0)
MIN_ELBOW_ANGLE = float(os.environ.get("MIN_ELBOW_ANGLE", "10"))
FOLD_MAX = 180.0 - MIN_ELBOW_ANGLE

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

JOG_ACCEL = 1200.0              # mm/s²
JOG_PERIOD = 0.05               # SwiftPro processes targets at 20 Hz
JOG_MAX_INTERVAL = 0.07         # cap stride when repeats arrive slowly
JOG_RESTART_GAP = 0.14          # reset acceleration between separate presses
TAP_MIN_DURATION = 0.18         # s a quick tap keeps moving (never a dead press)
PLAIN_HOLD_TIMEOUT = 0.22       # s a byte keeps its key held (no release events)
HELD_MAX_DURATION = 5.0         # s safety cap if a release event never arrives
MIN_MOVE = 0.2                  # deg, ignore smaller partial moves
POSE_EPSILON = 1e-9

JOG_KEYS = {"4": (0, 1), "6": (0, -1), "7": (1, -1),
            "9": (1, 1), "8": (2, -1), "2": (2, 1)}
REVERSE_KEY = {"4": "6", "6": "4", "7": "9", "9": "7", "8": "2", "2": "8"}
COMBO_KEYS = {"q": "78", "e": "72", "z": "98", "c": "92"}
CONTROL_KEYS = "\x03a0\r\n "
KITTY_KEY = re.compile(rb"(\d+)(?::[\d:]*)?(?:;(\d+)(?::([123]))?)?(?:;[\d:]*)?u")
CSI_END = re.compile(rb"[\x40-\x7e]")


def fk(a0, a1, a2):
    """Motor angles in degrees to a Cartesian pose in mm."""
    r0, r1, r2 = math.radians(a0), math.radians(a1), math.radians(a2)
    stretch = (MATH_LOWER_ARM * math.cos(r1)
               + MATH_UPPER_ARM * math.cos(r2)
               + MATH_BASE_OFFSET)
    height = (MATH_LOWER_ARM * math.sin(r1)
              - MATH_UPPER_ARM * math.sin(r2)
              + MATH_L1)
    return (stretch * math.sin(r0),
            -stretch * math.cos(r0),
            height - MATH_Z_OFFSET)


def ik(x, y, z):
    """Cartesian pose in mm to motor angles in degrees, or None."""
    a0 = math.degrees(math.atan2(x, -y))
    stretch = math.hypot(x, y)
    x_in = stretch - MATH_BASE_OFFSET
    z_in = z + MATH_Z_OFFSET - MATH_L1
    d = math.hypot(x_in, z_in)
    if d < 1e-6:
        return None

    cos_delta = ((d * d - MATH_LOWER_ARM ** 2 - MATH_UPPER_ARM ** 2)
                 / (2.0 * MATH_LOWER_ARM * MATH_UPPER_ARM))
    if cos_delta < -1.0 - 1e-9 or cos_delta > 1.0 + 1e-9:
        return None

    delta = -math.acos(max(-1.0, min(1.0, cos_delta)))
    phi = math.atan2(z_in, x_in)
    a1 = phi - math.atan2(MATH_UPPER_ARM * math.sin(delta),
                          MATH_LOWER_ARM + MATH_UPPER_ARM * math.cos(delta))
    return (a0, math.degrees(a1), math.degrees(-(a1 + delta)))


HOME_JOINTS = ik(*HOME) or (90.0, 103.4, 58.4)


def check_pose(joints):
    """Return the reason a joint set is rejected, or None."""
    for value, (lo, hi), name in zip(joints, JOINT_LIMITS, JOINT_NAMES):
        if value < lo - POSE_EPSILON or value > hi + POSE_EPSILON:
            return "%s joint limit (%.1f..%.1f deg)" % (name, lo, hi)

    if joints[1] + joints[2] > FOLD_MAX + 1e-9:
        return ("elbow folded too far (shoulder+elbow %.0f > %.0f deg)"
                % (joints[1] + joints[2], FOLD_MAX))

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


def distance(a, b):
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(a, b)))


def validated_pose_joints(x, y, z):
    """Read a saved or echoed pose, allowing for XYZ rounding at joint limits."""
    joints = ik(x, y, z)
    if joints is None:
        return None

    bounded = tuple(max(lo, min(hi, value))
                    for value, (lo, hi) in zip(joints, JOINT_LIMITS))
    if joints != bounded:
        if distance((x, y, z), fk(*bounded)) <= 0.001:
            joints = bounded
    return joints if check_pose(joints) is None else None


STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          ".arm_last_pose")


def save_pose(x, y, z):
    """Remember the last commanded Cartesian pose."""
    try:
        tmp = STATE_FILE + ".tmp"
        with open(tmp, "w") as handle:
            handle.write("%.3f %.3f %.3f\n" % (x, y, z))
        os.replace(tmp, STATE_FILE)
    except OSError:
        pass


def load_saved_pose():
    """Read the joints corresponding to the last saved command."""
    try:
        with open(STATE_FILE) as handle:
            x, y, z = [float(value) for value in handle.read().split()]
    except (OSError, ValueError):
        return None
    return validated_pose_joints(x, y, z)


def _check_kinematics():
    """Check FK and IK round trips, the home pose, and the allowed joint range."""
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
        print("IK/FK %s -> (%0.2f, %0.2f, %0.2f) -> err %.4f mm"
              % (p, joints[0], joints[1], joints[2], err))
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
        print("FK/IK %s -> (%0.2f, %0.2f, %0.2f) -> err %.4f deg"
              % (a, b[0], b[1], b[2], err))
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
from swiftpro.msg import SwiftproState, position, status

HELP = """
  Arm:   4 base left   6 base right
         7 upper arm up   9 upper arm down
         8 extend forearm  2 fold forearm        (jog tip speed up to %g mm/s)
  Combine: Q = 7+8   E = 7+2   Z = 9+8   C = 9+2
  Enter: vacuum ON + lift / OFF    Space: cancel lift    0: release + home
  Ctrl+C: exit
  A:     re-attach arm motors (press if keys are accepted but the arm is still)
"""


def say(text):
    """Write a line while the terminal is in raw mode."""
    sys.stdout.write(text.replace("\n", "\r\n") + "\r\n")
    sys.stdout.flush()


class Arm:
    """Local state for the last commanded joint angles."""

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


def furthest_valid(lo, hi, allowed):
    """Find the boundary between an allowed point and a rejected point."""
    for _ in range(14):
        mid = (lo + hi) * 0.5
        if allowed(mid):
            lo = mid
        else:
            hi = mid
    return lo


def jog_directions(keys):
    positive = set()
    negative = set()
    for key in keys:
        for digit in COMBO_KEYS.get(key, key):
            axis, direction = JOG_KEYS[digit]
            (positive if direction > 0 else negative).add(axis)
    return tuple(int(axis in positive) - int(axis in negative) for axis in range(3))


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
            if key and (key in JOG_KEYS or key in COMBO_KEYS or key in CONTROL_KEYS):
                events.append((key, event))
        return events


class HeldKeys:
    """Track which keys count as held, from Kitty events or plain bytes.

    Motion is driven from the control loop while a key is live, so a press
    starts moving on the next tick instead of waiting for the terminal's
    key-repeat delay. Kitty press/release events give exact edges, which is
    what lets two keys stay held together; a plain terminal only repeats the
    most recently pressed key, so a byte renews a hold that is long enough to
    bridge the gaps between repeats.
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
    """Use one plausible driver echo to initialise the local joint state."""

    def __init__(self, arm):
        self.arm = arm
        self.live_used = False
        self.lock = threading.Lock()

    def finish(self):
        with self.lock:
            self.live_used = True

    def callback(self, msg):
        if self.live_used:
            return
        if abs(msg.x) <= 1.0 and abs(msg.y) <= 1.0 and abs(msg.z) <= 1.0:
            return
        joints = validated_pose_joints(msg.x, msg.y, msg.z)
        if joints is None:
            return
        with self.lock:
            if self.live_used:
                return
            self.arm.sync_to(joints)
            self.live_used = True
        print("arm synced from /SwiftproState_topic: a0=%.1f a1=%.1f a2=%.1f"
              % tuple(joints))


def _status_report():
    """Print what is connected and what pose the script would start from."""
    rospy.init_node("arm_status", anonymous=True, argv=[sys.argv[0]])
    pos_pub = rospy.Publisher("/position_write_topic", position, queue_size=1)
    pump_pub = rospy.Publisher("/pump_topic", status, queue_size=1)

    arm = Arm()
    saved = load_saved_pose()
    if saved is not None:
        arm.sync_to(saved)
    reader = ArmStateReader(arm)
    rospy.Subscriber("/SwiftproState_topic", SwiftproState, reader.callback, queue_size=1)
    rospy.sleep(2.0)

    print("position_write_topic subscribers: %d" % pos_pub.get_num_connections())
    print("pump_topic subscribers:           %d" % pump_pub.get_num_connections())
    if saved is not None:
        print("saved pose (last commanded): x=%.1f y=%.1f z=%.1f" % fk(*saved))
    else:
        print("saved pose (last commanded): none")
    if arm.synced:
        print("start pose:                  x=%.1f y=%.1f z=%.1f" % fk(*arm.current))
        print("ready - arm keys will work")
    else:
        print("start pose:                  unknown")
        print("press 0 once when you run arm.py to establish the starting pose")
    return 0


def main():
    if not sys.stdin.isatty():
        print("arm.py needs a real terminal (termios raw mode).")
        print("Run it in an SSH window:  run python3 <path>/arm.py")
        print("Do not start it from roslaunch, nohup, or with stdin redirected.")
        return 1

    rospy.init_node("arm", anonymous=True, argv=[sys.argv[0]])

    pos_pub = rospy.Publisher("/position_write_topic", position, queue_size=1, tcp_nodelay=True)
    pump_pub = rospy.Publisher("/pump_topic", status, queue_size=1)
    status_pub = rospy.Publisher("/swiftpro_status_topic", status, queue_size=1)

    arm = Arm()
    saved = load_saved_pose()
    if saved is not None:
        arm.sync_to(saved)
        print("resumed from saved pose: x=%.1f y=%.1f z=%.1f (a0=%.1f a1=%.1f a2=%.1f)"
              % (fk(*saved) + tuple(saved)))
    state_reader = ArmStateReader(arm)
    rospy.Subscriber("/SwiftproState_topic", SwiftproState, state_reader.callback, queue_size=1)
    rospy.sleep(0.5)

    def wait_for(publisher, topic, hint, timeout=2.0):
        deadline = time.time() + timeout
        while (publisher.get_num_connections() == 0
               and time.time() < deadline
               and not rospy.is_shutdown()):
            rospy.sleep(0.1)
        if publisher.get_num_connections() == 0:
            print("WARNING: no subscriber on %s." % topic)
            print("         %s" % hint)

    wait_for(pos_pub, "/position_write_topic",
             "Start the arm driver: roslaunch swiftpro pro_control_nomoveit.launch\n"
             "         Arm keys and the pump will not work until it is running.")
    wait_for(pump_pub, "/pump_topic",
             "The arm driver (swiftpro) also provides the vacuum pump.")

    def attach_arm():
        """Send the driver its detach and attach sequence."""
        say("attaching arm motors (0 -> 1 on /swiftpro_status_topic)...")
        status_pub.publish(0)
        rospy.sleep(1.0)
        status_pub.publish(1)
        rospy.sleep(1.0)
        say("  arm attached - motors enabled")

    wait_for(status_pub, "/swiftpro_status_topic",
             "The arm driver (swiftpro) also handles motor attach.")
    if status_pub.get_num_connections() > 0:
        attach_arm()

    if not arm.synced:
        print("")
        print("  ARM NOT SYNCED - arm keys are ignored until the pose is known.")
        print("  Establish the starting pose with one home command:")
        print("      press 0 once  ->  release the pump and send the arm to home")
        print("  After that the pose is saved and later runs resume automatically.")
        print("  Run  python3 arm.py --status  to inspect the topics first.")
        print("")

    def set_pump(on):
        msg = status()
        msg.status = 1 if on else 0
        pump_pub.publish(msg)

    def publish_pose(x, y, z):
        msg = position()
        msg.x, msg.y, msg.z = x, y, z
        pos_pub.publish(msg)

    print(HELP % ARM_JOG_SPEED)
    print("  arm limits: base %g..%g  shoulder %.0f..%.0f  elbow %.0f..%.0f"
          % (A0_LIMITS + A1_LIMITS + A2_LIMITS))
    print("              shoulder+elbow <= %.0f   reach <= %.0f mm   z %.0f..%.0f mm"
          % (FOLD_MAX, MAX_REACH, Z_LIMITS[0], Z_LIMITS[1]))
    if _arm_ik_reachable is not None:
        print("              arm_ik.ik_reachable() is also checked")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    pump_on = False
    sync_warned = False
    dirty = False
    last_key = 0.0
    jog = JogController(arm)
    lift = VerticalLift(arm)
    keyboard = TerminalKeys()
    held_keys = HeldKeys()

    def send_target(pose, now):
        nonlocal dirty, last_key
        publish_pose(*pose)
        dirty = True
        last_key = now

    try:
        tty.setraw(fd)
        # Throw away keys typed during startup/attach instead of replaying
        # them all as one burst when the control loop begins.
        termios.tcflush(fd, termios.TCIFLUSH)
        if sys.stdout.isatty():
            sys.stdout.write("\x1b[>11u\x1b[?1004h\r\n")
            sys.stdout.flush()

        while not rospy.is_shutdown():
            now = time.monotonic()
            timeout = 0.1
            if lift.next_step is not None:
                timeout = min(timeout, max(0.0, lift.next_step - now))
            if held_keys:
                timeout = min(timeout, JOG_PERIOD)
            ready, _, _ = select.select([sys.stdin], [], [], timeout)
            if ready:
                data = os.read(fd, 4096)
                if not data:
                    break
                events = keyboard.feed(data)
                if b"\x03" in data or any(key == "\x03" and event != 3 for key, event in events):
                    break

                for key, event in events:
                    if key is None:
                        held_keys.clear()
                        jog.stop()
                        continue
                    if key in JOG_KEYS or key in COMBO_KEYS:
                        lift.stop()
                        held_keys.update(key, event, now)
                        continue
                    if event not in (0, 1):
                        continue

                    was_lifting = lift.next_step is not None
                    lift.stop()
                    jog.stop()
                    held_keys.clear()
                    if key == "a":
                        attach_arm()
                        # Do not replay keys typed during the 2 s attach.
                        termios.tcflush(fd, termios.TCIFLUSH)
                    elif key == " ":
                        if was_lifting:
                            say("Lift canceled. Pump stays on.")
                    elif key in ("\r", "\n"):
                        if not pump_on and pump_pub.get_num_connections() == 0:
                            say("Pump driver unavailable.")
                            continue
                        pump_on = not pump_on
                        set_pump(pump_on)
                        say("PUMP ON" if pump_on else "PUMP OFF")
                        if pump_on:
                            if not arm.synced or pos_pub.get_num_connections() == 0:
                                say("Lift needs a known pose and a connected driver.")
                            else:
                                state_reader.finish()
                                if lift.start(now):
                                    say("Lifting straight up to z=%.1f mm." % lift.goal)
                                else:
                                    say("Already at the highest reachable point here.")
                    elif key == "0":
                        pump_on = False
                        set_pump(False)
                        if pos_pub.get_num_connections() == 0:
                            say("RELEASE - arm driver not running, home not sent")
                        else:
                            state_reader.finish()
                            arm.home()
                            publish_pose(*fk(*arm.current))
                            save_pose(*fk(*arm.current))
                            dirty = False
                            say("RELEASE + HOME  (a0=%.1f a1=%.1f a2=%.1f)"
                                % tuple(arm.current))

            # Drive the arm every tick while a key is logically held, not only
            # when a key event arrives. Terminal key-repeat starts after a
            # delay (and Kitty taps can press+release in one read), so
            # event-driven jogging leaves dead presses and late bursts.
            now = time.monotonic()
            active = held_keys.live(now)
            if active:
                lift.stop()
                if arm.synced and pos_pub.get_num_connections() > 0:
                    state_reader.finish()
                    result = jog.press(active, now)
                    if result is not None:
                        moved, reason = result
                        pose = fk(*arm.current)
                        if moved:
                            send_target(pose, now)
                        if reason:
                            if len(active) == 1 and next(iter(active)) in REVERSE_KEY:
                                key = next(iter(active))
                                say(("arm limit on %s: %s. Try %s to move back. "
                                     "[x=%.0f y=%.0f z=%.0f]") %
                                    ((key, reason, REVERSE_KEY[key]) + pose))
                            else:
                                say(("arm limit: %s  [x=%.0f y=%.0f z=%.0f]") %
                                    ((reason,) + pose))
                elif not sync_warned:
                    say("Arm motion needs a known pose and a connected driver.")
                    sync_warned = True

            if lift.next_step is not None and now >= lift.next_step:
                if pos_pub.get_num_connections() == 0:
                    lift.stop()
                    say("Lift canceled. Arm driver disconnected.")
                else:
                    pose = lift.step(now)
                    if pose is None:
                        say("Lift stopped at the last valid pose.")
                    else:
                        send_target(pose, now)
                        if lift.next_step is None:
                            say("Lift complete at z=%.1f mm." % pose[2])

            if dirty and now - last_key > 0.3:
                save_pose(*fk(*arm.current))
                dirty = False

    finally:
        if sys.stdout.isatty():
            sys.stdout.write("\x1b[<u\x1b[?1004l")
            sys.stdout.flush()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        set_pump(False)
        if arm.synced:
            save_pose(*fk(*arm.current))
        time.sleep(0.1)
        set_pump(False)
        print("\nstopped - pump off")


if __name__ == "__main__":
    if "--status" in sys.argv:
        sys.exit(_status_report())
    sys.exit(main() or 0)
