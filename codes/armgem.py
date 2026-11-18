#!/usr/bin/env python3
"""
Optimized Arm-only keyboard teleop for the Spark-H uArm (base yaw, shoulder, elbow).

Arm keys (10 degrees per press, slew-limited for smooth execution):
    4  base rotate left          6  base rotate right
    7  raise upper arm           9  lower upper arm
    8  extend forearm            2  fold forearm

Other:
    Enter  Toggle vacuum pump (suck on / release off)
    0      Release pump and send arm to home pose
    Ctrl+C Stop everything safely (pump off, restore terminal)
    A      Re-attach arm motors if unresponsive
"""

import math
import os
import select
import sys
import termios
import time
import tty

# ---------------------------------------------------------------------------
# Arm Model Constants (matching swiftpro moveit/rviz nodes)
# ---------------------------------------------------------------------------
MATH_L1 = 106.6             # Shoulder height offset, mm
MATH_L2 = 13.2              # Base link offset, mm
MATH_LOWER_ARM = 142.07     # First link length, mm
MATH_UPPER_ARM = 158.81     # Second link length, mm
MATH_Z_OFFSET = 74.55       # Z offset used by driver IK
MATH_BASE_OFFSET = MATH_L2 + 56.55

HOME = (120.0, 0.0, 35.0)   # Home pose, mm

# Joint limits (degrees)
A0_LIMITS = (5.0, 175.0)      # Base yaw
A1_LIMITS = (0.0, 135.6)      # Shoulder
A2_LIMITS = (0.0, 100.7)      # Elbow
JOINT_LIMITS = (A0_LIMITS, A1_LIMITS, A2_LIMITS)

# Cartesian safety bounding box (mm). y is deliberately unbounded: the base
# joint sweeps a circle, so a fixed y window stops rotation part-way through
# whenever the forearm is extended. Base travel is limited by A0_LIMITS.
CART_LIMITS = {"x": (0.0, 350.0), "z": (-60.0, 170.0)}

# Control parameters (overridable via environment variables)
JOINT_STEP = float(os.environ.get("JOINT_STEP", "10"))
JOINT_SPEED = float(os.environ.get("JOINT_SPEED", "60"))
ARM_TIMEOUT = float(os.environ.get("ARM_TIMEOUT", "0.3"))
MAX_LEAD = float(os.environ.get("MAX_LEAD", "30"))
ARM_PERIOD = float(os.environ.get("ARM_PERIOD", "0.05"))

STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".arm_last_pose")


def fk(a0, a1, a2):
    """Motor angles (degrees) -> Cartesian pose (mm)."""
    r0, r1, r2 = math.radians(a0), math.radians(a1), math.radians(a2)
    stretch = (MATH_LOWER_ARM * math.cos(r1)
               + MATH_UPPER_ARM * math.cos(r2)
               + MATH_BASE_OFFSET)
    height = (MATH_LOWER_ARM * math.sin(r1)
              - MATH_UPPER_ARM * math.sin(r2)
              + MATH_L1)
    return (stretch * math.sin(r0), -stretch * math.cos(r0), height - MATH_Z_OFFSET)


def ik(x, y, z):
    """Cartesian pose (mm) -> motor angles (degrees), or None if unreachable."""
    a0 = math.degrees(math.atan2(x, -y))
    stretch = math.hypot(x, y)
    x_in = stretch - MATH_BASE_OFFSET
    z_in = z + MATH_Z_OFFSET - MATH_L1
    d = math.hypot(x_in, z_in)
    if d < 1e-6:
        return None

    cos_delta = ((d * d - MATH_LOWER_ARM ** 2 - MATH_UPPER_ARM ** 2)
                 / (2.0 * MATH_LOWER_ARM * MATH_UPPER_ARM))
    if not (-1.0 - 1e-9 <= cos_delta <= 1.0 + 1e-9):
        return None
    delta = -math.acos(max(-1.0, min(1.0, cos_delta)))
    phi = math.atan2(z_in, x_in)
    a1 = phi - math.atan2(MATH_UPPER_ARM * math.sin(delta),
                          MATH_LOWER_ARM + MATH_UPPER_ARM * math.cos(delta))
    return (a0, math.degrees(a1), math.degrees(-(a1 + delta)))


HOME_JOINTS = ik(*HOME) or (90.0, 103.4, 58.4)


def save_pose(x, y, z, force=False):
    """Persist the last commanded pose to disk safely (rate-limited/on-demand)."""
    try:
        tmp = STATE_FILE + ".tmp"
        with open(tmp, "w") as handle:
            handle.write(f"{x:.3f} {y:.3f} {z:.3f}\n")
        os.replace(tmp, STATE_FILE)
    except OSError:
        pass


def load_saved_pose():
    """Load joints from the last commanded pose file, or return None."""
    try:
        with open(STATE_FILE) as handle:
            x, y, z = [float(val) for val in handle.read().split()]
    except (OSError, ValueError):
        return None
    joints = ik(x, y, z)
    if joints is None:
        return None
    if not all(lo <= v <= hi for v, (lo, hi) in zip(joints, JOINT_LIMITS)):
        return None
    return joints


def _check_kinematics():
    """Self-check kinematics consistency."""
    ok = True
    for p in [HOME, (200.0, 0.0, 20.0), (250.0, 100.0, 50.0)]:
        joints = ik(*p)
        if joints is None:
            print(f"IK failed for {p}")
            ok = False
            continue
        q = fk(*joints)
        err = math.sqrt(sum((a - b) ** 2 for a, b in zip(p, q)))
        if err > 0.01:
            ok = False
    print("Kinematics OK" if ok else "Kinematics FAILED")
    return ok


if "--check-kinematics" in sys.argv:
    sys.exit(0 if _check_kinematics() else 1)

import rospy
from swiftpro.msg import SwiftproState, position, status

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_ik import ik_reachable

HELP = """
  Arm controls:
    4 / 6 : Base Left / Right
    7 / 9 : Upper Arm Up / Down
    8 / 2 : Extend / Fold Forearm  (10 deg per press)
  Enter   : Vacuum Pump ON/OFF
  0       : Release pump and send arm to Home
  A       : Re-attach arm motors
  Ctrl+C  : Exit safely
"""


def pose_ok(x, y, z):
    """Verify if Cartesian pose is within safety limits and reachable.

    y is unchecked on purpose (see CART_LIMITS): it would otherwise block base
    rotation part-way through. The base angle is bounded by A0_LIMITS.
    """
    if not (CART_LIMITS["x"][0] <= x <= CART_LIMITS["x"][1] and
            CART_LIMITS["z"][0] <= z <= CART_LIMITS["z"][1]):
        return False
    try:
        return ik_reachable(x, y, z)
    except Exception:
        return False


def clamp(val, limits):
    return min(max(val, limits[0]), limits[1])


class Arm:
    """Manages local joint state, target interpolation, and safety checks."""

    def __init__(self):
        self.current = list(HOME_JOINTS)
        self.target = list(HOME_JOINTS)
        self.synced = False
        self.last_pose = fk(*self.current)
        self.last_pub = 0.0
        self.last_limit_msg = 0.0
        self._last_saved_pose = None

    def sync_to(self, joints):
        self.current = list(joints)
        self.target = list(joints)
        self.last_pose = fk(*self.current)
        self.synced = True

    def home(self):
        self.sync_to(HOME_JOINTS)

    def jog(self, axis, direction):
        if not self.synced:
            return False
        limits = JOINT_LIMITS[axis]
        lo = max(limits[0], self.current[axis] - MAX_LEAD)
        hi = min(limits[1], self.current[axis] + MAX_LEAD)
        self.target[axis] = clamp(self.target[axis] + direction * JOINT_STEP, (lo, hi))
        return True

    def drop_target(self):
        self.target = list(self.current)

    def step(self, dt):
        max_step = JOINT_SPEED * dt
        desired = list(self.current)
        for i in range(3):
            delta = self.target[i] - self.current[i]
            if abs(delta) < 1e-9:
                continue
            step_val = max(-max_step, min(max_step, delta))
            desired[i] = clamp(self.current[i] + step_val, JOINT_LIMITS[i])

        if desired == self.current:
            return False

        x, y, z = fk(*desired)
        if pose_ok(x, y, z):
            self.current = desired
            return True

        # Fallback: move individual valid axes if combined move fails
        accepted = list(self.current)
        moved = False
        blocked = []
        for i in range(3):
            if desired[i] == self.current[i]:
                continue
            trial = list(accepted)
            trial[i] = desired[i]
            tx, ty, tz = fk(*trial)
            if pose_ok(tx, ty, tz):
                accepted = trial
                moved = True
            else:
                blocked.append(i)
                self.target[i] = self.current[i]

        if moved:
            self.current = accepted

        now = time.time()
        if blocked and now - self.last_limit_msg > 1.0:
            names = ", ".join(("base", "shoulder", "elbow")[i] for i in blocked)
            print(f"Arm limit reached on: {names}")
            self.last_limit_msg = now
        return moved


class ArmStateReader:
    """Initializes arm state from /SwiftproState_topic if available."""

    def __init__(self, arm):
        self.arm = arm
        self.live_used = False

    def callback(self, msg):
        if self.live_used or (abs(msg.x) <= 1.0 and abs(msg.y) <= 1.0 and abs(msg.z) <= 1.0):
            return
        joints = ik(msg.x, msg.y, msg.z)
        if joints and all(lo <= v <= hi for v, (lo, hi) in zip(joints, JOINT_LIMITS)):
            self.arm.sync_to(joints)
            self.live_used = True
            print(f"Arm synced from ROS topic: a0={joints[0]:.1f}, a1={joints[1]:.1f}, a2={joints[2]:.1f}")


def _status_report():
    rospy.init_node("arm_status", anonymous=True, argv=[sys.argv[0]])
    pos_pub = rospy.Publisher("/position_write_topic", position, queue_size=1)
    pump_pub = rospy.Publisher("/pump_topic", status, queue_size=1)
    
    arm = Arm()
    saved = load_saved_pose()
    if saved:
        arm.sync_to(saved)
    rospy.sleep(1.0)
    print(f"position_write_topic connections: {pos_pub.get_num_connections()}")
    print(f"pump_topic connections:           {pump_pub.get_num_connections()}")
    print(f"Saved pose available: {saved is not None}")
    return 0


def main():
    if not sys.stdin.isatty():
        print("Error: arm.py requires a direct terminal session (run via SSH).")
        return 1

    rospy.init_node("arm", anonymous=True, argv=[sys.argv[0]])

    pos_pub = rospy.Publisher("/position_write_topic", position, queue_size=1)
    pump_pub = rospy.Publisher("/pump_topic", status, queue_size=1)
    status_pub = rospy.Publisher("/swiftpro_status_topic", status, queue_size=1)

    arm = Arm()
    saved = load_saved_pose()
    if saved:
        arm.sync_to(saved)
        print(f"Resumed from saved pose: x={fk(*saved)[0]:.1f}, y={fk(*saved)[1]:.1f}, z={fk(*saved)[2]:.1f}")

    state_reader = ArmStateReader(arm)
    rospy.Subscriber("/SwiftproState_topic", SwiftproState, state_reader.callback, queue_size=1)
    rospy.sleep(0.5)

    def attach_arm():
        print("Attaching arm motors (M2019 -> M17)...")
        status_pub.publish(0)
        rospy.sleep(1.0)
        status_pub.publish(1)
        rospy.sleep(1.0)
        print("Arm attached and enabled.")

    if status_pub.get_num_connections() > 0:
        attach_arm()

    if not arm.synced:
        print("\nWARNING: Arm not synced. Press '0' once to home and enable arm keys.\n")

    def set_pump(on):
        msg = status()
        msg.status = 1 if on else 0
        pump_pub.publish(msg)

    def publish_pose(x, y, z):
        msg = position()
        msg.x, msg.y, msg.z = x, y, z
        pos_pub.publish(msg)
        arm.last_pose = (x, y, z)
        arm.last_pub = time.time()
        # Rate-limit disk writes: only write if pose changed significantly or on demand
        if arm._last_saved_pose != (x, y, z):
            save_pose(x, y, z)
            arm._last_saved_pose = (x, y, z)

    print(HELP)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    last_arm_key = 0.0
    pump_on = False
    last_loop = time.time()
    sync_warned = False

    try:
        tty.setraw(fd)
        while not rospy.is_shutdown():
            now = time.time()
            dt = min(max(now - last_loop, 0.0), 0.1)
            last_loop = now

            ready, _, _ = select.select([sys.stdin], [], [], 0.02)
            if ready:
                char = os.read(fd, 1).decode(errors="ignore")
                if char == "\x03":  # Ctrl+C
                    break
                elif char in ("a", "A"):
                    attach_arm()
                elif char in "482769":
                    last_arm_key = now
                    axis, direction = {
                        "4": (0, +1), "6": (0, -1),
                        "7": (1, -1), "9": (1, +1),
                        "8": (2, -1), "2": (2, +1),
                    }[char]
                    if not arm.jog(axis, direction) and not sync_warned:
                        print("Press '0' once to sync/home the arm.")
                        sync_warned = True
                elif char in ("\r", "\n"):
                    pump_on = not pump_on
                    set_pump(pump_on)
                    print(f"\nPUMP {'ON' if pump_on else 'OFF'}")
                elif char == "0":
                    pump_on = False
                    set_pump(False)
                    arm.home()
                    x, y, z = fk(*arm.current)
                    publish_pose(x, y, z)
                    print(f"\nRELEASE + HOME (a0={arm.current[0]:.1f}, a1={arm.current[1]:.1f}, a2={arm.current[2]:.1f})")

            if now - last_arm_key > ARM_TIMEOUT:
                arm.drop_target()

            moved = arm.step(dt)
            if now - arm.last_pub >= ARM_PERIOD:
                x, y, z = fk(*arm.current)
                if (x, y, z) != arm.last_pose or moved:
                    publish_pose(x, y, z)

            rospy.sleep(0.005)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        set_pump(False)
        rospy.sleep(0.1)
        print("\nStopped - pump off, terminal restored.")


if __name__ == "__main__":
    if "--status" in sys.argv:
        sys.exit(_status_report())
    sys.exit(main() or 0)