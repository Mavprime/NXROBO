#!/usr/bin/env python3
"""
Simplified keyboard teleop for Spark-H / uArm (SwiftPro).

Keys (same layout as before):
  4 / 6   base yaw left / right
  7 / 9   raise / lower upper arm (shoulder)
  8 / 2   extend / fold forearm (elbow)
  Enter   toggle vacuum
  0       pump off + go home
  A       re-attach motors
  Ctrl+C  stop (pump off)

Run alongside the arm driver:
  roslaunch swiftpro pro_control_nomoveit.launch
"""

import math
import os
import select
import sys
import termios
import time
import tty

import rospy
from swiftpro.msg import SwiftproState, position, status

# ---------------------------------------------------------------------------
# Geometry (copied from the official swiftpro driver)
# ---------------------------------------------------------------------------
MATH_L1          = 106.6
MATH_L2          = 13.2
MATH_LOWER_ARM   = 142.07
MATH_UPPER_ARM   = 158.81
MATH_Z_OFFSET    = 74.55
MATH_BASE_OFFSET = MATH_L2 + 56.55

HOME_CART  = (120.0, 0.0, 35.0)          # mm
HOME_JOINT = (90.0, 103.4, 58.4)         # deg (approx)

A0_LIMITS = (5.0, 175.0)
A1_LIMITS = (0.0, 135.6)
A2_LIMITS = (0.0, 100.7)
JOINT_LIMITS = (A0_LIMITS, A1_LIMITS, A2_LIMITS)

CART_LIMITS = {
    "x": (0.0, 350.0),
    "y": (-160.0, 160.0),
    "z": (-60.0, 170.0),
}

# Tunables (can still be overridden by env vars)
JOINT_STEP   = float(os.environ.get("JOINT_STEP",   "8"))   # deg per key
JOINT_SPEED  = float(os.environ.get("JOINT_SPEED",  "45"))  # deg/s
ARM_PERIOD   = float(os.environ.get("ARM_PERIOD",   "0.05"))
ARM_TIMEOUT  = float(os.environ.get("ARM_TIMEOUT",  "0.25"))

STATE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          ".arm_last_pose")


# ---------------------------------------------------------------------------
# Kinematics
# ---------------------------------------------------------------------------
def fk(a0, a1, a2):
    r0, r1, r2 = map(math.radians, (a0, a1, a2))
    stretch = (MATH_LOWER_ARM * math.cos(r1)
               + MATH_UPPER_ARM * math.cos(r2)
               + MATH_BASE_OFFSET)
    height  = (MATH_LOWER_ARM * math.sin(r1)
               - MATH_UPPER_ARM * math.sin(r2)
               + MATH_L1)
    return (stretch * math.sin(r0),
            -stretch * math.cos(r0),
            height - MATH_Z_OFFSET)


def ik(x, y, z):
    """Return (a0, a1, a2) in degrees or None if unreachable."""
    a0 = math.degrees(math.atan2(x, -y))
    stretch = math.hypot(x, y)
    x_in = stretch - MATH_BASE_OFFSET
    z_in = z + MATH_Z_OFFSET - MATH_L1
    d = math.hypot(x_in, z_in)
    if d < 1e-6:
        return None

    cos_delta = ((d*d - MATH_LOWER_ARM**2 - MATH_UPPER_ARM**2)
                 / (2.0 * MATH_LOWER_ARM * MATH_UPPER_ARM))
    if abs(cos_delta) > 1.0 + 1e-9:
        return None
    delta = -math.acos(max(-1.0, min(1.0, cos_delta)))
    phi = math.atan2(z_in, x_in)
    a1 = phi - math.atan2(MATH_UPPER_ARM * math.sin(delta),
                          MATH_LOWER_ARM + MATH_UPPER_ARM * math.cos(delta))
    a2 = -(a1 + delta)
    return (a0, math.degrees(a1), math.degrees(a2))


def pose_ok(x, y, z):
    if not (CART_LIMITS["x"][0] <= x <= CART_LIMITS["x"][1] and
            CART_LIMITS["y"][0] <= y <= CART_LIMITS["y"][1] and
            CART_LIMITS["z"][0] <= z <= CART_LIMITS["z"][1]):
        return False
    return ik(x, y, z) is not None


def clamp(v, limits):
    return max(limits[0], min(limits[1], v))


# ---------------------------------------------------------------------------
# Simple arm state
# ---------------------------------------------------------------------------
class Arm:
    def __init__(self):
        self.joints = list(HOME_JOINT)
        self.target = list(HOME_JOINT)
        self.last_pose = fk(*self.joints)
        self.last_pub = 0.0

    def set_joints(self, j):
        self.joints = list(j)
        self.target = list(j)
        self.last_pose = fk(*self.joints)

    def home(self):
        self.set_joints(HOME_JOINT)

    def jog(self, axis, direction):
        lo, hi = JOINT_LIMITS[axis]
        self.target[axis] = clamp(
            self.target[axis] + direction * JOINT_STEP, (lo, hi))

    def drop_target(self):
        self.target = list(self.joints)

    def step(self, dt):
        """Move joints toward target at JOINT_SPEED. Returns True if moved."""
        max_step = JOINT_SPEED * dt
        new_j = list(self.joints)
        moved = False

        for i in range(3):
            delta = self.target[i] - self.joints[i]
            if abs(delta) < 1e-6:
                continue
            step = max(-max_step, min(max_step, delta))
            trial = list(new_j)
            trial[i] = clamp(self.joints[i] + step, JOINT_LIMITS[i])
            x, y, z = fk(*trial)
            if pose_ok(x, y, z):
                new_j = trial
                moved = True
            else:
                # blocked → stop this axis
                self.target[i] = self.joints[i]

        if moved:
            self.joints = new_j
        return moved


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def save_pose(x, y, z):
    try:
        tmp = STATE_FILE + ".tmp"
        with open(tmp, "w") as f:
            f.write("%.3f %.3f %.3f\n" % (x, y, z))
        os.replace(tmp, STATE_FILE)
    except OSError:
        pass


def load_saved_pose():
    try:
        with open(STATE_FILE) as f:
            x, y, z = map(float, f.read().split())
        j = ik(x, y, z)
        if j and all(lo <= v <= hi for v, (lo, hi) in zip(j, JOINT_LIMITS)):
            return j
    except (OSError, ValueError):
        pass
    return None


def attach_arm(status_pub):
    print("attaching motors (0 → 1)...")
    status_pub.publish(status(status=0))
    rospy.sleep(0.8)
    status_pub.publish(status(status=1))
    rospy.sleep(0.8)
    print("motors attached")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    if not sys.stdin.isatty():
        print("Need a real terminal (raw mode). Run in an SSH window.")
        return 1

    rospy.init_node("arm_teleop", anonymous=True)

    pos_pub    = rospy.Publisher("/position_write_topic", position, queue_size=1)
    pump_pub   = rospy.Publisher("/pump_topic", status, queue_size=1)
    status_pub = rospy.Publisher("/swiftpro_status_topic", status, queue_size=1)

    # Wait a moment for the driver
    rospy.sleep(0.5)
    if pos_pub.get_num_connections() == 0:
        print("WARNING: no subscriber on /position_write_topic")
        print("         Start: roslaunch swiftpro pro_control_nomoveit.launch")

    arm = Arm()
    saved = load_saved_pose()
    if saved:
        arm.set_joints(saved)
        print("resumed last pose: x=%.1f y=%.1f z=%.1f" % fk(*saved))
    else:
        print("starting from home")

    # Attach motors on startup
    if status_pub.get_num_connections() > 0:
        attach_arm(status_pub)

    def set_pump(on):
        pump_pub.publish(status(status=1 if on else 0))

    def publish():
        x, y, z = fk(*arm.joints)
        if (x, y, z) != arm.last_pose:
            msg = position(x=x, y=y, z=z)
            pos_pub.publish(msg)
            arm.last_pose = (x, y, z)
            arm.last_pub = time.time()
            save_pose(x, y, z)

    print("""
  4/6  base left/right     7/9  upper arm up/down
  8/2  forearm extend/fold
  Enter  vacuum ON/OFF     0  release + home
  A      re-attach motors  Ctrl+C  quit
""")
    print("step = %.0f°   speed = %.0f°/s" % (JOINT_STEP, JOINT_SPEED))

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setraw(fd)

    last_key = 0.0
    pump_on = False
    last_loop = time.time()

    try:
        while not rospy.is_shutdown():
            now = time.time()
            dt = min(now - last_loop, 0.1)
            last_loop = now

            # Non-blocking key read
            if select.select([sys.stdin], [], [], 0.02)[0]:
                ch = os.read(fd, 1).decode(errors="ignore")

                if ch == "\x03":                     # Ctrl+C
                    break
                elif ch in ("a", "A"):
                    attach_arm(status_pub)
                elif ch in "482769":
                    last_key = now
                    axis, direction = {
                        "4": (0, +1), "6": (0, -1),
                        "7": (1, -1), "9": (1, +1),
                        "8": (2, -1), "2": (2, +1),
                    }[ch]
                    arm.jog(axis, direction)
                elif ch in ("\r", "\n"):
                    pump_on = not pump_on
                    set_pump(pump_on)
                    print("PUMP", "ON" if pump_on else "OFF")
                elif ch == "0":
                    pump_on = False
                    set_pump(False)
                    arm.home()
                    publish()
                    print("HOME  a0=%.1f a1=%.1f a2=%.1f" % tuple(arm.joints))

            # Stop moving shortly after key release
            if now - last_key > ARM_TIMEOUT:
                arm.drop_target()

            if arm.step(dt) or (now - arm.last_pub >= ARM_PERIOD):
                publish()

            rospy.sleep(0.005)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        set_pump(False)
        rospy.sleep(0.1)
        print("\nstopped – pump off")


if __name__ == "__main__":
    sys.exit(main() or 0)