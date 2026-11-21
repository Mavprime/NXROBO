#!/usr/bin/env python3
"""
Keyboard teleop for the Spark-H base + the 4-DOF uArm (base, shoulder, elbow).

Base (hold for continuous motion - terminal key repeat):
    w  forward          s  backward
    a  turn left        d  turn right

Arm (10 degrees per press, slew-limited so it moves smoothly):
    4  base rotate left          6  base rotate right
    7  raise upper arm           9  lower upper arm
    8  extend forearm            2  fold forearm

Other:
    Enter  toggle the vacuum pump: suck on the first press, release on the next
    0      release the pump and send the arm to its home pose
    Ctrl+C stop everything (base stopped, pump off)

How the arm works
-----------------
The driver only accepts a Cartesian pose on /position_write_topic
(swiftpro/position x,y,z in mm). There is no joint-angle topic. So this script keeps the three motor angles locally and
converts them to x,y,z with the same forward kinematics as the robot's
swiftpro_moveit_node.cpp:

    stretch = LOWER*cos(a1) + UPPER*cos(a2) + L2 + 56.55
    height  = LOWER*sin(a1) - UPPER*sin(a2) + L1
    x = stretch*sin(a0),  y = -stretch*cos(a0),  z = height - 74.55

where a0 = base yaw, a1 = shoulder, a2 = elbow, all in degrees. At the home
pose (120, 0, 35) mm the inverse gives about (90, 103.4, 58.4) degrees.

The key signs were chosen so that from home:
    4/6 move the base counter-clockwise/clockwise (left/right),
    7 decreases a1 and raises the arm, 9 increases a1 and lowers it,
    8 decreases a2 and extends the forearm, 2 increases a2 and folds it.
From a very different arm pose the visual direction can differ; press 0 to
return to the known home pose.

Each press adds JOINT_STEP degrees to the target. The control loop moves the
joint toward that target at JOINT_SPEED deg/s, so a 10 degree press takes about
0.17 s at the default speed instead of jumping. Holding a key (terminal key
repeat) keeps adding steps; letting go stops the arm after ARM_TIMEOUT.

Prerequisites
-------------
    base: roslaunch spark_bringup driver_bringup.launch camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6
    arm:  roslaunch swiftpro pro_control_nomoveit.launch

Usage
-----
    ssh spark@10.25.141.8
    source ~/spark_noetic/devel/setup.bash
    python3 ~/semi/key_fullbody.py
    python3 ~/semi/key_fullbody.py --check-kinematics   # no ROS needed

Environment overrides
---------------------
    LIN_SPEED=0.15      m/s
    ANG_SPEED=0.6       rad/s
    KEY_TIMEOUT=0.25    release timeout once the key-repeat stream is running
    HOLD_GRACE=0.7      timeout after a fresh press, before the first key repeat
                        arrives (the terminal's repeat delay is usually ~0.5 s)
    MAX_LIN_ACCEL=3.0   m/s^2 base acceleration
    MAX_LIN_DECEL=3.0   m/s^2 base braking when the key is released
    MAX_ANG_ACCEL=6.0   rad/s^2 turning acceleration
    MAX_ANG_DECEL=6.0   rad/s^2 turning braking
    JOINT_STEP=10       degrees added per arm key press
    JOINT_SPEED=60      deg/s slew rate for the arm
    ARM_TIMEOUT=0.3     seconds after the last arm key before the target is dropped
    MAX_LEAD=30         max degrees the target may run ahead of the arm
    ARM_PERIOD=0.05     seconds between pose publications

Topic summary
-------------
    /cmd_vel              geometry_msgs/Twist       base
    /position_write_topic swiftpro/position         arm x,y,z (mm)
    /pump_topic           swiftpro/status           vacuum 1/0
    /SwiftproState_topic  swiftpro/SwiftproState    last commanded pose

Safety
------
The arm moves immediately when a command is published. Keep hands, cables and
the field clear. Press 0 before the first arm move so the script's joint state
is synced with the physical arm. If this process is SIGKILLed while the pump is
on, the vacuum stays on; use Ctrl+C or press 0.
"""

import math
import os
import select
import sys
import termios
import time
import tty

# ---------------------------------------------------------------------------
# Arm model - constants copied from swiftpro_moveit_node.cpp and
# swiftpro_rviz_node.cpp on the robot.
# ---------------------------------------------------------------------------
MATH_L1 = 106.6             # shoulder height offset, mm
MATH_L2 = 13.2              # base link offset, mm
MATH_LOWER_ARM = 142.07     # first link length, mm
MATH_UPPER_ARM = 158.81     # second link length, mm
MATH_Z_OFFSET = 74.55       # z offset used by the driver IK
MATH_BASE_OFFSET = MATH_L2 + 56.55

HOME = (120.0, 0.0, 35.0)   # home pose, mm (same as keyboard_teleop.py)

# Conservative joint limits.  a1/a2 maxima come from the arm model constants;
# the base is kept in the front half-plane to avoid the sin(a0)=0 singularity.
A0_LIMITS = (5.0, 175.0)      # base yaw, degrees
A1_LIMITS = (0.0, 135.6)      # shoulder, degrees
A2_LIMITS = (0.0, 100.7)      # elbow, degrees
JOINT_LIMITS = (A0_LIMITS, A1_LIMITS, A2_LIMITS)

# Cartesian safety box (same spirit as keyboard_teleop.py).
CART_LIMITS = {"x": (0.0, 350.0), "y": (-160.0, 160.0), "z": (-60.0, 170.0)}

LIN_SPEED = float(os.environ.get("LIN_SPEED", "0.15"))       # m/s
ANG_SPEED = float(os.environ.get("ANG_SPEED", "0.6"))        # rad/s
KEY_TIMEOUT = float(os.environ.get("KEY_TIMEOUT", "0.25"))   # s
HOLD_GRACE = float(os.environ.get("HOLD_GRACE", "0.7"))      # s
MAX_LIN_ACCEL = float(os.environ.get("MAX_LIN_ACCEL", "3.0"))  # m/s^2
MAX_LIN_DECEL = float(os.environ.get("MAX_LIN_DECEL", "3.0"))  # m/s^2
MAX_ANG_ACCEL = float(os.environ.get("MAX_ANG_ACCEL", "6.0"))  # rad/s^2
MAX_ANG_DECEL = float(os.environ.get("MAX_ANG_DECEL", "6.0"))  # rad/s^2
JOINT_STEP = float(os.environ.get("JOINT_STEP", "10"))       # deg / press
JOINT_SPEED = float(os.environ.get("JOINT_SPEED", "60"))     # deg / s
ARM_TIMEOUT = float(os.environ.get("ARM_TIMEOUT", "0.3"))    # s
MAX_LEAD = float(os.environ.get("MAX_LEAD", "30"))           # deg
ARM_PERIOD = float(os.environ.get("ARM_PERIOD", "0.05"))     # s


def fk(a0, a1, a2):
    """Motor angles (degrees) -> Cartesian pose (mm)."""
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
    """Cartesian pose (mm) -> motor angles (degrees), or None if unreachable.

    This is the exact inverse of fk(). It selects the elbow configuration the
    robot uses (the same branch as swiftpro_rviz_node.cpp).
    """
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


def _check_kinematics():
    """Self-check: fk(ik(p)) == p and ik(fk(a)) == a."""
    ok = True
    cart_points = [HOME, (200.0, 0.0, 20.0), (250.0, 100.0, 50.0), (180.0, -80.0, 80.0)]
    for p in cart_points:
        joints = ik(*p)
        if joints is None:
            print("IK failed for %s" % (p,))
            ok = False
            continue
        q = fk(*joints)
        err = math.sqrt(sum((a - b) ** 2 for a, b in zip(p, q)))
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

    print("kinematics OK" if ok else "kinematics FAILED")
    return ok


if "--check-kinematics" in sys.argv:
    sys.exit(0 if _check_kinematics() else 1)

import rospy
from geometry_msgs.msg import Twist
from swiftpro.msg import SwiftproState, position, status

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_ik import ik_reachable

HELP = """
  Base:  w forward   s backward   a turn left   d turn right   (hold)
  Arm:   4 base left   6 base right
         7 upper arm up   9 upper arm down
         8 extend forearm  2 fold forearm        (10 deg per press)
  Enter: vacuum ON/OFF       0: release + arm home       Ctrl+C: stop
"""


def pose_ok(x, y, z):
    """True if a Cartesian pose is inside the safety box and arm-reachable."""
    if not (CART_LIMITS["x"][0] <= x <= CART_LIMITS["x"][1]):
        return False
    if not (CART_LIMITS["y"][0] <= y <= CART_LIMITS["y"][1]):
        return False
    if not (CART_LIMITS["z"][0] <= z <= CART_LIMITS["z"][1]):
        return False
    try:
        return ik_reachable(x, y, z)
    except Exception:
        return False


def clamp(value, limits):
    return min(max(value, limits[0]), limits[1])


def smooth_velocity(current, target, accel, decel, dt):
    """Ramp toward the target with separate acceleration and braking limits."""
    diff = target - current
    is_braking = (current > 0 and target < current) or (current < 0 and target > current)
    rate = decel if is_braking else accel
    max_change = rate * dt
    if abs(diff) <= max_change:
        return target
    return current + (max_change if diff > 0 else -max_change)


class Arm:
    """Local joint state plus slew-limited motion toward a target."""

    def __init__(self):
        self.current = list(HOME_JOINTS)
        self.target = list(HOME_JOINTS)
        self.synced = False
        self.last_pose = fk(*self.current)
        self.last_pub = 0.0
        self.last_limit_msg = 0.0

    def sync_to(self, joints):
        self.current = list(joints)
        self.target = list(joints)
        self.last_pose = fk(*self.current)
        self.synced = True

    def home(self):
        self.sync_to(HOME_JOINTS)

    def jog(self, axis, direction):
        """Add one step to a joint target; returns False if arm is not synced."""
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
        """Move the joints toward the target. Returns True if any joint moved.

        If the combined move is unreachable, each joint is tried on its own:
        the blocked joint is reset to its current angle and stops, while the
        other joints keep moving. This prevents one axis at a Cartesian limit
        from freezing the whole arm.
        """
        max_step = JOINT_SPEED * dt
        desired = list(self.current)
        for i in range(3):
            delta = self.target[i] - self.current[i]
            if abs(delta) < 1e-9:
                continue
            step = max(-max_step, min(max_step, delta))
            desired[i] = clamp(self.current[i] + step, JOINT_LIMITS[i])

        if desired == self.current:
            return False

        x, y, z = fk(*desired)
        if pose_ok(x, y, z):
            self.current = desired
            return True

        # Combined move is unreachable. Accept whatever axes can move alone.
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
                self.target[i] = self.current[i]   # stop this axis only

        if moved:
            self.current = accepted

        now = time.time()
        if blocked and now - self.last_limit_msg > 1.0:
            names = ", ".join(("base", "shoulder", "elbow")[i] for i in blocked)
            if moved:
                print("arm limit: %s blocked (other axes still active; x=%.1f y=%.1f z=%.1f)"
                      % (names, x, y, z))
            else:
                print("arm limit: %s blocked (move back to continue; x=%.1f y=%.1f z=%.1f)"
                      % (names, x, y, z))
            self.last_limit_msg = now
        return moved


class ArmStateReader:
    """Use /SwiftproState_topic to initialise the joint state when available."""

    def __init__(self, arm):
        self.arm = arm

    def callback(self, msg):
        if self.arm.synced:
            return
        if abs(msg.x) > 1.0 or abs(msg.y) > 1.0 or abs(msg.z) > 1.0:
            joints = ik(msg.x, msg.y, msg.z)
            if joints is None:
                return
            in_limits = all(lo <= v <= hi for v, (lo, hi) in zip(joints, JOINT_LIMITS))
            if not in_limits:
                return
            self.arm.sync_to(joints)
            print("arm synced from /SwiftproState_topic: a0=%.1f a1=%.1f a2=%.1f"
                  % tuple(joints))


def main():
    rospy.init_node("key_fullbody", anonymous=True, argv=[sys.argv[0]])

    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    pos_pub = rospy.Publisher("/position_write_topic", position, queue_size=1)
    pump_pub = rospy.Publisher("/pump_topic", status, queue_size=1)

    arm = Arm()
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

    wait_for(cmd_pub, "/cmd_vel",
             "Start the base driver: roslaunch spark_bringup driver_bringup.launch "
             "camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6")
    wait_for(pos_pub, "/position_write_topic",
             "Start the arm driver: roslaunch swiftpro pro_control_nomoveit.launch\n"
             "         Arm keys and the pump will not work until it is running.")
    wait_for(pump_pub, "/pump_topic",
             "The arm driver (swiftpro) also provides the vacuum pump.")

    if not arm.synced:
        print("  arm pose unknown - press 0 first to home/sync the arm.")

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
        arm.last_pose = (x, y, z)
        arm.last_pub = time.time()

    print(HELP)
    print("  base: %.2f m/s / %.2f rad/s   arm: %.0f deg/press at %.0f deg/s"
          % (LIN_SPEED, ANG_SPEED, JOINT_STEP, JOINT_SPEED))
    print("  arm limits: base %.0f..%.0f  shoulder %.0f..%.0f  elbow %.0f..%.0f"
          % (A0_LIMITS + A1_LIMITS + A2_LIMITS))

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    target_linear = target_angular = 0.0
    linear = angular = 0.0
    last_base_key = 0.0
    last_base_char = ""
    hold_timeout = HOLD_GRACE
    last_arm_key = 0.0
    pump_on = False
    last_loop = time.time()
    rate = rospy.Rate(20)         # matches the Spark base serial driver

    try:
        tty.setraw(fd)
        while not rospy.is_shutdown():
            now = time.time()
            dt = min(max(now - last_loop, 0.0), 0.1)
            last_loop = now

            # Drain all pending keys without blocking the 20 Hz control loop.
            stop = False
            while select.select([sys.stdin], [], [], 0)[0]:
                char = os.read(fd, 1).decode(errors="ignore")
                if not char:
                    break

                if char == "\x03":                       # Ctrl+C
                    stop = True
                    break
                elif char in "wWaAsSdD":                 # base
                    key = char.lower()
                    if key == "w":
                        target_linear, target_angular = LIN_SPEED, 0.0
                    elif key == "s":
                        target_linear, target_angular = -LIN_SPEED, 0.0
                    elif key == "a":
                        target_linear, target_angular = 0.0, ANG_SPEED
                    elif key == "d":
                        target_linear, target_angular = 0.0, -ANG_SPEED

                    # A terminal sends no key-up event: the only release signal
                    # is a pause in key repeats. The OS sends the FIRST repeat
                    # only after its repeat delay (~0.5 s), so a short timeout
                    # would brake right after the press. Use a long grace for a
                    # fresh press, then a short timeout once repeats are flowing.
                    gap = now - last_base_key
                    if key != last_base_char or gap > HOLD_GRACE:
                        # Fresh press or direction change: the first repeat is
                        # delayed by the terminal's key-repeat delay (~0.5 s),
                        # so do not brake until it has had time to arrive.
                        hold_timeout = HOLD_GRACE
                    else:
                        # Repeats are flowing: use the crisp release timeout.
                        hold_timeout = KEY_TIMEOUT
                    last_base_char = key
                    last_base_key = now
                elif char in "482769":                   # arm joints
                    last_arm_key = now
                    axis, direction = {
                        "4": (0, +1),   # base left
                        "6": (0, -1),   # base right
                        "7": (1, -1),   # shoulder: raise (was 8)
                        "9": (1, +1),   # shoulder: lower (was 2)
                        "8": (2, -1),   # elbow: extend (was 7)
                        "2": (2, +1),   # elbow: fold (was 9)
                    }[char]
                    if not arm.jog(axis, direction):
                        print("  arm not synced - press 0 first.")
                elif char in ("\r", "\n"):               # Enter
                    pump_on = not pump_on
                    set_pump(pump_on)
                    print("PUMP ON" if pump_on else "PUMP OFF")
                elif char == "0":                        # release + home
                    pump_on = False
                    set_pump(False)
                    set_base(0.0, 0.0)
                    target_linear = target_angular = 0.0
                    linear = angular = 0.0
                    if pos_pub.get_num_connections() == 0:
                        print("RELEASE - arm driver not running, home not sent")
                    else:
                        arm.home()
                        x, y, z = fk(*arm.current)
                        publish_pose(x, y, z)
                        print("RELEASE + HOME  (a0=%.1f a1=%.1f a2=%.1f)"
                              % tuple(arm.current))

            if stop:
                break

            # Base: the terminal has no key-up event, so the timeout is the
            # key release. hold_timeout is short while repeats are flowing and
            # long only during the initial key-repeat delay of a fresh press.
            if now - last_base_key > hold_timeout:
                target_linear = target_angular = 0.0

            # Slew the published velocity so starts and stops are smooth.
            linear = smooth_velocity(linear, target_linear,
                                     MAX_LIN_ACCEL, MAX_LIN_DECEL, dt)
            angular = smooth_velocity(angular, target_angular,
                                      MAX_ANG_ACCEL, MAX_ANG_DECEL, dt)
            set_base(linear, angular)

            # Arm: drop the remaining target when the key has been released.
            if now - last_arm_key > ARM_TIMEOUT:
                arm.drop_target()

            moved = arm.step(dt)
            if now - arm.last_pub >= ARM_PERIOD:
                x, y, z = fk(*arm.current)
                if (x, y, z) != arm.last_pose or moved:
                    publish_pose(x, y, z)

            rate.sleep()

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        set_base(0.0, 0.0)
        rospy.sleep(0.1)
        set_base(0.0, 0.0)
        set_pump(False)
        rospy.sleep(0.1)
        set_pump(False)
        print("\nstopped - base stopped, pump off")


if __name__ == "__main__":
    main()
