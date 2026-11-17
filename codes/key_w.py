#!/usr/bin/env python3
"""
Keyboard teleop for the Spark base and uArm.

Base (hold; smooth acceleration / braking):
    w / s      forward / backward
    a / d      turn left / right
    (hold w/s + a/d together for a smooth arc)
    Space      EMERGENCY STOP (instant base + arm halt; pump stays as-is)

Arm (hold; 20 Hz micro-steps):
    8 / 2      up / down          (z)
    7 / 9      forward / backward (x)
    4 / 6      swing CCW / CW     (y)
    5          stop arm jog (hold the current pose)

Other:
    Enter      toggle vacuum pump (suck / release)
    0          release pump + return the arm to home
    Ctrl+C     stop everything (base stopped, pump off)

How the motion is smoothed:
    * Base: velocity ramps toward the target with a configurable acceleration,
      and brakes harder when the key is released (smooth_velocity()).
    * Arm: the target pose is streamed in small steps at 20 Hz instead of one
      jump per key press. A key press moves the arm at ARM_SPEED mm/s.

Prerequisites:
    base: roslaunch spark_bringup driver_bringup.launch camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6
          (or the minimal server: roslaunch ~/codes/server.launch)
    arm:  roslaunch swiftpro pro_control_nomoveit.launch
          (included by server.launch)

Usage:
    ssh spark@10.25.141.8
    source ~/spark_noetic/devel/setup.bash
    python3 ~/codes/key_w.py

Environment overrides:
    LIN_SPEED=0.15        m/s
    ANG_SPEED=0.6         rad/s
    KEY_TIMEOUT=0.25      seconds of forward/back coast after the last key
    STEER_TIMEOUT=0.20    seconds of turn coast after the last key
    MAX_LIN_ACCEL=1.0     m/s^2
    MAX_LIN_DECEL=3.0     m/s^2
    MAX_ANG_ACCEL=3.0     rad/s^2
    MAX_ANG_DECEL=6.0     rad/s^2
    ARM_SPEED=35.0        mm/s
    ARM_PUB_PERIOD=0.05   seconds between arm pose publications (20 Hz)

Topics:
    /cmd_vel              geometry_msgs/Twist       base motion
    /position_write_topic swiftpro/position         arm x,y,z in mm (absolute)
    /pump_topic           swiftpro/status           vacuum 1/0
    /SwiftproState_topic  swiftpro/SwiftproState    current pose feedback
"""

import os
import select
import sys
import termios
import time
import tty

import rospy
from geometry_msgs.msg import Twist
from swiftpro.msg import SwiftproState, position, status

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_ik import ik_reachable

LIN_SPEED = float(os.environ.get("LIN_SPEED", "0.15"))          # m/s
ANG_SPEED = float(os.environ.get("ANG_SPEED", "0.6"))           # rad/s
KEY_TIMEOUT = float(os.environ.get("KEY_TIMEOUT", "0.25"))      # s
STEER_TIMEOUT = float(os.environ.get("STEER_TIMEOUT", "0.20"))  # s

MAX_LIN_ACCEL = float(os.environ.get("MAX_LIN_ACCEL", "1.0"))   # m/s^2
MAX_LIN_DECEL = float(os.environ.get("MAX_LIN_DECEL", "3.0"))   # m/s^2
MAX_ANG_ACCEL = float(os.environ.get("MAX_ANG_ACCEL", "3.0"))   # rad/s^2
MAX_ANG_DECEL = float(os.environ.get("MAX_ANG_DECEL", "6.0"))   # rad/s^2

ARM_SPEED = float(os.environ.get("ARM_SPEED", "35.0"))          # mm/s
ARM_PUB_PERIOD = float(os.environ.get("ARM_PUB_PERIOD", "0.05"))  # s
ARM_KEY_TIMEOUT = float(os.environ.get("ARM_KEY_TIMEOUT", "0.22"))  # s

HOME = (120.0, 0.0, 35.0)
ARM_LIMITS = {"x": (0.0, 350.0), "y": (-160.0, 160.0), "z": (-85.0, 170.0)}

# Numpad layout:
#   7 8 9      x-  z+  x+
#   4 5 6      y-  stop y+
#   1 2 3          z-
ARM_AXES = {
    "8": ("z", +1), "2": ("z", -1),
    "7": ("x", +1), "9": ("x", -1),
    "4": ("y", +1), "6": ("y", -1),
}

HELP = """
  === Spark keyboard teleop (key_w) ===
  Base:  w/s forward / backward   a/d turn left / right   (hold)
         hold w/s + a/d for a smooth arc    Space: emergency stop

  Arm:   8/2 up / down (z)     7/9 forward / backward (x)
         4/6 swing CCW / CW (y)        5: stop arm jog (hold pose)

  Pump:  Enter toggle vacuum     0: release + arm home
  Ctrl+C: stop everything
"""


def safe_print(*args, **kwargs):
    """Print safely while the terminal is in raw mode."""
    sep = kwargs.get("sep", " ")
    end = kwargs.get("end", "\r\n")
    text = sep.join(str(a) for a in args)
    text = text.replace("\r\n", "\n").replace("\n", "\r\n")
    sys.stdout.write("\r\033[K%s%s" % (text, end))
    sys.stdout.flush()


def update_hud(v_lin, v_ang, arm, pump_on):
    """One-line live status display."""
    sys.stdout.write(
        "\r\033[K[BASE: %+.2f m/s, %+.2f rad/s] "
        "[ARM: X=%5.1f Y=%5.1f Z=%5.1f] [PUMP: %s]"
        % (v_lin, v_ang, arm.x, arm.y, arm.z, "ON " if pump_on else "OFF")
    )
    sys.stdout.flush()


def clamp(value, limits):
    return min(max(value, limits[0]), limits[1])


def smooth_velocity(current, target, accel, decel, dt):
    """Ramp velocity toward the target; use decel when slowing down."""
    diff = target - current
    braking = (current > 0 and target < current) or (current < 0 and target > current)
    rate = decel if braking else accel
    max_change = rate * dt
    if abs(diff) <= max_change:
        return target
    return current + (max_change if diff > 0 else -max_change)


class ArmState:
    """Target pose tracked locally, initialised from /SwiftproState_topic."""

    def __init__(self):
        self.x, self.y, self.z = HOME
        self.initialised = False

    def callback(self, msg):
        plausible = abs(msg.x) > 1.0 or abs(msg.y) > 1.0 or abs(msg.z) > 1.0
        if not self.initialised and plausible:
            self.x, self.y, self.z = msg.x, msg.y, msg.z
            self.initialised = True


def main():
    rospy.init_node("key_w", anonymous=True, argv=[sys.argv[0]])

    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    pos_pub = rospy.Publisher("/position_write_topic", position, queue_size=1)
    pump_pub = rospy.Publisher("/pump_topic", status, queue_size=1)

    arm = ArmState()
    rospy.Subscriber("/SwiftproState_topic", SwiftproState, arm.callback, queue_size=1)
    rospy.sleep(0.3)

    deadline = time.time() + 2.0
    while time.time() < deadline:
        if cmd_pub.get_num_connections() > 0 and pos_pub.get_num_connections() > 0:
            break
        rospy.sleep(0.05)

    if cmd_pub.get_num_connections() == 0:
        safe_print("WARNING: no subscriber on /cmd_vel.")
        safe_print("         Start the base driver or ~/codes/server.launch")
    if pos_pub.get_num_connections() == 0:
        safe_print("WARNING: no subscriber on /position_write_topic.")
        safe_print("         Start the arm driver: roslaunch swiftpro pro_control_nomoveit.launch")
    if not arm.initialised:
        safe_print("  arm pose unknown - press 0 first to home/sync the arm.")

    def set_base(linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        cmd_pub.publish(twist)

    def set_pump(on):
        msg = status()
        msg.status = 1 if on else 0
        pump_pub.publish(msg)

    def publish_pose():
        msg = position()
        msg.x, msg.y, msg.z = arm.x, arm.y, arm.z
        pos_pub.publish(msg)

    def go_home():
        arm.x, arm.y, arm.z = HOME
        publish_pose()

    last_warn_time = 0.0

    def step_arm(axis, dt):
        nonlocal last_warn_time
        field, direction = ARM_AXES[axis]
        target = {"x": arm.x, "y": arm.y, "z": arm.z}
        target[field] = clamp(target[field] + ARM_SPEED * dt * direction, ARM_LIMITS[field])

        if ik_reachable(target["x"], target["y"], target["z"]):
            arm.x, arm.y, arm.z = target["x"], target["y"], target["z"]
            publish_pose()
        else:
            now = time.time()
            if now - last_warn_time > 0.5:
                safe_print("arm limit: %s" % field)
                last_warn_time = now

    safe_print(HELP)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    target_linear = target_angular = 0.0
    current_linear = current_angular = 0.0
    last_linear_key = 0.0
    last_angular_key = 0.0

    arm_axis = None
    last_arm_key = 0.0
    last_arm_pub = 0.0
    last_hud_time = 0.0
    pump_on = False

    rate = rospy.Rate(20)
    last_time = time.time()

    try:
        tty.setraw(fd)
        while not rospy.is_shutdown():
            now = time.time()
            dt = max(0.01, min(0.1, now - last_time))
            last_time = now

            # Read every pending key in one loop pass.
            while select.select([sys.stdin], [], [], 0)[0]:
                char = os.read(fd, 1).decode(errors="ignore")
                if not char:
                    break

                lower = char.lower()

                if char == "\x03":                    # Ctrl+C
                    return
                elif char == " ":                     # Emergency stop
                    target_linear = target_angular = 0.0
                    current_linear = current_angular = 0.0
                    set_base(0.0, 0.0)
                    arm_axis = None
                    safe_print(">> STOP (base & arm halted)")
                elif char in ("\r", "\n"):            # Enter: pump toggle
                    pump_on = not pump_on
                    set_pump(pump_on)
                    safe_print("PUMP ON" if pump_on else "PUMP OFF")
                elif lower in ("w", "s"):             # Forward / backward
                    target_linear = LIN_SPEED if lower == "w" else -LIN_SPEED
                    last_linear_key = now
                elif lower in ("a", "d"):             # Turn left / right
                    target_angular = ANG_SPEED if lower == "a" else -ANG_SPEED
                    last_angular_key = now
                    if target_linear != 0.0:
                        last_linear_key = now          # keep driving through the arc
                elif char in ARM_AXES:                # Arm jogging
                    arm_axis = char
                    last_arm_key = now
                elif char == "5":                     # Arm stop / hold pose
                    arm_axis = None
                    safe_print(">> ARM STOP (holding pose)")
                elif char == "0":                     # Release + home
                    pump_on = False
                    set_pump(False)
                    target_linear = target_angular = 0.0
                    current_linear = current_angular = 0.0
                    set_base(0.0, 0.0)
                    go_home()
                    arm_axis = None
                    safe_print("RELEASE + HOME")

            # Target speeds: keep the last direction while the key repeats,
            # then ramp to zero after the timeout.
            if now - last_linear_key > KEY_TIMEOUT:
                target_linear = 0.0
            else:
                target_linear = LIN_SPEED if target_linear > 0 else -LIN_SPEED
            if now - last_angular_key > STEER_TIMEOUT:
                target_angular = 0.0
            else:
                target_angular = ANG_SPEED if target_angular > 0 else -ANG_SPEED

            current_linear = smooth_velocity(
                current_linear, target_linear, MAX_LIN_ACCEL, MAX_LIN_DECEL, dt)
            current_angular = smooth_velocity(
                current_angular, target_angular, MAX_ANG_ACCEL, MAX_ANG_DECEL, dt)
            set_base(current_linear, current_angular)

            # Arm: stop jogging shortly after the key is released.
            if arm_axis is not None and now - last_arm_key > ARM_KEY_TIMEOUT:
                arm_axis = None

            # Arm: stream small Cartesian steps at 20 Hz.
            if arm_axis is not None and now - last_arm_pub >= ARM_PUB_PERIOD:
                step_dt = now - last_arm_pub if last_arm_pub > 0.0 else ARM_PUB_PERIOD
                last_arm_pub = now
                step_arm(arm_axis, step_dt)

            if now - last_hud_time >= 0.1:
                update_hud(current_linear, current_angular, arm, pump_on)
                last_hud_time = now

            rate.sleep()

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        set_base(0.0, 0.0)
        set_pump(False)
        sys.stdout.write("\r\033[K")
        safe_print("\nstopped - base stopped, pump off")


if __name__ == "__main__":
    main()
