#!/usr/bin/env python3
"""
Keyboard control for the Spark robot (base + uArm + vacuum pump).

Base (hold for continuous motion - terminal key repeat):
    w  forward          s  backward
    a  turn left        d  turn right

Arm (hold for continuous motion):
    8  up (z+)          2  down (z-)
    7  forward (x+)     9  backward (x-)
    4  swing counterclockwise around the base (y+)
    6  swing clockwise around the base          (y-)
    1  wrist counterclockwise (angle+)    (extra: end-effector spin)
    3  wrist clockwise        (angle-)

Other:
    Enter  toggle vacuum pump: grab / release (one toggle per press)
    0      release the box and return the arm to its resting pose
           (pump off, arm to x=120 y=0 z=35, wrist angle 0, base stopped)
    Ctrl+C stop everything (base stopped, pump off)

Topics:
    /cmd_vel              geometry_msgs/Twist       base motion
    /position_write_topic swiftpro/position         arm x,y,z in mm (absolute)
    /angle4th_topic       swiftpro/angle4th         wrist angle in degrees
    /pump_topic           swiftpro/status           vacuum 1/0
    /SwiftproState_topic  swiftpro/SwiftproState    current pose feedback

Prerequisites:
    base: roslaunch spark_bringup driver_bringup.launch camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6
    arm:  roslaunch swiftpro pro_control_nomoveit.launch

Usage:
    ssh spark@10.25.141.8
    source ~/spark_noetic/devel/setup.bash
    python3 ~/codes/keyboard_teleop.py

Environment overrides:
    LIN_SPEED=0.15   m/s
    ANG_SPEED=0.6    rad/s
    ARM_STEP=5       mm per 0.1 s (50 mm/s)
    WRIST_STEP=5     degrees per 0.1 s (50 deg/s)
    KEY_TIMEOUT=0.5  seconds of coast after the last repeated key

The arm stops at the workspace limit if the requested pose is unreachable
(checked with the same IK math as the firmware).
"""

import os
import select
import sys
import termios
import time
import tty

import rospy
from geometry_msgs.msg import Twist
from swiftpro.msg import SwiftproState, angle4th, position, status

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_ik import ik_reachable

LIN_SPEED = float(os.environ.get("LIN_SPEED", "0.15"))     # m/s
ANG_SPEED = float(os.environ.get("ANG_SPEED", "0.6"))      # rad/s
ARM_STEP = float(os.environ.get("ARM_STEP", "5"))          # mm / 0.1 s
WRIST_STEP = float(os.environ.get("WRIST_STEP", "5"))      # deg / 0.1 s
KEY_TIMEOUT = float(os.environ.get("KEY_TIMEOUT", "0.5"))  # s

HOME = (120.0, 0.0, 35.0)
HOME_ANGLE = 0.0
ARM_LIMITS = {"x": (0.0, 350.0), "y": (-160.0, 160.0), "z": (-60.0, 170.0)}
WRIST_LIMITS = (0.0, 180.0)
ARM_PUB_PERIOD = 0.1

HELP = """
  Base:  w forward   s backward   a turn left   d turn right
  Arm:   8 up   2 down   7 forward   9 backward
         4 swing CCW   6 swing CW       1 wrist CCW   3 wrist CW   (hold)
  Enter: grab/release (pump toggle)    0: release + arm home
  Ctrl+C: stop everything
"""


class ArmState:
    """Target pose tracked locally, initialised from SwiftproState_topic."""

    def __init__(self):
        self.x, self.y, self.z = HOME
        self.angle = HOME_ANGLE
        self.initialised = False

    def callback(self, msg):
        # Before the first move the driver reports 0,0,0; that is not a real
        # pose, so ignore it and keep the HOME default until a real one arrives.
        plausible = abs(msg.x) > 1 or abs(msg.y) > 1 or abs(msg.z) > 1
        if not self.initialised and plausible:
            self.x, self.y, self.z = msg.x, msg.y, msg.z
            self.angle = msg.motor_angle4
            self.initialised = True


def clamp(value, limits):
    return min(max(value, limits[0]), limits[1])


def main():
    rospy.init_node("keyboard_teleop", anonymous=True, argv=[sys.argv[0]])

    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    pos_pub = rospy.Publisher("/position_write_topic", position, queue_size=1)
    angle_pub = rospy.Publisher("/angle4th_topic", angle4th, queue_size=1)
    pump_pub = rospy.Publisher("/pump_topic", status, queue_size=1)

    arm = ArmState()
    rospy.Subscriber("/SwiftproState_topic", SwiftproState, arm.callback, queue_size=1)
    rospy.sleep(0.5)

    # Warn if the arm driver is not up: publishing to /position_write_topic
    # with no subscriber means the arm silently never moves.
    deadline = time.time() + 2.0
    while pos_pub.get_num_connections() == 0 and time.time() < deadline:
        rospy.sleep(0.1)
    # Same check for the base: no subscriber means w/a/s/d do nothing.
    deadline = time.time() + 2.0
    while cmd_pub.get_num_connections() == 0 and time.time() < deadline:
        rospy.sleep(0.1)
    if cmd_pub.get_num_connections() == 0:
        print("WARNING: no subscriber on /cmd_vel.")
        print("         Start the base driver: roslaunch spark_bringup driver_bringup.launch "
              "camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6")
        print("         Base keys will not move the robot until it is running.")

    if pos_pub.get_num_connections() == 0:
        print("WARNING: no subscriber on /position_write_topic.")
        print("         Start the arm driver: roslaunch swiftpro pro_control_nomoveit.launch")
        print("         Base keys still work; arm keys and the pump will not.")

    if not arm.initialised:
        print("  arm pose unknown - press 0 first to home/sync the arm.")

    def set_pump(on):
        message = status()
        message.status = 1 if on else 0
        pump_pub.publish(message)

    def set_base(linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        cmd_pub.publish(twist)

    def publish_pose():
        message = position()
        message.x, message.y, message.z = arm.x, arm.y, arm.z
        pos_pub.publish(message)

    def publish_angle():
        message = angle4th()
        message.angle4th = arm.angle
        angle_pub.publish(message)

    def go_home():
        arm.x, arm.y, arm.z = HOME
        arm.angle = HOME_ANGLE
        publish_pose()
        publish_angle()

    print(HELP)
    print("  base: %.2f m/s / %.2f rad/s   arm: %.0f mm/0.1s   wrist: %.0f deg/0.1s"
          % (LIN_SPEED, ANG_SPEED, ARM_STEP, WRIST_STEP))

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    linear = angular = 0.0
    last_base_key = 0.0
    arm_axis = None
    last_arm_key = 0.0
    last_arm_pub = 0.0
    pump_on = False

    try:
        tty.setraw(fd)
        while not rospy.is_shutdown():
            now = time.time()
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)

            if ready:
                char = os.read(fd, 1).decode(errors="ignore")

                if char == "\x03":                       # Ctrl+C
                    break
                elif char in ("\r", "\n"):               # Enter
                    pump_on = not pump_on
                    set_pump(pump_on)
                    print("PUMP ON" if pump_on else "PUMP OFF")
                elif char in "wWaAsSdD":                 # base
                    key = char.lower()
                    if key == "w":
                        linear, angular = LIN_SPEED, 0.0
                    elif key == "s":
                        linear, angular = -LIN_SPEED, 0.0
                    elif key == "a":
                        linear, angular = 0.0, ANG_SPEED
                    elif key== "d":
                        linear, angular = 0.0, -ANG_SPEED
                    last_base_key = now
                elif char in "82794613":                 # arm
                    arm_axis = {"8": "z+", "2": "z-", "7": "x+", "9": "x-",
                                "4": "y+", "6": "y-",
                                "1": "angle+", "3": "angle-"}[char]
                    last_arm_key = now
                elif char == "0":                        # release + home
                    pump_on = False
                    set_pump(False)
                    set_base(0.0, 0.0)
                    linear = angular = 0.0
                    go_home()
                    arm_axis = None
                    print("RELEASE + HOME")

            # base: stop after the last repeated key
            if now - last_base_key > KEY_TIMEOUT:
                linear = angular = 0.0
            set_base(linear, angular)

            # arm: move in steps while the key is repeating
            if arm_axis is not None and now - last_arm_key > KEY_TIMEOUT:
                arm_axis = None

            if arm_axis is not None and now - last_arm_pub >= ARM_PUB_PERIOD:
                last_arm_pub = now
                if arm_axis in ("x+", "x-"):
                    step = ARM_STEP if arm_axis == "x+" else -ARM_STEP
                    candidate = clamp(arm.x + step, ARM_LIMITS["x"])
                    if ik_reachable(candidate, arm.y, arm.z):
                        arm.x = candidate
                        publish_pose()
                    else:
                        print("arm limit: x")
                elif arm_axis in ("y+", "y-"):
                    step = ARM_STEP if arm_axis == "y+" else -ARM_STEP
                    candidate = clamp(arm.y + step, ARM_LIMITS["y"])
                    if ik_reachable(arm.x, candidate, arm.z):
                        arm.y = candidate
                        publish_pose()
                    else:
                        print("arm limit: y")
                elif arm_axis in ("z+", "z-"):
                    step = ARM_STEP if arm_axis == "z+" else -ARM_STEP
                    candidate = clamp(arm.z + step, ARM_LIMITS["z"])
                    if ik_reachable(arm.x, arm.y, candidate):
                        arm.z = candidate
                        publish_pose()
                    else:
                        print("arm limit: z")
                elif arm_axis in ("angle+", "angle-"):
                    step = WRIST_STEP if arm_axis == "angle+" else -WRIST_STEP
                    candidate = clamp(arm.angle + step, WRIST_LIMITS)
                    if candidate != arm.angle:
                        arm.angle = candidate
                        publish_angle()
                    else:
                        print("arm limit: wrist")

            rospy.sleep(0.01)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        set_base(0.0, 0.0)
        set_pump(False)
        print("\nstopped - base stopped, pump off")


if __name__ == "__main__":
    main()
