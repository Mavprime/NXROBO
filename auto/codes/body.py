#!/usr/bin/env python3
"""
Base-only keyboard teleop for the Spark robot.

Hold a key for continuous motion (terminal key repeat):
    w  forward          s  backward
    a  turn left        d  turn right

Ctrl+C stops the base.

This is the base half of the old key_fullbody.py. Run it in its own terminal
alongside arm.py; both connect to the same ROS master.

Prerequisites:
    base: roslaunch spark_bringup driver_bringup.launch camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6
          (or: roslaunch ~/semi/server.launch)

Usage:
    ssh spark@10.25.141.8
    source ~/spark_noetic/devel/setup.bash
    python3 ~/semi/body.py

Environment overrides:
    LIN_SPEED=0.15     m/s
    ANG_SPEED=0.6      rad/s
    KEY_TIMEOUT=0.5    seconds of coast after the last repeated key

Topic:
    /cmd_vel   geometry_msgs/Twist   base motion
"""

import os
import select
import sys
import termios
import time
import tty

import rospy
from geometry_msgs.msg import Twist

LIN_SPEED = float(os.environ.get("LIN_SPEED", "0.15"))    # m/s
ANG_SPEED = float(os.environ.get("ANG_SPEED", "0.6"))     # rad/s
KEY_TIMEOUT = float(os.environ.get("KEY_TIMEOUT", "0.5"))  # s

HELP = """
  Base:  w forward   s backward   a turn left   d turn right   (hold)
  Ctrl+C: stop
"""


def main():
    rospy.init_node("body", anonymous=True, argv=[sys.argv[0]])

    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.sleep(0.5)

    deadline = time.time() + 2.0
    while cmd_pub.get_num_connections() == 0 and time.time() < deadline:
        rospy.sleep(0.1)
    if cmd_pub.get_num_connections() == 0:
        print("WARNING: no subscriber on /cmd_vel.")
        print("         Start the base driver or ~/semi/server.launch")
        print("         Keys will not move the robot until it is running.")

    def set_base(linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        cmd_pub.publish(twist)

    print(HELP)
    print("  base: %.2f m/s / %.2f rad/s" % (LIN_SPEED, ANG_SPEED))

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    linear = angular = 0.0
    last_base_key = 0.0

    try:
        tty.setraw(fd)
        while not rospy.is_shutdown():
            now = time.time()
            ready, _, _ = select.select([sys.stdin], [], [], 0.02)

            if ready:
                char = os.read(fd, 1).decode(errors="ignore")

                if char == "\x03":                       # Ctrl+C
                    break
                elif char in "wWaAsSdD":                 # base
                    key = char.lower()
                    if key == "w":
                        linear, angular = LIN_SPEED, 0.0
                    elif key == "s":
                        linear, angular = -LIN_SPEED, 0.0
                    elif key == "a":
                        linear, angular = 0.0, ANG_SPEED
                    elif key == "d":
                        linear, angular = 0.0, -ANG_SPEED
                    last_base_key = now

            # Stop after the last repeated key.
            if now - last_base_key > KEY_TIMEOUT:
                linear = angular = 0.0
            set_base(linear, angular)

            rospy.sleep(0.005)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        set_base(0.0, 0.0)
        rospy.sleep(0.1)
        set_base(0.0, 0.0)
        print("\nstopped - base stopped")


if __name__ == "__main__":
    main()
