#!/usr/bin/env python3
"""
Turn the uArm vacuum pump on for N seconds, then off.

The pump is controlled by /pump_topic of type swiftpro/status
(uint8 status: 1 = vacuum on, 0 = off), exactly as the shipped grasp code does.

Prerequisite: the arm driver must be running:
    roslaunch swiftpro pro_control_nomoveit.launch

Usage:
    python3 ~/semi/pump_test.py          # 3 seconds
    python3 ~/semi/pump_test.py 5        # custom duration

Keep pump runs short (a few seconds). Do not leave it running unattended.
"""

import sys
import time

import rospy
from swiftpro.msg import status


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0

    rospy.init_node("pump_test", anonymous=True, argv=[sys.argv[0]])
    pump = rospy.Publisher("/pump_topic", status, queue_size=1)

    # Wait briefly for the swiftpro driver to connect; otherwise the command
    # disappears with no subscriber.
    deadline = time.time() + 5.0
    while pump.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.1)
    if pump.get_num_connections() == 0:
        print("WARNING: no subscriber on /pump_topic - is the arm driver running?")
        print("         roslaunch swiftpro pro_control_nomoveit.launch")

    msg = status()
    msg.status = 1
    pump.publish(msg)
    print("pump ON for %.1f s" % duration)

    rospy.sleep(duration)

    msg.status = 0
    pump.publish(msg)
    print("pump OFF")

    rospy.sleep(0.3)


if __name__ == "__main__":
    main()
