#!/usr/bin/env python3
"""
Move the uArm end effector to an absolute Cartesian pose.

Prerequisite: arm driver running on the robot:
    roslaunch swiftpro pro_control_nomoveit.launch

Usage:
    source ~/spark_noetic/devel/setup.bash
    python3 ~/semi/arm_move.py --x 200 --y 0 --z 20
    python3 ~/semi/arm_move.py --home
    python3 ~/semi/arm_move.py --x 198.8 --y 0.6 --z -50

Coordinates are in the same units as ~/thefile.txt (millimetres).
Home preset: (120, 0, 35).
"""

import sys

import rospy
from swiftpro.msg import position

HOME = (120.0, 0.0, 35.0)


def parse_args(argv):
    args = {"x": None, "y": None, "z": None, "home": False}
    i = 1
    while i < len(argv):
        token = argv[i]
        if token == "--home":
            args["home"] = True
            i += 1
        elif token in ("--x", "--y", "--z") and i + 1 < len(argv):
            args[token[2:]] = float(argv[i + 1])
            i += 2
        elif ":=" in token or token.startswith("__"):
            # ROS remapping / private args: leave them for rospy
            i += 1
        else:
            raise SystemExit("unknown argument: %s" % token)
    if not args["home"] and None in (args["x"], args["y"], args["z"]):
        raise SystemExit("give --x --y --z, or --home")
    return args


def main():
    args = parse_args(sys.argv)
    if args["home"]:
        x, y, z = HOME
    else:
        x, y, z = args["x"], args["y"], args["z"]

    rospy.init_node("arm_move", anonymous=True, argv=[sys.argv[0]])
    pub = rospy.Publisher("/position_write_topic", position, queue_size=10)
    rospy.sleep(0.5)  # let the publisher connect

    pose = position()
    pose.x, pose.y, pose.z = x, y, z
    print("moving arm to x=%.1f y=%.1f z=%.1f" % (x, y, z))
    pub.publish(pose)
    rospy.sleep(0.5)


if __name__ == "__main__":
    main()
