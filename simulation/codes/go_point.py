#!/usr/bin/env python3
"""Send one move_base goal in the arena/map frame.

Usage:
    python3 go_point.py X Y YAW_DEG

Example (D -> W -> D):
    python3 go_point.py 1.09 1.50 0
    python3 go_point.py 0.225 0.225 45

Blocks until the goal finishes, prints the actionlib state and returns
non-zero unless the goal SUCCEEDED. Goals must be sent with actionlib;
`rostopic pub /move_base/goal` is reported as PREEMPTED.
"""

from __future__ import annotations

import math
import sys

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

SUCCEEDED = 3


def main() -> int:
    if len(sys.argv) != 4:
        print(__doc__)
        return 2
    x, y, yaw_deg = (float(v) for v in sys.argv[1:4])

    rospy.init_node("go_point", anonymous=True)
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(15.0)):
        print("NO_ACTION_SERVER")
        return 1

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    yaw = math.radians(yaw_deg)
    goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)

    client.send_goal(goal)
    ok = client.wait_for_result(rospy.Duration(90.0))
    state = client.get_state()
    print("RESULT ok=%s state=%s" % (ok, state))
    return 0 if ok and state == SUCCEEDED else 1


if __name__ == "__main__":
    sys.exit(main())
