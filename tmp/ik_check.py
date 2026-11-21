#!/usr/bin/env python3
"""Print analytical IK reachability for candidate cake-grasp poses."""
import sys

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import match_logic

POSES = [(348, -26, 120), (348, -26, 95), (348, -26, 80), (348, -26, 60),
         (348, -26, 30), (348, -26, 0), (348, -26, -10), (348, -26, -15),
         (340, -26, 95), (340, -26, 0), (340, -26, -15),
         (350, -26, 75), (300, -16, 95), (250, -26, 95), (130, 0, 95),
         (50, 180, 160), (130, 0, 35)]
for pose in POSES:
    print("(%.0f, %.0f, %.0f): ik=%s limit=%s"
          % (pose[0], pose[1], pose[2],
             match_logic.ik_reachable(*pose),
             # is_in_arm_limit needs an ArmAction; replicate the cheap bounds
             (50.0 <= pose[0] <= 350.0 and -210.0 <= pose[1] <= 210.0
              and (pose[0] ** 2 + pose[1] ** 2) ** 0.5 <= 355.0)))
