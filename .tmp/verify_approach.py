#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Field verification of the fixed out-of-reach handling (approach only).

Runs exactly the production ``_prepare_target`` (scan -> reach check -> base
approach -> re-scan) but stops before the grasp sequence.  The distance the base
actually commands is recorded and undone at the end, so the robot is left where
it started.
"""

import sys

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")
sys.path.insert(0, "/home/spark/codes")

import rospy

import grasp_generic as gg
from runtask import ArmAction, CamAction


def main():
    rospy.init_node("verify_approach", anonymous=True, disable_signals=True)
    gg._install_signal_handlers()

    cam = CamAction()
    arm = ArmAction(cam=cam)
    gg._wait_for_arm_driver(arm)
    arm.interface.set_status(True)
    rospy.sleep(0.20)

    travelled = {"m": 0.0}
    original_drive = arm.drive_base

    def recording_drive(dist_m):
        original_drive(dist_m)
        travelled["m"] += float(dist_m)

    arm.drive_base = recording_drive

    scale, _cake_touch = gg.load_memory()
    entry = {"target": None}
    try:
        target = gg._prepare_target(
            cam, arm, scale, gg.CAKE_CUP_OFFSET_X, False, entry,
            conf_threshold=gg.DEFAULT_DETECT_CONF,
            cake_conf_threshold=gg.DEFAULT_CAKE_CONF,
            cake_site_conf_threshold=gg.DEFAULT_CAKE_SITE_CONF)
        print("APPROACH_RESULT: %s" % (target,))
        print("PIXEL_IN_NEAR_FIELD: %s"
              % gg.match_logic.in_near_field(*target["pixel"]))
    except Exception as exc:
        print("APPROACH_FAILED: %s: %s" % (type(exc).__name__, exc))
    finally:
        total = travelled["m"]
        print("BASE_TRAVELLED: %+.1f cm" % (total * 100.0))
        if abs(total) >= 0.02:
            print("RESTORING START POSITION: %+.1f cm" % (-total * 100.0))
            original_drive(-total)
            rospy.sleep(1.0)
        print("VERIFY_DONE")


main()
