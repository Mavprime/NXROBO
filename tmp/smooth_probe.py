#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Probe: does the uArm follow a streamed stream of waypoints smoothly?

Streams a vertical and a horizontal line by publishing interpolated
``position_write_topic`` targets at ~25 Hz (the driver's ``set_pose`` with
delay=0 only publishes), while watching the orange pad in the fixed camera.
If the observed pad pixel tracks the commanded motion smoothly and the lag
stays constant, the arm firmware handles waypoint streaming and we can replace
the step-and-wait motion with a continuous trajectory.

Nothing is grasped. The path is in open air at y = +90 mm.
"""
import sys
import time

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import rospy

import match_logic
from pad_detect import grab, pad_pixel
from runtask import ArmAction, CamAction

M = None
HZ = 25.0
DT = 1.0 / HZ
LINE1 = ((250.0, 90.0, 80.0), (250.0, 90.0, 20.0))
LINE2 = ((250.0, 90.0, 60.0), (180.0, 90.0, 60.0))


def project(x, y, z):
    return match_logic.project_arm(M, x, y, z)


def stream_line(arm, bridge, start, end, duration, tag):
    steps = max(2, int(duration * HZ))
    samples = []
    t0 = time.monotonic()
    for i in range(1, steps + 1):
        f = i / float(steps)
        x = start[0] + (end[0] - start[0]) * f
        y = start[1] + (end[1] - start[1]) * f
        z = start[2] + (end[2] - start[2]) * f
        arm.interface.set_pose(x, y, z, delay=0)
        time.sleep(DT)
        if i % 2 == 0 or i == steps:
            image = grab(bridge)
            pad = pad_pixel(image)
            pred = project(x, y, z)
            samples.append((time.monotonic() - t0, x, y, z, pad, pred))
    print("[STREAM] %s: %d waypoints in %.2f s (%.1f mm/s)"
          % (tag, steps, time.monotonic() - t0,
             ((start[0] - end[0]) ** 2 + (start[1] - end[1]) ** 2
              + (start[2] - end[2]) ** 2) ** 0.5 / max(duration, 1e-3)))
    prev_v = None
    for (t, x, y, z, pad, pred) in samples:
        if pad is None:
            print("  t=%.2f cmd=(%6.1f,%5.1f,%6.1f) pad=None pred=(%.1f,%.1f)"
                  % (t, x, y, z, pred[0], pred[1]))
        else:
            dv = "" if prev_v is None else " dv=%+5.1f" % (pad[1] - prev_v)
            print("  t=%.2f cmd=(%6.1f,%5.1f,%6.1f) pad=(%.1f,%.1f) a=%.0f "
                  "pred=(%.1f,%.1f) err=%+.1f%s"
                  % (t, x, y, z, pad[0], pad[1], pad[2], pred[0], pred[1],
                     pad[1] - pred[1], dv))
            prev_v = pad[1]
    return samples


rospy.init_node("smooth_probe", anonymous=True)
cam = CamAction()
arm = ArmAction(cam=cam)
M = cam.transform3d
bridge = __import__("cv_bridge").CvBridge()

try:
    arm.interface.set_status(True)
    rospy.sleep(0.3)
    # go to the start of line 1 using the ordinary move
    arm.interface.set_pose(130.0, 0.0, 80.0, delay=0.8)
    rospy.sleep(0.5)
    arm.interface.set_pose(*LINE1[0], delay=0.8)
    rospy.sleep(0.8)
    image = grab(bridge)
    print("[START] pad:", pad_pixel(image))

    stream_line(arm, bridge, LINE1[0], LINE1[1], 1.5, "vertical down 60mm")
    rospy.sleep(0.3)
    stream_line(arm, bridge, LINE1[1], LINE2[0], 1.5, "vertical up 60mm")
    rospy.sleep(0.3)
    stream_line(arm, bridge, LINE2[0], LINE2[1], 1.2, "horizontal 70mm")
    rospy.sleep(0.3)
    stream_line(arm, bridge, LINE2[1], LINE2[0], 1.2, "horizontal back 70mm")
finally:
    try:
        arm.arm_home()
    except Exception:
        pass
    print("[DONE]")
