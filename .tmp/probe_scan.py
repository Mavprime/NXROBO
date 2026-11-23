#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""READ-ONLY probe: what does the scan see right now, and why is it dropped?

Subscribes to /objects and the aligned depth image; never publishes a motion
command.  Prints every clustered detection with the scan gate verdict, the
depth-derived arm coordinates and the reach verdict.
"""

import sys
import time

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")
sys.path.insert(0, "/home/spark/codes")

import numpy as np
import rospy
from sensor_msgs.msg import Image

import match_logic
import grasp_generic
from runtask import CamAction


def grab_depth():
    msg = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw",
                                 Image, timeout=8)
    raw = np.frombuffer(msg.data, dtype=np.uint16)
    return raw.reshape(msg.height, msg.width).astype(np.float64) / 1000.0


def depth_at(depth, u, v, half=2):
    u, v = int(round(u)), int(round(v))
    patch = depth[max(0, v - half):v + half + 1, max(0, u - half):u + half + 1]
    valid = patch[patch > 0]
    return float(np.median(valid)) if len(valid) else None


rospy.init_node("probe_scan", anonymous=True, disable_signals=True)
cam = CamAction()
rospy.sleep(1.0)

detections = cam.detector(num_samples=5, timeout=4.0, conf_threshold=0.04,
                          cake_conf_threshold=0.90,
                          cake_site_conf_threshold=0.60) or []
print("RAW_DETECTIONS: %s" % (detections,))

depth = None
try:
    depth = grab_depth()
except Exception as exc:
    print("DEPTH_FAIL: %s" % exc)

scale = 0.386784
for item in detections:
    try:
        cid = int(item[0])
        u, v = float(item[1][0]), float(item[1][1])
    except Exception:
        continue
    gate = 130.0 <= v <= 450.0 and 80.0 <= u <= 560.0
    z_est = None
    xy = None
    if depth is not None:
        d = depth_at(depth, u, v)
        if d is not None:
            point = grasp_generic.solve_arm_point(
                cam.transform3d, grasp_generic.unproject(u, v, d), scale)
            z_est = float(point[2])
            xy = (float(point[0]), float(point[1]))
    if xy is None:
        xy = match_logic.pixel_to_xy_at_z(cam.transform3d, u, v, -100.0)
        z_est = None
    mode = "cake" if match_logic.is_cake(cid) else "floor"
    reach = None
    if xy is not None:
        reach = grasp_generic.reach_state(mode, xy[0], xy[1], z_est=z_est)
    print("DET id=%d px=(%.0f,%.0f) gate_pass=%s depth=%s z_est=%s xy=%s "
          "reach=%s"
          % (cid, u, v, gate, depth_at(depth, u, v) if depth is not None else None,
             None if z_est is None else round(z_est, 1),
             None if xy is None else (round(xy[0], 1), round(xy[1], 1)),
             reach))
print("PROBE_DONE")
