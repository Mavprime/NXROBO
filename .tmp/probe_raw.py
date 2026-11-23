#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""READ-ONLY probe: raw /objects boxes with size + depth at the centre."""
import sys

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import numpy as np
import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


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


rospy.init_node("probe_raw", anonymous=True, disable_signals=True)

frames = []


def callback(msg):
    rows = []
    for det in msg.detections:
        if not det.results:
            continue
        rows.append((int(det.results[0].id), round(float(det.results[0].score), 3),
                     int(det.bbox.center.x), int(det.bbox.center.y),
                     int(det.bbox.size_x), int(det.bbox.size_y)))
    frames.append(rows)


rospy.Subscriber("/objects", Detection2DArray, callback, queue_size=10)
rospy.sleep(3.0)
print("FRAMES: %d" % len(frames))
depth = None
try:
    depth = grab_depth()
except Exception as exc:
    print("DEPTH_FAIL: %s" % exc)

seen = set()
for rows in frames[-2:]:
    for row in rows:
        key = (row[0], row[2] // 10, row[3] // 10)
        if key in seen:
            continue
        seen.add(key)
        d = depth_at(depth, row[2], row[3]) if depth is not None else None
        print("id=%-2d score=%.3f px=(%d,%d) size=%dx%d depth=%s"
              % (row[0], row[1], row[2], row[3], row[4], row[5], d))
print("PROBE_DONE")
