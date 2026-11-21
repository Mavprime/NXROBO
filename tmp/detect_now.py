#!/usr/bin/env python3
"""Print current /objects detections (class, score, pixel) for the operator."""
import sys

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import rospy
from vision_msgs.msg import Detection2DArray

rospy.init_node("objects_probe", anonymous=True)
for _ in range(2):
    try:
        msg = rospy.wait_for_message("/objects", Detection2DArray, timeout=15)
    except Exception as exc:
        print("no /objects message: %s" % exc)
        break
    print("detections: %d" % len(msg.detections))
    for det in msg.detections:
        cid = det.results[0].id if det.results else "?"
        score = det.results[0].score if det.results else float("nan")
        print("  class=%s score=%.3f center=(%.0f, %.0f) size=(%.0f, %.0f)"
              % (cid, score, det.bbox.center.x, det.bbox.center.y,
                 det.bbox.size_x, det.bbox.size_y))
