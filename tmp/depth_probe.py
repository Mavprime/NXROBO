#!/usr/bin/env python3
"""Read the aligned depth frame and print depth around the cake top pixel.

Also prints the camera intrinsics from camera_info, so the operator can turn
pixel + depth into a 3D point.  Nothing moves.
"""
import sys

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image

rospy.init_node("depth_probe", anonymous=True)

info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=15)
print("color intrinsics: fx=%.3f fy=%.3f cx=%.3f cy=%.3f  frame=%s"
      % (info.K[0], info.K[4], info.K[2], info.K[5], info.header.frame_id))

msg = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw",
                              Image, timeout=15)
print("depth encoding=%s step=%d w=%d h=%d" %
      (msg.encoding, msg.step, msg.width, msg.height))

if msg.encoding in ("16UC1", "mono16"):
    depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
    scale = 0.001  # mm -> m
elif msg.encoding == "32FC1":
    depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    scale = 1.0
else:
    print("unexpected encoding, raw len=%d" % len(msg.data))
    sys.exit(1)

for (u, v, name) in [(362, 71, "cake top face"),
                     (364, 53, "pad probe pos"),
                     (351, 311, "front-row bear"),
                     (320, 240, "image centre"),
                     (320, 400, "near floor")]:
    patch = depth[max(0, v - 2):v + 3, max(0, u - 2):u + 3].astype(np.float64) * scale
    valid = patch[patch > 0]
    if len(valid):
        print("%-18s (%d,%d): median=%.4f m  n=%d  min=%.4f max=%.4f"
              % (name, u, v, float(np.median(valid)), len(valid),
                 float(valid.min()), float(valid.max())))
    else:
        print("%-18s (%d,%d): no valid depth (%d samples)"
              % (name, u, v, len(valid)))
