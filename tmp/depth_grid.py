#!/usr/bin/env python3
"""Print a coarse depth grid over the tower region to check registration."""
import sys

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import numpy as np
import rospy
from sensor_msgs.msg import Image

rospy.init_node("depth_grid", anonymous=True)
msg = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw",
                              Image, timeout=15)
d = np.frombuffer(msg.data, dtype=np.uint16).reshape(
    msg.height, msg.width).astype(np.float64) / 1000.0
print("depth grid (metres, x100 = no data), rows v=20..250, cols u=280..450")
header = "      " + " ".join("%5d" % u for u in range(280, 451, 20))
print(header)
for v in range(20, 251, 20):
    cells = []
    for u in range(280, 451, 20):
        patch = d[max(0, v - 1):v + 2, max(0, u - 1):u + 2]
        valid = patch[patch > 0.05]
        cells.append("%5.2f" % float(np.median(valid)) if len(valid) else "   --")
    print("%5d " % v + " ".join(cells))
