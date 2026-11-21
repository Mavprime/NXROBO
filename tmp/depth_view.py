#!/usr/bin/env python3
"""Save the colour frame and a colourised aligned-depth frame side by side.

Operator use: the depth edges must sit on the same image features as the colour
edges; a visible shift means the "aligned" depth stream is misregistered.
"""
import sys

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rospy.init_node("depth_view", anonymous=True)
bridge = CvBridge()

color = bridge.imgmsg_to_cv2(
    rospy.wait_for_message("/camera/color/image_raw", Image, timeout=15), "bgr8")
dmsg = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw",
                               Image, timeout=15)
depth = np.frombuffer(dmsg.data, dtype=np.uint16).reshape(
    dmsg.height, dmsg.width).astype(np.float64) / 1000.0

valid = depth > 0.05
scale = np.zeros_like(depth)
scale[valid] = np.clip((depth[valid] - 0.20) / (1.20 - 0.20), 0.0, 1.0)
grey = (scale * 255.0).astype(np.uint8)
depth_vis = cv2.applyColorMap(grey, cv2.COLORMAP_TURBO)
depth_vis[~valid] = (0, 0, 0)

cv2.imwrite("/home/spark/logs/color_view.jpg", color)
cv2.imwrite("/home/spark/logs/depth_view.png", depth_vis)
stack = np.vstack([color, depth_vis])
cv2.imwrite("/home/spark/logs/depth_pair.jpg", stack)
print("saved color_view.jpg depth_view.png depth_pair.jpg")
