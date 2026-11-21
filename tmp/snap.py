#!/usr/bin/env python3
"""Snapshot the D435 colour image to a JPEG (operator scene check)."""
import sys

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

OUT = sys.argv[1] if len(sys.argv) > 1 else "/home/spark/logs/snap.jpg"
FSYNC = sys.argv[2] if len(sys.argv) > 2 else "0"

rospy.init_node("snap_view", anonymous=True)
if FSYNC == "1":
    # Flush the detector's /objects UI redraw out of the way: the operator wants
    # the raw camera picture, not the annotated detection view.
    rospy.sleep(0.2)
msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=15)
image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
cv2.imwrite(OUT, image)
print("saved %s %s" % (OUT, image.shape))
