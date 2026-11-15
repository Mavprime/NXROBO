#!/usr/bin/env python3
"""
Save one camera frame plus its blue mask.

Useful when the detector reports "no blue box": copy the two PNG files to
your PC and look at them.

Run on the robot:

    source ~/spark_noetic/devel/setup.bash
    python3 ~/codes/blue_snap.py

Then from your PC:

    scp spark@10.25.141.8:/tmp/blue.png spark@10.25.141.8:/tmp/blue_mask.png .

Optional arguments:
    python3 blue_snap.py /tmp/my.png /tmp/my_mask.png
"""

import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

LOWER = np.array([90, 50, 50])
UPPER = np.array([135, 255, 255])

IMAGE_PATH = sys.argv[1] if len(sys.argv) > 1 else "/tmp/blue.png"
MASK_PATH = sys.argv[2] if len(sys.argv) > 2 else "/tmp/blue_mask.png"


def main():
    rospy.init_node("blue_snap", anonymous=True, argv=[sys.argv[0]])
    msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5.0)
    frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER, UPPER)

    cv2.imwrite(IMAGE_PATH, frame)
    cv2.imwrite(MASK_PATH, mask)
    print("saved %s and %s" % (IMAGE_PATH, MASK_PATH))
    print("blue pixels: %d" % int(mask.sum() / 255))


if __name__ == "__main__":
    main()
