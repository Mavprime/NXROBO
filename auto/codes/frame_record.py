#!/usr/bin/env python3
"""Record camera frames to JPEGs for N seconds (field debugging aid)."""
import os
import sys
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
FPS = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
OUT = os.path.expanduser("~/logs/cake_frames")

rospy.init_node("frame_recorder", anonymous=True)
os.makedirs(OUT, exist_ok=True)
for name in os.listdir(OUT):
    if name.endswith(".jpg"):
        try:
            os.remove(os.path.join(OUT, name))
        except OSError:
            pass

bridge = CvBridge()
state = {"last": 0.0, "n": 0, "t0": time.time()}


def cb(msg):
    now = time.time()
    if now - state["last"] < 1.0 / FPS:
        return
    state["last"] = now
    try:
        image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception:
        return
    state["n"] += 1
    stamp = now - state["t0"]
    path = os.path.join(OUT, "f%05d_%06.2f.jpg" % (state["n"], stamp))
    cv2.imwrite(path, image)


rospy.Subscriber("/camera/color/image_raw", Image, cb, queue_size=1)
end = time.time() + DURATION
while time.time() < end and not rospy.is_shutdown():
    rospy.sleep(0.1)
print("recorded %d frames in %s" % (state["n"], OUT))
