#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Live camera view with a GREEN box drawn around the orange suction pad.

Same mask and rejection rules as the real pipeline (pad_detect.py), so what
you see here is what runtask.py's startup pad check sees.  Use it to aim the
camera / position the wrist until the pad is boxed in green.

Run on the robot (camera driver must already be publishing):

    export DISPLAY=:0
    export XAUTHORITY=$HOME/.Xauthority
    source ~/spark_noetic/devel/setup.bash
    python3 ~/auto/codes/pad_view.py

Press Ctrl+C in the terminal, or `pkill -f pad_view.py`, to close it.
"""

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from pad_detect import LOWER, UPPER, MIN_AREA, BORDER_PX

WINDOW = "pad_view"
GREEN = (0, 255, 0)
RED = (0, 0, 255)


class PadViewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.kernel = np.ones((5, 5), np.uint8)

    def annotate(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER, UPPER)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        height, width = frame.shape[:2]

        if not contours:
            cv2.putText(frame, "no orange pad", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, RED, 2)
            return frame

        blob = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(blob)
        x, y, w, h = cv2.boundingRect(blob)
        clipped = (x <= BORDER_PX or y <= BORDER_PX
                   or x + w >= width - BORDER_PX or y + h >= height - BORDER_PX)

        if area < MIN_AREA or clipped:
            reason = "clipped" if clipped else "too small"
            cv2.putText(frame, "pad %s (area=%d)" % (reason, int(area)),
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, RED, 2)
            cv2.rectangle(frame, (x, y), (x + w, y + h), RED, 1)
            return frame

        moments = cv2.moments(blob)
        cx, cy = int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"])
        cv2.rectangle(frame, (x, y), (x + w, y + h), GREEN, 2)
        cv2.circle(frame, (cx, cy), 4, GREEN, -1)
        cv2.putText(frame, "PAD (%d,%d) area=%d" % (cx, cy, int(area)),
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, GREEN, 2)
        return frame

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow(WINDOW, self.annotate(frame))
        cv2.waitKey(1)

    def run(self):
        rospy.init_node("pad_view", anonymous=True)
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)
        rospy.loginfo("pad_view ready")
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    PadViewer().run()
