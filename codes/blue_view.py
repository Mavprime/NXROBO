#!/usr/bin/env python3
"""
Live blue-box camera view with detection overlay, shown on the robot screen.

Draws the largest blue contour as a green box, marks its centre and prints
the pixel coordinates and area on the image. Requires the camera driver and
a display:

    export DISPLAY=:0
    export XAUTHORITY=$HOME/.Xauthority
    python3 ~/codes/blue_view.py

Press Ctrl+C in the terminal, or `pkill -f blue_view.py`, to close it.
"""

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

LOWER = np.array([90, 50, 50])
UPPER = np.array([135, 255, 255])
MIN_AREA = 300
WINDOW = "blue_detect"


class BlueViewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), LOWER, UPPER)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        mask = cv2.dilate(cv2.erode(mask, None, iterations=4), None, iterations=4)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) > MIN_AREA]

        if contours:
            biggest = max(contours, key=cv2.contourArea)
            area = int(cv2.contourArea(biggest))
            x, y, w, h = cv2.boundingRect(biggest)
            moment = cv2.moments(biggest)
            if moment["m00"] > 0:
                cx = int(moment["m10"] / moment["m00"])
                cy = int(moment["m01"] / moment["m00"])
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
                cv2.putText(frame, "BLUE (%d,%d) area=%d" % (cx, cy, area),
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "no blue box", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        cv2.imshow(WINDOW, frame)
        cv2.waitKey(1)

    def run(self):
        rospy.init_node("blue_view", anonymous=True)
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)
        rospy.loginfo("blue_view ready")
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    BlueViewer().run()
