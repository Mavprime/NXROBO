import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node("snap_view", anonymous=True)
msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5.0)
cv_img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
cv2.imwrite("/home/spark/current_view.jpg", cv_img)
print("SUCCESS_SAVED")
