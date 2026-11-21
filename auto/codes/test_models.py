import os
import cv2
import yolov5
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node("test_models", anonymous=True)
msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5)
img = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")

m1_path = os.path.expanduser("~/spark_noetic/src/3rd_app/auto_match_25/auto_match/models/best.pt")
m2_path = os.path.expanduser("~/spark_noetic/src/3rd_app/auto_match_25/auto_match/models/banana.pt")

m1 = yolov5.load(m1_path)
m1.conf = 0.05
res1 = m1(img)
print("=== Model 1 (generic best.pt) ===")
for p in res1.xyxy[0]:
    x1, y1, x2, y2, conf, cls = [float(v) for v in p[:6]]
    name = m1.model.names[int(cls)]
    print(f"  {name:12s} cls={int(cls):2d} conf={conf:.2f} cx={(x1+x2)/2:.1f} cy={(y1+y2)/2:.1f} w={x2-x1:.1f} h={y2-y1:.1f}")

m2 = yolov5.load(m2_path)
m2.conf = 0.05
res2 = m2(img)
print("=== Model 2 (banana.pt) ===")
for p in res2.xyxy[0]:
    x1, y1, x2, y2, conf, cls = [float(v) for v in p[:6]]
    name = m2.model.names[int(cls)]
    print(f"  {name:12s} cls={int(cls):2d} conf={conf:.2f} cx={(x1+x2)/2:.1f} cy={(y1+y2)/2:.1f} w={x2-x1:.1f} h={y2-y1:.1f}")
