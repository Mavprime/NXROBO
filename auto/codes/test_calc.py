import rospy
from runtask import CamAction

rospy.init_node("check_dets", anonymous=True)
c = CamAction()
dets = c.detector(num_samples=6, timeout=6.0, conf_threshold=0.04)
print("Current detections (6 samples):", dets)
