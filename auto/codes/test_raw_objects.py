import rospy
from vision_msgs.msg import Detection2DArray

rospy.init_node("check_raw_objects", anonymous=True)
msg = rospy.wait_for_message("/objects", Detection2DArray, timeout=5.0)
print(f"Total detections: {len(msg.detections)}")
for i, d in enumerate(msg.detections):
    res = d.results[0]
    bb = d.bbox
    print(f"[{i}] id={res.id} score={res.score:.2f} center=({bb.center.x:.1f}, {bb.center.y:.1f}) size=({bb.size_x:.1f}, {bb.size_y:.1f})")
