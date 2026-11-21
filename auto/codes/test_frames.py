import rospy
from vision_msgs.msg import Detection2DArray

rospy.init_node("inspect_frames", anonymous=True)
for f in range(5):
    msg = rospy.wait_for_message("/objects", Detection2DArray, timeout=5.0)
    print(f"--- Frame {f+1} ({len(msg.detections)} detections) ---")
    for d in msg.detections:
        res = d.results[0]
        bb = d.bbox
        print(f"  id={res.id:2d} score={res.score:.2f} center=({bb.center.x:5.1f}, {bb.center.y:5.1f}) size=({bb.size_x:5.1f}, {bb.size_y:5.1f})")
    rospy.sleep(0.3)
