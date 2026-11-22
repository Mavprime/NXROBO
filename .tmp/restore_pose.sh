source /opt/ros/noetic/setup.bash
source ~/spark_noetic/devel/setup.bash
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
timeout 30 python3 -u -c '
import sys, math, rospy
sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")
sys.path.insert(0, "/home/spark/codes")
from runtask import ArmAction
rospy.init_node("restore_pose", anonymous=True)
arm = ArmAction()
arm.drive_base(-0.127)
rospy.sleep(0.5)
arm.turn_base(-math.radians(3.0))
rospy.sleep(0.5)
print("POSE_RESTORED")
' 2>&1 | tail -n 5
rm -f /home/spark/codes/verify_approach.py /home/spark/codes/probe_scan.py /home/spark/codes/probe_raw.py /home/spark/codes/test_scan_and_approach.py
echo "CLEANED"
ls /home/spark/codes | wc -l