source /opt/ros/noetic/setup.bash
source ~/spark_noetic/devel/setup.bash
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
timeout 30 python3 -u -c '
import sys, math, rospy
sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")
sys.path.insert(0, "/home/spark/codes")
from runtask import ArmAction
rospy.init_node("restore_heading3", anonymous=True)
arm = ArmAction()
arm.turn_base(-math.radians(63.5))
rospy.sleep(0.5)
print("HEADING_RESTORED_63")
' 2>&1 | tail -n 4