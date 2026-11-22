source /opt/ros/noetic/setup.bash
source ~/spark_noetic/devel/setup.bash
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
timeout 90 python3 -u ~/codes/grasp_generic.py --scan-only 2>&1 | tail -n 10