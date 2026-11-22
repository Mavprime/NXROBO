echo "=== grasp_test_run.log ==="
cat /home/spark/logs/grasp_test_run.log 2>&1
echo "=== grasp_scan_run.log ==="
cat /home/spark/logs/grasp_scan_run.log 2>&1
echo "=== NAV TEST DIR ==="
ls -lt /home/spark/logs/nav_test 2>&1 | head -10
echo "=== ARM DRIVER CHECK ==="
pgrep -af "swiftpro|uarm|uArm" 2>&1
echo "=== ROS NODES ==="
source /opt/ros/noetic/setup.bash 2>/dev/null
rosnode list 2>&1 | head -30