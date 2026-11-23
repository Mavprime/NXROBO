echo "=== ROBOT ONLINE CHECK ==="
date
source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/spark/spark_noetic/devel/setup.bash 2>/dev/null
timeout 5 rosnode list 2>&1 | grep -E "detector_node|swiftpro_write|spark_base_server"
echo "=== CURRENT DETECTOR PROCESS ==="
pgrep -af "detect_ros.py" 2>&1
echo "=== DONE ==="