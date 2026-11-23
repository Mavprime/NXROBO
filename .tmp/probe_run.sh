source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/spark/spark_noetic/devel/setup.bash 2>/dev/null
echo "=== NODES ==="
timeout 5 rosnode list 2>&1 | grep -E "detector_node|swiftpro|spark_base_server"
echo "=== OLD DETECTOR: raw /objects confidence distribution (6 s) ==="
timeout 20 python3 -u /tmp/conf_probe.py 2>&1
echo "=== DONE ==="