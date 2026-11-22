echo "=== LAST JSONL LINES ==="
tail -n 3 /home/spark/logs/grasp_generic.jsonl
echo "=== ARM STATE AFTER ABORT ==="
source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/spark/spark_noetic/devel/setup.bash 2>/dev/null
timeout 8 rostopic echo -n 1 /SwiftproState_topic 2>&1 | head -12
echo "=== NODES ==="
timeout 5 rosnode list 2>&1 | grep -E "swiftpro|detector|spark_base_server"
echo "=== DONE ==="