echo "=== LAST RUN LOG LINE ==="
tail -n 3 /home/spark/logs/grasp_generic.jsonl 2>&1
echo "=== ARM FINAL STATE (/SwiftproState_topic) ==="
source /opt/ros/noetic/setup.bash 2>/dev/null
timeout 8 rostopic echo -n 1 /SwiftproState_topic 2>&1 | head -30
echo "=== PUMP TOPIC INFO ==="
timeout 5 rosnode list 2>&1 | grep -E "swiftpro|detector|spark_base_server"
echo "=== DONE ==="