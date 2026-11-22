echo "=== DETECTOR GATE LOG ==="
grep -a "publish_conf" /home/spark/logs/detect.log 2>/dev/null | tail -2
echo "=== ARM STATE ==="
source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/spark/spark_noetic/devel/setup.bash 2>/dev/null
timeout 8 rostopic echo -n 1 /SwiftproState_topic 2>&1 | grep -E "^(x|y|z|pump):"
echo "=== LAST RUN LOG LINE ==="
tail -n 1 /home/spark/logs/grasp_generic.jsonl
echo "=== DONE ==="