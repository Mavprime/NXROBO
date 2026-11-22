source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/spark/spark_noetic/devel/setup.bash 2>/dev/null
echo "=== ARM STATE AFTER CAKE RUN ==="
timeout 8 rostopic echo -n 1 /SwiftproState_topic 2>&1 | grep -E "^(x|y|z|pump):"
echo "=== LAST GRASP LOG LINE ==="
tail -n 1 /home/spark/logs/grasp_generic.jsonl
echo "=== FRESH VIEW (no park) ==="
python3 -u /home/spark/codes/robot_view.py --no-park 2>&1 | tail -n 25
echo "=== DONE ==="