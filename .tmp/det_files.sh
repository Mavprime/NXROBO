echo "=== DETECTOR FILE VERSIONS ==="
ls -l /home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/detect_ros.py /home/spark/codes/detect_ros.py 2>&1
md5sum /home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/detect_ros.py /home/spark/codes/detect_ros.py 2>&1
echo "=== CAKE PUBLISH HACK IN NODES COPY ==="
grep -n "is_cake(item_id)\|PUBLISH_CONF\|min_conf" /home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/nodes/detect_ros.py 2>&1
echo "=== DETECTOR PROCESSES ==="
pgrep -af "detect_ros.py" 2>&1
echo "=== NODES ==="
source /opt/ros/noetic/setup.bash 2>/dev/null
timeout 5 rosnode list 2>&1 | grep -E "detector"
echo "=== DONE ==="