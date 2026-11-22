source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/spark/spark_noetic/devel/setup.bash 2>/dev/null
python3 -u /home/spark/codes/robot_view.py --no-park
echo "=== EXIT: $? ==="
ls -l /home/spark/logs/latest_view.jpg