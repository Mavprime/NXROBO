source /opt/ros/noetic/setup.bash 2>/dev/null
source /home/spark/spark_noetic/devel/setup.bash 2>/dev/null
python3 -u /home/spark/codes/snap.py 2>&1
echo "=== SNAP DONE ==="
ls -l /home/spark/current_view.jpg