source /opt/ros/noetic/setup.bash 2>/dev/null
echo "===== DEFAULT rospy signal handling ====="
timeout 15 python3 -u /tmp/sigtest.py default 2>&1
echo "===== disable_signals + custom handler ====="
timeout 15 python3 -u /tmp/sigtest.py custom 2>&1
echo "===== DONE ====="