echo "=== rospy/timer.py sleep ==="
sed -n '1,220p' /opt/ros/noetic/lib/python3/dist-packages/rospy/timer.py
echo "=== rostime wallsleep / is_wallclock ==="
grep -n -A20 "def wallsleep\|def is_wallclock" /opt/ros/noetic/lib/python3/dist-packages/rospy/rostime.py
echo "=== core _ros_signal ==="
grep -n -A40 "def _ros_signal" /opt/ros/noetic/lib/python3/dist-packages/rospy/core.py
echo "=== topics publish ==="
grep -n -B2 -A25 "def publish" /opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py | head -80
echo "=== use_sim_time ==="
rosparam get /use_sim_time 2>&1
echo "=== DONE ==="