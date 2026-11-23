echo "=== NAV / DETECTOR / ARM NODES ==="
source /opt/ros/noetic/setup.bash 2>/dev/null
rosnode list 2>&1 | grep -E "move_base|amcl|detector|swiftpro|map_server" 
echo "=== CMD_VEL PUBLISHERS ==="
timeout 5 rostopic info /cmd_vel 2>&1
echo "=== CUBES ON /objects (latest frame) ==="
timeout 6 rostopic echo -n 1 /objects 2>&1 | head -40
echo "=== ARM DRIVER PROCESS ==="
pgrep -af "swiftpro|pro_control" 2>&1
echo "=== ACTIVE NAV LOGS ==="
tail -n 5 /home/spark/logs/nav_test/nav.log 2>&1