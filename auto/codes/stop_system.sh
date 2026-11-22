#!/usr/bin/env bash
# Stop all Spark subsystems safely: zero velocity, release pump, close bag.

source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/spark_noetic/devel/setup.bash 2>/dev/null

echo "Stopping all processes..."

# Stop chassis motion and release vacuum pump while drivers are still alive
if timeout 2 rosnode list >/dev/null 2>&1; then
    timeout 2 rostopic pub -1 /cmd_vel geometry_msgs/Twist '{}' >/dev/null 2>&1 || true
    timeout 2 rostopic pub -1 /pump_topic swiftpro/status "status: 0" >/dev/null 2>&1 || true
fi

fuser -k 8080/tcp 2>/dev/null

# Stop the recorder first so the bag file is closed cleanly.
if pgrep -f "rosbag record" >/dev/null; then
    pkill -INT -f "rosbag record" 2>/dev/null
    sleep 2
    pkill -f "rosbag record" 2>/dev/null
    echo "  + mission bag recorder stopped (file closed)"
fi

pkill -f "runtask.py" 2>/dev/null
pkill -f "detect_ros.py" 2>/dev/null
pkill -f "camera_stream.py" 2>/dev/null
pkill -f "nav_bringup.sh" 2>/dev/null
pkill -f "ydlidar_ros_driver_node" 2>/dev/null
pkill -f "delta_lidar_node" 2>/dev/null
pkill -f "pro_control_nomoveit" 2>/dev/null
pkill -f "spark_bringup" 2>/dev/null
pkill -f "spark_base_server" 2>/dev/null
rosnode kill -a 2>/dev/null
echo "All processes stopped."
