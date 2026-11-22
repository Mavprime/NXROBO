#!/usr/bin/env bash
# Run RViz on this PC (X display :0) against the robot-side Gazebo sim on master 11312.
#
#   ROBOT_IP=<robot-ip> ./run_pc_rviz.sh   # PC IP auto-detected
#
# The robot IP is DHCP and is not stored in this repository. Read it from the
# launcher / robot_ip.txt (gitignored) and pass it explicitly.
#
# Requires: spark_sim:noetic image, docker group, robot sim running.
# The robot sim must use ROS_IP=<robot LAN IP> (start_sim.sh does).

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
ROBOT_IP="${ROBOT_IP:-}"
PC_IP="${PC_IP:-$(hostname -I | awk '{print $1}')}"
DISPLAY_NUM="${DISPLAY:-:0}"
IMAGE="${IMAGE:-spark_sim:noetic}"

if [ -z "$ROBOT_IP" ]; then
    echo "ROBOT_IP is not set. Pass the robot's current DHCP address:"
    echo "  ROBOT_IP=<robot-ip> ./run_pc_rviz.sh"
    exit 1
fi

xhost +local:root >/dev/null 2>&1 || true
docker rm -f pc_rviz >/dev/null 2>&1 || true

docker run -d --name pc_rviz --net=host \
    -e DISPLAY="$DISPLAY_NUM" \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$ROOT:/ws" \
    "$IMAGE" \
    bash -lc "source /opt/ros/noetic/setup.bash; \
              export ROS_PACKAGE_PATH=/ws/assets:\$ROS_PACKAGE_PATH; \
              export ROS_MASTER_URI=http://${ROBOT_IP}:11312; \
              export ROS_IP=${PC_IP}; \
              rosrun rviz rviz -d /ws/ros/sim_view.rviz"

echo "RViz container 'pc_rviz' started on ${DISPLAY_NUM}"
echo "  robot master: http://${ROBOT_IP}:11312   PC ROS_IP: ${PC_IP}"
echo "  logs: docker logs pc_rviz"
echo "  stop: docker stop pc_rviz"
