#!/usr/bin/env bash
# Launch Gazebo Classic + the Spark chassis in the arena, inside a Noetic container.
#
# Prerequisites (one-time, needs sudo):
#   sudo apt-get update && sudo apt-get install -y docker.io
#   sudo usermod -aG docker "$USER"      # then log out/in
#   sudo docker pull osrf/ros:noetic-desktop-full
#
# Usage:
#   ./run_gazebo.sh             # headless gzserver (recommended on 8 GB RAM)
#   ./run_gazebo.sh true        # GUI, needs X11: xhost +local:root first
#
# The container mounts simulation/ at /ws; the robot model is found through
# ROS_PACKAGE_PATH=/ws/assets.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
GUI="${1:-false}"
IMAGE="${IMAGE:-spark_sim:noetic}"

if ! command -v docker >/dev/null 2>&1; then
    echo "docker is not installed. Run:"
    echo "  sudo apt-get update && sudo apt-get install -y docker.io"
    echo "  sudo usermod -aG docker \"$USER\"   # log out/in afterwards"
    exit 1
fi

if ! docker info >/dev/null 2>&1; then
    echo "Cannot access the docker socket from this shell."
    echo "Use a shell with the docker group:"
    echo "  newgrp docker          # then run this script again"
    echo "  sg docker -c ./run_gazebo.sh"
    exit 1
fi

if [ "$GUI" = "true" ]; then
    echo "GUI mode needs X11 access: run 'xhost +local:root' on the host first."
fi

# `-t` fails when this script is run without a TTY (CI/background); use `-i` then.
TTY_FLAGS=(-i)
if [ -t 0 ]; then TTY_FLAGS=(-it); fi

docker rm -f spark_gazebo >/dev/null 2>&1 || true

exec docker run --rm "${TTY_FLAGS[@]}" --net=host --ipc=host --name spark_gazebo \
    -e DISPLAY="${DISPLAY:-:0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$ROOT:/ws" \
    "$IMAGE" \
    bash -lc "source /opt/ros/noetic/setup.bash; \
              export ROS_PACKAGE_PATH=/ws/assets:\$ROS_PACKAGE_PATH; \
              roslaunch /ws/gazebo/spark_arena.launch gui:=$GUI & \
              sleep 6; \
              roslaunch /ws/ros/nav_arena.launch use_sim_time:=true \
                  map:=/ws/maps/arena_map.yaml \
                  move_base_launch:=/ws/ros/move_base_arena.launch & \
              wait"
