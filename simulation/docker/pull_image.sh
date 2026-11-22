#!/usr/bin/env bash
# Pull osrf/ros:noetic-desktop-full through a reachable mirror (Docker Hub is blocked
# on some networks) and tag it under the canonical name.
#
# Run with the docker group available:
#   newgrp docker              # then ./pull_image.sh
#   sg docker -c ./pull_image.sh
#
# The mirrors are third-party pull-through caches. Treat the image as untrusted unless
# you can verify its digest against a trusted source.

set -euo pipefail

CANONICAL="osrf/ros:noetic-desktop-full"
MIRRORS=(
  "docker.1panel.live/osrf/ros:noetic-desktop-full"
  "docker.m.daocloud.io/osrf/ros:noetic-desktop-full"
  "docker.1ms.run/osrf/ros:noetic-desktop-full"
  "hub.rat.dev/osrf/ros:noetic-desktop-full"
  "dockerproxy.net/osrf/ros:noetic-desktop-full"
)

if ! docker info >/dev/null 2>&1; then
  echo "docker socket not accessible; use newgrp docker or sg docker -c"
  exit 1
fi

for ref in "${MIRRORS[@]}"; do
  echo "[pull] trying $ref"
  if docker pull "$ref"; then
    docker tag "$ref" "$CANONICAL"
    echo "[pull] tagged $ref as $CANONICAL"
    exit 0
  fi
  echo "[pull] failed: $ref (continuing)"
done

echo "[pull] all mirrors failed. Alternatives:"
echo "  - configure a proxy for the docker daemon"
echo "  - run Gazebo directly on the robot (Noetic + Gazebo are installed there)"
exit 1
