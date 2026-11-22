# ROS Noetic + Gazebo + the navigation stack.
#
# The `osrf/ros:noetic-desktop-full` image available through the current mirror is
# trimmed: it has Gazebo but NOT map_server/amcl/move_base, and no ROS apt source.
# This layer adds the navigation metapackage.
#
# `trusted=yes` avoids fetching the ROS signing key in a restricted network. It means
# apt does not verify the repository signature; use a proper key in a trusted network.
FROM osrf/ros:noetic-desktop-full

RUN echo "deb [trusted=yes] http://packages.ros.org/ros/ubuntu focal main" \
        > /etc/apt/sources.list.d/ros1.list \
    && apt-get update -qq \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        ros-noetic-navigation \
    && rm -rf /var/lib/apt/lists/*
