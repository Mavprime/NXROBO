#!/usr/bin/env bash
# Spark Remote Subsystem Diagnostic Runner
source /opt/ros/noetic/setup.bash 2>/dev/null
source ~/spark_noetic/devel/setup.bash 2>/dev/null

python3 /home/spark/semi/test_all_systems.py
