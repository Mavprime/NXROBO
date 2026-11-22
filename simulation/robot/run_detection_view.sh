#!/usr/bin/env bash
# Live detection view on the robot: DLT bridge + RViz markers.
#
#   bash ~/codes/robot/run_detection_view.sh              # base_footprint
#   FRAME=map bash ~/codes/robot/run_detection_view.sh    # map (AMCL must be up)
#
# Then in RViz (run on the robot via VNC):
#   Fixed Frame: base_footprint (or map)
#   Add -> By topic -> MarkerArray -> /detection_markers
# Camera image: http://<robot-ip>:8080/
#
# Passive: publishes markers, moves nothing. Ctrl+C stops both nodes.

set -o pipefail
source /opt/ros/noetic/setup.bash
source "$HOME/spark_noetic/devel/setup.bash"

FRAME="${FRAME:-base_footprint}"

python3 -u "$HOME/codes/detection_bridge.py" \
    _mode:=dlt _target_frame:="$FRAME" &
BRIDGE=$!

python3 -u "$HOME/codes/detections_viz.py" \
    _frame:="$FRAME" _topic:=/detections_world &
VIZ=$!

trap 'kill $BRIDGE $VIZ 2>/dev/null' INT TERM
echo "bridge + markers running (frame $FRAME); RViz topic /detection_markers"
wait
