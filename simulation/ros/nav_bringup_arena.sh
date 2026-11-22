#!/usr/bin/env bash
# Navigation bring-up for the 0..3 m arena map.
#
# DEPLOY: copy this to ~/codes/nav_bringup.sh and chmod +x. start_system.sh runs it
#         automatically when executable, then waits for /move_base.
#         Maps: copy simulation/maps/arena_map.{pgm,yaml} to ~/codes/maps/.
#
# Uses the static map in the arena frame (origin at the SW corner, map == arena),
# so no mark_map.pkl / mark_nav dependency is needed.
#
# move_base overrides (goal tolerance) come from ~/codes/config/move_base_tuning.yaml;
# the vendor auto_match params are not modified.

set -o pipefail

source /opt/ros/noetic/setup.bash || exit 1
source "$HOME/spark_noetic/devel/setup.bash" || exit 1

PKG="$(rospack find auto_match)" || { echo "[nav] auto_match package not found"; exit 1; }
MAP="${MAP_FILE:-$HOME/codes/maps/arena_map.yaml}"
INIT_X="${INIT_X:-0.225}"
INIT_Y="${INIT_Y:-2.775}"
INIT_A="${INIT_A:--0.785398}"   # facing the arena centre from the A collection square
TUNING="${TUNING_FILE:-$HOME/codes/config/move_base_tuning.yaml}"
if [ ! -f "$TUNING" ]; then
    echo "[nav] WARNING: tuning file not found: $TUNING (vendor goal tolerance 0.05 m)"
fi

echo "[nav] map: $MAP"
if [ ! -f "$MAP" ]; then
    echo "[nav] ERROR: map file not found: $MAP"
    exit 1
fi

rosrun map_server map_server "$MAP" &

roslaunch "$PKG/config/amcl.launch.xml" \
    initial_pose_x:="$INIT_X" initial_pose_y:="$INIT_Y" initial_pose_a:="$INIT_A" &

roslaunch "$PKG/config/move_base_param/move_base.launch.xml" \
    custom_param_file:="$TUNING" &

echo "[nav] requested map_server + amcl + move_base (map frame = arena)"
wait
