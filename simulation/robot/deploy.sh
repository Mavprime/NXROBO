#!/usr/bin/env bash
# Deploy the simulation tree to a Spark robot, laid out under ~/codes where the run
# scripts read it. Idempotent: re-run after any edit; rsync sends only changed files.
#
#   bash robot/deploy.sh spark@<robot-ip>
#   ROBOT=spark@<robot-ip> bash robot/deploy.sh
#
# Stops a sim left running by a previous deploy first (master 11312 only; the hardware
# stack on 11311 is never touched). Prints the three packages/commands the sim needs.

set -euo pipefail

ROBOT="${1:-${ROBOT:-}}"
if [ -z "$ROBOT" ]; then
    echo "usage: bash $0 spark@<robot-ip>" >&2
    exit 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM="$(cd "$HERE/.." && pwd)"
echo "== deploy: $SIM -> $ROBOT:~/codes"

# 1. Stop a sim left running by a previous deploy.
ssh "$ROBOT" 'if [ -x "$HOME/codes/robot/stop_sim.sh" ]; then bash "$HOME/codes/robot/stop_sim.sh"; fi' || true

# 2. rsync creates only the final directory, never its parents. A fresh Spark has no
#    ~/codes, so create the tree first (this is what made config/ and maps/ fail on
#    2026-11-23).
ssh "$ROBOT" 'mkdir -p "$HOME/codes/config" "$HOME/codes/maps" "$HOME/codes/ros" \
                       "$HOME/codes/robot" "$HOME/codes/sim" "$HOME/codes/assets"'

# 3. Copy into the exact layout the scripts read.
rsync -av --exclude='__pycache__/' "$SIM/config/" "$ROBOT":~/codes/config/
rsync -av "$SIM/maps/" "$ROBOT":~/codes/maps/
rsync -av --exclude='__pycache__/' --exclude='tests/' "$SIM/codes/" "$ROBOT":~/codes/
rsync -av "$SIM/ros/" "$ROBOT":~/codes/ros/
rsync -av "$SIM/robot/" "$ROBOT":~/codes/robot/
rsync -av "$SIM/gazebo/arena.world" "$SIM/gazebo/spark_arena.launch" \
          "$SIM/assets/spark1_description/urdf/spark1_description_nav.urdf" \
          "$ROBOT":~/codes/sim/
# Bundled vendor packages (auto_match configs + Spark model): fallback for hosts that
# do not have them in ~/spark_noetic.
rsync -av "$SIM/assets/" "$ROBOT":~/codes/assets/

# 4. Exec bits, hardware nav bring-up copy, rospack refresh, preflight report.
#    rospack profile matters: a stale cache hid auto_match and spark1_description on
#    2026-11-23 even though both are in ~/spark_noetic/src.
ssh "$ROBOT" 'chmod +x "$HOME/codes/robot/"*.sh
cp "$HOME/codes/ros/nav_bringup_arena.sh" "$HOME/codes/nav_bringup.sh"
chmod +x "$HOME/codes/nav_bringup.sh"
source /opt/ros/noetic/setup.bash 2>/dev/null || true
source "$HOME/spark_noetic/devel/setup.bash" 2>/dev/null || true
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}:$HOME/codes/assets"
rospack profile >/dev/null 2>&1 || true
echo "   auto_match:         $(rospack find auto_match 2>/dev/null || echo MISSING)"
echo "   spark1_description: $(rospack find spark1_description 2>/dev/null || echo MISSING)"
echo "   gazebo:             $(command -v gzserver || echo MISSING)"
'
echo "== deployed."
echo "   start: ssh $ROBOT bash ~/codes/robot/start_sim.sh"
echo "   stop:  ssh $ROBOT bash ~/codes/robot/stop_sim.sh"
