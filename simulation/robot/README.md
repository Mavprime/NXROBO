# Gazebo on the robot NUC

Running the simulation on the robot itself avoids Docker and the blocked registry.
The robot already has ROS Noetic + Gazebo Classic 11 and the Spark model.

## Safety / isolation

- The sim runs on **ROS master 11312**; the hardware stack stays on 11311.
- `stop_sim.sh` never touches 11311.
- No hardware driver is started; the sim uses Gazebo only.

## Deploy

From `simulation/`:

```bash
ssh spark@<IP> 'mkdir -p ~/codes/{sim,ros,maps,robot,config}'
scp gazebo/arena.world            spark@<IP>:~/codes/sim/
scp assets/spark1_description/urdf/spark1_description_nav.urdf spark@<IP>:~/codes/sim/
scp gazebo/spark_arena.launch     spark@<IP>:~/codes/sim/
scp ros/nav_arena.launch ros/move_base_arena.launch spark@<IP>:~/codes/ros/
scp maps/arena_map.*              spark@<IP>:~/codes/maps/
scp robot/*.sh                    spark@<IP>:~/codes/robot/
scp config/arena_map.yaml config/mission_demo.yaml config/gate.yaml spark@<IP>:~/codes/config/
ssh spark@<IP> 'chmod +x ~/codes/robot/*.sh'
```

`spark1_description_nav.urdf` is the model without the camera sensor: the depth camera
rendering pushed the NUC to load ~6/8, and navigation does not need it. The full model
remains available for Level 2; pass `model:=.../spark1_description.urdf`.

## Run

```bash
ssh spark@<IP> 'bash ~/codes/robot/start_sim.sh'
# logs
ssh spark@<IP> 'tail -f ~/logs/sim/gazebo.log ~/logs/sim/nav.log'
# stop
ssh spark@<IP> 'bash ~/codes/robot/stop_sim.sh'
```

Everything uses `ROS_MASTER_URI=http://127.0.0.1:11312`.

## Verified 2026-11-22

- Gazebo spawned the Spark; `/clock`, `/odom`, `/scan` (10 Hz) present.
- `map_server` + AMCL + `move_base` + velocity smoother up.
- `map → base_footprint` TF resolves; AMCL localized at the start pose.
- Sent `move_base_simple/goal` to `Sorting_N` `(1.5, 1.88, -π/2)`:
  - result **status 3, "Goal reached."**
  - odom `(0.229, 2.778)` → `(1.497, 1.935)`
  - AMCL `(1.462, 1.874)`

## Live detection view (hardware)

```bash
bash ~/codes/robot/run_detection_view.sh              # base_footprint
FRAME=map bash ~/codes/robot/run_detection_view.sh    # map, AMCL required
```

Then in RViz on the robot desktop (VNC 5900): Fixed Frame `base_footprint` (or `map`),
Add → MarkerArray → `/detection_markers`. Camera: `http://<robot-ip>:8080/`.
For numbers only, run `bridge_check.py` (see `docs/BRIDGE_VALIDATION.md`).

## Full mission

```bash
bash ~/codes/robot/start_sim.sh              # Gazebo + nav on 11312
bash ~/codes/robot/run_mission_robot_sim.sh  # truth detections + arm mock + mission
bash ~/codes/robot/start_viz_robot_sim.sh    # /mission_markers, /detection_markers
```

**Verified 2026-11-22:** mission state `DONE`, cake `delivered`, all steps `ok`:
goto 15.4 s, pick 3.4 s, goto 9.8 s, place 2.0 s, verify 0.5 s (~31.7 s).

## View it (VNC)

One command (starts RViz on display `:0` with a ready layout):

```bash
bash ~/codes/robot/start_rviz_robot_sim.sh
```

Then connect a VNC viewer to `<robot-ip>:5900` (credentials as configured on the robot;
they are not stored in this repo). The layout includes
Map, LaserScan, TF, RobotModel, `/mission_markers` and `/detection_markers`.

Manual equivalent over SSH (the missing `DISPLAY` is why plain `rosrun rviz rviz` fails):

```bash
export ROS_MASTER_URI=http://127.0.0.1:11312
source /opt/ros/noetic/setup.bash
source ~/spark_noetic/devel/setup.bash
DISPLAY=:0 XAUTHORITY=$HOME/.Xauthority rosrun rviz rviz
```

- Fixed Frame: `map`
- Add: `Map` (`/map`), `LaserScan` (`/scan`), `TF`, `RobotModel`
- Add: `MarkerArray` → `/mission_markers` (zones, cube memory)
- Add: `MarkerArray` → `/detection_markers` (truth detections)

Optional 3D world: run `gzclient` on the VNC desktop (heavier; the server is headless).

## Notes

- Load average reached ~6.5 on 8 cores with Gazebo + nav + mission. Stop the sim when
  not testing.
- The nav build uses the robot's `auto_match` configs and vendor `move_base`
  (`spark_navigation` + velocity smoother are installed there).
- The sorting stand-off is 0.38 m; the vendor costmap does not mark floor cubes, which
  is required for the arm to reach the cluster.
- The mission node and detection bridge have not been run on this sim yet; next step.
