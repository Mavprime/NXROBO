# Gazebo in Docker

The simulation targets the robot's stack (ROS Noetic + Gazebo Classic 11). This PC is
Ubuntu 22.04 with ROS Humble only, so Noetic runs in a container.

## One-time setup (needs your password)

```bash
sudo apt-get update
sudo apt-get install -y docker.io
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"        # log out and back in
sudo docker pull osrf/ros:noetic-desktop-full   # ~3 GB
```

Check: `docker run --rm osrf/ros:noetic-desktop-full gazebo --version`.

### If Docker Hub is unreachable

On some networks `registry-1.docker.io` times out. Pull through a mirror and tag it:

```bash
newgrp docker                  # or: sg docker -c ./pull_image.sh
./pull_image.sh
```

`pull_image.sh` tries several third-party mirrors and tags the result as
`osrf/ros:noetic-desktop-full`. These are caches, not official distribution; treat the
image as untrusted unless you can verify its digest.

### If `permission denied ... docker.sock`

`usermod -aG docker` only affects new logins. In the current terminal use:

```bash
newgrp docker                  # opens a shell with the docker group
# or prefix a single command:
sg docker -c ./run_gazebo.sh
```

## Run

The mirror image is **trimmed**: it has Gazebo but not `map_server`/`amcl`/`move_base`.
Build the derived image once (adds `ros-noetic-navigation`):

```bash
cd simulation/docker
newgrp docker
docker build -t spark_sim:noetic .
```

Then run:

```bash
./run_gazebo.sh              # headless Gazebo + nav
./run_gazebo.sh true         # GUI (run `xhost +local:root` first)
```

`run_gazebo.sh` defaults to `spark_sim:noetic`; override with `IMAGE=...` if needed.

The container mounts `simulation/` at `/ws`, so the world, model and launch files are
edited on the host. `ROS_PACKAGE_PATH=/ws/assets` lets Gazebo resolve the model's
`package://spark1_description/...` meshes and the copied `auto_match` configs.

The script starts Gazebo, waits 6 s, then starts `map_server` + AMCL + `move_base`.
`move_base_arena.launch` is used instead of the vendor launch because the latter pulls
in the `spark_navigation` velocity smoother, which is not copied into this workspace.

Run the full mission loop (synthetic detections + arm mock + mission node) while the
container runs:

```bash
docker exec spark_gazebo bash /ws/robot/run_mission_container.sh
```

Or start the pieces by hand in a shell:

```bash
docker exec -it spark_gazebo bash
source /opt/ros/noetic/setup.bash
python3 /ws/codes/gazebo_truth_detections.py _world:=/ws/config/arena_map.yaml &
python3 /ws/codes/arm_mock_node.py &
python3 /ws/codes/mission_node.py \
  _world:=/ws/config/arena_map.yaml _mission:=/ws/config/mission_demo.yaml &
rostopic pub /task_start_flag std_msgs/String "data: 'start'" --once
```

The mission node's clock uses the container's `/clock` (sim time) when `use_sim_time`
is true.

## What exists

- `gazebo/arena.world` — 3×3 arena generated from `config/arena_map.yaml`.
- `assets/spark1_description/` — Spark chassis model from the robot; lidar reduced to
  720 beams at 10 Hz and its topic changed to `/scan` for the nav stack.
- `gazebo/spark_arena.launch` — empty world + spawn at the A collection square.
- `ros/nav_arena.launch` — map_server + AMCL + move_base (run in a second terminal
  inside the same container, or on the robot).

## Known limitations

- The arm is a single fixed link in the model; no pick/place physics. Level 1 validates
  chassis navigation, TF, lidar and costmaps.
- The camera plugin publishes `/camera/rgb/camera_info` with frame
  `virtual_camera_link`, not the D435 optical frame; the detection bridge needs
  `~camera_info_topic`/`~camera_frame` overrides for Gazebo (Level 2).
- **Verified 2026-11-22:** Gazebo spawned the Spark, `/scan` ran at 10 Hz, AMCL
  localized, and a `move_base` goal to `Sorting_N` returned **status 3, "Goal
  reached."** (odom `(0.232, 2.781)` → `(1.567, 1.982)`, AMCL `(1.520, 1.890)`).
- **Full mission verified 2026-11-22:** `mission_node` + `arm_mock_node` +
  `gazebo_truth_detections` → state **DONE**, cake **delivered**, all steps `ok`:
  goto 40.6 s, pick 4.5 s, goto 8.8 s, place 3.0 s, verify 0.5 s. The first goto
  includes AMCL convergence; per-step telemetry is in `/mission_state`. Detections and
  the arm are synthetic; this validates the mission/nav/gate loop, not perception.
- The vendor costmap intentionally leaves floor cubes out; the arm needs a 0.38 m
  stand-off. If you enable the scan obstacle layer, lower the robot radius/inflation
  or the sorting goals become unreachable.
- The diff-drive plugin publishes `odom -> base_footprint` (`publishOdomTF` defaults
  true); AMCL depends on it.
- For mission detections in Gazebo, run the bridge with the sim camera overrides; the
  sim camera frame/intrinsics have not been validated for projection yet.
