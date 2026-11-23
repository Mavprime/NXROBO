# ROS integration (Noetic)

The mission layer is ROS-free; this document is the thin ROS 1 adapter around it and
the deployment procedure.

> Status: the nodes are written and syntax-checked, but **not yet run on the robot**
> (this PC has no ROS Noetic). Expect to debug topic names and TF on the first run.

## Node graph

```
static map (arena frame) ── map_server ──▶ /map
                                  AMCL ◀── /scan + /tf ──▶ map→odom
                                            │
move_base action ◀────────────── mission_node ──────────▶ /mission_state ──▶ mission_viz
                                   │    ▲  ▲
                     /arm/pick     │    │  │ /detections_world ◀── detection_bridge ◀── /objects
                     /arm/place    ▼    │  │ /classification_board
                                arm service  │
                     /arm/result ─────────────┘
```

The bridge needs `/camera/color/camera_info` and TF `map <- camera_color_optical_frame`
(D435). On Gazebo the camera topics/frame differ; override the bridge parameters.

The mission node uses the **move_base action** with goals in the map frame. The static
map has its origin at the arena SW corner, so **`map` == `arena`**; no extra TF is
needed. This replaces the `mark_nav` / `mark_map.pkl` route (which is tied to the old
demo map).

## Message contracts

All custom payloads are `std_msgs/String` containing JSON. This avoids adding a catkin
package and keeps the nodes deployable as scripts.

### Detections — `/detections_world`

```json
{
  "stamp": 123.4,
  "objects": [
    {"class": "cake", "confidence": 0.82, "x": 1.500, "y": 1.500},
    {"class": "doll", "confidence": 0.71, "x": 1.355, "y": 1.645}
  ]
}
```

- `x, y` are in the `map` frame (the gate compares them with the cube positions in
  `arena_map.yaml`).
- Objects without a pose are ignored.
- The publisher must transform camera pixels to the map frame and stamp them at capture
  time. Do not publish stale poses.
- Production `/objects` is `vision_msgs/Detection2DArray` in image coordinates.
  `codes/detection_bridge.py` converts bbox centres to this topic.
  - **`~mode:=dlt` (real robot):** 4×3 `transform_3d.dat` → arm mm → URDF mount
    (`base_link → Base = 0.044, 0, 0.138`) → `base_footprint` → `map` via AMCL.
  - **`~mode:=pinhole` (sim):** camera_info + camera TF + floor plane.

  The real camera is top-down while the URDF camera TF is horizontal; pinhole mode is
  wrong on the robot (it puts a nearby cube at ~10 m). See
  [`BRIDGE_VALIDATION.md`](BRIDGE_VALIDATION.md). The mission layer consumes world
  poses, not pixels, so the ROS-side geometry stays in one place.

### Arm — `/arm/pick`, `/arm/place`, `/arm/result`

Pick request:

```json
{"id": "a1b2c3", "op": "pick", "class": "cake", "station": ""}
```

Place request:

```json
{"id": "d4e5f6", "op": "place", "zone": "", "level": 1}
```

Result:

```json
{"id": "a1b2c3", "ok": true, "confirmed_class": "cake",
 "reason": "", "duration_s": 4.0}
```

Rules:

- `ok` must reflect a real grasp/place, not an assumption.
- `confirmed_class` must be what was actually grasped; the mission uses it to update
  memory and will log a mismatch.
- The mission only marks a cube picked when `ok` is true.
- `arm_mock_node.py` implements this contract for testing.

### Board — `/classification_board`

```json
["doll", "clock", "banana"]
```

Left → right as seen by a robot facing the table (south → north), mapping to zones
B, C, D. If the topic is absent and the mission step has `optional: true`, the fallback
mapping from `arena_map.yaml` is used.

### Mission state — `/mission_state`

JSON snapshot: `state`, `current`, `steps[]`, `mapping`, `memory` (instances with
`x, y`), `clock`. `mission_viz.py` turns it into RViz markers.

## Node parameters

| Node | Parameter | Default |
|---|---|---|
| `mission_node.py` | `~world` | `~/codes/config/arena_map.yaml` |
| | `~mission` | `~/codes/config/mission_demo.yaml` |
| | `~gate` | `~/codes/config/gate.yaml` |
| | `~frame_id` | `map` |
| | `~base_frame` | `base_footprint` |
| | `~move_base_action` | `move_base` |
| | `~detections_topic` | `/detections_world` |
| | `~board_topic` | `/classification_board` |
| | `~arm_pick_topic` | `/arm/pick` |
| | `~arm_place_topic` | `/arm/place` |
| | `~arm_result_topic` | `/arm/result` |
| | `~arm_reach` | `0.45` |
| | `~frame_period` | `0.1` |
| | `~auto_start` | `false` |
| `arm_mock_node.py` | `~pick_s`, `~place_s`, `~fail_prob` | `4.0`, `3.0`, `0.0` |
| `mission_viz.py` | `~world` | `~/codes/config/arena_map.yaml` |
| `detection_bridge.py` | `~mode` | `dlt` (robot) / `pinhole` (sim) |
| | `~objects_topic` | `/objects` |
| | `~target_frame` | `map` |
| | `~matrix`, `~mount_x`, `~mount_y` | `~/codes/transform_3d.dat`, `0.044`, `0.0` |
| | `~z_center_default`, `~z_center_by_class` | `-100`, `{cake: 0}` |
| | `~camera_info_topic`, `~camera_frame` | pinhole only |
| | `~plane_z_default`, `~plane_z_by_class` | pinhole only, `0.05`, `{cake: 0.15}` |
| | `~conf_min`, `~arena_filter`, `~world` | `0.2`, `true`, arena map |
| `bridge_check.py` | `~mode`, `~expected_class/x/y`, `~samples` | `dlt`, unset, `20` |

## Map

`static_map.py` generates the map_server files:

```bash
cd simulation/codes
python3 static_map.py           # -> ../maps/arena_map.{pgm,yaml,png}
```

- 60×60 cells at 0.05 m, origin `[0, 0, 0]`, walls and table occupied.
- Cubes are **not** in the static map: they move. The lidar/local costmap should treat
  them as dynamic obstacles.
- A hand-built map is preferred over gmapping for a fixed arena: repeatable origin,
  no drift, `map == arena`.

## Deploy

From this repo:

```bash
bash simulation/robot/deploy.sh spark@<IP>
```

The script copies `config/`, `maps/`, `codes/`, `ros/`, `robot/`, the Gazebo
world/launch + nav URDF into `~/codes/sim/`, and the bundled packages into
`~/codes/assets/`; it then installs `~/codes/nav_bringup.sh` and refreshes the remote
rospack index. Re-run it after any edit.

`start_system.sh` picks up `~/codes/nav_bringup.sh` automatically.

## Run sequence

Terminal 1 — chassis, camera, lidar:

```bash
source ~/spark_noetic/devel/setup.bash
roslaunch spark_bringup driver_bringup.launch camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6
```

Terminal 2 — arm driver:

```bash
roslaunch swiftpro pro_control_nomoveit.launch
```

Terminal 3 — navigation:

```bash
bash ~/codes/nav_bringup.sh
# or: roslaunch ~/codes/ros/nav_arena.launch
```

Terminal 4 — mission (and optional mock/viz):

```bash
python3 ~/codes/mission_node.py
python3 ~/codes/detection_bridge.py _mode:=dlt   # /objects -> /detections_world
# optional, if the real arm service is not ready:
python3 ~/codes/arm_mock_node.py
# optional:
python3 ~/codes/mission_viz.py
```

Gazebo overrides for the bridge (the sim camera is not a D435):

```bash
python3 ~/codes/detection_bridge.py \
  _camera_info_topic:=/camera/rgb/camera_info _camera_frame:=virtual_camera_link
```

Start / stop:

```bash
rostopic pub /task_start_flag std_msgs/String "data: 'start'" --once
rostopic pub /task_start_flag std_msgs/String "data: 'stop'" --once
```

Watch: `/mission_state`, `/mission_markers` in RViz (fixed frame `map`),
`/move_base/result`, `/detections_world`.

## Known gaps before a real run

1. **Detection bridge validation** — DLT mode is passively validated on the real
   detector (bear at base `(0.246, −0.010)` m, stable). Still to do: a tape-measured
   expected comparison, and the map/AMCL run (`_target_frame:=map`). See
   [`BRIDGE_VALIDATION.md`](BRIDGE_VALIDATION.md).
2. **Arm service** — the friend's pick/place implementation must speak the JSON contract
   above; `arm_mock_node.py` only simulates it.
3. **Board reader** — no node publishes `/classification_board`; the demo mission does
   not read the board. For a competition mission, add the `read_board` step and a reader.
4. **move_base tuning** — speed, inflation and recovery are still the vendor defaults;
   see Phase 2 in `auto/docs/PLAN.md`.
5. **This code has not been executed on the robot.** Expect topic/frame fixes on the
   first run; start with one `goto`, then one pick, before the full mission.
