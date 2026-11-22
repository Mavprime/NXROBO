# Detection bridge validation (real robot)

Goal: prove that a bbox centre from `/objects` becomes the correct world position before
the mission gate uses it.

## Key finding (2026-11-22)

The real D435 image is **top-down** (snapshot shows the cubes from above), but the URDF
camera TF says the optical frame is **horizontal** (`+z` = robot forward, camera at
0.674 m). The pinhole+TF projection therefore puts a nearby cube at **10.85 m**. The
production pipeline does not use TF for this: it uses the calibrated 4×3 DLT matrix
`transform_3d.dat` (pixel → arm coordinates). The bridge now has a **DLT mode** for the
real robot; pinhole mode remains for the synthetic simulation.

## Two modes

| Mode | Inputs | Use |
|---|---|---|
| `dlt` (default) | `/objects`, `transform_3d.dat`, URDF arm mount, TF target ← `base_footprint` | real robot |
| `pinhole` | `/objects`, `camera_info`, TF target ← `camera_color_optical_frame`, floor plane | synthetic sim |

The DLT mode maps the pixel to arm `(x, y)` mm at a fixed height `z`, then to
`base_footprint` with the URDF mount `base_link → Base = (0.044, 0, 0.138)` (no rotation),
then to `map` with the robot pose if requested.

Heights: floor cubes `z = -100 mm`, cake (tier 2) `z = 0 mm`.

## Prerequisites

- Camera driver and production detector running (`/objects` at ~3–4 Hz).
- `~/codes/{bridge_check.py,projection.py,mission_logic.py}`,
  `~/codes/config/{arena_map.yaml,transform_3d.dat}`.
- For `target_frame:=map`, AMCL/nav running and localized.

## 1. Place one cube at a known position

- **Base frame:** put a cube in front of the robot, measure its `(x, y)` from the robot
  centre. Keep the robot still.
- **Map frame:** put it on a taped arena mark whose coordinates you know.

## 2. Check the inputs

```bash
rostopic hz /objects
rostopic echo -n1 /camera/color/camera_info/header
rostopic echo -n1 /camera/color/camera_info/K
```

The DLT mode does not need the camera TF; it needs the matrix and (for map) the robot TF.

## 3. Run the check

Base frame, cube measured 0.30 m ahead:

```bash
python3 -u ~/codes/bridge_check.py \
  _mode:=dlt _matrix:=$HOME/codes/transform_3d.dat \
  _target_frame:=base_footprint \
  _expected_class:=bear _expected_x:=0.30 _expected_y:=0.00 \
  _samples:=20
```

Map frame:

```bash
python3 -u ~/codes/bridge_check.py \
  _mode:=dlt _target_frame:=map \
  _expected_class:=cake _expected_x:=1.5 _expected_y:=1.5
```

Real passive result on the robot (no cube moved, detector running):

```
[bear] px=(348,285) conf=0.93 -> (0.246, -0.010) arm=(202.0,-9.6)mm z=-100mm base=(0.246,-0.010)
```

Stable to ±1–2 mm across frames. The bear in the snapshot is directly in front of the
robot, so `0.246 m` is the physical distance to confirm with a tape measure.

## 4. Acceptance

| Distance | Median error | Action |
|---|---|---|
| 0.2–0.6 m | **< 0.02 m** | pass |
| 0.6–1.0 m | **< 0.04 m** | pass with caution |
| any | > 0.06 m | do not trust the gate |

A constant offset means the mount or arm-frame origin is wrong; scatter means bbox
jitter or a stale frame.

## 5. Then run the bridge

```bash
python3 ~/codes/detection_bridge.py \
  _mode:=dlt _target_frame:=map \
  _matrix:=$HOME/codes/transform_3d.dat \
  _world:=$HOME/codes/config/arena_map.yaml _debug:=true
```

Check `/detections_world`: it should match the check. On the robot with the hardware
stack only (no AMCL), use `_target_frame:=base_footprint` and a test output topic:

```bash
python3 ~/codes/detection_bridge.py _mode:=dlt _target_frame:=base_footprint \
  _output_topic:=/detections_world_test _debug:=true
```

Verified 2026-11-22: `/detections_world_test` carried
`{"class":"bear","confidence":0.93,"x":0.246,"y":-0.01}`.

## Troubleshooting

| Symptom | Likely cause | Check |
|---|---|---|
| `cannot load DLT matrix` | wrong path or non-NumPy file | `python3 -c "import numpy as np; print(np.load('...').shape)"` → `(4,3)` |
| arm position implausible | wrong `z` for the class, matrix stale | try `_z_center_default:=-100` / `_z_center_by_class:={cake: 0}` |
| constant x/y bias | arm mount or `Base` origin | verify `base_link → Base` in the URDF; adjust `_mount_x/_mount_y` |
| TF `map <- base_footprint` missing | AMCL/nav not running | use `base_footprint`, or start nav |
| pinhole mode gives 10+ m | camera TF is horizontal while the camera is top-down | use `dlt`; pinhole is sim-only |
| class never matches | raw YOLO ids | normalized: `1/10→bear`, `2/20→cake`, `3/30→clock`, `4/40→banana` |

## Seeing it live

**Console** (numbers, no RViz):

```bash
python3 -u ~/codes/bridge_check.py _mode:=dlt _target_frame:=base_footprint \
  _expected_class:=bear _expected_x:=0.30 _expected_y:=0.00 _samples:=100
```

**RViz** (visual): on the robot, start the bridge + marker publisher:

```bash
bash ~/codes/robot/run_detection_view.sh              # base_footprint
FRAME=map bash ~/codes/robot/run_detection_view.sh    # map, AMCL required
```

Then in RViz (robot desktop over VNC, port 5900):

- Fixed Frame: `base_footprint` (or `map`)
- Add → By topic → MarkerArray → `/detection_markers`
- Live camera: `http://<robot-ip>:8080/`

Markers are coloured per class with a `class conf` label; they update at 10 Hz.

**Mission view:** when running `mission_node.py`, `mission_viz.py` adds
`/mission_markers` (zones, cube memory and statuses) from `/mission_state`.

**Simulation view:** `cd simulation/codes && python3 run_live.py` serves the kinematic
world at `http://127.0.0.1:8090/`; Gazebo is headless unless started with `gui:=true`
and X11.

## Notes

- The DLT matrix is robot-specific and may be updated after recalibration; copy the new
  `~/codes/transform_3d.dat` into `simulation/config/transform_3d.dat` if you want the
  tests to follow it.
- The check is passive: it moves nothing.
