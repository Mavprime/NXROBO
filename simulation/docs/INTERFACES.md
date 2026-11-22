# Interfaces

Contracts between the mission/navigation layer, the simulator, and the arm/perception
side. These are intentionally small: the mission layer must not know about IK, detectors
or Gazebo.

---

## 1. Layer split

```
            ┌─────────────────────────────────────────┐
            │ Mission + Supervisor (this workstream)  │
            │  - task list, gate, memory, recovery    │
            └───────┬───────────────┬─────────────────┘
        topics/actions        service calls
            ┌───────▼───────┐   ┌───▼──────────────────┐
            │ Navigation     │   │ Arm service (mock now,│
            │ move_base/     │   │ friend's code later)  │
            │ fake world     │   └───┬──────────────────┘
            └───────┬────────┘       │
            ┌───────▼────────────────▼───────────────┐
            │ Perception adapter                      │
            │ synthetic (L0/L1) or YOLO (L2)          │
            └──────────────────────────────────────────┘
```

---

### 1.1 Mission platform protocol (implemented in `codes/mission_logic.py`)

The mission layer talks to the world and the arm through a duck-typed platform object.
Required: `goto(point) -> bool`, `scan_window(n) -> frames of Detection`, `pick(cls) ->
PickResult`, `place(level) -> PlaceResult`. Optional: `read_board()`, `time_left()`,
`stop_requested()`, `wait(s)`, `precheck()`, `clear_costmaps()`, `rotate_scan()`,
`backup()`, `reframe(station)`, and an `approachable(x, y)` callable for G4.

`codes/mock_adapter.py` implements it on the discrete mock; `codes/fake_world.py`
(`SpatialPlatform`) implements it on the continuous 2D world with lidar and camera poses.
The ROS node will wrap the real `move_base` / detector / arm service in the same methods.

---

## 2. Topic contract

Topics marked **real** already exist on the robot; **sim** topics are provided by the
fake world and should be adapted to the real one when moving to the robot.

### 2.1 Navigation / chassis

| Topic | Type | Dir | Notes |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | mission/recovery → base | real |
| `/odom` | `nav_msgs/Odometry` | base → all | real (sim: fake world) |
| `/scan` | `sensor_msgs/LaserScan` | lidar → costmaps | real |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | base/camera/lidar → all | real |
| `/move_base/status` | `actionlib_msgs/GoalStatusArray` | nav → mission | real |
| `move_base` action | `move_base_msgs/MoveBaseAction` | mission → nav | real |
| `/mark_nav` | `std_msgs/String` (`go <name>`, `arrive <name>`) | mission → nav shim | real (on robot) |

The mission layer talks **only** to the move_base action (or a shim that wraps
`/mark_nav`). Named points are resolved from the world config, not from the action.

### 2.2 Perception

Production uses `/objects` as `vision_msgs/Detection2DArray` with a bbox and class id.
For the mission layer, the adapter converts that to a world-pose detection list. In
simulation the fake world publishes the same topic name, so the mission code is identical.

| Topic | Type | Dir | Notes |
|---|---|---|---|
| `/objects` | `vision_msgs/Detection2DArray` | detector → mission | real topic name |
| `/objects_world` | `sim_msgs/DetectedObjects` (sim only) | sim → mission | optional shortcut for Level 0; must be adapted before Level 2 |

`DetectedObjects`:

```
DetectedObject {
  string  instance_id        # optional; empty if not tracked
  string  class              # canonical: cake | bear | clock | banana
  float32 confidence
  geometry_msgs/PoseStamped pose     # in `map`
  float32 stamp_sec
}
DetectedObjects { DetectedObject[] objects }
```

Class names are canonical (normalised). The production alias set (`1|10`, `2|20`,
`3|30`, `4|40`) is resolved by the adapter, not by the mission code. Note: the generated
sim scenario uses `doll` for the bear — either rename it there or translate
`doll -> bear` in the sim adapter (README, "Open naming conflicts").

**Detection window for the gate:** the mission takes the last N=5 frames (not just the
latest) and applies temporal voting with K=3. In sim, frames arrive at a fixed rate so
tests are deterministic.

### 2.3 Mission control

| Topic | Type | Dir | Notes |
|---|---|---|---|
| `/task_start_flag` | `std_msgs/String` | operator → mission | real: `start|1|go` / `stop|0|abort` |
| `/mission_state` | `std_msgs/String` (JSON) | mission → operator/RViz | new; full belief snapshot |
| `/mission_event` | `std_msgs/String` (JSONL) | mission → log/bag | optional, one line per event |

`/mission_state` example:

```json
{
  "state": "GATE",
  "step": 5,
  "step_desc": "pick cake at M",
  "clock_remaining_s": 512.4,
  "robot_pose": [-0.6, -0.6, 0.785],
  "target": {"class": "cake", "station": "M"},
  "gate": {"G1": "pass", "G2": "pass", "G3": "pass", "G4": "pass", "G5": "pass"},
  "cubes": {
    "cake": {"status": "pending", "attempts": 1},
    "bear_1": {"status": "pending", "station": "N"},
    "clock_1": {"status": "pending", "station": "E"}
  },
  "mapping": {"bear": "B", "clock": "C", "banana": "D"}
}
```

### 2.4 Arm service

The mission never commands joint poses. It requests an **intent**; the arm side owns
detection, alignment, vacuum and verification.

```
srv Pick:
  string class              # canonical class to pick
  string station            # named station, for logging/recovery only
  ---
  bool   ok
  string confirmed_class    # what the arm actually grasped ("" if failed)
  string reason             # machine-readable: not_seen | misaligned | vacuum_failed | ...
  float32 duration_s

srv Place:
  string zone               # A|B|C|D
  int32  level              # 1|2|3
  ---
  bool   ok
  string reason

srv Home:
  ---
  bool ok
```

The existing in-process mock (`codes/mock_world.py`) already exposes `pick(type)`,
`drop()`, `goto_local(name)`, `read_board()` and `detector(want=None)`, so the mission
logic can be developed against it before any ROS service exists. The services below are
the same contract at the node boundary.

Mock behaviour (Level 0), configured by `config/gate.yaml` injection and
`mock_world.Config`:

- `pick` succeeds iff the gate passed **and** the simulated target exists at the station.
- Optional fault injection: `pick_fail_probability`, `place_fail_probability`,
  `confirmed_class_override` (to test gate/verification paths).
- `place` succeeds unless fault injection fires.

This is the contract the friend implements with real uArm code. Keep the service name
stable (`/arm/pick`, `/arm/place`, `/arm/home`).

---

## 3. Config schemas

### 3.1 World (`config/arena_map.yaml`, canonical)

```yaml
frame: arena
arena: {size: [3.0, 3.0], origin: [0.0, 0.0]}
bricks:
  - {name: brick_1_1, type: clock, row: 1, col: 1, layer: 1, x: 1.5, y: 1.5}
  - {name: cake, type: cake, row: 1, col: 1, layer: 2, x: 1.5, y: 1.5}
board_left_to_right: [doll, clock, banana]
board_zones: [B, C, D]
zones: {A: {transport_center: [0.5, 2.5], collection_center: [0.225, 2.775]}}
start_pose: {x: 0.225, y: 2.775, yaw: -0.785398}
points: {Classification_area: [0.6, 1.5], Sorting_N: [1.5, 1.91]}
```

`make_arena.py` derives `scenario.json` in the same frame for the discrete mock.

Agreement-gate thresholds and fault injection live separately in `config/gate.yaml`.
Mission step format is in `config/mission_demo.yaml`.

### 3.2 Mission (`config/mission_*.yaml`)

```yaml
name: ...
clock_budget_s: 540
steps:
  - goto:    {point: Sorting_N, tol: 0.08}
  - pick:    {class: cake, station: N, attempts: 3}
  - place:   {zone: A, level: 1}
  - verify:  {class: cake, zone: A, timeout_s: 3.0}
```

Point names are resolved as: explicit `[x, y, yaw]` → named zone centre → named station
approach pose → error (never a silent default).

---

## 4. RViz visualisation (the debugging tool)

Publish one `MarkerArray` per source so contradictions are visible at a glance:

| Namespace | Colour | Contents |
|---|---|---|
| `map/zones` | grey | zone squares, labels A–D |
| `map/stations` | blue | station approach poses + arrows (heading) |
| `perception/detections` | green | live detections, size ∝ confidence |
| `memory/pending` | yellow | cubes believed still pending, at last known pose |
| `memory/carried` | cyan | the cube currently on the arm |
| `memory/delivered` | dim green | delivered cubes at their zones |
| `memory/conflict` | red | perception/memory disagreements |
| `plan/route` | white line | current path to the next waypoint |
| `robot/goal` | magenta arrow | the active target pose |

This is the fastest way to see the gate's reasoning: if a green detection sits away from
the blue station, G2 will fail; if a yellow cube and a green detection coincide after
delivery, G3 will conflict.

---

## 5. What the mission layer needs from the friend's side

1. Implement `/arm/pick`, `/arm/place`, `/arm/home` with the semantics above.
2. Return `confirmed_class` honestly — the mission uses it to update memory.
3. Do not perform a pick when `ok=false` was already determined; the mission may have
   decided to skip.
4. Publish grasp/vacuum telemetry if it exists (the mission logs it but does not depend
   on it; see SI-28 — there is currently no vacuum feedback).
5. Detection class names must be canonical (`cake|bear|clock|banana`), not raw YOLO ids
   and not the sim's `doll`.
