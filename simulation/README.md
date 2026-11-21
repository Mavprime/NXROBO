# Spark Simulation — Chassis Navigation & Mission Algorithm

Algorithm-first workspace for the **chassis side** of the autonomous competition robot:
navigation, mission/task management, and the cross-check ("agreement gate") between
what the map says, what the camera sees, and what the mission has already done.

This is **not** the arm/perception work — that is owned separately (recognition +
grasping on the uArm side). Here the arm is a service with a fixed contract, and a mock
implements it.

## Status

- [x] Design: architecture, algorithm, interfaces, roadmap
- [x] **Level 0A — discrete mission model** (existing, runs on a laptop):
  `codes/make_arena.py` generates the arena and scenario; `codes/mock_world.py`
  simulates time, failures and scoring; `python3 mock_world.py --runs 200` self-tests.
- [x] **Mission logic** — `codes/mission_logic.py`: executor, agreement gate, memory,
  recovery, clock. 42 unit tests (`python3 -m unittest discover -s tests`), and
  `codes/mock_adapter.py` runs the demo mission against the discrete mock (seeds 0–9
  all complete; scores 2–3).
- [x] **Visualisation** — `codes/view_run.py` renders each run to `runs/` as a top-down
  storyboard PNG and an animated GIF. This is a *picture of the discrete log*, not a
  physics simulation.
- [x] **Level 0B — spatial world** — `codes/fake_world.py`: continuous differential
  drive, raycast lidar, camera detections with poses, mocked arm and scoring.
  `codes/run_live.py` + `codes/live_view.py` serve a live browser view. Demo completes
  10/10 seeds (score 3, 29.3 s at measured chassis speeds, 0 collisions); `verify` is
  now `ok`. Per-step timings are in [`docs/TIMING.md`](docs/TIMING.md).
  Not ROS topics yet — the ROS adapter is still pending.
- [ ] Level 1 — Gazebo Classic + `move_base` + static map
- [ ] Level 2 — rendered RGB camera + real detector

## Layout

```
simulation/
├── README.md
├── docs/
│   ├── ALGORITHM.md           # mission FSM, agreement gate, pseudocode, failure policy
│   ├── INTERFACES.md          # ROS topics/services, arm contract, config schemas, RViz
│   ├── TIMING.md              # measured speeds, per-step timing, calibration
│   ├── FIELD_RISKS.md         # sim → real risk register + pre-flight checklist
│   └── ROADMAP.md             # staged plan, prerequisites, acceptance criteria, risks
├── config/
│   ├── robot.yaml             # measured chassis/arm parameters + sources
│   ├── gate.yaml              # agreement-gate thresholds + fault injection
│   └── mission_demo.yaml      # A -> center -> pick -> deliver
└── codes/
    ├── make_arena.py          # generates arena.world + scenario.json (seed -> layout)
    ├── mock_world.py          # discrete mission simulator + scoring + self-test
    ├── mission_logic.py       # ROS-free executor, agreement gate, memory, recovery
    ├── mock_adapter.py        # runs mission_logic against MockWorld (demo runner)
    ├── view_run.py            # renders a discrete run to ../runs/*.png + *.gif
    ├── fake_world.py          # Level 0B continuous world + SpatialPlatform
    ├── live_view.py           # HTTP server + canvas page (/state, /stop)
    ├── run_live.py            # spatial sim with live view on :8090
    ├── tests/                 # 42 unit tests, plain unittest
    ├── arena.world            # generated, not hand-edited
    ├── scenario.json          # generated world truth (bricks, board, points, start)
    └── README.md              # how to run, current baseline, planned nodes
└── runs/                      # generated visualisations (PNG storyboard, GIF path)
```

World geometry has **one** source of truth: `codes/scenario.json`, produced by
`make_arena.py`. Config files never duplicate it.

## Decisions taken

| Decision | Choice | Why |
|---|---|---|
| Simulator | **Gazebo Classic 11** | Robot already ships the Spark Gazebo model (`spark1_description`), URDFs and `gazebo_ros`; same Noetic nav stack as the real robot. Webots would need a new model + integration for no gain in a 3×3 m arena. |
| Host | **ROS Noetic in Docker on this PC** | This PC is Ubuntu 22.04 + ROS Humble only. Docker gives the exact robot stack without touching the host; the robot NUC stays free for hardware work. |
| Camera | **Staged**: synthetic detections first, rendered RGB later | Mission logic does not need pixels. Synthetic detections are deterministic and fast; rendered RGB + YOLO is Level 2. |
| First scenario | A → center → pick → destination | Small enough to verify end-to-end, exercises navigation, the gate and memory. |
| Cross-check | **Agreement gate** — map + perception + memory must agree before any physical action | Disagreement triggers a re-scan or re-plan, never a blind action. |
| Level 0 shape | **Two models**: discrete (time/scoring) and spatial (geometry/gate) | The discrete model is fast for strategy and scoring; only the spatial model can validate the gate. |

## Arena convention (from `codes/make_arena.py`)

- Origin at arena centre, `map` frame, metres, ROS axes: +x east, +y north.
- Zones: **A top-left** `(-,+)`, **B top-right** `(+,+)`, **C bottom-right** `(+,-)`,
  **D bottom-left** `(-,-)`; transport squares 1×1 m, collection squares 0.45×0.45 m.
- Robot starts in the A collection zone at `(-1.275, +1.275)`, yaw `-π/2`.
- 3×3 first-tier brick cluster at the centre (10 cm bricks, 14.5 cm pitch), cake on top
  of the centre brick; sorting approaches `Sorting_N/S/E/W`; classification table on the
  west wall (`Classification_area`).
- Board is read left→right by a robot facing west: left = south = `-y`; the three cards
  map to zones B, C, D in that order.

## Relationship to the robot code

- Mission logic should be **ROS-independent** (same pattern as `auto/codes/match_logic.py`),
  with a thin ROS adapter. Then it runs in the discrete mock, the spatial mock, the
  simulator and on the robot unchanged.
- The discrete mock deliberately mirrors `runtask.py` names (`goto_local`, `step_go`,
  `step_back`, `pick`, `drop`), so the state machine can be written once against either.
- Topic names follow the real stack where they exist (`/task_start_flag`, `/objects`,
  `/cmd_vel`, `/move_base`), so the adapter to `auto/codes/runtask.py` stays small.
- The sim is the intended place to develop and validate `nav_bringup.sh`, named points
  and `move_base` behaviour that Phase 2 needs (SI-21 in `auto/docs/PLAN.md`).

## Watch it live

```bash
cd simulation/codes
python3 run_live.py            # 1x real time; open http://127.0.0.1:8090/
python3 run_live.py --speed 4  # 4x
```

The page shows the robot, lidar, trail, cubes, score and the step list updating in real
time. `STOP MISSION` in the page (or Ctrl+C) stops it. This is still a 2D world — the 3D
Gazebo view comes at Level 1.

## Open naming conflicts

1. **`doll` vs `bear`** — the generated scenario calls the bear class `doll`; production
   `match_logic.norm_id()` canonicalises it as `bear` (id 1). Pick one spelling before
   the spatial model is written.
2. **`Sorting_M`** — production `CAKE_SORTING_STATION = "Sorting_M"`, but `make_arena.py`
   has no `M`; the cake sits on the centre brick and is picked from any `Sorting_*`
   approach. Either add `M` to the sim map or change the production constant after the
   real arena is measured.
