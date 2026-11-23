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
  recovery, clock. 80 unit tests (`python3 -m unittest discover -s tests`), and
  `codes/mock_adapter.py` runs the demo mission against the discrete mock (seeds 0–9
  all complete; scores 2–3).
- [x] **Visualisation** — `codes/view_run.py` renders each run to `runs/` as a top-down
  storyboard PNG and an animated GIF. This is a *picture of the discrete log*, not a
  physics simulation.
- [x] **Level 0B — spatial world** — `codes/fake_world.py`: continuous differential
  drive, raycast lidar, camera detections with poses, mocked arm and scoring.
  `codes/run_live.py` + `codes/live_view.py` serve a live browser view. Demo completes
  10/10 seeds (score 3, 28.6 s at measured chassis speeds, 0 collisions); `verify` is
  now `ok`. Per-step timings are in [`docs/TIMING.md`](docs/TIMING.md).
- [x] **ROS adapter (written, not yet run on the robot)** — static map generator,
  `mission_node.py`, `arm_mock_node.py`, `mission_viz.py`, nav launch/script. See
  [`docs/ROS_INTEGRATION.md`](docs/ROS_INTEGRATION.md). No ROS Noetic on this PC.
- [x] **Detection bridge, real-robot DLT path** — the real D435 is top-down while the
  URDF camera TF is horizontal, so pinhole mode is simulation-only. `dlt` mode uses the
  production `transform_3d.dat` + URDF arm mount; passive validation on the real
  detector gives a bear at `(0.246, −0.010)` m in base frame, stable to 1–2 mm.
  Tape-measured expected comparison and the map/AMCL run are still pending.
- [x] **Gazebo validated on both hosts** — Spark spawned and a `move_base` goal to
  `Sorting_N` returned **status 3, "Goal reached."** on the robot NUC (isolated ROS
  master 11312) and in Docker. Odom and AMCL positions matched the goal. See
  [`robot/README.md`](robot/README.md) and [`docker/README.md`](docker/README.md).
- [x] **Full 10-cube kinematic mission** — `mission_competition.yaml`: read board,
  cake first to A, then 3 bears→B, 3 clocks→C, 3 bananas→D via the adaptive `fetch`
  step. **Score 21, DONE, 340.1 s simulated, 0 collisions**, all placements verified.
- [x] **Level 1 mission loop in Gazebo** — `mission_node` + arm mock + synthetic
  world-truth detections completed the demo mission in **Docker and on the robot NUC**:
  state `DONE`, cake `delivered`, all steps `ok`.
  Docker: goto 40.6/8.8 s, pick 4.5, place 3.0, verify 0.5.
  Robot sim: goto 15.4/9.8 s, pick 3.4, place 2.0, verify 0.5 (~31.7 s total).
- [ ] Level 2 — rendered RGB camera + real detector

## Layout

```
simulation/
├── README.md
├── docs/
│   ├── commands.html/.docx    # simulation command reference (all hosts)
│   ├── ALGORITHM.md           # mission FSM, agreement gate, pseudocode, failure policy
│   ├── INTERFACES.md          # ROS topics/services, arm contract, config schemas, RViz
│   ├── TIMING.md              # measured speeds, per-step timing, calibration
│   ├── FIELD_RISKS.md         # sim → real risk register + pre-flight checklist
│   ├── ROS_INTEGRATION.md     # topics, JSON contracts, deploy/run procedure
│   └── ROADMAP.md             # staged plan, prerequisites, acceptance criteria, risks
├── config/
│   ├── arena_map.yaml         # canonical 0..3 m arena (official layout, labels, bricks)
│   ├── transform_3d.dat       # production 4x3 DLT pixel -> arm calibration
│   ├── robot.yaml             # measured chassis/arm parameters + sources
│   ├── gate.yaml              # agreement-gate thresholds + fault injection
│   ├── mission_demo.yaml      # A -> center -> pick -> deliver
│   └── mission_competition.yaml  # cake first, then all 9 cubes (adaptive fetch)
├── codes/
│   ├── make_arena.py          # arena_map.yaml -> arena.world + scenario.json
│   ├── mock_world.py          # discrete mission simulator + scoring + self-test
│   ├── mission_logic.py       # ROS-free executor, agreement gate, memory, recovery
│   ├── mock_adapter.py        # runs mission_logic against MockWorld (demo runner)
│   ├── view_run.py            # renders a discrete run to ../runs/*.png + *.gif
│   ├── fake_world.py          # Level 0B continuous world + SpatialPlatform
│   ├── planner.py             # A* around inflated cube faces + table
│   ├── live_view.py           # HTTP server + canvas page (/state, /stop)
│   ├── run_live.py            # spatial sim with live view on :8090
│   ├── static_map.py          # arena_map.yaml -> map_server PGM/YAML
│   ├── ros_contract.py        # JSON contracts + goal resolution (ROS-free)
│   ├── mission_node.py        # ROS 1 adapter (move_base, detections, arm)
│   ├── arm_mock_node.py       # mock /arm/pick|place|result for testing
│   ├── mission_viz.py         # RViz markers from /mission_state
│   ├── projection.py          # pinhole pixel -> ray -> plane (unit-tested)
│   ├── detection_bridge.py    # /objects -> /detections_world (dlt or pinhole)
│   ├── bridge_check.py        # live validation against a measured cube
│   ├── gazebo_truth_detections.py  # synthetic detections for the Gazebo mission test
│   ├── tests/                 # 80 unit tests, plain unittest
│   ├── arena.world            # generated, not hand-edited
│   ├── scenario.json          # generated scenario (derived from arena_map.yaml)
│   └── README.md              # how to run, current baseline, planned nodes
├── assets/
│   ├── spark1_description/    # Gazebo model (full + nav-only URDF)
│   └── auto_match/            # nav configs copied from the robot (minimal package)
├── docker/                    # Dockerfile, run_gazebo.sh, pull_image.sh
├── gazebo/                    # arena.world + spark_arena.launch
├── maps/                      # generated map_server files (PGM + YAML + preview)
├── robot/                     # start/stop scripts for the robot-NUC sim
├── ros/                       # nav_arena.launch + move_base_arena.launch
└── runs/                      # generated visualisations (PNG storyboard, GIF path)
```

World geometry has **one** source of truth: `config/arena_map.yaml`. `make_arena.py`
derives `arena.world` (Gazebo) and `scenario.json` (discrete mock) from it; the spatial
sim and live viewer read the YAML directly. Config files never duplicate the geometry.

## Decisions taken

| Decision | Choice | Why |
|---|---|---|
| Simulator | **Gazebo Classic 11** | Robot already ships the Spark Gazebo model (`spark1_description`), URDFs and `gazebo_ros`; same Noetic nav stack as the real robot. Webots would need a new model + integration for no gain in a 3×3 m arena. |
| Host | **ROS Noetic in Docker on this PC** | This PC is Ubuntu 22.04 + ROS Humble only. Docker gives the exact robot stack without touching the host; the robot NUC stays free for hardware work. |
| Camera | **Staged**: synthetic detections first, rendered RGB later | Mission logic does not need pixels. Synthetic detections are deterministic and fast; rendered RGB + YOLO is Level 2. |
| First scenario | A → center → pick → destination | Small enough to verify end-to-end, exercises navigation, the gate and memory. |
| Cross-check | **Agreement gate** — map + perception + memory must agree before any physical action | Disagreement triggers a re-scan or re-plan, never a blind action. |
| Level 0 shape | **Two models**: discrete (time/scoring) and spatial (geometry/gate) | The discrete model is fast for strategy and scoring; only the spatial model can validate the gate. |

## Arena convention (`config/arena_map.yaml`)

Official layout from the CN rules diagram (page 15):

```
y=3  A (top-left)       B (top-right)
     table |  centre     |
y=0  D (bottom-left)    C (bottom-right)
     x=0                 x=3
```

- `arena` frame, origin at the **SW corner**, metres, +x east, +y north.
- Transport squares 1×1 m in the corners; red collection squares 0.45×0.45 m in the
  outer corner of each zone.
- Corners: A `(0,3)`, B `(3,3)`, C `(3,0)`, D `(0,0)`; collection centres are 0.225 m
  in from the corner, approaches 0.35 m inward from those.
- Robot starts in the A collection square at `(0.225, 2.775)`, facing the centre
  (yaw `-π/4`).
- 3×3 first-tier brick cluster at `(1.474, 1.562)` (measured 2026-11-24) (10 cm bricks, 14.5 cm pitch), cake on
  top of the centre brick; sorting approaches `Sorting_N/S/E/W` at 0.38 m.
- Classification table on the **left wall between A and D** (`Classification_area` at
  `(0.6, 1.5)`, facing west). Cards left→right for that robot are south→north and map to
  zones B, C, D.

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
