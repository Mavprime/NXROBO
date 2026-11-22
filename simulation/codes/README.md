# codes/

Everything here is pure Python and runs on this laptop; no ROS required yet.

## Existing

| File | Purpose |
|---|---|
| `make_arena.py` | Reads the canonical `config/arena_map.yaml` and generates `arena.world` (Gazebo 11 / SDF 1.6) and `scenario.json`. The map lists brick types explicitly; `--randomize-types` shuffles the nine first-tier types. |
| `mock_world.py` | Discrete-event mission simulator: time costs, failure probabilities, rule scoring, drop locations. Mirrors `runtask.py` names (`goto_local`, `step_go`, `step_back`, `pick`, `drop`). Includes a baseline strategy and hand-checked scoring assertions. |
| `mission_logic.py` | **The algorithm.** ROS-free mission executor, agreement gate (G1–G5, V2/V3), cube memory, recovery ladder, clock. Platform protocol in the module docstring. |
| `mock_adapter.py` | Implements that protocol on `MockWorld` and runs `config/mission_demo.yaml` end-to-end. |
| `view_run.py` | Renders a discrete run to `../runs/`: a static top-down storyboard (`*.png`) and an animated path (`*.gif`). |
| `fake_world.py` | **Level 0B spatial world**: continuous differential drive, raycast lidar, camera detections with positions, mocked arm, scoring. Also `SpatialPlatform` (the mission protocol on top). |
| `planner.py` | Grid A* around cube faces and the table inflated by robot radius; used by `SpatialPlatform.goto`. |
| `live_view.py` | Stdlib HTTP server + canvas page: `/` view, `/state` JSON, `/stop`. |
| `run_live.py` | Runs the demo mission in the spatial world and serves the live view. |
| `static_map.py` | Generates `../maps/arena_map.{pgm,yaml,png}` (map_server occupancy grid, origin at the arena SW corner). |
| `ros_contract.py` | ROS-free JSON contracts and goal resolution shared by the node and tests. |
| `mission_node.py` | ROS 1 adapter: `/task_start_flag`, move_base action, `/detections_world`, arm JSON topics, `/mission_state`. |
| `arm_mock_node.py` | Mock `/arm/pick`, `/arm/place`, `/arm/result` for integration testing. |
| `mission_viz.py` | RViz markers from `/mission_state`. |
| `projection.py` | Pinhole pixel → camera ray → cube plane; pure and unit-tested. |
| `detection_bridge.py` | `/objects` (Detection2DArray) → `/detections_world`; `dlt` mode for the real robot, `pinhole` for sim. |
| `bridge_check.py` | Live projection validation against a measured cube (dlt/pinhole, base/map). |
| `detections_viz.py` | RViz MarkerArray for `/detections_world` (real-time cubes + labels). |
| `gazebo_truth_detections.py` | Synthetic `/detections_world` for the Gazebo mission test; updates the cube list on arm pick/place. |
| `tests/` | 80 unit tests across mission logic, static map, ROS contracts, projection, DLT and adaptive fetch. |
| `arena.world`, `scenario.json` | Generated from `config/arena_map.yaml`. The map is the source of truth; do not hand-edit these. |

## Run

```bash
cd codes
python3 make_arena.py --out .              # regenerate world + scenario from arena_map
python3 -m unittest discover -s tests -v   # 80 tests
python3 static_map.py                      # -> ../maps/arena_map.{pgm,yaml,png}
python3 mock_adapter.py --seed 3           # demo mission against the discrete mock
python3 mock_adapter.py --json             # full report + event trace
python3 view_run.py --seed 3               # -> ../runs/mission_demo_seed3.png + .gif
python3 run_live.py                        # spatial sim + live view on :8090 (1x)
python3 run_live.py --speed 4              # faster; --no-viewer for console only
python3 run_live.py --report ../runs/run.json  # per-step timings + score as JSON
python3 run_live.py --lin-speed 0.45       # upper bound: teleop FAST mode
python3 mock_world.py --runs 200           # Monte Carlo baseline (strategy level)
```

## Current results

- **Tests:** 80/80 pass.
- **Full competition mission (kinematic):** board read → cake to A → 3 bears→B,
  3 clocks→C, 3 bananas→D. **Score 21, DONE, 340.1 s, 0 collisions**, all verified.
  Run: `python3 run_live.py --mission ../config/mission_competition.yaml --speed 2`.
- **Real-robot bridge:** DLT mode tested passively: bear at base `(0.246, −0.010)` m,
  stable to 1–2 mm. Tape-measured expected and map/AMCL run pending.
- **Demo mission** (`mission_demo.yaml`: A → center → pick cake → A)
  completes (`DONE`) on seeds 0–9. Scores 2–3 in the discrete mock; the 2s are the
  mock's 70 % "drops in collection" coin flip — the arm reports success but the cube
  lands in the transport square. `verify` reports `unverified` because the discrete
  mock has no poses, which is exactly the gap Level 0B closes.
- **Monte Carlo baseline** (seed 3, 200 runs): mean 22.5, median 26, p10 1, p90 31,
  max 33, mean time 501 s. Costs and failure rates in `mock_world.Config` are assumed,
  not measured.
- **Visualisation (discrete):** `runs/mission_demo_seed*.png` shows the arena, bricks,
  board, the numbered path and the step table; the matching `.gif` animates the robot.
- **Spatial run (Level 0B):** seeds 0–9 all `DONE`, score 3, **28.6 s** simulated at
  measured chassis speeds (0.18 m/s, 0.75 rad/s) and a realistic arm reach (0.45 m
  effective), 0 collisions; `verify` is now `ok` (the discrete mock could only say
  `unverified`). Breakdown: goto 11.0 s, pick 4.5 s, goto 9.6 s, place 3.0 s,
  verify 0.5 s — see `docs/TIMING.md`.
- **Live view:** `run_live.py` serves `http://127.0.0.1:8090/` — robot, lidar, trail,
  cubes, score and step list update in real time. Verified end-to-end over HTTP.

### Arena refactor (2026-11-21)

The world now uses the official 0..3 m layout from the rules diagram: origin at the
SW corner, **A top-left, B top-right, C bottom-right, D bottom-left**, classification
table on the left wall between A and D. `config/arena_map.yaml` is the source of truth;
`make_arena.py` derives the Gazebo world and `scenario.json`.

Collection points sit at the red-square centres and `Collection_X_approach` points are
0.35 m inward, so placing 0.30 m ahead lands inside the scoring square. Sorting points
are 0.38 m from the centre cluster, within the modelled arm reach. The discrete scoring
is unaffected.

## Implementation decisions worth knowing

- **A failed pick never marks the cube carried.** Only `result.ok` changes memory;
  otherwise a retry would be blocked by G3 ("already carrying"). Regression test
  `test_failed_pick_does_not_mark_carried`.
- **The memory conflict re-check disables G3.** When perception contradicts memory,
  G1/G2/G4/G5 decide; memory is revised only if the second window passes.
- **No pose → no spatial claim.** Verification returns `unverified` when detections
  carry no world pose, instead of pretending the cube is `misplaced`.

## ROS nodes (require Noetic on the robot)

```bash
python3 mission_node.py          # mission executor
python3 detection_bridge.py      # /objects -> /detections_world
python3 arm_mock_node.py         # test double for the arm
python3 mission_viz.py           # RViz markers
```

Full contracts and deployment: `../docs/ROS_INTEGRATION.md`. These nodes cannot run on
this PC (ROS Humble only).

## Remaining

| Item | Purpose |
|---|---|
| Detection bridge validation | TF/intrinsics/timing on the robot and in Gazebo |
| Board reader | Publish `/classification_board` |
| Gazebo execution | Docker install, then Level 1 runs |
| move_base tuning | speed/inflation/recovery for the arena |

Keep `mission_logic.py` free of `rospy`/ROS imports — same pattern as
`auto/codes/match_logic.py`.
