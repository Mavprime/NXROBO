# Roadmap

Four stages. Each is useful on its own; do not start the next until the acceptance
criteria of the current one pass.

---

## Level 0A — Discrete mission model (exists)

**Goal:** strategy, time budget and scoring under uncertainty, with no geometry.

- `codes/make_arena.py` reads `config/arena_map.yaml` (the canonical 0..3 m map) and
  generates `arena.world` + `scenario.json`; brick types are listed in the map.
- `codes/mock_world.py` — `MockWorld` with `goto_local` / `step_go` / `step_back` /
  `pick` / `drop`, assumed costs and failure rates, rule scoring, baseline strategy,
  `--runs N` Monte Carlo + hand-checked scoring assertions.
- Current baseline (seed 3, 200 runs): mean **22.5**, median 26, p10 1, p90 31, max 33,
  mean time 501 s.

Acceptance (met): `python3 mock_world.py --runs 200` runs, assertions pass, scores match
the rule formulas. Costs and failure rates are placeholders until measured on hardware.

Limitations — why Level 0B is still required:

- no geometry, pose, lidar or camera, so the agreement gate cannot be exercised;
- `pick()` is a coin flip with no class confirmation;
- `drop()` has no level/tower handling and no placement verification;
- no recovery ladder, no relocalization.

---

## Level 0B — Spatial fake world (no Gazebo)

**Goal:** the mission algorithm, gate and memory run end-to-end against a deterministic
spatial world, with a mocked arm.

### Components to write (in `simulation/codes/`)

| File | Role | ROS-dependent? | Status |
|---|---|---|---|
| `mission_logic.py` | steps, statuses, gate, memory, metrics — pure Python | **no** | ✅ done |
| `tests/` | mission logic + static map + ROS contracts + projection + DLT + fetch (80) | no | ✅ done |
| `mock_adapter.py` | mission logic against the discrete `MockWorld` | no | ✅ done |
| `fake_world.py` | 3×3 arena, kinematic diff drive, raycast lidar, camera detections with poses, mocked arm, scoring | **no** | ✅ done |
| `planner.py` | A* around inflated cube faces/table; cube collision checks | **no** | ✅ done |
| `live_view.py` + `run_live.py` | browser view (`/state`, `/stop`), real-time pacing | no | ✅ done |
| `arm_mock.py` (integrated) | pick/place/home behaviour inside `fake_world.py` | no | ✅ done |
| `static_map.py` | map_server PGM/YAML from `arena_map.yaml` | no | ✅ done |
| `ros_contract.py` | JSON contracts, goal resolution, state serialisation | no | ✅ done |
| `mission_node.py` | ROS adapter: `/task_start_flag`, move_base action, `/detections_world`, arm JSON, `/mission_state` | yes | ✅ written, not run |
| `arm_mock_node.py` | mock arm over `/arm/pick|place|result` | yes | ✅ written, not run |
| `mission_viz.py` | RViz MarkerArrays from `/mission_state` | yes | ✅ written, not run |
| `projection.py` | pinhole pixel → ray → plane, unit-tested | **no** | ✅ done |
| `detection_bridge.py` | `/objects` → `/detections_world`; DLT mode for the robot | yes | ✅ DLT passively validated; map/tape pending |
| `bridge_check.py` | live projection check against a measured cube | yes | ✅ used on the real detector |
| `ros/nav_arena.launch` + `nav_bringup_arena.sh` | map_server + AMCL + move_base on the arena map | yes | ✅ written, not run |
| `gazebo/` + `assets/spark1_description` | arena world + Spark model, validated with `gz sdf`/`check_urdf` | yes | ✅ prepared |

The split matters: `mission_logic.py` is imported by the ROS node and by tests, the same
way `auto/codes/match_logic.py` is used now.

### Acceptance

- [x] Demo mission (`config/mission_demo.yaml`) completes in seeds 0–9 (score 3,
  28.6 s simulated at measured chassis speeds, 0 collisions; V2 verification passes).
- [x] Full 10-cube mission (`config/mission_competition.yaml`) — cake first to A, then
  all bears/clocks/bananas to their board zones: score 21, 340.1 s, 0 collisions.
- [ ] All 7 injected faults from `ALGORITHM.md` §6 behave as specified **in the spatial
  world** (gate logic is unit-tested; scenario-level injection is not wired yet).
- [x] Unit tests cover: G1 voting, G2 belief revision, G3 conflict revision, G5 clock
  denial, V1/V2/V3 outcomes, recovery ladder transitions, adaptive fetch (80 tests).
- [ ] Trace JSONL reconstructs each source's belief per action (events exist in the
  report; persistence to JSONL is pending).

### Environment

Runs in the Noetic container below (or natively on the robot), but the pure-logic tests
run with plain `python3` on this PC — no ROS needed.

---

## Level 1 — Gazebo Classic + real navigation stack

**Goal:** the same mission code, unchanged, against `move_base` and a static map in
Gazebo. Validates geometry, costmaps, arrival tolerances and recovery on a physics sim.

### Prerequisites

1. ✅ **Docker** installed 2026-11-22. Docker Hub is blocked on this network; the
   image came through a mirror and is trimmed (no navigation). `docker/Dockerfile`
   adds `ros-noetic-navigation` and produces `spark_sim:noetic`.
2. ✅ **Spark Gazebo model**: `simulation/assets/spark1_description/` copied from the
   robot; lidar reduced to 720 beams/10 Hz and topic changed to `/scan`. The xacro'd
   model passes `check_urdf`; the arena world passes `gz sdf -k` on the robot.
3. ✅ **Static map** of the arena: `codes/static_map.py` generates
   `maps/arena_map.{pgm,yaml}` from `arena_map.yaml` (origin at the SW corner).
4. ✅ **Nav launch**: `ros/nav_arena.launch` + `ros/nav_bringup_arena.sh`
   (map_server, AMCL, move_base). This also unblocks Phase 2 / SI-21 on the real robot.
   Deployment needs the `auto_match` package on the robot.

### Container

```bash
docker run --rm -it --net=host --ipc=host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$PWD/simulation:/ws" \
  osrf/ros:noetic-desktop-full
```

- Gazebo headless (`gui:=false`) for automated runs; GUI or RViz only when debugging.
- X11 forwarding gives RViz from inside the container.
- Watch RAM: this machine has 7.4 GB total. One headless Gazebo + move_base + RViz is
  roughly 2–3 GB. Close the browser during runs.

### Acceptance

- [x] Gazebo + `move_base` reach a real goal: `Sorting_N` returned status 3 on both
  the robot NUC (isolated master 11312) and the Docker container.
- [x] Same demo mission passes with `move_base` in Gazebo, using synthetic
  world-truth detections and the arm mock — on Docker (40.6/8.8 s gotos) and on the
  robot NUC (15.4/9.8 s; ~31.7 s total; `DONE`, cake `delivered`).
- [ ] Arrival error at each waypoint measured and within tolerance in ≥ 9/10 runs.
- [ ] Recovery ladder triggers on an injected obstacle and completes the route.
- [ ] Costmap inflation does not clip the 3×3 arena; robot does not scrape walls.
- [ ] Mission duration recorded as the baseline for the real arena.

---

## Level 2 — Rendered camera + real detector

**Goal:** replace synthetic detections with the real perception path, still without the
arm's fine alignment.

- Add an RGB camera to the Gazebo model (the `spark1_description` Gazebo URDF already
  has a kinect/openni camera link; verify it renders at a usable rate).
- Run the real detector (`detect_ros.py` models) on rendered frames.
- Compare detection output against ground truth: per-class recall/precision at the
  production threshold (0.35), especially banana (SI-25) and the double-box problem
  (SI-26).
- Keep the synthetic path available as a test double; the mission code must not know
  which one is running.

### Acceptance

- Gate verdicts on rendered frames match synthetic verdicts in neutral conditions.
- The known failure modes (banana low confidence, one cube two boxes) are reproduced in
  sim and the gate behaves safely (skip, not blind pick).
- Frame-rate budget: detection window of 5 frames completed inside the mission's
  per-step time allowance.

---

## Explicit non-goals

- Simulating the uArm grasp and vacuum physics. The arm is a service; its mock is
  enough for mission logic. If the friend needs a Gazebo arm, that is a separate
  effort (the bundled Gazebo URDF has the arm as a single fixed link).
- Replacing the field validation of the arena geometry. The map is built from the rules
  diagram; verify the real arena dimensions and point positions on the field (Phase 2).
- Match-day scoring optimisation. Mission duration is measured, but route optimisation
  is Phase 5.

---

## Order of work (suggested)

1. ✅ Level 0A discrete model (`make_arena.py`, `mock_world.py`) — exists and runs.
2. ✅ `mission_logic.py` + tests — executor, gate, memory, recovery.
3. ✅ `mock_adapter.py` — demo mission runs against the discrete mock.
4. ✅ `fake_world.py` + arm mock + live browser view — the spatial loop is closed.
5. ✅ Static map + ROS adapter (`static_map.py`, `mission_node.py`, `arm_mock_node.py`,
   `mission_viz.py`, nav launch) — written, syntax-checked, not yet run on hardware.
6. ✅ Detection bridge (`projection.py` + `detection_bridge.py`) — real-robot DLT mode
   passively validated; tape-measured expected and map/AMCL run pending.
7. Fault-injection suite against the spatial world — prove the gate under G2/V2/V3.
8. Board reader — the last missing input on the robot.
9. ✅ Docker + Gazebo nav validated (Sorting_N goal reached on the robot and in
   Docker). Full demo mission in Gazebo is next.
10. Camera + detector — Level 2.
