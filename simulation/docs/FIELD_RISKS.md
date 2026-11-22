# Field risks: sim → real arena

What the simulation assumes, what the real robot does, and what to check before a
physical run. Cross-references are to `auto/docs/PLAN.md` (SI register).

Severity: **P0** = mission cannot run or is unsafe; **P1** = wrong behaviour, lost
points; **P2** = slower / less robust.

---

## P0 — blockers

| ID | Risk | Sim assumes | Real symptom | Mitigation |
|---|---|---|---|---|
| **R1** | **No navigation stack / wrong map** (SI-21, Phase 2) | `goto` always reaches the named point | Every `goto_local` waits 60 s and fails; mission skips stations or aborts | Deploy `nav_bringup.sh` (`auto/codes/nav_bringup.sh.example`), confirm `/move_base` action server, re-capture `mark_map.pkl` on the real arena. The current file is from a 20×20 m demo map. |
| **R2** | **Arm reach is an estimate** | effective reach **0.45 m** from robot centre; `Sorting_N` at **0.38 m** from the cluster | uArm reaches ~0.32 m from its base (X 90–320 mm); the mount offset is a guess. If the real reach is under ~0.40 m from the robot centre, the centre cake at 0.38 m is marginal | Measure the real reach from the robot centre (arm base + extension), update `arm_reach_m` in `config/robot.yaml`, adjust the sorting stand-off. Validate with one physical centre-cake pick before the run. |
| **R3** | **Named points do not match the real map** | `config/arena_map.yaml` coordinates (from the rules diagram) are the truth | Robot stops at the wrong place; zones/board/collection offsets wrong; possible wall contact | Drive to each named point with teleop, measure, update `arena_map.yaml`. Confirm map frame orientation and the A/B/C/D corner convention on the field. |
| **R4** | **Camera→arm calibration stale or wrong** (SI-29, SI-34) | pixel→arm matrix exact; gate matches cube positions | Arm aims 6–7 cm off; every grasp misses; `verify` fails | Run `python3 calib3d.py --check` before the session; the startup guard rejects >6 px error. Recalibrate after any camera bump. |
| **R5** | **Arm USB re-enumeration** | Arm always responds | Hardcoded `/dev/ttyACM0`; after a replug the old tty is gone. The arm "accepts" poses but never moves; no respawn | Check `ls -l /dev/ttyACM0` before each run; move the arm once and watch it physically; add a udev symlink/respawn (not done). |
| **R6** | **Stop / e-stop wiring** | `STOP` button sets an event checked every 0.05 s | If the ROS adapter only checks `/task_start_flag` at step boundaries, the robot keeps driving/arm moving for seconds | Thread the stop flag into `goto`/`scan`/`pick` loops; test mid-goto; keep the physical e-stop reachable; rules penalise operator contact. |
| **R7** | **Placement geometry** | Cube placed 0.30 m in front; zone square 0.45 m | Arm places at its own reach/offset; chassis heading error moves the drop point; cube lands in the transport square (0 points) | Use `Collection_X_approach` points; verify V3 after every place; measure placement error physically; keep the red square 0.45 m clear. |

---

## P1 — wrong behaviour, lost points

| ID | Risk | Sim assumes | Real symptom | Mitigation |
|---|---|---|---|---|
| **R8** | **`/objects` pose adaptation** | `detection_bridge.py` DLT mode: `transform_3d.dat` → arm → URDF mount → base → map | The real camera is top-down but the URDF camera TF is horizontal; pinhole/TF gives ~10 m errors. DLT mode is validated passively, but map/tape and mount offsets still need a field check | Use `_mode:=dlt`, run `bridge_check.py` against a tape-measured cube, then verify the map/AMCL frame. Do not use pinhole on the robot. |
| **R9** | **G2 has no expected cube position after porting** | memory instances carry brick `(x,y)` from `arena_map.yaml` | On the robot, memory instances have no positions → G2 falls back to the station pose (0.38 m), and every cube at 0.15–0.2 m fails G2 → all picks skipped | Populate memory positions from the map (central cluster geometry) or use a per-station working-area pose with a radius that covers the cluster. |
| **R10** | **Detector thresholds / duplicates / window** (SI-25, SI-26, SI-27) | class seen 3/5 frames at conf ≥ 0.35, boxes separated | Banana sits at 0.35–0.52; one cube yields two boxes 67 px apart; `_frames` deque maxlen 4 < `votes_n=5` | Calibrate per-class threshold or lighting; raise `DEDUPE_DIST_PX` to ~80; raise `_frames` maxlen to 16 or set `votes_n=4`. |
| **R11** | **Vacuum loss in transit** (SI-28) | a successful pick stays held | No pressure sensor; a dropped cube is only discovered at the destination; mission delivers nothing | Keep V2 verification; add a mid-route camera check or pressure sensor; on failure, re-plan a re-pick. |
| **R12** | **Board misread / wrong mapping** (SI-14) | ground-truth board from `arena_map.yaml` | Cards read in the wrong order or not at all; fallback mapping sends classes to wrong zones | Read the board from a fixed pose and orientation; vote across frames; validate that the three classes are distinct; log the mapping and compare with a human spot-check. |
| **R13** | **Cake-first rule** | demo picks cake first | A different first cube voids the cake score; mission order is data, so a wrong route silently loses points | Keep cake as the first pick in every competition mission; add an assertion/log if the first delivered class is not cake. |
| **R14** | **Stacking levels** | only level 1, cube placed on the floor | Levels 2/3 need tower alignment and arm heights; not implemented in the mission | Do not enable tower steps until the arm side provides and tests them. |
| **R15** | **Camera occlusion by the arm** | detections always see the scene | At home pose the arm covers the camera (production note: 1 detection in 40 frames); scans return nothing → station retired | Scan only with the arm in the service pose; make the arm move aside for every `scan_window`; verify the pose physically. |
| **R16** | **Verify false alarms** | the placed cube is in view, unobstructed | The arm or chassis may hide the cube; V2 returns `unverified` and the mission logs a failure that did not happen | Retract the arm before verification; look from a second pose; treat `unverified` as a warning, not a delivery failure. |

---

## P2 — timing and robustness

| ID | Risk | Sim assumes | Real symptom | Mitigation |
|---|---|---|---|---|
| **R17** | **Real time longer than 28.6 s** | straight-line paths, 0.18 m/s, 4.0 s pick | `move_base` speed unknown (Phase 2); detours, detection inside `grasp()`, re-scans, odometry corrections | Treat 28.6 s as a baseline; measure the physical run; update `config/robot.yaml`; keep the 540 s budget and G5 reserve. |
| **R18** | **No recovery on the robot** (4.2b) | recovery ladder clears costmaps, rotates, backs up | `move_base` abort just skips the station; the robot can wedge in a corner | Port the ladder or add `cancel_goal` + backup + clear_costmap; test with an injected obstacle. |
| **R19** | **Open-loop drive/turn error** (SI-23) | exact arrival within 0.06 m | 10–15 % distance error; turns overshoot; re-scan after every correction | Use `move_base` feedback for station-to-station; keep re-scan after small corrections; calibrate `drive_base`/`turn_base`. |
| **R20** | **Costmap / collisions** | straight paths, no inflation | Robot scrapes walls, clips the table, stuck IR bumper (`ir_bumper_front` reads True) | Tune `inflation_radius`, `max_vel_x`, `acc_lim_x`; test near walls; check `/spark_base/sensor`. |
| **R21** | **ROS integration** | in-process calls, no latency | Blocking `goto` starves callbacks; service timeouts; no respawn; `runtask.py` and the sim competing for `/cmd_vel` | One owner of `/cmd_vel`; use `rospy.Rate` loops with shutdown checks; add respawn and clear diagnostics. |
| **R22** | **Rules / operator** | no humans, no false start | Touching the robot = penalty; false start −6; starting before the whistle | Pre-flight checklist; start only on the official signal; keep hands off during the run. |

---

## What the simulation cannot tell you

- **Pick success probability.** The sim's arm either finds a cube in reach or returns
  `ok=false`; it never slips, misaligns, or grabs the wrong neighbour.
- **Detection error rates.** Ground-truth-ish detections with 6 % dropout; real YOLO has
  class confusion, low banana confidence and duplicate boxes.
- **Path length.** Straight lines; the real planner adds detours and recovery.
- **Physics.** No wheel slip, no inertia beyond the velocity ramp, no collisions except
  the table, no arm collision with cubes or the table.
- **Sensor mount.** No camera occlusion by the arm and no lidar blind spots.

---

## Pre-flight checklist (in order)

1. `calib3d.py --check` → PASS (≤6 px), matrix age and LOO error printed.
2. Topics alive: `/camera/color/image_raw`, `/scan`, `/objects`, `/spark_base/sensor`.
3. Arm: correct `/dev/ttyACM0`; move it once and watch it move; pump on/off.
4. Navigation: `/move_base` action server up; map matches the arena; drive to each named
   point with teleop and correct the coordinates.
5. One leg dry run: `A → Sorting_N`; compare time with the sim's 11.0 s.
6. One pick + one place + verify at reduced scope; compare with 4.5 s / 3.0 s / 0.5 s.
7. Stop test: publish `stop` mid-goto; confirm the robot halts within one control cycle.
8. Full mission, no cubes (dry run), then one cube, then the full task.
9. Record bags (`start_system.sh` already writes `~/logs/bags/match_*.bag`).
10. Update `config/robot.yaml` with measured speeds and arm times; re-run the sim and
    compare the per-step report.
