# Timing model

Goal: predict how long the physical Spark takes for a mission, so simulation runs can be
compared with real runs.

## Measured parameters

All values live in [`config/robot.yaml`](../config/robot.yaml).

| Parameter | Value | Where it comes from |
|---|---|---|
| Linear speed | **0.18 m/s** | `auto/codes/runtask.py:538` (`drive_base`) |
| Angular speed | **0.75 rad/s** | `auto/codes/runtask.py:507` (`turn_base`) |
| Linear accel / decel | **3.0 / 5.0 m/s²** | `auto/codes/keypatched2v.py:75-76` |
| Angular accel / decel | **3.0 / 8.0 rad/s²** | `auto/codes/keypatched2v.py:77-78` |
| Post-move settle | **0.10 s** | `runtask.py:525,555` |
| Arm reach (from robot centre) | **0.45 m** (estimate) | uArm ≈0.32 m from its base + ~0.10 m mount offset + margin; **measure this** |
| Pick duration | **4.0 s** (estimate) | `runtask.grasp()` delays: 0.9 + 0.1 + 0.8 + 0.4 + 0.8 ≈ 3.2 s + margin |
| Place duration | **3.0 s** (estimate) | `runtask.drop()`/`pick_direct` delays: 0.8 + 0.4 + 0.6 + 0.8 + 0.4 ≈ 3.0 s |
| Detection frame period | **0.10 s** | 10 Hz voting window (`GRASP_FRAMES`) |
| Scan timeout | **3.0 s** | production `detector(num_samples=6, timeout=3.0)` |

**Teleop FAST mode can do 0.45 m/s** (`keypatched2v.py:61`), but the autonomous
`drive_base()` used by the mission is 0.18 m/s. Use `--lin-speed 0.45` to see the upper
bound.

## What the simulator models

- differential-drive kinematics with acceleration and deceleration ramps;
- rotate-in-place for large heading errors, then drive; a 0.10 s settle at each waypoint;
- straight-line go-to between named points (no path planner);
- detection windows at 10 Hz;
- arm pick/place as fixed durations with the cube changing hands at the right moment;
- a 600 s match clock and the 540 s mission budget.

## Current run (seed 3, measured chassis)

| Step | Simulated time |
|---|---:|
| goto Sorting_N | 11.7 s |
| pick cake | 4.5 s (4.0 arm + 0.5 scan) |
| goto Collection_A_approach | 9.6 s |
| place cake | 3.0 s |
| verify cake | 0.5 s |
| **total** | **29.3 s** |

Score 3, zero collisions. This is the number to compare against a physical run.

## What is NOT modelled (real runs will be slower)

1. **`move_base` path planning.** The sim drives straight. The real robot uses the nav
   stack; its `max_vel_x` is not configured yet (Phase 2), and it may detour around the
   brick cluster or the table. Speed may be above or below 0.18 m/s.
2. **The arm's own detection and alignment.** Production `grasp()` calls
   `cam.detector()` again (up to 3 s) and may turn/drive the base to align. Pick can be
   1–4 s longer than 4.0 s.
3. **Odometry error.** `drive_base()`/`turn_base()` are open-loop; the docs already
   record 10–15 % distance error, which triggers re-scans and corrections.
4. **Localization, costmap inflation, recovery behaviours, ROS/service latency,
   battery voltage effects.**
5. **Tower stacking.** Only level 1 is modelled.

So treat 29.3 s as the **drive + arm baseline at production speeds**, not a guarantee.
A real run of the same mission will likely be somewhat longer until the nav stack is
tuned.

## How to calibrate with a real run

1. Run the same mission physically and record the time per step (the mission log already
   prints each step; add timestamps if needed).
2. Update `config/robot.yaml` with the measured values:
   - speed: drive a 1 m straight line, time it → `max_linear_mps = 1 / t`;
   - turn: turn 90°, time it → `max_angular_rps = (π/2) / t`;
   - pick/place: time the arm sequence from the logs.
3. Re-run and compare:

```bash
cd simulation/codes
python3 run_live.py --report ../runs/run_seed3.json
```

`run_seed3.json` contains `steps[].elapsed_s`, `sim.t`, the score and the parameters used.

4. If the real navigation runs at a different speed, override without editing the file:

```bash
python3 run_live.py --lin-speed 0.30 --ang-speed 0.75
```
