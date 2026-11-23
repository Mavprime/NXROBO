# NXROBO Spark: Autonomous Competition Robot (ROS Noetic)

[![Platform](https://img.shields.io/badge/Platform-Ubuntu%2020.04%20LTS-orange.svg)](https://ubuntu.com/)
[![ROS](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Hardware](https://img.shields.io/badge/Robot-NXROBO%20Spark%20%7C%20uArm%20Swift%20Pro-green.svg)](https://www.nxrobo.com/)
[![Python](https://img.shields.io/badge/Python-3.8%2B-yellow.svg)](https://www.python.org/)

Complete software stack for the **NXROBO Spark** mobile manipulator competing in the
**10-minute fully autonomous sorting challenge** (自动任务赛 "Treasure Hunt" / «Полный автоном»).

The stack covers the whole chain: object detection → rule-aware decision making →
inverse-kinematics-validated grasping → waypoint navigation → tier stacking →
one-click operations, diagnostics and a unit-test suite.

---

## ✅ What Works Right Now

| Subsystem | Status | Notes |
|---|---|---|
| Chassis & wheel motors | 🟢 Verified | IR bumper freeze fixed, `/cmd_vel` publishing fixed |
| uArm Swift Pro manipulator | 🟢 Verified | All servos, angles and positioning checked on hardware |
| Vacuum pump | 🟢 Verified | Hold and release tested through `/pump_topic` |
| Physical cube grasp & release | 🟢 Verified | In-place grasp, re-scan after rotation, vacuum dwell, safe vertical retract |
| Arm service/tuck pose | 🟢 Verified | Retracts to `(50, 180, 160)` mm to clear camera and LiDAR FOV |
| RealSense camera stream | 🟢 Verified | MJPEG over HTTP on `http://<robot_ip>:8080/` |
| YOLO detection + classification | 🟢 Verified | 10-frame temporal voting + spatial clustering (45 px) removes flicker |
| Autonomous mission FSM (`runtask.py`) | 🟢 Verified | Cake-first rule, board mapping, 10-item sorting loop, tiered drops |
| Waypoint navigation | 🟢 Verified | Named locations + step-back/step-go maneuvers around the arena |
| 1-click operator dashboard | 🟢 Verified | Options 1–8, auto-detects the latest teleop script |
| Detached background processes | 🟢 Verified | Every node is started with `setsid` and survives SSH/terminal closure |
| Hardware self-diagnostics | 🟢 Verified | `test_all_systems.py` / dashboard option 7 |
| Logic unit tests | 🟢 Implemented | Pure-Python suite, no ROS required (see below) |
| Live log viewer | 🟢 Verified | `view_log.py` exits instantly on `q` / `Enter` / `Esc` |

---

## 🎯 Competition Rules Implemented

The robot must operate fully autonomously inside a 3 × 3 m arena for 10 minutes:

1. **Strict priority — Cake first.** The Cake on the 2nd central tier (Z = 45 mm) **must be
   delivered to Zone A first**. The FSM enforces this twice: the very first grasp uses the
   tier-2 routine `grasp1()`, and the drop target is hard-pinned to `Collection_A` for the
   first item and for every cake detected afterwards. Any cake delivered later also goes to
   Zone A. Missing this yields 0 points for the cake.
2. **Dynamic sorting of the remaining 9 items.** After the Cake, 3 Teddy Bears, 3 Clocks and
   3 Bananas are classified from the **classification board** and sorted into Zones B, C and D
   according to the board layout detected at runtime (not a hardcoded mapping).
3. **Tier stacking.** Red target squares (0.45 × 0.45 m) score `N + 2^(N-1)`
   (Tier 1 = 2 pts, Tier 2 = 4 pts, Tier 3 = 7 pts). `drop()`, `drop1()` and `drop2()`
   place items at the calibrated tier heights (Z = −55 / 60 / 160 mm).

---

## 🏗️ System Architecture

```text
Host PC (Windows 10/11)
  ├── run_grasp_test.bat     --> grasp/view/scan test: uploads the field code and runs it
  ├── run_scan_only.bat      --> shortcut for run_grasp_test.bat --scan-only
  ├── auto\launchers\START_ROBOT.bat --> 1-click launcher: asks for the IP and opens the dashboard
  ├── auto\launchers\upload.ps1      --> SCP code sync + automatic background restart of runtask.py
  ├── auto\launchers\connect.ps1     --> Quick SSH terminal launcher
  └── auto\docs\remote_spark_ops.py  --> Deep remote diagnostics over SSH (USB, processes, ROS, logs)
          │
      (SSH / SCP)
          ▼
Robot onboard NUC (Ubuntu 20.04 LTS + ROS Noetic)
  ├── ~/codes/dashboard.sh       --> Unified operator interface (options 1–8)
  ├── ~/codes/start_system.sh    --> Idempotent background bringup of every subsystem
  ├── ~/codes/stop_system.sh     --> Safe shutdown, pump/motor stop, bag close
  ├── ~/codes/keypatched2v.py    --> Smooth keyboard teleoperation (self-bootstrapping ROS env)
  ├── ~/codes/camera_stream.py   --> MJPEG web stream on port 8080
  ├── ~/codes/detect_ros.py      --> YOLO detector publishing /objects (Detection2DArray)
  ├── ~/codes/robot_view.py      --> Annotated camera snapshot (no motion)
  ├── ~/codes/grasp_generic.py   --> Field grasp test: scan -> grasp -> lift -> hold -> return
  ├── ~/codes/arm_ik.py          --> Compatibility shim over match_logic kinematics
  ├── ~/codes/match_logic.py     --> ROS-free rules, IK, vision and scoring logic
  ├── ~/codes/runtask.py         --> Competition FSM (perception → grasp → navigate → drop)
  ├── ~/codes/test_all_systems.py --> Hardware/ROS diagnostics
  ├── ~/codes/view_log.py        --> Interactive log viewer
  └── ~/codes/tests/             --> Pure-Python unit tests for the mission logic
```

---

## ⚙️ Feature Highlights

### Autonomous mission FSM — `runtask.py`

* Waits for `/task_start_flag`, then runs the full mission in a worker thread;
  `/grasp 1` / `/grasp 0` still accept manual grasp/release commands.
* Navigates to `Classification_area`, reads the board for up to 6 s and builds a
  `zone → item` mapping with temporal voting (≥ 0.45 confidence, 12 accepted samples).
* Executes a sort-plan over `Sorting_W/S/E/N/M` (2 picks per station) and delivers
  to `Collection_A/B/C/D`.
* Chooses an approach waypoint (`Step_NW/NE/SW/SE`) and a `step_back()` maneuver
  so the robot leaves each sorting station without colliding with the centre board.
* Rule guard: if navigation to `Classification_area` fails, the mission stops with a
  clear error instead of continuing with an empty mapping.

### Perception pipeline

* YOLO detector (`detect_ros.py`) publishes detections on `/objects`.
* **Temporal voting:** frames are accumulated (10 frames in `CamAction.detector()`)
  and the majority class wins — eliminates class flicker between consecutive frames.
* **Spatial clustering:** detections within 45 px are merged and their coordinates
  averaged, giving a stable target centre instead of a jittering bounding box.
* ID normalization (`norm_id`) unifies raw model IDs and "site" IDs
  (1/10 → teddy, 2/20 → cake, 3/30 → clock, 4/40 → banana).

### Grasp pipeline — perception-driven, IK-validated

* **Direct in-place grasp:** if the cube is already inside the arm workspace
  (X ∈ [120, 320] mm, Y ∈ [−190, 190] mm, IK-validated), the wheeled base does
  **not** move — the arm picks the cube immediately.
* **Out-of-reach recovery:** the base rotates in place towards the cube
  (visual servoing, `delta_u` → radians). Forward motion only when the cube is
  farther than 295 mm (`x > 295`), so the bumper never pushes the cube.
* **Re-scan after motion:** waits 0.20 s for chassis vibration to settle, re-runs
  detection and recomputes coordinates from the new viewpoint. The originally
  targeted class is preserved to avoid class flips sending an item to the wrong zone.
* **Reliable vacuum grasp:** `Z_touch = −35 mm` (contact top face, not the floor),
  pump enabled before descent, 0.40 s vacuum dwell, then a confident vertical lift
  (`z_lift` computed from the measured cube surface).
* **Reachability filter:** `ik_reachable(x, y, z)` reproduces the firmware's own IK
  mathematics, so unreachable poses are rejected before the arm silently refuses to move.
* **Safe release:** the pump is released (0.25 s vacuum relief), the suction cup lifts
  vertically **+80 mm**, and only then does the arm retract to the service pose —
  preventing friction from tipping the placed cube.

### Navigation

* Named waypoints: `Classification_area`, `Sorting_W/S/E/N/M`, `Collection_A/B/C/D`,
  `Step_NW/NE/SW/SE`.
* `goto_local()` / `arriveto_local()` with arrival verification; `step_go()` / `step_back()`
  for short, collision-safe repositioning.

### Operator dashboard — `dashboard.sh` (options 1–8)

```text
1) START ALL SYSTEMS          -> start_system.sh (idempotent)
2) TEST CUBE GRASP            -> publishes /grasp 1
3) TEST CUBE RELEASE          -> publishes /grasp 0
4) SMOOTH KEYBOARD TELEOP     -> auto-detects the newest key*.py / *teleop*.py
5) VIEW LIVE RUNTASK LOG      -> view_log.py, instant exit on q/Enter/Esc
6) STOP ALL SYSTEMS           -> stop_system.sh
7) DIAGNOSTICS & SYSTEM TEST  -> test_all_systems.sh
8) VIEW DETECTOR LOG          -> follows ~/logs/detect.log
0) Exit to bash
```

### Process management

* `start_system.sh` is **idempotent** — every subsystem is checked with `rosnode list` /
  `pgrep` before launch, so re-running it never creates duplicates.
* Every long-running process (bringup, uArm, camera stream, detector, `runtask.py`) is
  started with `setsid nohup … < /dev/null` and logs to `~/logs/*.log`. Closing SSH,
  MobaXterm or the terminal **no longer kills the robot**.
* `upload.ps1` restarts the background `runtask.py` after every code sync, so the robot
  always runs the file you just uploaded.

### Teleoperation

* `keypatched2v.py` and `keyboard_teleop.py` accept smooth WASD-style driving.
* Teleop scripts bootstrap the ROS environment into `sys.path`, so they can be run
  directly from a clean shell without `source`-ing the workspace manually.

### Remote diagnostics — `remote_spark_ops.py`

Connects over SSH (key first, password fallback) and reports USB devices, running
processes, ROS node list, serial ports, calibration files and recent logs — useful
when the robot is unreachable through the dashboard.

---

## 🚀 Quick Start

### 0. Prerequisites

* Robot: NXROBO Spark with Ubuntu 20.04 + ROS Noetic, uArm Swift Pro, vacuum pump,
  RealSense D435, YDLIDAR.
* Host: Windows 10/11 with PowerShell and an SSH client.

### 1. Connect and open the dashboard

Double-click **`auto\launchers\START_ROBOT.bat`** (or run `auto\launchers\connect.ps1`):

1. It shows the last saved IP (stored in `robot_ip.txt`, git-ignored).
2. Press `Enter` to accept it or type the robot's new Wi-Fi IP.
3. The interactive dashboard opens automatically.

If any subsystem is not running, it is started automatically in the background.

### 2. Sync code to the robot

```powershell
.\auto\launchers\upload.ps1
```

Copies the current `auto\codes\` files to `~/codes/` and the ROS nodes on the robot
(`~/spark_noetic/.../auto_match/nodes/`), then restarts the background `runtask.py`.

### 3. Field grasp test (optional)

```powershell
.\run_grasp_test.bat              # full grasp cycle: scan -> pick -> lift -> hold -> return
.\run_grasp_test.bat --view       # annotated camera snapshot, no grasp motion
.\run_scan_only.bat               # text-only target assessment, no motion
```

The launcher uploads the field-test files itself and works from the repo root or from
`auto\` (it resolves the repository root). It deliberately does **not** overwrite
`~/transform_3d.dat`, so a calibration fitted on the robot survives a code deploy; use
`auto\launchers\upload.ps1 -PushCalib` when you really mean to push the repo matrix.

### 3. Start the competition mission

From the dashboard choose `1` (start all systems), then publish the mission start:

```bash
rostopic pub /task_start_flag std_msgs/String "data: 'start'" --once
```

### 4. Watch the live camera

Open `http://<robot_ip>:8080/` in any browser — real-time MJPEG from the RealSense D435.

---

## 🧪 Tests & Diagnostics

### Logic unit tests (no ROS required)

```bash
cd ~/semi
python3 tests/run_all_tests.py        # on the robot
# or, from any Python 3 host:
python3 -m unittest discover -s tests -p "test_*.py"
```

| Test file | Covers |
|---|---|
| `test_normalizer.py` | Raw ID → canonical item ID normalization (1/10, 2/20, 3/30, 4/40) |
| `test_fsm_transitions.py` | Classification-board decoding into B/C/D mapping, Cake pinned to A |
| `test_voting_and_clustering.py` | Temporal voting and 45 px spatial clustering of detections |
| `test_ik_reachability.py` | uArm IK reachability for standard competition poses |
| `test_scoring_rules.py` | `N + 2^(N-1)` scoring for standard and red stacking zones |

`arm_ik.py` is deliberately ROS-free so the IK mathematics can be tested on any machine.

### Hardware self-diagnostics

From the dashboard choose `7`, or on the robot:

```bash
bash ~/codes/test_all_systems.sh
```

Checks: factory calibration files (`thefile.txt`, `thefile2.txt`), every required
process (roscore, base, RealSense, uArm, camera stream, detector, `runtask`), serial
ports (`/dev/ttyACM*`, `/dev/ttyUSB*`), the web stream on :8080 and required ROS topics.

### Deep remote diagnostics

```powershell
python auto\docs\remote_spark_ops.py
```

---

## 📂 Repository Layout

```text
.
├── run_grasp_test.bat         # Field grasp/view/scan test (canonical launcher)
├── run_scan_only.bat          # Shortcut for run_grasp_test.bat --scan-only
├── README.md                  # This file
├── auto/
│   ├── codes/                 # Production code deployed to the robot
│   │   ├── match_logic.py     # ROS-free rules, IK, vision and scoring logic
│   │   ├── runtask.py         # Competition FSM
│   │   ├── detect_ros.py      # YOLO detector node (dual model -> /objects)
│   │   ├── grasp_generic.py   # Field grasp test (scan -> pick -> lift -> return)
│   │   ├── robot_view.py      # Annotated camera snapshot (no motion)
│   │   ├── calib3d.py         # Hand-eye calibration -> ~/transform_3d.dat
│   │   ├── pad_detect.py      # Suction-pad blob detector
│   │   ├── arm_ik.py          # Compatibility shim over match_logic
│   │   ├── dashboard.sh       # Operator dashboard
│   │   ├── start_system.sh    # Idempotent bringup
│   │   ├── stop_system.sh     # Safe shutdown
│   │   ├── camera_stream.py   # MJPEG stream server
│   │   ├── test_all_systems.py# Hardware diagnostics
│   │   ├── view_log.py        # Live log viewer
│   │   ├── transform_3d.dat   # Shipped hand-eye matrix (numpy 4x3)
│   │   └── tests/             # Unit test suite (157 tests)
│   ├── launchers/             # START_ROBOT.bat, connect.ps1, upload.ps1, robot_ip.txt
│   └── docs/                  # Handoff, operator docs, remote diagnostics
├── semi/                      # Legacy teleoperation/diagnostics (not deployed)
├── simulation/                # Chassis simulator track
├── Manuals/                   # Organiser documentation
└── snapshots/                 # Downloaded view images (git-ignored)
```

---

## 🔧 Calibration & Configuration

* `thefile.txt` — camera→arm coefficients for the **ground tier** (Z = −55 mm).
* `thefile2.txt` — camera→arm coefficients for the **2nd tier** (Z = 45 mm).
* Both files are factory-calibrated and stored in the robot's home directory; the
  diagnostics script verifies they exist and contain four coefficients.
* `~/transform_3d.dat` — the hand-eye matrix fitted by `calib3d.py` (numpy 4×3). It is
  loaded by `runtask.py` / `grasp_generic.py`; on startup the code sanity-checks it
  (`check_transform`) and compares its MD5 with `~/transform_3d.meta.json`.
* `~/transform_3d.meta.json` — sidecar written by `calib3d.py`: fit date, poses,
  leave-one-out residuals, intrinsics, the matrix MD5 and one reference pad pixel.
  A mismatch between the MD5 in the sidecar and the matrix on disk means the matrix
  was replaced after that fit (usually by a deploy that pushed an older matrix): the
  sidecar check frame is then ignored and the FSM performs an active pad check before
  it will run. Re-run `python3 -u ~/codes/calib3d.py` to fit a fresh, consistent pair.
* The `run_grasp_test.bat` launcher and `upload.ps1` do **not** overwrite
  `~/transform_3d.dat` (only `upload.ps1 -PushCalib` does), so a matrix fitted on the
  robot is never silently replaced by the one in the repository.
* Waypoint names come from the robot's saved map; edit `runtask.py` if your arena
  uses different location names.
* The robot IP is stored locally in `robot_ip.txt` and is **git-ignored**; it is not
  hardcoded into the launch flow.

---

## 🔒 Safety & Repository Hygiene

* SSH keys (`.ssh_key`, `.ssh_key.pub`, `*.pem`, `*.key`, `id_rsa*`) and the local
  `robot_ip.txt` are excluded via `.gitignore`.
* Heavy vendor installers/executables (`.exe`, `.msi`, `.dat`, MobaXterm bundle) are
  not part of this source tree.
* This is a closed team repository. Do not publish robot credentials outside the team.

---

## 📌 Known Limitations / Next Steps

* `run_task()` aborts cleanly if navigation to `Classification_area` fails, but a
  pre-defined fallback mapping for B/C/D is still on the roadmap (SI-14).
* No `rosbag record` is started automatically during competition runs (SI-15).
* The dead `target` variable in `grasp()`/`grasp1()` should be removed (SI-13).
* Unit tests are runnable on demand but not yet wired into CI.

---

## 👥 Team

Built by the **Mavprime** competition team. Contributions and fixes are welcome
through pull requests against `main`.
