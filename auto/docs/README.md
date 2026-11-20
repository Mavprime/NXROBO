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
| YOLO detection + classification | 🟢 Implemented | Two models merged by `dedupe_boxes`; grasp decision uses a 4-frame majority vote + 45 px clustering |
| Rule engine (`match_logic.py`) | 🟢 Tested | ROS-free module: voting/clustering, board mapping, zone/tier planning, match clock, command parsing |
| Autonomous mission FSM (`runtask.py`) | 🟡 Code complete | Cake-first with class check, board mapping, 10-cube loop, match clock, station retirement — **not yet run end-to-end on the arena** |
| Waypoint navigation | ⛔ Blocked | Needs the arena map + `mark_map.pkl` + a `nav_bringup.sh`; `move_base` is not launched by default |
| 1-click operator dashboard | 🟢 Verified | Options 0–9, auto-detects the latest teleop script |
| Detached background processes | 🟢 Verified | Every node is started with `setsid` and survives SSH/terminal closure |
| Mission bag recording | 🟢 Implemented | `start_system.sh` records `~/logs/bags/match_*.bag` (SI-15) |
| Hardware self-diagnostics | 🟢 Verified | `test_all_systems.py` / dashboard option 7 (now also checks `/objects`, `move_base`, `mark_map.pkl` and the bag) |
| Logic unit tests | 🟢 Verified | 90 tests, ROS-free, importing the production modules |
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
  ├── START_ROBOT.bat        --> 1-click launcher: asks for the IP and opens the dashboard
  ├── upload.ps1             --> SCP code sync + automatic background restart of runtask.py
  ├── connect.ps1            --> Quick SSH terminal launcher
  └── remote_spark_ops.py    --> Deep remote diagnostics over SSH (USB, processes, ROS, logs)
          │
      (SSH / SCP)
          ▼
Robot onboard NUC (Ubuntu 20.04 LTS + ROS Noetic)
  ├── codes/dashboard.sh     --> Unified operator interface (options 0–9)
  ├── codes/start_system.sh  --> Idempotent bringup of every subsystem + mission bag
  ├── codes/stop_system.sh   --> Safe shutdown, bag close, pump/motor reset
  ├── codes/keypatched2v.py  --> Smooth keyboard teleoperation (self-bootstrapping ROS env)
  ├── codes/camera_stream.py --> MJPEG web stream on port 8080
  ├── codes/detect_ros.py    --> Dual YOLO detector publishing /objects (de-duplicated)
  ├── codes/arm_ik.py        --> Compatibility shim re-exporting the kinematics
  ├── codes/match_logic.py   --> ROS-free match logic: voting, board, zones, tiers, clock
  ├── codes/runtask.py       --> Competition FSM (thin ROS layer over match_logic)
  ├── codes/test_all_systems.py --> Hardware + mission-readiness diagnostics
  ├── codes/view_log.py      --> Interactive log viewer
  ├── codes/nav_bringup.sh   --> (operator-provided, Phase 2) map + move_base + mark_nav
  └── codes/tests/           --> 90 ROS-free unit tests against the production modules
```

---

## ⚙️ Feature Highlights

### Autonomous mission FSM — `runtask.py`

* Waits for `/task_start_flag`, then runs the full mission in a worker thread;
  `/grasp 1` / `/grasp 0` still accept manual grasp/release commands.
* `start`/`1`/`go`/`on`/`true` start the mission, `stop`/`0`/`halt`/`off`/`abort` stop it,
  anything else is ignored with a warning (an unknown payload never starts a run).
* A `MatchClock` caps the mission at **540 s** of the 600 s match; it is checked before
  every station and before every grasp, so the robot finishes instead of paying overtime.
* Navigates to `Classification_area`, reads the board for up to 6 s and builds a
  `zone → item` mapping with temporal voting (≥ 0.45 confidence, 12 accepted samples).
* If the board area is unreachable or the board is unreadable, the mission continues with
  `DEFAULT_MAPPING` (cake→A, bear→B, clock→C, banana→D) instead of stopping.
* Executes a sort-plan over `Sorting_W/S/E/N/M` (2 picks per station) and delivers
  to `Collection_A/B/C/D`; a station that yields nothing twice is retired, so the loop
  always terminates.
* Chooses an approach waypoint (`Step_NW/NE/SW/SE`) and a `step_back()` maneuver
  so the robot leaves each sorting station without colliding with the centre board.
* Cake first: drives to `CAKE_SORTING_STATION` (`Sorting_M`, verify on the arena), uses the
  tier-2 grasp and checks the grasped **class** before pinning the drop to `Collection_A`.

### Perception pipeline

* YOLO detector (`detect_ros.py`) publishes detections on `/objects`. Two models run per
  frame and their boxes are merged with `match_logic.dedupe_boxes()` (same class, 60 px)
  so one physical cube is one detection.
* **Temporal voting:** `CamAction.detector()` collects `GRASP_FRAMES = 4` frames that arrive
  *after* the call (so a post-rotation re-scan never reads pre-rotation frames) and picks the
  majority class — eliminates class flicker.
* **Spatial clustering:** detections within 45 px are merged and their coordinates averaged,
  giving a stable target centre instead of a jittering bounding box.
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
* **Reliable vacuum grasp:** heights are derived from the measured cube surface
  (`contact_heights()`: hover +45 mm, touch +8 mm with a −120 mm floor, lift +100 mm
  capped at 165 mm), pump enabled before descent, 0.50 s vacuum dwell, then a confident
  vertical lift.
* **Reachability filter:** `ik_reachable(x, y, z)` reproduces the firmware's own IK
  mathematics, so unreachable poses are rejected before the arm silently refuses to move.
* **Safe release:** the pump is released (0.25 s vacuum relief), the suction cup lifts
  vertically **+80 mm**, and only then does the arm retract to the service pose —
  preventing friction from tipping the placed cube.

### Navigation

* Named waypoints: `Classification_area`, `Sorting_W/S/E/N/M`, `Collection_A/B/C/D`,
  `Step_NW/NE/SW/SE`.
* `goto_local()` / `arriveto_local()` with arrival verification and one retry; `step_go()` /
  `step_back()` for short, collision-safe repositioning.
* **Not launched by `start_system.sh`:** the navigation stack arrives with the arena map
  (Phase 2). Put the exact command in an executable `~/codes/nav_bringup.sh`; the startup
  script runs it, waits for `/move_base`, and warns loudly when navigation is missing.

### Operator dashboard — `dashboard.sh` (options 0–9)

```text
1) START ALL SYSTEMS          -> start_system.sh (idempotent, also starts the bag)
2) TEST CUBE GRASP            -> publishes /grasp 1
3) TEST CUBE RELEASE          -> publishes /grasp 0
4) SMOOTH KEYBOARD TELEOP     -> auto-detects the newest key*.py / *teleop*.py
5) VIEW LIVE RUNTASK LOG      -> view_log.py, instant exit on q/Enter/Esc
6) STOP ALL SYSTEMS           -> stop_system.sh (closes the bag cleanly)
7) DIAGNOSTICS & SYSTEM TEST  -> test_all_systems.sh
8) VIEW DETECTOR LOG          -> follows ~/logs/detect.log
9) MISSION BAGS               -> list ~/logs/bags, stop the recorder
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

Double-click **`START_ROBOT.bat`** (or run `connect.ps1`):

1. It shows the last saved IP (stored in `robot_ip.txt`, git-ignored).
2. Press `Enter` to accept it or type the robot's new Wi-Fi IP.
3. The interactive dashboard opens automatically.

If any subsystem is not running, it is started automatically in the background.

### 2. Sync code to the robot

```powershell
.\upload.ps1
```

Copies the current `codes/` to `~/codes/` on the robot and restarts the background
`runtask.py`.

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
cd ~/codes
python3 tests/run_all_tests.py        # on the robot
# or, from any Python 3 host:
python3 -m unittest discover -s tests -p "test_*.py"
```

| Test file | Covers |
|---|---|
| `test_normalizer.py` | Raw ID → canonical item ID normalization (1/10, 2/20, 3/30, 4/40) |
| `test_fsm_transitions.py` | Board decoding, zone/tier planning, station routing and exit manoeuvres |
| `test_voting_and_clustering.py` | Temporal voting, 45 px clustering, cross-model de-duplication, target selection |
| `test_ik_reachability.py` | uArm IK reachability + proof that the shim and `match_logic` share one implementation |
| `test_scoring_rules.py` | `N + 2^(N-1)` scoring and the end-of-match score estimate |
| `test_match_clock.py` | Match time budget and `/task_start_flag` payload parsing |
| `test_wiring.py` | The production files really call the tested logic (and no test copies it) |

The suite imports `match_logic.py` / `arm_ik.py` directly — the same modules that run on the
robot. `test_wiring.py` fails if a test file ever re-implements production logic again, which
is exactly how the previous "27/27 green" suite hid three real defects.

`arm_ik.py` is deliberately ROS-free so the IK mathematics can be tested on any machine.

### Hardware self-diagnostics

From the dashboard choose `7`, or on the robot:

```bash
bash ~/codes/test_all_systems.sh
```

Checks: factory calibration files (`thefile.txt`, `thefile2.txt`), every required
process (roscore, base, RealSense, uArm, camera stream, detector, `runtask`), serial
ports (`/dev/ttyACM*`, `/dev/ttyUSB*`), the web stream on :8080, required ROS topics
(including `/objects`), and mission readiness: `move_base`, `mark_map.pkl`, the bag
recorder and the optional `~/transform_3d.dat` calibration.

### Deep remote diagnostics

```powershell
python remote_spark_ops.py
```

---

## 📂 Repository Layout

```text
.
├── START_ROBOT.bat            # Windows 1-click launcher
├── upload.ps1                 # SCP sync + remote runtask restart
├── connect.ps1                # SSH terminal launcher
├── remote_spark_ops.py        # Remote diagnostics over SSH
├── README.md                  # This file
├── PLAN.md                    # Roadmap, verified phases, issue register
├── CONTEX.md                  # Hardware parameters, topics, verified status
├── robot_startup.md           # Operator pre-match checklist
├── codes/                     # Auxiliary/experimental robot scripts
├── Commit/                    # Self-contained verified release bundle
│   ├── codes/                 # Production code executed on the robot
│   ├── launchers/             # Launchers shipped with the release
│   └── README.md              # Deployment notes for the release
└── spark_humble_ws/
    ├── codes/                 # Main development source tree
    │   ├── runtask.py         # Competition FSM
    │   ├── detect_ros.py      # YOLO detector node
    │   ├── camera_stream.py   # MJPEG stream server
    │   ├── dashboard.sh       # Operator dashboard
    │   ├── start_system.sh    # Idempotent bringup
    │   ├── stop_system.sh     # Safe shutdown
    │   ├── arm_ik.py          # IK reachability
    │   ├── test_all_systems.* # Hardware diagnostics
    │   ├── view_log.py        # Live log viewer
    │   └── tests/             # Unit test suite
    ├── Manuals/               # Organiser documentation
    └── ...                    # Competition materials and vendor tools
```

---

## 🔧 Calibration & Configuration

* `thefile.txt` — camera→arm coefficients for the **ground tier** (Z = −55 mm).
* `thefile2.txt` — camera→arm coefficients for the **2nd tier** (Z = 45 mm).
* Both files are factory-calibrated and stored in the robot's home directory; the
  diagnostics script verifies they exist and contain four coefficients.
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
