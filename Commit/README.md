# 📦 Project Update Package: NXROBO Spark Autonomous Robot

> **Repository:** [https://github.com/Mavprime/NXROBO](https://github.com/Mavprime/NXROBO)  
> **Target Platform:** NXROBO Spark (Ubuntu 20.04 LTS + ROS Noetic)  
> **Status:** Phase 0 & Phase 1 are 100% complete and physically verified on hardware. Ready for Arena Mapping (Phase 2) and full autonomous FSM match runs (Phase 3).

---

## 📖 Overview & Competition Rules

This package contains the fully tested code, automation launchers, and architectural documentation for the **10-minute autonomous competition category** («Полный автоном» / 自动任务赛 / "Treasure Hunt").

### Key Competition Constraints:
* **Match Duration:** 10 minutes total autonomous run (no manual intervention allowed).
* **Target Items (10 cubes total):**
  * **1 × Cake** (located on the 2nd tier in the center) $\to$ **MUST BE DELIVERED FIRST to Zone A**. Delivering any other cube first disqualifies the Cake score.
  * **3 × Teddy Bear, 3 × Clock, 3 × Banana** (1st ground tier) $\to$ delivered to Zones B, C, D based on the dynamic classification board.
* **Scoring Matrix:**
  * Red Corner Target (0.45 × 0.45 m): Tier 1 = 2 pts, Tier 2 = 4 pts, Tier 3 = 7 pts (Formula: $N + 2^{N-1}$).
  * Standard Drop Area: Tier 1 = 1 pt, Tier 2 = 2 pts, Tier 3 = 4 pts.
  * Base departure: +1 pt.

---

## 📂 Folder Structure

```text
Commit/
├── codes/              # Production code deployed to the robot's onboard computer
│   ├── runtask.py          # Core competition FSM (Cake-first rule, Temporal Voting, IK filter)
│   ├── arm_ik.py           # Geometric kinematics solver for uArm Swift Pro
│   ├── keypatched2v.py     # Smooth keyboard teleop (WASD) with automatic ROS sys.path bootstrapping
│   ├── dashboard.sh        # Single-window interactive terminal dashboard (options 1-7)
│   ├── start_system.sh     # Background multi-launch startup with unbuffered logging (python3 -u)
│   ├── stop_system.sh      # Clean process shutdown & emergency pump/motor reset
│   ├── camera_stream.py    # Lightweight HTTP MJPEG video stream on port 8080
│   └── detect_ros.py       # Real-time YOLO object detection node publishing to /objects
│
├── launchers/          # Operator deployment tools for Windows workstation
│   ├── START_ROBOT.bat     # 1-click desktop launcher connecting to robot's interactive dashboard
│   ├── upload.ps1          # Automated code synchronization via SCP
│   ├── connect.ps1         # Quick SSH terminal launcher
│   └── .gitignore          # Repository hygiene (excludes SSH keys, large binaries, logs)
│
└── docs/               # Technical manuals and match runbooks
    ├── PLAN.md             # Master project plan, phase tracking (0-5), and side issue register
    ├── CONTEX.md           # Engineering handoff: credentials, topics, calibration parameters
    └── robot_startup.md    # Pre-match operator checklist and step-by-step procedures
```

---

## 🛠️ Detailed Component Descriptions

### 1. Onboard Robot Scripts (`codes/`)

* **`runtask.py`**  
  * **Role:** Master Finite State Machine (FSM) controlling perception, manipulation, and navigation.
  * **Key Features:**
    * **Cake-First Priority:** Enforces mandatory delivery of the 2nd-tier Cake ($Z = 45$ mm) to Zone A at `total_drop_times == 0`.
    * **Temporal Voting:** Buffers 10 consecutive camera frames to compute majority vote, eliminating label flickering.
    * **Spatial Bounding-Box Clustering:** Groups detections within 45 px and averages coordinates to eliminate arm positioning jitter.
    * **Class Alias Normalizer (`norm_id()`):** Resolves aliased competition IDs (`1` $\to$ Bear, `2/10/20` $\to$ Cake, `3/30` $\to$ Clock, `40` $\to$ Banana).
    * **Inverse Kinematics Reachability Filter (`ik_reachable`):** Evaluates physical reachability of all detected cubes and picks the closest achievable candidate.
    * **Calibrated Gripping Heights:** Calibrated against physical robot files: $Z = -55$ mm (1st floor, `thefile.txt`) and $Z = 45$ mm (2nd floor Cake, `thefile2.txt`).
    * **Camera-Clear Tuck Pose:** Retracts arm to $(50, 180, 160)$ after drop to keep camera and LiDAR FOV completely unobstructed.
  * **Deployment Target:** `/home/spark/codes/runtask.py` and ROS package path.

* **`arm_ik.py`**  
  * **Role:** Analytical Forward & Inverse Kinematics solver for the uArm Swift Pro 4-DOF robotic arm.
  * **Key Features:** Validates coordinates $(X, Y, Z)$ against physical joint limits and prevents arm stall or singular configurations.
  * **Deployment Target:** `/home/spark/codes/arm_ik.py`.

* **`keypatched2v.py`**  
  * **Role:** Smooth manual keyboard teleoperation node.
  * **Key Features:** Smooth acceleration/deceleration profiles to prevent wheel slippage. Includes a self-bootstrapping `sys.path` injection that resolves ROS Python packages automatically without requiring `source /opt/ros/noetic/setup.bash` first.
  * **Deployment Target:** `/home/spark/codes/keypatched2v.py`.

* **`dashboard.sh`**  
  * **Role:** Unified interactive command center for the operator.
  * **Key Features:** Provides numbered menu (1–7: Start stack, Test grasp, Test release, Teleop, Live logs, Stop, Exit). Shielded against accidental exit on `Ctrl+C` and uses `less +F` for real-time unbuffered log monitoring.
  * **Deployment Target:** `/home/spark/codes/dashboard.sh`.

* **`start_system.sh` & `stop_system.sh`**  
  * **Role:** Lifecycle orchestration scripts.
  * **Key Features:** Spawns `driver_bringup.launch`, `pro_control_nomoveit.launch`, and `auto_match.launch` in the background with unbuffered logging (`python3 -u`). Stop script kills active ROS nodes and cuts power to the suction pump.
  * **Deployment Target:** `/home/spark/codes/`.

* **`camera_stream.py`**  
  * **Role:** Standalone HTTP MJPEG video streamer.
  * **Key Features:** Streams RealSense D435 color feed over HTTP on port 8080 (`http://<robot_ip>:8080/`), allowing visual inspection via browser without heavy X11 GUI forwarding.
  * **Deployment Target:** `/home/spark/codes/camera_stream.py`.

---

### 2. Operator Workstation Launchers (`launchers/`)

* **`START_ROBOT.bat`**  
  * 1-click Windows batch launcher. Reads target IP from `robot_ip.txt` and establishes a pseudo-terminal SSH connection directly into `dashboard.sh`.
* **`upload.ps1`**  
  * PowerShell deployment script. Synchronizes all modified scripts from the PC to the robot via SCP and applies execution permissions (`chmod +x`).
* **`connect.ps1`**  
  * Fast interactive SSH wrapper for terminal debugging.
* **`.gitignore`**  
  * Protects private SSH credentials (`.ssh_key`), local IP configs (`robot_ip.txt`), and large diagnostic archives from entering version control.

---

## 🚀 Quick Start Guide

### 1. Deploying to Hardware
From the Windows host PC in PowerShell:
```powershell
.\upload.ps1
```

### 2. Launching Operator Dashboard
Double-click **`START_ROBOT.bat`** on your Windows desktop:
* Press **`1`** to initialize all robot hardware (mobile base, LiDAR, RealSense, uArm).
* Press **`2`** / **`3`** to verify physical cube grasp and release.
* Press **`4`** for smooth keyboard driving (`WASD`).
* Press **`5`** to monitor live mission logs.

### 3. Starting Autonomous Match
Once the arena is mapped and the robot is placed in the starting zone:
```bash
rostopic pub /task_start_flag std_msgs/String "1"
```
Emergency abort:
```bash
rostopic pub /task_start_flag std_msgs/String "0"
```
