# NXROBO Spark: Autonomous Competition Robot (ROS Noetic)

[![Platform](https://img.shields.io/badge/Platform-Ubuntu%2020.04%20LTS-orange.svg)](https://ubuntu.com/)
[![ROS](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Hardware](https://img.shields.io/badge/Robot-NXROBO%20Spark%20%7C%20uArm%20Swift%20Pro-green.svg)](https://www.nxrobo.com/)
[![Status](https://img.shields.io/badge/Phase%200%20%26%201-100%25%20Verified-brightgreen.svg)](Commit/README.md)

Autonomous competition software suite for the **NXROBO Spark** mobile manipulator preparing for the **10-minute autonomous sorting challenge** («Полный автоном» / 自动任务赛 / "Treasure Hunt").

---

## 🎯 Competition Mission & Rules

The robot must operate completely autonomously inside a 3 × 3 m arena for 10 minutes without operator intervention:
1. **Rule #1 (Strict Priority):** The **Cake** situated on the 2nd central tier ($Z = 45$ mm) **MUST be transported to Zone A FIRST**. Failure to deliver the Cake first yields 0 points for the cake.
2. **Dynamic Sorting:** The remaining 9 items (3 Teddy Bears, 3 Clocks, 3 Bananas) must be classified at the central board and sorted into Zones B, C, and D.
3. **Multi-Tier Stacking:** Red corner target squares (0.45 × 0.45 m) award stacked cubes according to $N + 2^{N-1}$ (Tier 1 = 2 pts, Tier 2 = 4 pts, Tier 3 = 7 pts).

---

## 🏗️ System Architecture

```text
Host PC (Windows 10/11)
  ├── START_ROBOT.bat        --> 1-click launcher connecting to robot's interactive dashboard
  ├── upload.ps1             --> Instant SCP code synchronization to onboard computer
  └── connect.ps1            --> Quick SSH terminal launcher
          │
      (SSH / SCP)
          ▼
Robot Onboard NUC (Ubuntu 20.04 LTS + ROS Noetic)
  ├── codes/dashboard.sh     --> Unified terminal operator interface (options 1-7)
  ├── codes/start_system.sh  --> Background bringup (chassis, LiDAR, RealSense, uArm, vision)
  ├── codes/stop_system.sh   --> Safe shutdown and emergency pump/motor reset
  ├── codes/keypatched2v.py  --> Smooth teleoperation with self-bootstrapping ROS environment
  ├── codes/camera_stream.py --> Real-time HTTP MJPEG video stream on port 8080
  ├── codes/arm_ik.py        --> Analytical 4-DOF inverse kinematics & reachability validator
  └── codes/runtask.py       --> Competition Finite State Machine (perception -> grasp -> drop)
```

---

## ⚡ Key Implemented Improvements

* **Temporal Voting (10 frames):** Accumulates multi-frame object detections with majority voting, completely eliminating detection flickering.
* **Spatial Clustering (<45 px):** Bounding-box clustering and coordinate averaging to guarantee stable manipulator centering without jitter.
* **Reachability Filter:** Analytical Inverse Kinematics (`arm_ik.py`) checks physical joint limits and picks the closest achievable cube first.
* **Tier Calibrations:** Validated on physical hardware: $Z = -55$ mm (ground tier, `thefile.txt`) and $Z = 45$ mm (Cake 2nd tier, `thefile2.txt`).
* **Camera-Clear Tuck Pose:** The arm automatically retreats to $(50, 180, 160)$ (extreme left and elevated) post-release to clear camera and LiDAR field-of-view.
* **Zero Sourcing Dependency:** Python teleop scripts auto-inject ROS package paths into `sys.path` to permit direct execution in clean shells.

---

## 🚀 Quick Start Guide

### 1. Sync Code to Robot
```powershell
.\upload.ps1
```

### 2. Operator Dashboard (Windows 1-Click)
Run **`START_ROBOT.bat`** and choose from the interactive menu:
* `1` — Bring up all ROS subsystems (base, sensors, arm, perception).
* `2` — Test physical cube grasp in place.
* `3` — Test physical cube release.
* `4` — Smooth keyboard driving (`WASD`).
* `5` — View live mission execution logs (`less +F`).
* `6` — Stop all systems.

### 3. Web Camera Stream
Navigate to `http://<robot_ip>:8080/` in any web browser to view the RealSense D435 color feed in real time.

---

## 📂 Repository Contents

* [`Commit/`](Commit/README.md) — Self-contained verified release bundle with deployment notes.
* [`Commit/codes/`](Commit/codes/) — Clean production code executed on the robot.
* [`PLAN.md`](PLAN.md) — Comprehensive roadmap, rules, phase progression, and side issues.
* [`CONTEX.md`](CONTEX.md) — Hardware parameters, topic mappings, and network credentials.
* [`robot_startup.md`](robot_startup.md) — Pre-match operator checklist.
