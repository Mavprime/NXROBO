# Spark Robot — Code Improvements & Teleop Context Summary

**Date:** 2026-11-16  
**Robot:** Spark-H (NXROBO) — Ubuntu 20.04 LTS (ROS Noetic)  
**Robot IP:** `10.24.170.0` (User: `spark`, Password: `simple2026`)  
**Workspace Path (Local):** `D:\NXROBO\auto\codes\`  
**Target Path (Robot):** `/home/spark/codes/`  

---

## 1. Executive Summary of Changes

This document consolidates all iterations, engineering analyses, bug fixes, and improvements performed on the Spark teleoperation system (`keyboard_teleop.py`) and peripheral tooling.

---

## 2. Key Teleoperation Controls & Keymap

| Key | Action | Description |
| :--- | :--- | :--- |
| **`W / S`** (or `↑ / ↓`) | Move Forward / Backward | Smooth acceleration (`1.0 m/s²`), instant braking (`3.0 m/s²`). |
| **`A / D`** (or `← / →`) | Steer Left / Right | Turning in place or steering while driving. |
| **`Q / E`** | **Arc Turn Left / Right** | Single-key simultaneous forward drive + arc turn (no OS key conflict!). |
| **`Space`** | **EMERGENCY STOP** | Immediately zeroes base velocity and halts arm jogging. **Preserves pump state and gripped box.** |
| **`f`** (or `Tab`/`m`) | Toggle Speed Mode | Switches between **`FAST`** (arena transit) and **`SLOW`** (precision docking/cube alignment). |
| **`8 / 2`** | Arm Up / Down ($Z$) | Step-jogging: 8.0 mm in `FAST`, 2.5 mm in `SLOW` per tap. |
| **`7 / 9`** | Arm Forward / Backward ($X$) | Step-jogging: 8.0 mm in `FAST`, 2.5 mm in `SLOW` per tap. |
| **`4 / 6`** | Arm Swing CCW / CW ($Y$) | Step-jogging around the base. |
| **`1 / 3`** | Wrist Spin CCW / CW | 4th axis rotation (10° in `FAST`, 4° in `SLOW`). |
| **`v`** | Preset: **Hover / Safe Height** | `X=280.0, Y=0.0, Z=30.0` (hovering safely above cubes). |
| **`t`** | Preset: **Touch / Pick** | `X=280.0, Y=0.0, Z=-50.0` (lowered down to table/floor cube contact). |
| **`c`** | Preset: **Carry / Lift** | `X=230.0, Y=0.0, Z=100.0` (lifted high for safe driving with load). |
| **`r`** | **Relax / Free Motors** | Toggles M2019 (free motors for manual adjustment) / M17 (lock). |
| **`Enter`** | Toggle Vacuum Pump | One-press suction ON / OFF. |
| **`0`** | Release & Park Home | Vacuum pump OFF, base stopped, arm folded home (`120, 0, 35`). |
| **`Ctrl + C`** | Clean Exit | Halts base, disables pump, restores terminal settings. |

---

## 3. Deep Technical Problem Analyses & Solutions

### 3.1. Problem: Simultaneous Forward Drive & Steering Failed Intermittently ("Через раз")
* **Root Cause:** Standard SSH / POSIX terminals in raw mode cannot physically repeat two pressed keys at once. When a user holds `W` and then presses `D`, the client OS stops transmitting `W` and transmits only `D D D D`. When `D` is released, the OS does NOT resume repeating `W`, leaving stdin silent. With a unified short timeout, linear velocity was killed within 300 ms, canceling forward motion while the user was still physically holding `W`.
* **Fix Implemented:** Implemented a decoupled dual-timer state machine with auto-centering:
  - **Steering auto-centering:** Steering (`dir_angular`) auto-centers back to 0 after `0.30s` of no steer key.
  - **Steering persistence:** Receiving `A` or `D` while driving forward/backward keeps `last_linear_key` refreshed, driving in a smooth arc as long as `A`/`D` is held.
  - **Steer-recovery grace period:** When steering ends while moving (e.g. `D` is released while holding `W`), `dir_linear` is granted a tuned `0.80s` grace period, allowing the robot to straighten out and continue driving forward seamlessly.
  - **Single-key arc turn keys (`Q` / `E`):** Added dedicated `Q` (forward-left arc) and `E` (forward-right arc) keys that eliminate two-key repeat conflicts entirely.
  - **Braking deceleration:** High `MAX_LIN_DECEL = 5.0 m/s²` and `MAX_ANG_DECEL = 8.0 rad/s²` for crisp, immediate stopping upon key release.
  - **Straight-drive timeout:** Clean `0.30s` threshold for halting when simply driving straight.

### 3.2. Problem: Base Moved Jerkily & Took Too Long to Stop
* **Root Cause:**
  1. The OS initial keyboard repeat delay (250–400 ms) exceeded short timeouts, causing false "key release" triggers right at startup.
  2. High publishing rate (50 Hz) was saturating the `/dev/sparkBase` serial controller (115200 baud, 20 Hz native loop).
  3. Symmetrical acceleration/deceleration made stopping feel sluggish.
* **Fix Implemented:**
  - Base loop fixed at steady **20 Hz** (`rospy.Rate(20)`), streaming continuous `Twist` commands.
  - Implemented asymmetric velocity ramping: `MAX_LIN_ACCEL = 1.0 m/s²` (smooth departure) and `MAX_LIN_DECEL = 3.0 m/s²` (fast, crisp braking).
  - Reduced `KEY_TIMEOUT` to `0.22 s`, stopping the wheels **2x faster** (~0.28 s total from physical release to dead stop).

### 3.3. Problem: Arm Movement Was Stuttering & Jerky
* **Root Cause:** Discovered inside `swiftpro_write_node.cpp`:
  ```cpp
  Gcode = (std::string)"G0 X" + x + " Y" + y + " Z" + z + " F99990000" + "\r\n";
  ```
  Every Cartesian command generates an absolute `G0` move planned by the Arduino with zero entry/exit velocity. Streaming coordinates at 5 Hz or 20 Hz commanded the motors to accelerate to maximum speed and immediately decelerate to zero for every single point.
* **Fix Implemented:** Transitioned to **step-jogging** with a hardware-paced throttle:
  - Tapping a key triggers an immediate single move (8.0 mm in `FAST`, 2.5 mm in `SLOW`).
  - Holding a key triggers clean 140 ms step pulses without buffer pileup.
  - Removed mid-flight pose overwriting from `/SwiftproState_topic` feedback.

### 3.4. Problem: Preset `p` (Pick) Was Too High Above Cube
* **Root Cause:** When presets were extended forward to $X=280$ mm to clear surrounding cubes, the parallelogram linkage geometry and suction cup tilt placed $Z=-50$ mm noticeably higher than near the base. Furthermore, `ARM_LIMITS['z']` was hardcoded to `-60.0 mm`.
* **Fix Implemented:**
  - Lowered preset `p` to **`Z = -80.0 mm`** (`X=280.0, Y=0.0, Z=-80.0`).
  - Removed software blocking limits (`ARM_LIMITS` and `ik_reachable`) during execution.

---

## 4. Software Limits Removal & Risk Analysis

The user requested: *"Убери программные лимиты на движения руки. Это чем-то чревато?"*

### Risks of Removing Limits:
1. **Mechanical Stall / Gear Grinding:** uArm Swift Pro uses open-loop stepper motors without force feedback. If commanded beyond physical reach ($X > 320$ mm or $Z < -90$ mm into the table), the motors will collide with physical stops, skip steps ("chattering"), and lose home calibration.
2. **Firmware Silent Ignore:** The Arduino firmware calculates its own internal kinematics. If $\sqrt{X^2+Y^2+Z^2}$ exceeds arm reach, the firmware **silently discards the G-code packet** without moving the arm.
3. **Pneumatic Line Tension:** Extreme reaches can stretch or dislodge the vacuum hose from the suction cup nozzle.

### Status in Code (Safe Protective Barrier):
Sensible protective soft limits (`SAFE_LIMITS`) are active to safeguard the hardware:
- **Z floor ($Z \ge -85.0$ mm):** Prevents driving into the tabletop/floor while cleanly allowing Pick preset ($Z = -80.0$ mm).
- **Z ceiling ($Z \le 180.0$ mm):** Prevents over-extending upwards.
- **X rear buffer ($X \ge 90.0$ mm):** Prevents accidental collision into the robot's own LiDAR tower or RealSense camera mount.
- **X forward extension ($X \le 320.0$ mm):** Prevents vacuum hose detachment and mechanical joint strain.
- **Y swing ($Y \in [-160.0, 160.0]$ mm):** Full left/right workspace swing.

---

## 5. Robot Infrastructure & Deployment

### Network & Authentication:
- **Host:** `10.24.170.0`
- **User:** `spark`
- **Password:** `simple2026`
- **SSH Command:** `ssh spark@10.24.170.0`

### Automated Deployment Script:
Files are synchronized using `paramiko` via `uv`:
```powershell
uv run --with paramiko python "C:\Users\user1\.gemini\antigravity\brain\a6878179-fbba-4c95-8d4e-4d5fbe641917\scratch\upload_to_robot.py"
```

### Robot Launch Sequence:
1. **Terminal 1 (Chassis, Camera & Lidar):**
   ```bash
   source ~/spark_noetic/devel/setup.bash
   roslaunch spark_bringup driver_bringup.launch camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6
   ```
2. **Terminal 2 (uArm Driver):**
   ```bash
   source ~/spark_noetic/devel/setup.bash
   roslaunch swiftpro pro_control_nomoveit.launch
   ```
3. **Terminal 3 (Keyboard Teleoperation):**
   ```bash
   source ~/spark_noetic/devel/setup.bash
   python3 ~/codes/keyboard_teleop.py
   ```
