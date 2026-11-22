# Handoff package — NXROBO Spark

**Prepared:** 22 November 2026, ~20:35 (Asia/Shanghai)  
**Repository:** `D:\NXROBO`  
**Current git:** `main` @ `6a9ad54` + uncommitted changes from 22.11  
**Purpose:** let an external assistant read the whole project as a few plain files,
without a folder upload.

---

## 1. What to attach

Attach these files (drag them into the chat):

1. **`CONTEXT_TRANSFER.md`** — this folder: current state of the project: what works,
   what is broken, architecture, ROS topics/parameters, how to run, next plan.
2. **`project_bundle.txt`** — this folder: complete manifest + production source code +
   tests + calibration data. It is self-contained and starts with a read-first header.
3. **`PLAN.md`** — optional: this folder: historical task plan and journal. Read the
   `2026-11-22` addendum at the end; the earlier test counts are historical.

The `CONTEXT_TRANSFER.md` and `PLAN.md` copies in this folder are **redacted**
(passwords replaced with `<REDACTED-...>`).  Do not attach the originals from
`../CONTEXT_TRANSFER.md` or `../PLAN.md`: they contain team credentials.

Do **not** attach `../project_bundle.txt` (dated 14.11.2026): it is stale and
currently locked by another process. The current bundle is the one in this folder.

If the assistant has a small context window, attach only files 1 and 2 first; add
`PLAN.md` only when it asks for the history/roadmap.

---

## 2. The project in one paragraph

NXROBO Spark / Spark-H is a wheeled mobile manipulator (Intel NUC, Ubuntu 20.04,
ROS Noetic) built for a 10-minute fully autonomous sorting challenge. The robot must
deliver the tier-2 Cake to Zone A first, then sort 3 Teddy Bears, 3 Clocks and
3 Bananas into zones B/C/D according to a classification board, optionally stacking
them for bonus points. Hardware: RealSense D435, YDLidar G6, differential base,
uArm Swift Pro with a vacuum pump.

The software is split into a ROS-free logic module (`match_logic.py`), a thin ROS
finite-state machine (`runtask.py`), a dual-YOLO detector (`detect_ros.py`) and a
field-test grasp script (`grasp_generic.py`).

---

## 3. Current status (22.11.2026)

| Area | Status |
|---|---|
| Floor cube grasp | ✅ verified in field tests (banana, 19:11) |
| Tier-2 Cake grasp | ✅ full field cycle completes (3 runs, 5 s hold, return); no programmatic hold check |
| Anti-glare cake gates | ✅ implemented: raw id 2 gate `0.90`, raw id 20 (`cake_site`) gate `0.60` |
| Far-field approach | 🟡 code + 29 unit tests; **not tested on hardware**, not wired into `runtask.grasp()` |
| Competition FSM | 🟡 code complete, **not run end-to-end** |
| Navigation / arena | 🔴 blocked: no real 3×3 m arena, no real `mark_map.pkl`, no `nav_bringup.sh` |
| Unit tests | ✅ **141/141 PASS** |
| Uncommitted work | ⚠️ all 22.11 core changes + this handoff are uncommitted |

### Critical caveats

1. **`runtask.grasp1()` is stale.** It uses `cup_offset_x=30 mm`, `z_touch=40`,
   `z_lift=85`; the verified field pipeline uses `125 mm`, `37`, `80`. With offset 30
   the FSM aims ~95 mm short of the Cake and breaks the cake-first rule.
2. **The hand-eye matrix on the robot is uncertain.** A 22.11 15:38 refit existed, but
   `run_grasp_test.bat` at 19:20 uploaded the local 21.11 matrix. The meta file still
   says 15:38, so log dates are misleading. Check md5/backups before trusting it.
3. **Far-field is not hardware-validated** and is only in `grasp_generic.py`.
4. **Everything from 22.11 is uncommitted.**
5. The current `README.md` still says "90 tests"; `PLAN.md` now has a `2026-11-22`
   addendum, but its earlier numbers are historical.  Treat `CONTEXT_TRANSFER.md`
   as the source of truth.

---

## 4. What is inside `project_bundle.txt`

- read-first orientation, current state and critical caveats;
- repository tree and a manifest of every relevant file (path, lines, bytes, SHA256,
  role, whether its text is included);
- full source of the production core:
  `match_logic.py`, `runtask.py`, `grasp_generic.py`, `detect_ros.py`,
  `robot_view.py`, `calib3d.py`, `arm_ik.py`, `start_system.sh`, `stop_system.sh`,
  `dashboard.sh`, `run_grasp_test.sh`, `nav_bringup.sh.example`;
- supporting runtime: `patch_swiftpro.py`, `pad_detect.py`, `snap.py`,
  `server.launch`, `thefile.txt`, `thefile2.txt`;
- all 9 test files (`141` tests); `pick_direct.py`, `keypatched2v.py`,
  `test_all_systems.py` and `camera_stream.py` are listed in the manifest but their
  text is deliberately not included (reference/diagnostic, not runtime);
- Windows launchers (`START_ROBOT.bat`, `connect.ps1`, `upload.ps1`,
  `run_grasp_test.bat`, `run_scan_only.bat`);
- decoded `transform_3d.dat` matrix and both calibration coefficient files;
- an explicit list of omitted legacy/binary/secret files.

It does **not** include: SSH keys, `robot_ip.txt`, `.docx`/`.pdf` manuals, images,
`__pycache__`, the simulation code, or documentation (those are separate files).

---

## 5. Suggested reading order for the assistant

1. `project_bundle.txt` → header (project, caveats) and manifest.
2. `CONTEXT_TRANSFER.md` → sections 1 (status), 6 (what is broken), 7 (architecture),
   8 (topics/parameters), 9 (how to run), 10 (plan).
3. `project_bundle.txt` → `match_logic.py` → `runtask.py` → `grasp_generic.py` →
   `detect_ros.py` → tests.
4. `PLAN.md` → only if the history/roadmap is needed.
