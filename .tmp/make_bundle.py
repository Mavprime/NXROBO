#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate auto/docs/project_bundle.txt for an external LLM review.

The bundle is deliberately plain UTF-8 text: it can be attached to any chat
without a folder upload.  It contains:
  * a read-first orientation + current state + critical caveats,
  * the repository manifest (every relevant file, role, hash, included/omitted),
  * the full source of the production core, tests and Windows launchers,
  * decoded calibration data,
  * an explicit list of what is NOT in the bundle and why.
"""

import base64
import datetime
import hashlib
from pathlib import Path

ROOT = Path(r"D:\NXROBO")
OUT = ROOT / "auto" / "docs" / "handoff" / "project_bundle.txt"
OUT.parent.mkdir(parents=True, exist_ok=True)


def read_text(path):
    return path.read_text(encoding="utf-8", errors="replace")


#: Never publish credentials in a file meant for an external assistant.
SAFE_REPLACEMENTS = {
    "simple2026": "<REDACTED-ROBOT-PASSWORD>",
    "123456": "<REDACTED-ROOT-PASSWORD>",
}


def safe_text(path):
    text = read_text(path)
    for secret, replacement in SAFE_REPLACEMENTS.items():
        text = text.replace(secret, replacement)
    return text


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def lines(path):
    return len(read_text(path).splitlines())


def git_revision():
    try:
        head = (ROOT / ".git" / "HEAD").read_text(encoding="utf-8").strip()
        if head.startswith("ref: "):
            ref = ROOT / ".git" / head[5:]
            if ref.exists():
                return ref.read_text(encoding="utf-8").strip()[:7]
            packed = (ROOT / ".git" / "packed-refs").read_text(encoding="utf-8")
            for line in packed.splitlines():
                if line.endswith(" " + head[5:]):
                    return line.split()[0][:7]
            return "unknown"
        return head[:7]
    except Exception:
        return "unknown"


# ---------------------------------------------------------------------------
# What is included in full
# ---------------------------------------------------------------------------
INCLUDE = [
    # Core runtime
    "auto/codes/match_logic.py",
    "auto/codes/runtask.py",
    "auto/codes/grasp_generic.py",
    "auto/codes/detect_ros.py",
    "auto/codes/robot_view.py",
    "auto/codes/calib3d.py",
    "auto/codes/arm_ik.py",
    "auto/codes/start_system.sh",
    "auto/codes/stop_system.sh",
    "auto/codes/dashboard.sh",
    "auto/codes/run_grasp_test.sh",
    "auto/codes/nav_bringup.sh.example",
    # Supporting runtime
    "auto/codes/patch_swiftpro.py",
    "auto/codes/pad_detect.py",
    "auto/codes/snap.py",
    "auto/codes/server.launch",
    "auto/codes/thefile.txt",
    "auto/codes/thefile2.txt",
    # Tests
    "auto/codes/tests/*.py",
    # Windows entry points
    "auto/launchers/START_ROBOT.bat",
    "auto/launchers/connect.ps1",
    "auto/launchers/upload.ps1",
    "run_grasp_test.bat",
    "run_scan_only.bat",
]

ROLES = {
    "match_logic.py": "Core (ROS-free): rules, scoring, board, clock, IK, scan window, clustering, cake gates",
    "runtask.py": "Core: competition FSM (ROS layer over match_logic) + arm/base/nav actions",
    "grasp_generic.py": "Core: field-test grasp (near/far scan, approach, pick, 5 s hold, return) + reusable API",
    "detect_ros.py": "Core: dual YOLO detector, de-duplication, anti-glare cake gates -> /objects",
    "robot_view.py": "Operator view: annotated JPEG of detector output + clustered grab targets",
    "calib3d.py": "DLT hand-eye calibration + --check + transform_3d.meta.json",
    "arm_ik.py": "Compatibility shim re-exporting IK from match_logic",
    "start_system.sh": "Idempotent bringup: base, lidar, camera, arm, stream, detector, nav hook, runtask, bag",
    "stop_system.sh": "Safe shutdown, bag close, pump/motor reset",
    "dashboard.sh": "Operator dashboard, options 0-9",
    "run_grasp_test.sh": "Driver check/bringup + field test / view / scan-only launcher",
    "nav_bringup.sh.example": "Template for map_server + amcl + move_base + mark_nav (Phase 2)",
    "pick_direct.py": "Reference direct floor grasp: DLT 3D + re-scan + camera check",
    "keypatched2v.py": "Smooth keyboard teleoperation with ROS path bootstrap",
    "test_all_systems.py": "Hardware + ROS + mission-readiness diagnostics",
    "camera_stream.py": "HTTP MJPEG camera stream on port 8080",
    "patch_swiftpro.py": "Patch uArm driver for dynamic /dev/ttyACM0..2; returns 10 if recompile needed",
    "pad_detect.py": "HSV suction-pad detector used by calibration",
    "snap.py": "One camera frame -> ~/current_view.jpg",
    "thefile.txt": "Legacy 2D calibration coefficients (floor tier)",
    "thefile2.txt": "Dedicated affine calibration for the tier-2 Cake",
    "START_ROBOT.bat": "Windows 1-click launcher: IP prompt -> SSH dashboard",
    "connect.ps1": "Windows PowerShell SSH/dashboard connector",
    "upload.ps1": "Windows PowerShell code sync + runtask restart",
    "run_grasp_test.bat": "Windows 1-click field test: full / view-only / scan-only + upload",
    "run_scan_only.bat": "Windows 1-click visibility scan without motion",
    "_bootstrap.py": "Test bootstrap: imports the production match_logic/arm_ik modules",
    "__init__.py": "Test package marker",
}


def expand(patterns):
    result = []
    for pattern in patterns:
        matches = sorted(ROOT.glob(pattern))
        if not matches:
            raise SystemExit("pattern matched nothing: %s" % pattern)
        result.extend(matches)
    # unique, keep order
    seen = set()
    unique = []
    for path in result:
        if path.is_file() and path not in seen:
            seen.add(path)
            unique.append(path)
    return unique


INCLUDED = expand(INCLUDE)
INCLUDED_SET = {p.resolve() for p in INCLUDED}


# ---------------------------------------------------------------------------
# Manifest of the whole repository (text files only)
# ---------------------------------------------------------------------------
MANIFEST_PATTERNS = [
    "auto/codes/*.py",
    "auto/codes/*.sh",
    "auto/codes/*.launch",
    "auto/codes/*.example",
    "auto/codes/*.txt",
    "auto/codes/*.dat",
    "auto/codes/tests/*.py",
    "auto/launchers/*",
    "auto/docs/*.md",
    "auto/docs/*.py",
    "auto/docs/*.ps1",
    "auto/docs/*.txt",
    "auto/docs/*.html",
    "simulation/*.md",
    "simulation/codes/*.py",
    "simulation/codes/tests/*.py",
    "simulation/config/*.yaml",
    "simulation/docs/*.md",
    "run_grasp_test.bat",
    "run_scan_only.bat",
    "README.md",
    ".gitignore",
]

SKIP_NAMES = {
    "project_bundle.txt",
    "robot_ip.txt",
    ".ssh_key",
    ".ssh_key.pub",
}


def manifest_role(path):
    name = path.name
    if name in ROLES:
        return ROLES[name]
    if "tests" in path.parts:
        return "Unit test (ROS-free)"
    if "simulation" in path.parts:
        return "Simulation track (separate workstream)"
    if path.suffix in (".bat", ".ps1"):
        return "Windows launcher / script"
    if "docs" in path.parts:
        return "Documentation"
    if path.suffix == ".dat":
        return "Binary calibration data (decoded separately)"
    return "Diagnostic / legacy / supporting script"


manifest = []
for pattern in MANIFEST_PATTERNS:
    for path in sorted(ROOT.glob(pattern)):
        if not path.is_file():
            continue
        if path.name in SKIP_NAMES or path.name.startswith(".ssh"):
            continue
        if "__pycache__" in path.parts:
            continue
        manifest.append(path)
manifest = sorted(set(manifest), key=lambda p: p.relative_to(ROOT).as_posix())


def status_mark(path):
    return "FULL" if path.resolve() in INCLUDED_SET else "manifest only"


def role_for(path):
    rel = path.relative_to(ROOT).as_posix()
    if rel in ROLES:
        return ROLES[rel]
    return manifest_role(path)


# ---------------------------------------------------------------------------
# Calibration decode
# ---------------------------------------------------------------------------
def decode_matrix():
    path = ROOT / "auto" / "codes" / "transform_3d.dat"
    try:
        import numpy as np
        data = np.load(str(path))
        return "numpy array shape=%s dtype=%s\n%s" % (
            data.shape, data.dtype, np.array2string(data, precision=6, suppress_small=False))
    except Exception as exc:
        raw = path.read_bytes()
        b64 = base64.b64encode(raw).decode("ascii")
        return "decoded with numpy failed: %s\nsize=%d bytes\nbase64:\n%s" % (
            exc, len(raw), b64)


# ---------------------------------------------------------------------------
# Build the document
# ---------------------------------------------------------------------------
now = datetime.datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S %z")
rev = git_revision()

out = []
add = out.append

add("=" * 100)
add("NXROBO SPARK — PROJECT BUNDLE FOR EXTERNAL REVIEW")
add("=" * 100)
add("")
add("Generated:    %s" % now)
add("Repository:   D:\\NXROBO")
add("Git revision: main @ %s + uncommitted working-tree changes (22.11.2026)" % rev)
add("Companion:    CONTEXT_TRANSFER.md — current project state, architecture, parameters, plan")
add("Optional:     PLAN.md — historical task plan/journal (read the 2026-11-22 addendum)")
add("")
add("This file lives in auto/docs/handoff/.  The old auto/docs/project_bundle.txt is from")
add("14.11.2026 and is stale (and currently locked by another process); do not attach it.")
add("")
add("WHY THIS FILE EXISTS")
add("-" * 100)
add("A folder upload is not readable by most chat assistants.  This is ONE plain UTF-8")
add("text file that contains the manifest, the production source code, the unit tests and")
add("the calibration data, so the project can be reviewed without opening the repository.")
add("")
add("RECOMMENDED READING ORDER")
add("-" * 100)
add("1. This header (sections 0-3): current status, caveats, repository map, manifest.")
add("2. Section 4: the production code in the order match_logic -> runtask -> grasp_generic")
add("   -> detect_ros -> robot_view -> bringup/launchers.")
add("3. Section 5: calibration data.")
add("4. Section 6: what is intentionally omitted.")
add("5. Then read CONTEXT_TRANSFER.md (state/parameters) and, if attached, PLAN.md.")
add("")
add("PROJECT IN ONE PAGE")
add("-" * 100)
add("* NXROBO Spark / Spark-H: wheeled mobile manipulator for the 10-minute fully")
add("  autonomous sorting challenge (自动任务赛 / «Полный автоном»).")
add("* Arena 3x3 m, four corner zones A-D, 10 cubes.  The tier-2 Cake MUST be delivered")
add("  to Zone A first; the other 9 cubes are sorted by the classification board.")
add("* Hardware: Intel NUC (Ubuntu 20.04, ROS Noetic), RealSense D435, YDLidar G6,")
add("  differential base, uArm Swift Pro 4-DOF + vacuum pump.")
add("* Software: match_logic.py (ROS-free rules/IK/vision logic) + runtask.py (FSM) +")
add("  detect_ros.py (two YOLO models -> /objects) + grasp_generic.py (field grasp test).")
add("* Current state: floor cubes and the tier-2 Cake are grasped in field tests; the")
add("  10-minute FSM has NOT yet run end-to-end because the physical arena/map is missing.")
add("* Tests: 141/141 PASS (ROS-free unittest suite, importing the production modules).")
add("")
add("CRITICAL CAVEATS — READ BEFORE TRUSTING THE CODE")
add("-" * 100)
add("1. runtask.grasp1() is STALE.  It uses cup_offset_x=30 mm, z_touch=40 mm and")
add("   z_lift=85 mm.  The verified 22.11 field pipeline uses cup_offset_x=125 mm,")
add("   z_touch=37 mm and z_lift=80 mm.  With offset 30 the FSM aims ~95 mm short and")
add("   will miss the Cake, breaking the cake-first rule.")
add("2. The hand-eye matrix on the robot is UNCERTAIN.  A 22.11 15:38 refit existed,")
add("   but run_grasp_test.bat at 19:20 uploaded the local 21.11 matrix; the meta file")
add("   still says 15:38, so the date in the logs is misleading.  Check md5/backups.")
add("3. Far-field scan/approach + approach_cost target choice (cube beyond ~35 cm)")
add("   are implemented and unit-tested (29 tests) but not validated on hardware, and")
add("   are not wired into runtask.grasp().")
add("4. Navigation is blocked: no arena, no real mark_map.pkl, no nav_bringup.sh, so")
add("   start_system.sh does not launch move_base.")
add("5. All 22.11 work is uncommitted (core changes + this handoff package).")
add("")
add("Credentials in the excerpts below are redacted.  The bundle contains no SSH keys and")
add("no robot_ip.txt; do not expect to log in with anything found in this file.")
add("")
add("=" * 100)
add("SECTION 1 — REPOSITORY TREE (relevant part)")
add("=" * 100)
add("")
add("D:\\NXROBO\\")
add("  README.md")
add("  run_grasp_test.bat              # 1-click field test (full/view/scan)")
add("  run_scan_only.bat               # 1-click visibility scan")
add("  auto/")
add("    codes/                        # production code for the NUC")
add("      match_logic.py              # ROS-free rules/scoring/board/clock/IK/scan")
add("      runtask.py                  # competition FSM (ROS layer)")
add("      grasp_generic.py            # field grasp test + reusable grasp API")
add("      detect_ros.py               # dual YOLO detector -> /objects")
add("      robot_view.py               # annotated detector view")
add("      calib3d.py                  # DLT hand-eye calibration")
add("      pick_direct.py              # reference direct floor grasp")
add("      start_system.sh             # idempotent bringup + bag")
add("      stop_system.sh              # safe shutdown")
add("      dashboard.sh                # operator dashboard 0-9")
add("      run_grasp_test.sh           # driver bringup + field test")
add("      nav_bringup.sh.example      # Phase-2 navigation template")
add("      tests/                      # 141 ROS-free unit tests")
add("      thefile.txt/thefile2.txt    # legacy / Cake affine calibration")
add("      transform_3d.dat            # 4x3 DLT matrix (numpy .npy renamed)")
add("    docs/                         # CONTEXT_TRANSFER.md, PLAN.md, README.md, manuals")
add("    launchers/                    # START_ROBOT.bat, connect.ps1, upload.ps1")
add("  simulation/                     # separate chassis mission simulation (Level 0A/0B)")
add("  semi/                           # M-Robots OS semi-automatic track")
add("  spark_humble_ws/                # previous workspace + Chinese vendor material")
add("  Manuals/                        # official competition PDFs")
add("")
add("=" * 100)
add("SECTION 2 — FILE MANIFEST")
add("=" * 100)
add("")
add("%-58s %7s %9s %12s  %-13s %s" % ("PATH", "LINES", "BYTES", "SHA256[0:12]", "IN BUNDLE", "ROLE"))
add("-" * 160)
for path in manifest:
    rel = path.relative_to(ROOT).as_posix()
    try:
        line_count = lines(path)
    except Exception:
        line_count = -1
    add("%-58s %7d %9d %12s  %-13s %s" % (
        rel, line_count, path.stat().st_size, sha256(path)[:12], status_mark(path), role_for(path)))
add("")
add("NOTE: 'manifest only' files are listed and hashed here but their text is not")
add("included below (legacy diagnostics, simulation track, manuals, images).  Ask for")
add("a specific file if it is needed for the review.")
add("")

add("=" * 100)
add("SECTION 3 — CRITICAL PARAMETERS AT A GLANCE")
add("=" * 100)
add("")
add("Confidence:        CONF_THRESHOLD=0.35 (grasp), PUBLISH_CONF=0.25 (detector)")
add("Cake gates:        CAKE_CONF_THRESHOLD=0.90 (raw id 2)")
add("                   CAKE_SITE_CONF_THRESHOLD=0.60 (raw id 20, the real Cake)")
add("Clustering:        GRASP_FRAMES=4, CLUSTER_DIST_PX=45, DEDUPE_DIST_PX=60")
add("Floor grasp:       z_center=-100, z_touch=-58, z_approach=70, z_lift=60, cup +20")
add("Cake grasp:        z_touch=37 (state 40 - press 3), z_approach=75, z_lift=80,")
add("                   cup_offset_x=125, clamp x<=348, affine from thefile2.txt")
add("Arm park:          (50, 180, 160) mm")
add("Stacking:          1st -55, 2nd +60, 3rd +160 mm; retract +80")
add("Scan window:       near field u=80..560, v=130..450; far window u=20..620, v=25..470")
add("Far approach:      step <= 0.20 m, MAX_APPROACH_ATTEMPTS=4, OUT_OF_REACH code 6")
add("Match clock:       540 s internal budget for a 600 s match")
add("")

add("=" * 100)
add("SECTION 4 — SOURCE CODE")
add("=" * 100)
add("")
add("Every file starts with its path, role, line count, byte size and SHA256.  The")
add("full content follows the dashed separator and ends at the next FILE banner.")
add("")
for index, path in enumerate(INCLUDED, 1):
    rel = path.relative_to(ROOT).as_posix()
    add("")
    add("=" * 100)
    add("FILE %02d/%02d: %s" % (index, len(INCLUDED), rel))
    add("ROLE:   %s" % role_for(path))
    add("SIZE:   %d lines, %d bytes, sha256=%s" % (lines(path), path.stat().st_size, sha256(path)))
    add("=" * 100)
    add("")
    add(safe_text(path).rstrip("\n"))
    add("")

add("=" * 100)
add("SECTION 5 — CALIBRATION DATA")
add("=" * 100)
add("")
add("transform_3d.dat is a numpy .npy file renamed to .dat (4x3 float64 DLT matrix).")
add("Decoded content:")
add("")
add(decode_matrix())
add("")
add("thefile.txt (legacy floor 2D coefficients):")
add("  " + safe_text(ROOT / "auto" / "codes" / "thefile.txt").strip())
add("")
add("thefile2.txt (tier-2 Cake affine coefficients):")
add("  " + safe_text(ROOT / "auto" / "codes" / "thefile2.txt").strip())
add("")
add("On the robot, transform_3d.meta.json records the fit date, residuals, px/mm, md5")
add("and the reference suction-pad pixel.  It is NOT in the repository; it exists only")
add("on the robot at ~/transform_3d.meta.json.")
add("")

add("=" * 100)
add("SECTION 6 — WHAT IS NOT IN THIS BUNDLE")
add("=" * 100)
add("")
add("Excluded on purpose:")
add("  * Secrets and local config: auto/docs/.ssh_key*, auto/launchers/robot_ip.txt")
add("  * Binary manuals: commands.docx, Spark_Commands.docx, rules.docx, Manuals/*.pdf")
add("  * Images/snapshots: *.jpg, *.png, snapshots/, auto/docs/scan_only_2026-11-22.jpg")
add("  * Python caches: __pycache__/, *.pyc")
add("  * Large legacy/diagnostic scripts not needed for the runtime review:")
add("      arm.py, armgem.py, armgrok.py, body.py, cake_smooth.py, cake_final.py,")
add("      cake_pick_place.py, cube_check.py, calib_pad.py, servo_grasp.py,")
add("      approach.py, arm_move.py, touch_blue.py, pick_hold_release.py,")
add("      keyboard_teleop.py, keyboard_a.py, key_fullbody.py, blue_*.py,")
add("      eval_*.py, test_reach_*.py, test_models.py, test_frames.py,")
add("      runtask_backup.py, detect_ros_backup.py")
add("  * The simulation track is listed in the manifest but its code is not included.")
add("  * Documentation files are separate attachments; this bundle does not duplicate")
add("    CONTEXT_TRANSFER.md / PLAN.md / README.md.")
add("")
add("If the reviewer needs one of the omitted files, generate a focused excerpt instead")
add("of attaching the whole repository.")
add("")
add("=" * 100)
add("END OF BUNDLE")
add("=" * 100)

OUT.write_text("\n".join(out) + "\n", encoding="utf-8")
print("WROTE:", OUT)
print("FILES INCLUDED:", len(INCLUDED))
print("MANIFEST ROWS:", len(manifest))
print("BYTES:", OUT.stat().st_size)
