#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""One full recognition cycle for the pre-field check.

scan -> report what YOLO saw -> grasp -> lift -> carry ~10 cm -> return ->
place -> report.  Exactly ONE cube per invocation, so the operator can swap
the cube between runs.  Every run appends a JSON line to
``~/logs/cube_check.jsonl``.

Prerequisites (must already be running):
    roslaunch spark_bringup driver_bringup.launch ...
    roslaunch swiftpro pro_control_nomoveit.launch
    detect_ros.py  (publishes /objects)

The autonomous FSM (runtask.py) must NOT run at the same time - it also
publishes /cmd_vel.  The script refuses to move while a node whose name
contains ``auto_match`` is alive (override with --force).

Usage:
    python3 -u ~/codes/cube_check.py --dry-run    # recognition only
    python3 -u ~/codes/cube_check.py              # full cycle
    python3 -u ~/codes/cube_check.py --no-drive   # grasp + place, no base
    python3 -u ~/codes/cube_check.py --cycles 3   # several cycles in a row

Exit codes: 0 ok | 3 no cube detected | 4 grasp failed | 5 FSM is running
"""

import argparse
import json
import math
import os
import sys
import time

import rospy
from vision_msgs.msg import Detection2DArray

import match_logic
import runtask as runtask_module
from runtask import CamAction, ArmAction, SwiftProInterface
try:                       # older runtask.py variants have no voting window
    from runtask import GRASP_FRAMES
except ImportError:
    GRASP_FRAMES = 4

NAMES = {1: "TEDDY(bear)", 2: "CAKE", 3: "CLOCK", 4: "BANANA"}
LOG_PATH = os.path.expanduser("~/logs/cube_check.jsonl")


def active_nodes():
    try:
        import rosnode
        return rosnode.get_node_names()
    except Exception:
        return []


def fmt_raw(frame):
    """Render one raw detection frame: 'id:NAME@score(cx,cy)'."""
    if not frame:
        return "(empty)"
    out = []
    for det in frame:
        try:
            raw_id, score, cx, cy = det[:4]
            name = NAMES.get(match_logic.norm_id(int(raw_id)), "?")
            out.append("%d:%s@%.2f(%d,%d)" % (int(raw_id), name, score, cx, cy))
        except Exception:
            continue
    return " ".join(out) if out else "(empty)"


def scan(cam, timeout=8.0, samples=None):
    wanted = int(samples) if samples else GRASP_FRAMES
    cubes = cam.detector(num_samples=wanted, timeout=timeout)
    with cam._lock:
        recent = list(cam._frames)
    raw = recent[-1][1] if recent else []
    frames_seen = len(recent)
    return cubes, raw, frames_seen


def measure():
    """Sample depth + arm coordinates over the image grid. NO motion at all.

    Answers "is the depth camera actually returning the cube top, and at what
    wrist height z?" - the grasp path silently falls back to a hard-coded
    -127 mm when depth is unusable, and for a tall cube that drives the arm
    into the table.
    """
    cam = CamAction()
    print("[MEASURE] depth sampling, the arm is NOT moving")
    print("  %-18s %-26s %s" % ("pixel", "depth (m, camera)", "wrist (mm)"))
    for v in range(150, 400, 50):
        for u in range(160, 500, 85):
            point = cam.point3d(u, v)
            if point is None:
                print("  u=%3d v=%3d      ---" % (u, v))
                continue
            wrist = cam.point_to_arm(point)
            if wrist is None:
                print("  u=%3d v=%3d      depth ok, transform failed" % (u, v))
                continue
            print("  u=%3d v=%3d      (%.3f, %.3f, %.3f)   (%.1f, %.1f, %.1f)"
                  % (u, v, point[0], point[1], point[2],
                     wrist[0], wrist[1], wrist[2]))
    return 0


def append_log(entry):
    try:
        os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
        with open(LOG_PATH, "a") as handle:
            handle.write(json.dumps(entry, ensure_ascii=True) + "\n")
    except Exception as exc:            # never lose the run because of logging
        print("[WARN] cannot write %s: %s" % (LOG_PATH, exc))


def probe(count=40, timeout=15.0, tuck=False):
    """Dump raw /objects frames WITHOUT any confidence filter.

    Answers "does the robot see the cube at all, and with what score?" -
    the production grasp path drops everything below CONF_THRESHOLD, so a
    correct-but-timid detection looks exactly like an empty scene there.

    ``tuck`` first parks the arm in the service pose: parked in ``home``
    (130, 0, 35) the arm sits in the middle of the camera view and hides
    the floor in front of the robot.
    """
    if tuck:
        print("[PROBE] tucking the arm into the service pose first")
        arm = ArmAction()
        arm.arm_grasp_ready(block=True)
        rospy.sleep(1.5)

    frames = []

    def callback(msg):
        detections = []
        for obj in msg.detections:
            try:
                res = obj.results[0]
                detections.append((int(res.id),
                                   float(getattr(res, "score", 0.0) or 0.0),
                                   float(obj.bbox.center.x),
                                   float(obj.bbox.center.y)))
            except Exception:
                continue
        frames.append(detections)

    sub = rospy.Subscriber("/objects", Detection2DArray, callback, queue_size=10)
    deadline = time.time() + timeout
    while len(frames) < count and time.time() < deadline and not rospy.is_shutdown():
        rospy.sleep(0.05)
    sub.unregister()

    print("[PROBE] %d frames, no confidence filter" % len(frames))
    best = {}
    for index, frame in enumerate(frames, 1):
        print("  frame %02d: %s" % (index, fmt_raw(frame)))
        for det in frame:
            try:
                cid = match_logic.norm_id(int(det[0]))
                best[cid] = max(best.get(cid, 0.0), float(det[1]))
            except Exception:
                continue
    for cid in sorted(best):
        print("  best %-12s = %.2f  (production threshold %.2f -> %s)"
              % (NAMES.get(cid, "?"), best[cid], match_logic.CONF_THRESHOLD,
                 "PASS" if best[cid] >= match_logic.CONF_THRESHOLD else "DROPPED"))
    return 0


def one_cycle(arm, cam, args, index):
    entry = {
        "ts": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "cycle": index,
        "mode": "dry" if args.dry_run else ("no-drive" if args.no_drive else "full"),
        "detected": [],
        "chosen": None,
        "frames": 0,
        "grasp_ok": None,
        "carry_ok": None,
        "place_ok": None,
        "result": None,
    }

    print("\n[CYCLE %d] -------------------------------------------------" % index)

    # The arm itself occludes the camera when parked in the middle of the
    # view, so tuck it away first (this is what run_task() also does).
    arm.arm_grasp_ready(block=True)
    rospy.sleep(1.2)

    print("[SCAN] waiting for %d fresh detection frames (min_hit_ratio=%.2f)"
          % (args.samples or GRASP_FRAMES, match_logic.MIN_HIT_RATIO))
    cubes, raw, frames_seen = scan(cam, samples=args.samples)
    entry["frames"] = frames_seen
    entry["samples"] = args.samples or GRASP_FRAMES
    entry["conf"] = getattr(runtask_module, "CONF_THRESHOLD",
                            match_logic.CONF_THRESHOLD)
    entry["min_hit_ratio"] = match_logic.MIN_HIT_RATIO
    print("[RAW ] last frame: %s" % fmt_raw(raw))

    if not cubes:
        print("[CUBE] none")
        entry["result"] = "NO_CUBE"
        return entry, 3

    for cube in cubes:
        cid = cube[0]
        name = NAMES.get(cid, "?")
        print("[CUBE] class=%d %-12s px=(%d,%d)" % (cid, name, cube[1][0], cube[1][1]))
        entry["detected"].append({"id": cid, "name": name,
                                  "u": cube[1][0], "v": cube[1][1]})

    if args.dry_run:
        entry["result"] = "DETECTED_ONLY"
        return entry, 0

    print("[GRASP] moving to the cube")
    target_id = arm.grasp()
    entry["chosen"] = target_id
    if not target_id:
        print("[GRASP] FAILED - nothing grasped")
        entry["grasp_ok"] = False
        entry["result"] = "GRASP_FAIL"
        return entry, 4
    entry["grasp_ok"] = True
    entry["chosen_name"] = NAMES.get(target_id, "?")
    print("[GRASP] holding class=%d %s" % (target_id, NAMES.get(target_id, "?")))

    if args.no_drive:
        entry["carry_ok"] = None
    else:
        # The front IR bumper reads True quite often (SI-2).  Driving into
        # whatever triggered it would be a real collision, so backward is the
        # safe default for the carry leg; the robot still returns to its start
        # pose afterwards.
        step = 0.10 if args.carry_dir == "forward" else -0.10
        print("[CARRY] %s 10 cm (%s)" % (args.carry_dir, "clear bumper" if step > 0 else "safe direction"))
        arm.drive_base(step)
        rospy.sleep(0.8)
        print("[CARRY] returning 10 cm to the start pose")
        arm.drive_base(-step)
        rospy.sleep(0.8)
        entry["carry_ok"] = True
        entry["carry_dir"] = args.carry_dir

    print("[PLACE] lowering and releasing")
    entry["place_ok"] = bool(arm.drop())
    entry["result"] = "OK"

    print("[REPORT] recognized: class=%d %s | grasp=%s carry=%s place=%s"
          % (target_id, NAMES.get(target_id, "?"),
             entry["grasp_ok"], entry["carry_ok"], entry["place_ok"]))
    return entry, 0


def main():
    parser = argparse.ArgumentParser(description="One recognition cycle")
    parser.add_argument("--dry-run", action="store_true", help="recognition only")
    parser.add_argument("--no-drive", action="store_true", help="no base motion")
    parser.add_argument("--carry-dir", choices=("back", "forward"), default="back",
                        help="direction of the 10 cm carry leg (default: back, "
                             "the front IR bumper is unreliable)")
    parser.add_argument("--force", action="store_true", help="ignore running FSM")
    parser.add_argument("--cycles", type=int, default=1, help="number of cycles")
    parser.add_argument("--conf", type=float, default=None,
                        help="override CONF_THRESHOLD for this run only")
    parser.add_argument("--samples", type=int, default=0,
                        help="frames in the voting window (production: %d)" % GRASP_FRAMES)
    parser.add_argument("--min-hit-ratio", type=float, default=None,
                        help="share of frames a cluster must appear in "
                             "(production: %.2f)" % match_logic.MIN_HIT_RATIO)
    parser.add_argument("--probe", type=int, default=0, metavar="N",
                        help="print N raw frames without the confidence filter")
    parser.add_argument("--tuck", action="store_true",
                        help="with --probe: park the arm in the service pose first")
    parser.add_argument("--measure", action="store_true",
                        help="sample the depth camera over the image grid (no motion)")
    parser.add_argument("--turn-deg", dest="turn_deg", type=float, default=0.0,
                        help="rotate the base by DEG degrees and exit")
    parser.add_argument("--min-z", type=float, default=None, metavar="MM",
                        help="hard safety floor: never command the wrist below MM")
    parser.add_argument("--raise-mm", dest="raise_mm", type=float, default=0.0,
                        metavar="MM",
                        help="shift every assumed height up by MM (touch/hover/lift "
                             "and the pixel->arm projection)")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("cube_check", anonymous=True)

    if args.measure:
        return measure()

    if args.probe:
        return probe(args.probe, tuck=args.tuck)

    if args.raise_mm:
        # The camera->arm transform on this robot is off (the whole floor maps
        # to z ~ -240 mm), so target_from_pixel() falls back to the hard-coded
        # -127 mm and the arm digs into anything taller than ~5 cm.  Shifting
        # expected_z keeps the pixel->arm projection consistent at the height
        # the operator actually sees, instead of faking only the final Z.
        rise = float(args.raise_mm)
        original_target_from_pixel = ArmAction.target_from_pixel

        def shifted_target_from_pixel(self, u, v, kb_x, kb_y,
                                      expected_z, z_lo, z_hi):
            return original_target_from_pixel(
                self, u, v, kb_x, kb_y,
                expected_z + rise, z_lo + rise, z_hi + rise)

        ArmAction.target_from_pixel = shifted_target_from_pixel
        print("[RAISE] all assumed heights shifted by %+.0f mm" % rise)

    if args.min_z is not None:
        # A real 15 cm cube sits well above the -127 mm the grasp path assumes
        # when depth is unusable.  This clamp is independent of the grasp math:
        # whatever it computes, the wrist physically cannot go below the floor.
        original_set_pose = SwiftProInterface.set_pose
        floor_z = float(args.min_z)

        def guarded_set_pose(self, x, y, z, delay=0.35):
            if z < floor_z:
                print("[SAFETY] z=%.1f is below the floor %.1f -> clamped"
                      % (z, floor_z))
                z = floor_z
            return original_set_pose(self, x, y, z, delay=delay)

        SwiftProInterface.set_pose = guarded_set_pose
        print("[SAFETY] wrist floor = %.1f mm" % floor_z)

    prod_conf = match_logic.CONF_THRESHOLD
    prod_hits = match_logic.MIN_HIT_RATIO
    if args.conf is not None:
        runtask_module.CONF_THRESHOLD = float(args.conf)
    if args.min_hit_ratio is not None:
        match_logic.MIN_HIT_RATIO = float(args.min_hit_ratio)

    print("[INIT] effective conf=%.2f min_hit_ratio=%.2f samples=%d "
          "(production: %.2f / %.2f / %d)"
          % (getattr(runtask_module, "CONF_THRESHOLD", prod_conf),
             match_logic.MIN_HIT_RATIO,
             args.samples or GRASP_FRAMES, prod_conf, prod_hits, GRASP_FRAMES))

    nodes = active_nodes()
    fsm = [n for n in nodes if "auto_match" in n or "runtask" in n]
    if fsm and not args.force:
        print("[ABORT] the autonomous FSM is running: %s" % ", ".join(fsm))
        print("        stop it first: rosnode kill %s" % fsm[0])
        return 5

    cam = CamAction()
    arm = ArmAction(cam=cam)
    print("[INIT] cube_check ready")

    if args.turn_deg:
        arm.turn_base(math.radians(args.turn_deg))
        rospy.sleep(0.5)
        print("[TURN] rotated %+.1f deg" % args.turn_deg)
        return 0

    code = 0
    try:
        for index in range(1, max(1, args.cycles) + 1):
            entry, code = one_cycle(arm, cam, args, index)
            append_log(entry)
            if code not in (0,):
                break
    except KeyboardInterrupt:
        print("\n[STOP] interrupted by operator")
        code = 130
    finally:
        # Never leave the pump energised or the arm in a travel pose.
        try:
            arm.interface.set_pump(False)
            rospy.sleep(0.2)
            arm.arm_home()
            rospy.sleep(0.5)
            print("[SAFE] pump off, arm home")
        except Exception as exc:
            print("[WARN] cleanup failed: %s" % exc)

    print("[DONE] exit=%d" % code)
    return code


if __name__ == "__main__":
    sys.exit(main())
