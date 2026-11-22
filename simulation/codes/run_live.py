#!/usr/bin/env python3
"""Run the demo mission in the continuous 2D world and serve a live browser view.

    python3 run_live.py                 # 1x real time, http://127.0.0.1:8090/
    python3 run_live.py --speed 4       # 4x faster
    python3 run_live.py --no-viewer     # console only
    python3 run_live.py --no-realtime   # as fast as possible (tests)

The mission, gate and memory are the same code that runs against the discrete mock.
Only the platform changes: `SpatialPlatform` drives a continuous robot with a lidar and
camera detections instead of jumping between named points.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time

from fake_world import FakeWorld, RobotParams, SpatialPlatform
from live_view import ViewerServer
from mission_logic import MissionMemory, MissionRunner, WorldModel, load_mission


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--speed", type=float, default=1.0, help="real-time multiplier")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--linger", type=float, default=0.0,
                    help="seconds to keep serving after the mission (0 = Ctrl+C)")
    ap.add_argument("--start-delay", type=float, default=2.0,
                    help="seconds to wait after starting the viewer before the mission")
    ap.add_argument("--mission", default=os.path.join(here, "..", "config", "mission_demo.yaml"))
    ap.add_argument("--scenario", default=os.path.join(here, "..", "config", "arena_map.yaml"),
                    help="arena_map.yaml (canonical) or a generated scenario.json")
    ap.add_argument("--robot", default=os.path.join(here, "..", "config", "robot.yaml"),
                    help="measured chassis/arm parameters")
    ap.add_argument("--lin-speed", type=float, default=None, help="override m/s")
    ap.add_argument("--ang-speed", type=float, default=None, help="override rad/s")
    ap.add_argument("--pick-s", type=float, default=None)
    ap.add_argument("--place-s", type=float, default=None)
    ap.add_argument("--no-viewer", action="store_true")
    ap.add_argument("--no-realtime", action="store_true",
                    help="do not sleep; run the mission as fast as possible")
    ap.add_argument("--report", default=None, help="write the run report as JSON")
    args = ap.parse_args()

    if args.scenario.endswith((".yaml", ".yml")):
        import yaml

        with open(args.scenario) as f:
            scenario = yaml.safe_load(f)
    else:
        with open(args.scenario) as f:
            scenario = json.load(f)

    params = RobotParams.from_yaml(args.robot) if args.robot else RobotParams()
    if args.lin_speed is not None:
        params.max_linear_mps = args.lin_speed
    if args.ang_speed is not None:
        params.max_angular_rps = args.ang_speed
    if args.pick_s is not None:
        params.pick_s = args.pick_s
    if args.place_s is not None:
        params.place_s = args.place_s

    world = FakeWorld(
        scenario, seed=args.seed, speed=args.speed,
        real_time=not args.no_realtime, params=params,
    )
    world_model = WorldModel.from_scenario(scenario)
    stop = threading.Event()
    platform = SpatialPlatform(world, world_model, stop_event=stop)
    mission = load_mission(args.mission)
    memory = MissionMemory.from_inventory(scenario["bricks"], world_model.default_mapping)

    def on_step(record):
        elapsed = "   -  " if record.elapsed_s is None else "%5.1fs" % record.elapsed_s
        print("  [%d] %-10s %-10s %s %s" % (
            record.index + 1, record.kind, record.outcome.value, elapsed, record.reason))

    runner = MissionRunner(
        mission, platform, world_model, memory=memory,
        approachable=platform.approachable, on_step=on_step,
    )

    def state_fn():
        current = "-"
        if runner.current is not None:
            current = runner.current.kind
            params = runner.current.params or {}
            label = params.get("point") or params.get("class")
            if label:
                current += " " + str(label)
        steps = [{"kind": r.kind, "outcome": r.outcome.value, "reason": r.reason}
                 for r in runner.records]
        extra = {
            "state": runner.state,
            "current": current,
            "steps": steps,
            "mapping": dict(memory.mapping),
            "board": scenario.get("board_left_to_right"),
            "clock": platform.time_left(),
        }
        return world.snapshot(memory.mapping, extra)

    server = None
    if not args.no_viewer:
        server = ViewerServer(args.port, scenario, state_fn, stop)
        server.start()
        print("viewer: http://127.0.0.1:%d/   (speed %.1fx)" % (args.port, args.speed))

    if args.start_delay > 0:
        print("open the viewer now — starting in %.0fs (Ctrl+C to cancel)"
              % args.start_delay)
        time.sleep(args.start_delay)

    print("chassis: %.2f m/s  %.2f rad/s  (accel %.1f/%.1f, turn %.1f/%.1f)" % (
        params.max_linear_mps, params.max_angular_rps,
        params.lin_accel_mps2, params.lin_decel_mps2,
        params.ang_accel_rps2, params.ang_decel_rps2))
    print("arm    : pick %.1fs  place %.1fs" % (params.pick_s, params.place_s))
    print("mission: %s  seed %d" % (mission.name, args.seed))
    report = None
    try:
        report = runner.run()
    except KeyboardInterrupt:
        stop.set()
        print("\ninterrupted")

    if report is not None:
        score, detail = world.score(memory.mapping)
        print()
        print("state : %s %s" % (report.state, report.reason))
        print("score : %d  %s" % (score, detail))
        print("sim   : %.1f s  collisions=%d" % (world.t, world.collisions))
        print("board : %s  mapping: %s" % (
            scenario.get("board_left_to_right"), memory.mapping))
        if args.report:
            payload = report.to_dict()
            payload["sim"] = {"t": world.t, "collisions": world.collisions}
            payload["score"] = score
            payload["score_detail"] = detail
            payload["params"] = {
                "max_linear_mps": params.max_linear_mps,
                "max_angular_rps": params.max_angular_rps,
                "lin_accel_mps2": params.lin_accel_mps2,
                "lin_decel_mps2": params.lin_decel_mps2,
                "ang_accel_rps2": params.ang_accel_rps2,
                "ang_decel_rps2": params.ang_decel_rps2,
                "pick_s": params.pick_s,
                "place_s": params.place_s,
            }
            with open(args.report, "w") as f:
                json.dump(payload, f, indent=2)
            print("report: %s" % args.report)

    if server is not None:
        if args.linger > 0:
            print("serving final state for %.0fs..." % args.linger)
            server.linger(args.linger)
        else:
            print("viewer still live — Ctrl+C to exit")
            server.linger(0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
