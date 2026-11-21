#!/usr/bin/env python3
"""Run mission_logic against the existing discrete MockWorld.

This is the bridge that proves the mission executor and the agreement gate work with
the simulator already in this directory. The spatial platform (Level 0B) and the ROS
node will implement the same small protocol, so the mission code stays unchanged.

Usage:
    python3 mock_adapter.py                    # mission_demo.yaml, seed 3
    python3 mock_adapter.py --seed 7 --json    # dump the full report as JSON
"""

from __future__ import annotations

import argparse
import json
import os

from mission_logic import (
    Detection,
    MissionMemory,
    MissionRunner,
    PickResult,
    PlaceResult,
    WorldModel,
    canonical_class,
    load_mission,
)
from mock_world import Config, MockWorld


class MockWorldPlatform:
    """The mission_layer protocol on top of MockWorld's discrete interface."""

    def __init__(self, world: MockWorld, confidence: float = 0.95) -> None:
        self.w = world
        self.confidence = confidence
        # The scenario spells the bear "doll"; map canonical -> scenario spelling.
        self._raw = {}
        for brick in world.sc["bricks"]:
            self._raw.setdefault(canonical_class(brick["type"]), brick["type"])

    # navigation ------------------------------------------------------------
    def goto(self, point: str) -> bool:
        return self.w.goto_local(point)

    # perception ------------------------------------------------------------
    def read_board(self):
        return [canonical_class(c) for c in self.w.read_board()]

    def scan_window(self, n: int):
        # The discrete model has no frames: one detection pass replicated n times.
        dets = [Detection(t, self.confidence) for t in self.w.detector()]
        return [list(dets) for _ in range(n)]

    # arm -------------------------------------------------------------------
    def pick(self, cls: str) -> PickResult:
        raw = self._raw.get(cls, cls)
        ok = self.w.pick(raw)
        return PickResult(
            ok=bool(ok),
            confirmed_class=cls if ok else "",
            reason="" if ok else "grasp failed",
        )

    def place(self, level: int) -> PlaceResult:
        ok = self.w.drop()
        return PlaceResult(ok=bool(ok), reason="" if ok else "drop failed")

    # clock / stop ----------------------------------------------------------
    def time_left(self) -> float:
        return self.w.time_left()

    def now(self) -> float:
        return self.w.t

    def stop_requested(self) -> bool:
        return False

    # No recovery ladder on the discrete model: goto_local has no costmaps and
    # backing up would not change its outcome.


def run(mission_path: str, scenario_path: str, seed: int) -> tuple:
    with open(scenario_path) as f:
        scenario = json.load(f)
    world = WorldModel.from_scenario(scenario)
    mock = MockWorld(scenario, Config(), seed=seed)
    platform = MockWorldPlatform(mock)
    mission = load_mission(mission_path)
    memory = MissionMemory.from_inventory(world.bricks, world.default_mapping)
    report = MissionRunner(mission, platform, world, memory=memory).run()
    return report, mock


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mission", default=os.path.join(here, "..", "config", "mission_demo.yaml"))
    ap.add_argument("--scenario", default=os.path.join(here, "scenario.json"))
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--json", action="store_true", help="print the full report as JSON")
    args = ap.parse_args()

    report, mock = run(args.mission, args.scenario, args.seed)
    score, detail = mock.score(overtime_s=max(0.0, mock.t - 600))

    if args.json:
        print(json.dumps(report.to_dict(), indent=2))
        return

    print("mission : %s" % os.path.basename(args.mission))
    print("state   : %s %s" % (report.state, ("(%s)" % report.reason) if report.reason else ""))
    print("clock   : %.0fs left" % report.clock_remaining)
    print("board   : %s" % mock.sc["board_left_to_right"])
    print("mapping : %s" % report.mapping)
    print()
    for r in report.records:
        print("  %d. %-10s %-10s %s" % (r.index + 1, r.kind, r.outcome.value, r.reason))
    print()
    print("score   : %d  (mock time %.0fs)" % (score, mock.t))
    print("detail  : %s" % (detail,))


if __name__ == "__main__":
    main()
