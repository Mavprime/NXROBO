#!/usr/bin/env python3
"""Unit tests for mission_logic.py — no ROS, no simulator, plain unittest.

Run from simulation/codes:

    python3 -m unittest discover -s tests -v
"""

import json
import os
import sys
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CODE_DIR = os.path.dirname(HERE)
sys.path.insert(0, CODE_DIR)

from mission_logic import (  # noqa: E402
    Detection,
    Gate,
    GateAction,
    Mission,
    MissionMemory,
    MissionRunner,
    Outcome,
    PickResult,
    PlaceResult,
    Step,
    Thresholds,
    WorldModel,
    canonical_class,
    load_mission,
)

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

SCENARIO = {
    "points": {
        "Classification_area": [-0.90, 0.0],
        "Sorting_N": [0.0, 0.55],
        "Sorting_S": [0.0, -0.55],
        "Sorting_E": [0.55, 0.0],
        "Sorting_W": [-0.55, 0.0],
        "Collection_A": [-1.275, 1.275],
        "Collection_B": [1.275, 1.275],
        "Collection_C": [1.275, -1.275],
        "Collection_D": [-1.275, -1.275],
    },
    "zones": {
        "A": {"transport_center": [-1.0, 1.0], "collection_center": [-1.275, 1.275]},
        "B": {"transport_center": [1.0, 1.0], "collection_center": [1.275, 1.275]},
        "C": {"transport_center": [1.0, -1.0], "collection_center": [1.275, -1.275]},
        "D": {"transport_center": [-1.0, -1.0], "collection_center": [-1.275, -1.275]},
    },
    "bricks": (
        [{"name": "cake", "type": "cake"}]
        + [{"name": "doll_%d" % i, "type": "doll"} for i in range(3)]
        + [{"name": "clock_%d" % i, "type": "clock"} for i in range(3)]
        + [{"name": "banana_%d" % i, "type": "banana"} for i in range(3)]
    ),
    "board_left_to_right": ["doll", "clock", "banana"],
    "board_zones": ["B", "C", "D"],
    "start_pose": {"x": -1.275, "y": 1.275, "yaw": -1.5708},
}

WORLD = WorldModel.from_scenario(SCENARIO)
ZONE_A = WORLD.zone_center("A")
STATION_N = WORLD.station_pose("N")


def det(cls, conf=0.9, x=None, y=None):
    return Detection(cls, conf, x, y)


def frames(detection, n=5):
    return [[detection] for _ in range(n)]


def make_memory():
    return MissionMemory.from_inventory(SCENARIO["bricks"], WORLD.default_mapping)


def mission(*steps, budget=540):
    return Mission("test", budget, [Step(kind, params) for kind, params in steps])


class FakePlatform:
    """Scriptable platform: queues of results, call recording, no ROS."""

    def __init__(
        self,
        windows=None,
        goto=None,
        picks=None,
        places=None,
        board=None,
        time_left=500.0,
        stop=False,
        precheck_ok=True,
    ):
        self.windows = list(windows or [])
        self.goto_results = list(goto or [])
        self.picks = list(picks or [])
        self.places = list(places or [])
        self.board = board
        self._time_left = time_left
        self.stop = stop
        self.precheck_ok = precheck_ok

        self.goto_calls = []
        self.pick_calls = []
        self.place_calls = []
        self.scan_calls = 0
        self.recovery = []
        self.reframes = []
        self.waits = []

    # protocol --------------------------------------------------------------
    def goto(self, point):
        self.goto_calls.append(point)
        return self.goto_results.pop(0) if self.goto_results else True

    def scan_window(self, n):
        self.scan_calls += 1
        if self.windows:
            return self.windows.pop(0)
        return [[] for _ in range(n)]

    def pick(self, cls):
        self.pick_calls.append(cls)
        return self.picks.pop(0) if self.picks else PickResult(True, confirmed_class=cls)

    def place(self, level):
        self.place_calls.append(level)
        return self.places.pop(0) if self.places else PlaceResult(True)

    def read_board(self):
        return self.board

    def time_left(self):
        return self._time_left

    def stop_requested(self):
        return self.stop

    def precheck(self):
        return (True, "") if self.precheck_ok else (False, "no calibration")

    def wait(self, seconds):
        self.waits.append(seconds)

    # recovery --------------------------------------------------------------
    def clear_costmaps(self):
        self.recovery.append("clear_costmaps")

    def rotate_scan(self):
        self.recovery.append("rotate_scan")

    def backup(self):
        self.recovery.append("backup")

    def reframe(self, station):
        self.reframes.append(station)


# ---------------------------------------------------------------------------
# Class normalisation
# ---------------------------------------------------------------------------


class ClassNormalisationTests(unittest.TestCase):
    def test_aliases(self):
        self.assertEqual(canonical_class("doll"), "bear")
        self.assertEqual(canonical_class("1"), "bear")
        self.assertEqual(canonical_class("10"), "bear")
        self.assertEqual(canonical_class("20"), "cake")
        self.assertEqual(canonical_class("30"), "clock")
        self.assertEqual(canonical_class("40"), "banana")
        self.assertEqual(canonical_class("BANANA"), "banana")
        self.assertEqual(canonical_class(None), "")

    def test_detection_is_canonical(self):
        self.assertEqual(det("doll").cls, "bear")
        self.assertEqual(det("40").cls, "banana")


# ---------------------------------------------------------------------------
# Gate: pick
# ---------------------------------------------------------------------------


class GatePickTests(unittest.TestCase):
    def setUp(self):
        self.gate = Gate()
        self.mem = make_memory()

    def gate_pick(self, window, cls="cake", mem=None, time_left=500.0, approachable=None):
        return self.gate.pick(
            window, cls, STATION_N, mem or self.mem,
            time_left=time_left, approachable=approachable,
        )

    def test_g1_two_votes_fails(self):
        window = frames(det("cake", x=0.0, y=0.55), n=2) + [[] for _ in range(3)]
        verdict = self.gate_pick(window)
        self.assertFalse(verdict.passed)
        self.assertEqual(verdict.action, GateAction.RESCAN)
        self.assertIn("G1", verdict.reason)

    def test_g1_three_votes_passes(self):
        verdict = self.gate_pick(frames(det("cake", x=0.0, y=0.55)))
        self.assertTrue(verdict.passed, verdict.reason)

    def test_g1_low_confidence_does_not_count(self):
        window = frames(det("cake", conf=0.20, x=0.0, y=0.55))
        verdict = self.gate_pick(window)
        self.assertFalse(verdict.passed)
        self.assertEqual(verdict.action, GateAction.RESCAN)

    def test_g1_other_class_does_not_count(self):
        window = frames(det("clock", x=0.0, y=0.55))
        verdict = self.gate_pick(window)
        self.assertFalse(verdict.passed)
        self.assertEqual(verdict.action, GateAction.RESCAN)

    def test_g1_prefers_target_pose(self):
        far = det("bear", x=-0.25, y=0.55)
        near = det("bear", x=0.05, y=0.55)
        window = [[far, near] for _ in range(5)]
        verdict = self.gate.pick(
            window, "bear", STATION_N, self.mem, target_pose=(0.0, 0.55)
        )
        self.assertTrue(verdict.passed, verdict.reason)
        self.assertAlmostEqual(verdict.evidence.x, 0.05)
        self.assertAlmostEqual(verdict.evidence.y, 0.55)

    def test_g1_target_prefers_across_frames(self):
        near = det("bear", conf=0.45, x=0.05, y=0.55)
        far = det("bear", conf=0.95, x=-0.25, y=0.55)
        window = [[near]] + [[far] for _ in range(4)]
        verdict = self.gate.pick(
            window, "bear", STATION_N, self.mem, target_pose=(0.0, 0.55)
        )
        self.assertTrue(verdict.passed, verdict.reason)
        self.assertAlmostEqual(verdict.evidence.x, 0.05)

    def test_g2_pose_mismatch_revises_belief(self):
        verdict = self.gate_pick(frames(det("cake", x=0.0, y=0.0)))
        self.assertFalse(verdict.passed)
        self.assertEqual(verdict.action, GateAction.RESCAN)
        self.assertIn("G2", verdict.reason)
        self.assertAlmostEqual(verdict.revise[0], 0.0)
        self.assertAlmostEqual(verdict.revise[1], 0.0)

    def test_g2_pose_within_radius_passes(self):
        verdict = self.gate_pick(frames(det("cake", x=0.0, y=0.50)))
        self.assertTrue(verdict.passed, verdict.reason)

    def test_g3_carrying_skips(self):
        self.mem.mark_picked("bear")
        verdict = self.gate_pick(frames(det("bear", x=0.0, y=0.55)), cls="bear")
        self.assertFalse(verdict.passed)
        self.assertEqual(verdict.action, GateAction.SKIP)

    def test_g3_memory_empty_is_conflict(self):
        self.mem.mark_picked("cake")
        self.mem.mark_placed(zone="A", verified=True)
        verdict = self.gate_pick(frames(det("cake", x=0.0, y=0.55)))
        self.assertFalse(verdict.passed)
        self.assertEqual(verdict.action, GateAction.CONFLICT)

    def test_g4_unapproachable_reframes(self):
        verdict = self.gate_pick(
            frames(det("cake", x=0.0, y=0.55)), approachable=lambda x, y: False
        )
        self.assertFalse(verdict.passed)
        self.assertEqual(verdict.action, GateAction.REFRAME)

    def test_g5_not_enough_time_replans(self):
        verdict = self.gate_pick(frames(det("cake", x=0.0, y=0.55)), time_left=5.0)
        self.assertFalse(verdict.passed)
        self.assertEqual(verdict.action, GateAction.REPLAN)


# ---------------------------------------------------------------------------
# Gate: verify
# ---------------------------------------------------------------------------


class GateVerifyTests(unittest.TestCase):
    def setUp(self):
        self.gate = Gate()

    def test_verified_in_zone(self):
        window = frames(det("cake", x=ZONE_A[0], y=ZONE_A[1]))
        status, evidence = self.gate.verify(window, "cake", ZONE_A, 0.45)
        self.assertEqual(status, "verified")
        self.assertIsNotNone(evidence)

    def test_misplaced_outside_zone(self):
        window = frames(det("cake", x=0.0, y=0.0))
        status, _ = self.gate.verify(window, "cake", ZONE_A, 0.45)
        self.assertEqual(status, "misplaced")

    def test_unverified_without_pose(self):
        window = frames(det("cake"))
        status, _ = self.gate.verify(window, "cake", ZONE_A, 0.45)
        self.assertEqual(status, "unverified")

    def test_unverified_too_few_votes(self):
        window = frames(det("cake", x=ZONE_A[0], y=ZONE_A[1]), n=2)
        status, _ = self.gate.verify(window, "cake", ZONE_A, 0.45)
        self.assertEqual(status, "unverified")


# ---------------------------------------------------------------------------
# Memory
# ---------------------------------------------------------------------------


class MemoryTests(unittest.TestCase):
    def setUp(self):
        self.mem = make_memory()

    def test_inventory_counts(self):
        self.assertEqual(self.mem.available("bear"), 3)
        self.assertEqual(self.mem.available("clock"), 3)
        self.assertEqual(self.mem.available("banana"), 3)
        self.assertEqual(self.mem.available("cake"), 1)
        self.assertEqual(len(self.mem.instances), 10)

    def test_pick_and_place(self):
        self.mem.mark_picked("bear")
        self.assertEqual(self.mem.available("bear"), 2)
        self.assertIsNotNone(self.mem.carried)
        self.mem.mark_placed(zone="B", verified=True)
        self.assertIsNone(self.mem.carried)
        self.assertEqual(self.mem.last_placed.status, "delivered")
        self.assertEqual(self.mem.last_placed.zone, "B")

    def test_place_without_verification(self):
        self.mem.mark_picked("cake")
        self.mem.mark_placed(zone="A", verified=None)
        self.assertEqual(self.mem.last_placed.status, "delivered_unverified")

    def test_mark_picked_uses_evidence_pose(self):
        mem = MissionMemory.from_inventory([
            {"name": "b_near", "type": "doll", "x": 1.35, "y": 1.5},
            {"name": "b_far", "type": "doll", "x": 1.65, "y": 1.35},
        ])
        inst = mem.mark_picked("bear", pose=(1.65, 1.35))
        self.assertEqual(inst.name, "b_far")
        self.assertEqual(mem.available("bear"), 1)
        self.assertEqual(mem.carried.name, "b_far")

    def test_confirmed_class_wins(self):
        inst = self.mem.mark_picked("bear", confirmed="clock")
        self.assertEqual(inst.cls, "clock")
        self.assertEqual(self.mem.carried.cls, "clock")
        self.assertEqual(self.mem.available("bear"), 3)
        self.assertEqual(self.mem.available("clock"), 2)

    def test_revise_delivered(self):
        self.mem.mark_picked("cake")
        self.mem.mark_placed(zone="A", verified=True)
        self.assertEqual(self.mem.available("cake"), 0)
        self.mem.revise_delivered("cake")
        self.assertEqual(self.mem.available("cake"), 1)
        self.assertTrue(any(e["kind"] == "memory_revised" for e in self.mem.events))

    def test_mark_skipped(self):
        self.mem.mark_skipped("bear")
        self.assertEqual(self.mem.available("bear"), 2)
        self.assertEqual(len(self.mem.by_status("skipped")), 1)


# ---------------------------------------------------------------------------
# World model
# ---------------------------------------------------------------------------


class WorldModelTests(unittest.TestCase):
    def test_queries(self):
        self.assertEqual(WORLD.station_pose("N"), (0.0, 0.55))
        self.assertEqual(WORLD.zone_center("A"), (-1.275, 1.275))
        self.assertEqual(WORLD.default_mapping["bear"], "B")
        self.assertEqual(WORLD.default_mapping["cake"], "A")

    def test_actual_scenario_json(self):
        with open(os.path.join(CODE_DIR, "scenario.json")) as f:
            sc = json.load(f)
        world = WorldModel.from_scenario(sc)
        self.assertEqual(len(world.bricks), 10)
        self.assertEqual(world.station_pose("S"), (1.5, 1.09))
        mem = MissionMemory.from_inventory(world.bricks, world.default_mapping)
        self.assertEqual(mem.available("bear"), 3)
        self.assertEqual(mem.available("cake"), 1)

    def test_load_demo_mission(self):
        path = os.path.join(CODE_DIR, "..", "config", "mission_demo.yaml")
        m = load_mission(path)
        kinds = [s.kind for s in m.steps]
        self.assertEqual(kinds[0], "goto")
        self.assertIn("pick", kinds)
        self.assertIn("place", kinds)
        self.assertIn("verify", kinds)
        self.assertEqual(m.clock_budget_s, 540)


# ---------------------------------------------------------------------------
# Executor
# ---------------------------------------------------------------------------


class ExecutorTests(unittest.TestCase):
    def run_mission(self, m, platform, memory=None, thresholds=None):
        return MissionRunner(m, platform, WORLD, memory=memory, thresholds=thresholds).run()

    def test_pick_passes_and_calls_arm(self):
        plat = FakePlatform(windows=[frames(det("cake", x=0.0, y=0.55))])
        m = mission(("pick", {"class": "cake", "station": "N", "attempts": 1}))
        report = self.run_mission(m, plat)
        self.assertEqual(report.state, "DONE")
        self.assertEqual(report.records[0].outcome, Outcome.OK)
        self.assertEqual(plat.pick_calls, ["cake"])

    def test_gate_failure_never_calls_arm(self):
        plat = FakePlatform(windows=[frames(det("clock", x=0.0, y=0.55))])
        m = mission(("pick", {"class": "cake", "station": "N", "attempts": 2}))
        report = self.run_mission(m, plat)
        self.assertEqual(report.records[0].outcome, Outcome.SKIPPED)
        self.assertEqual(plat.pick_calls, [], "must never pick blind")

    def test_failed_pick_does_not_mark_carried(self):
        # A failed grasp must leave the cube pending so the retry is not blocked by G3.
        window = frames(det("cake", x=0.0, y=0.55))
        plat = FakePlatform(
            windows=[window, window],
            picks=[PickResult(False, reason="grasp failed"), PickResult(True, confirmed_class="cake")],
        )
        m = mission(("pick", {"class": "cake", "station": "N", "attempts": 2}))
        report = self.run_mission(m, plat)
        self.assertEqual(report.records[0].outcome, Outcome.OK)
        self.assertEqual(plat.pick_calls, ["cake", "cake"])
        self.assertEqual(report.records[0].attempts, 2)

    def test_goto_recovers_then_succeeds(self):
        plat = FakePlatform(goto=[False, True])
        m = mission(("goto", {"point": "Sorting_N"}))
        report = self.run_mission(m, plat)
        self.assertEqual(report.records[0].outcome, Outcome.OK)
        self.assertEqual(report.records[0].attempts, 2)
        self.assertEqual(plat.recovery, ["clear_costmaps", "rotate_scan", "backup"])

    def test_goto_fails_and_mission_continues(self):
        plat = FakePlatform(goto=[False, False, True])
        m = mission(
            ("goto", {"point": "Sorting_N"}),
            ("goto", {"point": "Collection_A"}),
        )
        report = self.run_mission(m, plat)
        self.assertEqual(report.records[0].outcome, Outcome.FAILED)
        self.assertEqual(report.records[1].outcome, Outcome.OK)
        self.assertEqual(report.state, "DONE")

    def test_place_retries_once(self):
        mem = make_memory()
        mem.mark_picked("cake")
        plat = FakePlatform(places=[PlaceResult(False), PlaceResult(True)])
        m = mission(("place", {"zone": "A", "level": 1}))
        report = self.run_mission(m, plat, memory=mem)
        self.assertEqual(report.records[0].outcome, Outcome.OK)
        self.assertEqual(report.records[0].attempts, 2)
        self.assertEqual(mem.last_placed.status, "delivered_unverified")

    def test_verify_confirms_delivery(self):
        mem = make_memory()
        mem.mark_picked("cake")
        mem.mark_placed(zone="A")
        plat = FakePlatform(windows=[frames(det("cake", x=ZONE_A[0], y=ZONE_A[1]))])
        m = mission(("verify", {"class": "cake", "zone": "A", "timeout_s": 1.0}))
        report = self.run_mission(m, plat, memory=mem)
        self.assertEqual(report.records[0].outcome, Outcome.OK)
        self.assertEqual(mem.last_placed.status, "delivered")

    def test_verify_unverified_keeps_status(self):
        mem = make_memory()
        mem.mark_picked("cake")
        mem.mark_placed(zone="A")
        plat = FakePlatform()
        m = mission(("verify", {"class": "cake", "zone": "A", "timeout_s": 1.0}))
        report = self.run_mission(m, plat, memory=mem)
        self.assertEqual(report.records[0].outcome, Outcome.UNVERIFIED)
        self.assertEqual(mem.last_placed.status, "delivered_unverified")

    def test_read_board_mapping(self):
        plat = FakePlatform(board=["doll", "clock", "banana"])
        m = mission(("read_board", {}))
        report = self.run_mission(m, plat)
        self.assertEqual(report.state, "DONE")
        self.assertEqual(report.mapping, {"bear": "B", "clock": "C", "banana": "D", "cake": "A"})

    def test_read_board_optional_falls_back(self):
        plat = FakePlatform(board=None)
        m = mission(("read_board", {"optional": True}))
        report = self.run_mission(m, plat)
        self.assertEqual(report.records[0].outcome, Outcome.OK)
        self.assertEqual(report.mapping, WORLD.default_mapping)
        self.assertTrue(any(e["kind"] == "board_fallback" for e in report.events))

    def test_read_board_invalid_fails(self):
        plat = FakePlatform(board=["clock", "clock", "banana"])
        m = mission(("read_board", {}))
        report = self.run_mission(m, plat)
        self.assertEqual(report.records[0].outcome, Outcome.FAILED)

    def test_conflict_revises_memory_then_picks(self):
        mem = make_memory()
        mem.mark_picked("cake")
        mem.mark_placed(zone="A", verified=True)
        window = frames(det("cake", x=0.0, y=0.55))
        plat = FakePlatform(windows=[window, window])
        m = mission(("pick", {"class": "cake", "station": "N", "attempts": 1}))
        report = self.run_mission(m, plat, memory=mem)
        self.assertEqual(report.records[0].outcome, Outcome.OK)
        self.assertEqual(plat.pick_calls, ["cake"])
        self.assertTrue(any(e["kind"] == "memory_revised" for e in report.events))

    def test_conflict_unresolved_skips(self):
        mem = make_memory()
        mem.mark_picked("cake")
        mem.mark_placed(zone="A", verified=True)
        plat = FakePlatform(windows=[
            frames(det("cake", x=0.0, y=0.55)),
            frames(det("clock", x=0.0, y=0.55)),
        ])
        m = mission(("pick", {"class": "cake", "station": "N", "attempts": 1}))
        report = self.run_mission(m, plat, memory=mem)
        self.assertEqual(report.records[0].outcome, Outcome.SKIPPED)
        self.assertEqual(plat.pick_calls, [])

    def test_stop_flag_aborts(self):
        plat = FakePlatform(stop=True)
        m = mission(("goto", {"point": "Sorting_N"}))
        report = self.run_mission(m, plat)
        self.assertEqual(report.state, "ABORT")
        self.assertIn("stop", report.reason)
        self.assertEqual(plat.goto_calls, [])

    def test_clock_expiry_aborts(self):
        plat = FakePlatform(time_left=0.0)
        m = mission(("goto", {"point": "Sorting_N"}))
        report = self.run_mission(m, plat)
        self.assertEqual(report.state, "ABORT")
        self.assertIn("clock", report.reason)

    def test_precheck_failure_aborts(self):
        plat = FakePlatform(precheck_ok=False)
        m = mission(("goto", {"point": "Sorting_N"}))
        report = self.run_mission(m, plat)
        self.assertEqual(report.state, "ABORT")
        self.assertIn("precheck", report.reason)

    def test_fetch_adaptive_station_and_zone(self):
        # One bear at the left-middle cell: the fetch should choose Sorting_W,
        # pick it, and deliver to zone B from the mapping.
        with open(os.path.join(CODE_DIR, "..", "config", "arena_map.yaml")) as f:
            world = WorldModel.from_scenario(yaml.safe_load(f))
        memory = MissionMemory.from_inventory(
            [{"name": "bear_x", "type": "doll", "x": 1.355, "y": 1.5}],
            {"bear": "B", "clock": "C", "banana": "D", "cake": "A"},
        )
        plat = FakePlatform(
            windows=[
                frames(det("bear", x=1.355, y=1.5)),      # pick gate at Sorting_W
                frames(det("bear", x=2.775, y=2.775)),    # verify at zone B centre
            ],
            goto=[True, True],
            picks=[PickResult(True, confirmed_class="bear")],
            places=[PlaceResult(True)],
        )
        m = mission(("fetch", {"class": "bear", "count": 1, "level": 1}))
        report = MissionRunner(m, plat, world, memory=memory).run()
        self.assertEqual(report.state, "DONE")
        self.assertEqual(report.records[0].outcome, Outcome.OK)
        self.assertEqual(plat.goto_calls, ["Sorting_W", "Collection_B_approach"])
        self.assertEqual(plat.pick_calls, ["bear"])
        self.assertEqual(plat.place_calls, [1])
        self.assertEqual(memory.last_placed.status, "delivered")
        self.assertEqual(report.records[0].details.get("delivered"), 1)

    def test_full_demo_mission(self):
        steps = [
            ("goto", {"point": "Sorting_N", "tol": 0.08}),
            ("goto", {"point": "Collection_A", "tol": 0.08}),
            ("goto", {"point": "Sorting_N", "tol": 0.06}),
            ("pick", {"class": "cake", "station": "N", "attempts": 1}),
            ("goto", {"point": "Collection_A", "tol": 0.06}),
            ("place", {"zone": "A", "level": 1}),
            ("verify", {"class": "cake", "zone": "A", "timeout_s": 1.0}),
        ]
        plat = FakePlatform(windows=[
            frames(det("cake", x=0.0, y=0.55)),
            frames(det("cake", x=ZONE_A[0], y=ZONE_A[1])),
        ])
        report = self.run_mission(mission(*steps), plat)
        self.assertEqual(report.state, "DONE")
        self.assertEqual([r.outcome for r in report.records], [Outcome.OK] * 7)
        self.assertEqual(plat.pick_calls, ["cake"])
        self.assertEqual(plat.place_calls, [1])
        self.assertEqual(report.memory["last_placed"], "cake")
        counts = report.memory["counts"]["cake"]
        self.assertEqual(counts.get("delivered"), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
