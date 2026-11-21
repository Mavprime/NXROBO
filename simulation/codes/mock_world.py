#!/usr/bin/env python3
"""Mock robot / arm / camera / scorer for testing the auto-task logic with no hardware.

Pure Python (no ROS import) so it also runs on a laptop. The interface mirrors the names in the
stock runtask.py (goto_local, step_go, step_back, grasp, drop, ...) plus the pick(type) call your
friend's arm side will provide, so your state machine can be written once and run against either
this mock or the real classes.

All time costs and failure rates are ASSUMED numbers (see Config). Replace them with logged values
from real runs; until then results only show logic bugs, not real scores.

Self-test:   python3 mock_world.py --runs 200
"""
import argparse
import json
import random
from dataclasses import dataclass, field

# -- scoring constants taken from the rules (auto-task group) ---------------------------------
RUN_SECONDS = 600
LEAVE_A_POINTS = 1
OVERTIME_PENALTY_PER_S = 2
EARLY_START_PENALTY = 6


def collection_points(layer):
    return layer + 2 ** (layer - 1)          # N + 2^(N-1)


def transport_points(layer):
    return 2 ** (layer - 1)                  # 2^(N-1)


@dataclass
class Config:
    # ASSUMED costs in seconds -- replace with measured values
    nav_seconds: float = 12.0
    step_seconds: float = 2.0
    detect_seconds: float = 3.0
    pick_seconds: float = 10.0
    drop_seconds: float = 5.0
    # ASSUMED failure rates -- replace with measured values
    p_nav_fail: float = 0.05
    p_pick_fail: float = 0.15
    p_drop_in_collection: float = 0.7        # else lands in transport zone
    p_misdetect: float = 0.05                # board read swaps two neighbours
    nav_timeout: float = 60.0                # matches runtask.goto_local


class MockWorld:
    def __init__(self, scenario, cfg=None, seed=0):
        self.sc = scenario
        self.cfg = cfg or Config()
        self.rng = random.Random(seed)
        self.t = 0.0
        self.left_a = False
        self.pose_name = "A_start"
        self.held = None                      # brick dict or None
        self.remaining = [dict(b) for b in scenario["bricks"]]
        self.zone_stacks = {z: [] for z in "ABCD"}   # list of (type, in_collection)
        self.log = []
        self.zone_of_type = dict(zip(scenario["board_left_to_right"], scenario["board_zones"]))
        self.zone_of_type["cake"] = "A"
        self.cake_done = False
        self.penalties = 0

    # -- helpers -------------------------------------------------------------------------
    def _spend(self, s):
        self.t += s

    def _log(self, what, **kw):
        self.log.append({"t": round(self.t, 1), "what": what, **kw})

    def time_left(self):
        return RUN_SECONDS - self.t

    # -- robot (names follow runtask.py) --------------------------------------------------
    def step_go(self, meters=0.2):
        self._spend(self.cfg.step_seconds)
        if self.pose_name == "A_start":
            self.left_a = True
        self._log("step_go", m=meters)

    def step_back(self, meters=0.2):
        self._spend(self.cfg.step_seconds)
        self._log("step_back")

    def goto_local(self, name):
        """True on success, False on failure/timeout, like runtask.goto_local."""
        if name not in self.sc["points"]:
            self._log("goto_local", name=name, ok=False, why="unknown point")
            return False
        if self.rng.random() < self.cfg.p_nav_fail:
            self._spend(self.cfg.nav_timeout)
            self._log("goto_local", name=name, ok=False, why="timeout")
            return False
        self._spend(self.cfg.nav_seconds)
        self.pose_name = name
        if name != "A_start":
            self.left_a = True
        self._log("goto_local", name=name, ok=True)
        return True

    # -- camera --------------------------------------------------------------------------
    def read_board(self):
        """Types left to right on the sorting board; may be wrong with p_misdetect."""
        self._spend(self.cfg.detect_seconds)
        board = list(self.sc["board_left_to_right"])
        if self.rng.random() < self.cfg.p_misdetect:
            i = self.rng.randrange(2)
            board[i], board[i + 1] = board[i + 1], board[i]
        self._log("read_board", board=board)
        return board

    def detector(self, want=None):
        """Types visible at the centre (layer-1 bricks plus the cake on top)."""
        self._spend(self.cfg.detect_seconds)
        vis = [b["type"] for b in self.remaining]
        return [t for t in vis if want is None or t == want]

    # -- arm (the pick(type) call the friend provides) -----------------------------------
    def pick(self, ty):
        """Return True and hold a brick of type ty, False on failure. Fails if hands full,
        if not at the centre, or if the type is no longer on the field."""
        self._spend(self.cfg.pick_seconds)
        if self.held is not None:
            self._log("pick", ty=ty, ok=False, why="hand full")
            return False
        if not self.pose_name.startswith("Sorting_"):
            self._log("pick", ty=ty, ok=False, why="not at centre")
            return False
        cands = [b for b in self.remaining if b["type"] == ty]
        if ty != "cake":                      # cake sits on top of the centre clock
            cands = [b for b in cands if b["layer"] == 1]
        if not cands:
            self._log("pick", ty=ty, ok=False, why="none left")
            return False
        if self.rng.random() < self.cfg.p_pick_fail:
            self._log("pick", ty=ty, ok=False, why="grasp failed")
            return False
        b = cands[0]
        self.remaining.remove(b)
        self.held = b
        self._log("pick", ty=ty, ok=True)
        return True

    def drop(self):
        """Drop whatever is held at the current point. Returns True if something was dropped."""
        self._spend(self.cfg.drop_seconds)
        if self.held is None:
            self._log("drop", ok=False, why="nothing held")
            return False
        if not self.pose_name.startswith("Collection_"):
            self._log("drop", ok=False, why="not at a collection point")
            self.held = None                  # lost on the floor in the public area
            return True
        z = self.pose_name.split("_")[1]
        in_collection = self.rng.random() < self.cfg.p_drop_in_collection
        self.zone_stacks[z].append((self.held["type"], in_collection))
        self._log("drop", zone=z, ty=self.held["type"], collection=in_collection)
        if self.held["type"] == "cake" and z == "A":
            self.cake_done = True
        self.held = None
        return True

    # -- scoring (rules: auto-task group) ------------------------------------------------
    def score(self, overtime_s=0.0):
        pts = LEAVE_A_POINTS if self.left_a else 0
        detail = []
        cake_first_ok = self.cake_done and all(
            e["what"] != "drop" or e.get("ty") == "cake" or e["t"] >= self._cake_drop_time()
            for e in self.log)
        for z, stack in self.zone_stacks.items():
            layer = 0
            for ty, in_coll in stack:
                correct = self.zone_of_type.get(ty) == z
                if ty == "cake" and not cake_first_ok:
                    correct = False
                if not correct:
                    detail.append((z, ty, 0))      # wrong zone: 0, and no layer is consumed
                    continue
                layer = min(layer + 1, 2)          # cap at 2 layers (assumed stack limit)
                p = collection_points(layer) if in_coll else transport_points(layer)
                pts += p
                detail.append((z, ty, p))
        pts -= int(overtime_s) * OVERTIME_PENALTY_PER_S
        return max(pts, 0), detail

    def _cake_drop_time(self):
        for e in self.log:
            if e["what"] == "drop" and e.get("ty") == "cake":
                return e["t"]
        return 1e9


# -- a deliberately simple baseline strategy so the self-test has something to run ------------
def baseline_run(world):
    w = world
    w.step_go(0.2)
    if not w.goto_local("Classification_area"):
        return
    board = w.read_board()
    zmap = dict(zip(board, ["B", "C", "D"]))
    zmap["cake"] = "A"
    # cake first
    for _ in range(2):
        if w.goto_local("Sorting_N") and w.pick("cake"):
            break
    if w.held is not None and w.goto_local("Collection_A"):
        w.drop()
        w.step_back()
    # brick loop
    stuck = 0
    while w.time_left() > 45 and stuck < 6:
        vis = [t for t in w.detector() if t in zmap and t != "cake"]
        if not vis:
            break
        ty = vis[0]
        ok = w.goto_local("Sorting_N") and w.pick(ty)
        if not ok:
            stuck += 1
            continue
        if w.goto_local("Collection_%s" % zmap[ty]):
            w.drop()
            w.step_back()
        else:
            w.drop()                          # carry failed: drop where we are (scores 0)


def selftest(runs):
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    sc = json.load(open(os.path.join(here, "scenario.json")))
    totals, times = [], []
    for i in range(runs):
        w = MockWorld(sc, Config(), seed=i)
        baseline_run(w)
        over = max(0.0, w.t - RUN_SECONDS)
        s, _ = w.score(overtime_s=over)
        totals.append(s)
        times.append(w.t)
    totals.sort()
    n = len(totals)
    print("runs=%d  mean=%.1f  median=%d  p10=%d  p90=%d  max=%d  mean_time=%.0fs" % (
        n, sum(totals) / n, totals[n // 2], totals[n // 10], totals[(9 * n) // 10],
        totals[-1], sum(times) / n))
    # hand-checked cases
    w = MockWorld(sc, Config(p_nav_fail=0, p_pick_fail=0, p_drop_in_collection=1.0, p_misdetect=0), 0)
    w.step_go()
    assert w.score()[0] == 1, "leaving A should give 1 point"
    ok = w.goto_local("Classification_area")
    assert ok
    w.goto_local("Sorting_N")
    assert w.pick("cake")
    w.goto_local("Collection_A")
    w.drop()
    s, d = w.score()
    assert s == 1 + collection_points(1), (s, d)    # free point 1 + cake in A collection zone 2 = 3
    print("hand-check ok: free point + cake in A collection =", s)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=200)
    selftest(ap.parse_args().runs)
