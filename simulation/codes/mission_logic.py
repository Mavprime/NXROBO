#!/usr/bin/env python3
"""ROS-free mission logic for the Spark chassis: executor, agreement gate, memory.

This module is deliberately free of ROS imports so it can be tested with plain
`python3` on a laptop, the same way `auto/codes/match_logic.py` is. A ROS node or the
discrete mock plugs into it through the small platform protocol described below.

Platform protocol (duck-typed, all optional except goto/scan_window/pick/place):

    goto(point: str) -> bool                 # arrived within tolerance
    scan_window(n: int) -> list[list[Detection]]   # n frames, oldest first
    pick(cls: str) -> PickResult             # canonical class
    place(level: int) -> PlaceResult
    read_board() -> list[str] | None         # three cards, left -> right
    time_left() -> float                     # seconds of match left
    stop_requested() -> bool
    wait(seconds)
    clear_costmaps(); rotate_scan(); backup(); reframe(station)   # recovery, optional
    precheck() -> (bool, reason)             # optional start-up check

Everything else (mission steps, gate rules, cube statuses, recovery policy) lives here.

See docs/ALGORITHM.md for the design this implements.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Iterable, Optional, Sequence

__all__ = [
    "canonical_class",
    "Detection",
    "PickResult",
    "PlaceResult",
    "Thresholds",
    "CubeInstance",
    "MissionMemory",
    "WorldModel",
    "GateVerdict",
    "Gate",
    "GateAction",
    "Outcome",
    "Step",
    "Mission",
    "load_mission",
    "MissionClock",
    "StepRecord",
    "MissionReport",
    "MissionRunner",
]

# ---------------------------------------------------------------------------
# Classes
# ---------------------------------------------------------------------------

# Production ids from match_logic.norm_id():
#   bear: 1 | 10   cake: 2 | 20   clock: 3 | 30   banana: 4 | 40
# The generated sim scenario spells the bear as "doll"; both map to "bear".
CLASS_ALIASES = {
    "1": "bear", "10": "bear", "doll": "bear", "bear": "bear", "teddy": "bear",
    "2": "cake", "20": "cake", "cake": "cake",
    "3": "clock", "30": "clock", "clock": "clock",
    "4": "banana", "40": "banana", "banana": "banana",
}


def canonical_class(raw: Any) -> str:
    """Map raw YOLO ids / scenario spellings to one canonical class name."""
    if raw is None:
        return ""
    key = str(raw).strip().lower()
    return CLASS_ALIASES.get(key, key)


# ---------------------------------------------------------------------------
# Small value types
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Detection:
    """One detection in one frame. Pose is optional: the discrete mock has none."""

    cls: str
    confidence: float = 1.0
    x: Optional[float] = None
    y: Optional[float] = None
    stamp: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "cls", canonical_class(self.cls))

    def pose(self) -> Optional[tuple[float, float]]:
        if self.x is None or self.y is None:
            return None
        return (float(self.x), float(self.y))


@dataclass(frozen=True)
class PickResult:
    ok: bool
    confirmed_class: str = ""
    reason: str = ""
    duration_s: float = 0.0
    retryable: bool = True
    confirmed_pose: Optional[tuple[float, float]] = None

    def __post_init__(self) -> None:
        if self.confirmed_class:
            object.__setattr__(self, "confirmed_class", canonical_class(self.confirmed_class))


@dataclass(frozen=True)
class PlaceResult:
    ok: bool
    reason: str = ""
    retryable: bool = True


@dataclass
class Thresholds:
    """Gate thresholds. Defaults mirror config/gate.yaml."""

    conf_min: float = 0.35
    votes_n: int = 5
    votes_k: int = 3
    cluster_radius_m: float = 0.06
    r_station_m: float = 0.12
    r_zone_m: float = 0.15
    arrival_settle_s: float = 0.5
    arrival_speed_eps: float = 0.02
    nav_tolerance_default_m: float = 0.08
    recovery_backup_m: float = 0.20
    relocalize_cov_max: float = 0.05
    time_reserve_s: float = 20.0
    pick_cost_estimate_s: float = 10.0
    verify_interval_s: float = 0.5

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Thresholds":
        known = {f.name for f in cls.__dataclass_fields__.values()}  # type: ignore[attr-defined]
        return cls(**{k: v for k, v in (data or {}).items() if k in known})

    @classmethod
    def from_yaml(cls, path: str) -> "Thresholds":
        import yaml

        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return cls.from_dict(data.get("thresholds", data))


# ---------------------------------------------------------------------------
# Memory
# ---------------------------------------------------------------------------

CUBE_STATUSES = (
    "pending",
    "carried",
    "delivered",
    "delivered_unverified",
    "misplaced",
    "skipped",
    "lost",
)


@dataclass
class CubeInstance:
    name: str
    cls: str
    status: str = "pending"
    zone: Optional[str] = None
    x: Optional[float] = None
    y: Optional[float] = None

    def __post_init__(self) -> None:
        self.cls = canonical_class(self.cls)
        if self.status not in CUBE_STATUSES:
            raise ValueError("unknown cube status %r" % self.status)


@dataclass
class MissionMemory:
    """Task memory: which cube instances are pending / carried / delivered.

    Instance-level statuses, class-level destinations (from the board). The cake is the
    only instance-pinned cube and must reach zone A first.
    """

    instances: list[CubeInstance] = field(default_factory=list)
    mapping: dict[str, str] = field(default_factory=dict)
    carried: Optional[CubeInstance] = None
    last_placed: Optional[CubeInstance] = None
    events: list[dict[str, Any]] = field(default_factory=list)

    # -- construction -------------------------------------------------------

    @classmethod
    def from_inventory(
        cls, bricks: Iterable[dict[str, Any]], mapping: Optional[dict[str, str]] = None
    ) -> "MissionMemory":
        instances: list[CubeInstance] = []
        used: dict[str, int] = {}
        for i, brick in enumerate(bricks):
            raw = brick.get("type", brick.get("class", ""))
            ccls = canonical_class(raw)
            used[ccls] = used.get(ccls, 0) + 1
            name = brick.get("name") or "%s_%d" % (ccls, used[ccls])
            x = float(brick["x"]) if brick.get("x") is not None else None
            y = float(brick["y"]) if brick.get("y") is not None else None
            instances.append(CubeInstance(name=name, cls=ccls, x=x, y=y))
        return cls(instances=instances, mapping=dict(mapping or {}))

    # -- events -------------------------------------------------------------

    def log(self, kind: str, **kw: Any) -> None:
        entry = {"kind": kind}
        entry.update(kw)
        self.events.append(entry)

    # -- queries ------------------------------------------------------------

    def available(self, cls: str) -> int:
        ccls = canonical_class(cls)
        return sum(1 for i in self.instances if i.cls == ccls and i.status == "pending")

    def pending(self, cls: str) -> list[CubeInstance]:
        ccls = canonical_class(cls)
        return [i for i in self.instances if i.cls == ccls and i.status == "pending"]

    def nearest_pose(self, cls: str, point: tuple[float, float]) -> Optional[tuple[float, float]]:
        """Nearest pending instance of `cls` whose map position is known."""
        ccls = canonical_class(cls)
        best = None
        best_dist = None
        for inst in self.instances:
            if inst.cls != ccls or inst.status != "pending":
                continue
            if inst.x is None or inst.y is None:
                continue
            d = math.hypot(inst.x - point[0], inst.y - point[1])
            if best_dist is None or d < best_dist:
                best, best_dist = (inst.x, inst.y), d
        return best

    def by_status(self, status: str) -> list[CubeInstance]:
        return [i for i in self.instances if i.status == status]

    def counts(self) -> dict[str, dict[str, int]]:
        out: dict[str, dict[str, int]] = {}
        for inst in self.instances:
            out.setdefault(inst.cls, {})
            out[inst.cls][inst.status] = out[inst.cls].get(inst.status, 0) + 1
        return out

    # -- transitions --------------------------------------------------------

    def mark_picked(self, cls: str, confirmed: Optional[str] = None,
                    pose: Optional[tuple[float, float]] = None) -> CubeInstance:
        """Put a pending instance of `cls` into the carried state.

        If the arm confirmed a different class, that class is what is physically carried;
        the caller passes it as `confirmed`. When the gate's evidence pose is known, the
        nearest pending instance is marked — otherwise the wrong same-class cube can be
        marked and picked twice.
        """
        ccls = canonical_class(confirmed or cls)
        candidates = [i for i in self.instances
                      if i.cls == ccls and i.status == "pending"]
        inst = None
        if pose is not None:
            located = [i for i in candidates if i.x is not None and i.y is not None]
            if located:
                inst = min(located, key=lambda i: math.hypot(i.x - pose[0],
                                                             i.y - pose[1]))
        if inst is None and candidates:
            inst = candidates[0]
        if inst is None:
            # The gate said one was available; if it vanished, reacquire it rather than
            # silently carrying nothing.
            inst = CubeInstance(
                name="%s_reacquired_%d" % (ccls, len(self.instances)), cls=ccls
            )
            self.instances.append(inst)
            self.log("instance_reacquired", cls=ccls)
        inst.status = "carried"
        self.carried = inst
        self.log("picked", cls=ccls, instance=inst.name)
        return inst

    def mark_placed(self, zone: Optional[str] = None, verified: Optional[bool] = None) -> Optional[CubeInstance]:
        """Finish the carried instance. verified=None means 'no independent evidence yet'."""
        inst = self.carried
        if inst is None:
            self.log("place_without_cube")
            return None
        inst.zone = zone
        if verified is True:
            inst.status = "delivered"
        elif verified is False:
            inst.status = "misplaced"
        else:
            inst.status = "delivered_unverified"
        self.last_placed = inst
        self.carried = None
        self.log("placed", cls=inst.cls, instance=inst.name, zone=zone, status=inst.status)
        return inst

    def confirm_delivered(self, cls: str, zone: Optional[str] = None) -> Optional[CubeInstance]:
        ccls = canonical_class(cls)
        target = self.last_placed
        if target is None or target.cls != ccls:
            target = next(
                (
                    i
                    for i in self.instances
                    if i.cls == ccls and i.status in ("delivered_unverified", "misplaced")
                ),
                None,
            )
        if target is None:
            return None
        target.status = "delivered"
        if zone:
            target.zone = zone
        self.log("verified", cls=ccls, instance=target.name, zone=target.zone)
        return target

    def mark_misplaced(self, cls: str) -> Optional[CubeInstance]:
        ccls = canonical_class(cls)
        target = self.last_placed if self.last_placed and self.last_placed.cls == ccls else None
        if target is None:
            target = next(
                (i for i in self.instances if i.cls == ccls and i.status == "delivered_unverified"),
                None,
            )
        if target is None:
            return None
        target.status = "misplaced"
        self.log("misplaced", cls=ccls, instance=target.name)
        return target

    def mark_skipped(self, cls: str,
                     pose: Optional[tuple[float, float]] = None) -> Optional[CubeInstance]:
        ccls = canonical_class(cls)
        candidates = [i for i in self.instances
                      if i.cls == ccls and i.status == "pending"]
        inst = None
        if pose is not None:
            located = [i for i in candidates if i.x is not None and i.y is not None]
            if located:
                inst = min(located, key=lambda i: math.hypot(i.x - pose[0],
                                                             i.y - pose[1]))
        if inst is None and candidates:
            inst = candidates[0]
        if inst is None:
            self.log("skip_without_instance", cls=ccls)
            return None
        inst.status = "skipped"
        self.log("skipped", cls=ccls, instance=inst.name)
        return inst

    def revise_delivered(self, cls: str, note: str = "perception saw it again") -> Optional[CubeInstance]:
        """Belief revision: memory said gone, direct evidence says otherwise."""
        ccls = canonical_class(cls)
        target = next(
            (
                i
                for i in self.instances
                if i.cls == ccls
                and i.status in ("delivered", "delivered_unverified", "misplaced")
            ),
            None,
        )
        if target is None:
            target = CubeInstance(
                name="%s_reacquired_%d" % (ccls, len(self.instances)), cls=ccls
            )
            self.instances.append(target)
            self.log("instance_reacquired", cls=ccls)
        target.status = "pending"
        self.log("memory_revised", cls=ccls, instance=target.name, note=note)
        return target

    def summary(self) -> dict[str, Any]:
        return {
            "counts": self.counts(),
            "carried": self.carried.name if self.carried else None,
            "last_placed": self.last_placed.name if self.last_placed else None,
            "instances": [
                {"name": i.name, "cls": i.cls, "status": i.status, "zone": i.zone,
                 "x": i.x, "y": i.y}
                for i in self.instances
            ],
        }


# ---------------------------------------------------------------------------
# World model (from scenario.json / the real map config)
# ---------------------------------------------------------------------------


@dataclass
class WorldModel:
    points: dict[str, tuple[float, float]] = field(default_factory=dict)
    zones: dict[str, dict[str, Any]] = field(default_factory=dict)
    bricks: list[dict[str, Any]] = field(default_factory=list)
    board: list[str] = field(default_factory=list)
    board_zones: list[str] = field(default_factory=lambda: ["B", "C", "D"])
    default_mapping: dict[str, str] = field(default_factory=dict)
    start_pose: Optional[dict[str, float]] = None
    zone_size: float = 0.45  # collection square

    @classmethod
    def from_scenario(cls, sc: dict[str, Any]) -> "WorldModel":
        points = {k: (float(v[0]), float(v[1])) for k, v in (sc.get("points") or {}).items()}
        board = [canonical_class(c) for c in (sc.get("board_left_to_right") or [])]
        board_zones = list(sc.get("board_zones") or ["B", "C", "D"])
        mapping = {c: z for c, z in zip(board, board_zones)}
        mapping["cake"] = "A"  # mandatory rule
        return cls(
            points=points,
            zones=dict(sc.get("zones") or {}),
            bricks=list(sc.get("bricks") or []),
            board=board,
            board_zones=board_zones,
            default_mapping=mapping,
            start_pose=sc.get("start_pose"),
        )

    # -- queries ------------------------------------------------------------

    def point_pose(self, name: str) -> Optional[tuple[float, float]]:
        return self.points.get(name)

    def station_pose(self, station: Optional[str]) -> Optional[tuple[float, float]]:
        if not station:
            return None
        return self.points.get("Sorting_%s" % station) or self.points.get(station)

    def zone_center(self, zone: str) -> Optional[tuple[float, float]]:
        z = self.zones.get(zone) or {}
        centre = z.get("collection_center") or z.get("transport_center")
        if centre is None:
            return None
        return (float(centre[0]), float(centre[1]))

    def classes(self) -> set[str]:
        return {canonical_class(b.get("type", b.get("class", ""))) for b in self.bricks}


# ---------------------------------------------------------------------------
# Agreement gate
# ---------------------------------------------------------------------------


class GateAction(str, Enum):
    PASS = "pass"
    RESCAN = "rescan"
    REFRAME = "reframe"
    SKIP = "skip"
    REPLAN = "replan"
    CONFLICT = "conflict"


@dataclass
class GateVerdict:
    passed: bool
    action: GateAction
    reason: str
    evidence: Optional[Detection] = None
    revise: Optional[tuple[float, float]] = None


def _point_in_square(x: float, y: float, centre: tuple[float, float], size: float) -> bool:
    return abs(x - centre[0]) <= size / 2.0 and abs(y - centre[1]) <= size / 2.0


def _best_per_frame(frame: Sequence[Detection], cls: str, conf_min: float,
                    target_pose: Optional[tuple[float, float]] = None) -> Optional[Detection]:
    """Best detection of `cls` in a frame.

    With a target pose, prefer the detection nearest that target (same-class cubes can
    be far apart); otherwise prefer the highest confidence.
    """
    best = None
    for d in frame or ():
        if d.cls != cls or d.confidence < conf_min:
            continue
        if best is None:
            best = d
            continue
        if target_pose is not None:
            dp = d.pose()
            bp = best.pose()
            if (dp is not None and bp is not None
                    and math.hypot(dp[0] - target_pose[0], dp[1] - target_pose[1])
                    < math.hypot(bp[0] - target_pose[0], bp[1] - target_pose[1])):
                best = d
        elif d.confidence > best.confidence:
            best = d
    return best


class Gate:
    """Admission tests from docs/ALGORITHM.md §4. Three sources must agree."""

    def __init__(self, thresholds: Optional[Thresholds] = None) -> None:
        self.th = thresholds or Thresholds()

    # -- before pick: G1..G5 ------------------------------------------------

    def pick(
        self,
        window: Sequence[Sequence[Detection]],
        requested_class: str,
        station_pose: Optional[tuple[float, float]],
        memory: MissionMemory,
        time_left: float = float("inf"),
        approachable: Optional[Callable[[float, float], bool]] = None,
        check_memory: bool = True,
        target_pose: Optional[tuple[float, float]] = None,
    ) -> GateVerdict:
        cls = canonical_class(requested_class)
        th = self.th

        # G1: enough votes for the requested class. If the caller knows which cube it
        # wants (fetch), each frame prefers the detection nearest that target; this
        # stops a far same-class cube from being picked and failing G4.
        qualifying = [d for d in (_best_per_frame(f, cls, th.conf_min, target_pose)
                                  for f in window) if d]
        if len(qualifying) < th.votes_k:
            return GateVerdict(
                False, GateAction.RESCAN,
                "G1 class unconfirmed (%d/%d votes)" % (len(qualifying), th.votes_k),
            )
        if target_pose is not None:
            located = [d for d in qualifying if d.pose() is not None]
            if located:
                best = min(located, key=lambda d: math.hypot(
                    d.pose()[0] - target_pose[0], d.pose()[1] - target_pose[1]))
            else:
                best = max(qualifying, key=lambda d: (d.confidence, d.stamp))
        else:
            best = max(qualifying, key=lambda d: (d.confidence, d.stamp))

        # G2: observed pose agrees with where the map expects this cube. The target
        # pose only guides G1's detection choice; G2 uses the nearest pending instance
        # so a reachable same-class cube is not rejected when the exact target is not
        # the one detected.
        pose = best.pose()
        expected = station_pose
        if pose is not None:
            nearest = memory.nearest_pose(cls, pose)
            if nearest is not None:
                expected = nearest
        if expected is not None and pose is not None:
            dist = math.hypot(pose[0] - expected[0], pose[1] - expected[1])
            if dist > th.r_station_m:
                return GateVerdict(
                    False, GateAction.RESCAN,
                    "G2 pose mismatch %.0f mm" % (dist * 1000.0),
                    evidence=best, revise=pose,
                )

        # G3: memory must allow the action. Skipped only for the conflict re-check,
        # where perception is deciding whether to revise the memory.
        if check_memory:
            if memory.carried is not None:
                return GateVerdict(False, GateAction.SKIP, "G3 already carrying a cube")
            if memory.available(cls) <= 0:
                return GateVerdict(
                    False, GateAction.CONFLICT,
                    "G3 memory has no pending %s but perception sees one" % cls,
                    evidence=best,
                )

        # G4: physically approachable.
        if approachable is not None and pose is not None and not approachable(*pose):
            return GateVerdict(
                False, GateAction.REFRAME, "G4 target not approachable", evidence=best
            )

        # G5: enough match time for the pick plus the reserve.
        if time_left < th.pick_cost_estimate_s + th.time_reserve_s:
            return GateVerdict(False, GateAction.REPLAN, "G5 not enough match time")

        return GateVerdict(True, GateAction.PASS, "all checks passed", evidence=best)

    # -- after place: V2/V3 -------------------------------------------------

    def verify(
        self,
        window: Sequence[Sequence[Detection]],
        cls: str,
        zone_centre: Optional[tuple[float, float]],
        zone_size: float,
    ) -> tuple[str, Optional[Detection]]:
        """Return ('verified'|'unverified'|'misplaced', evidence)."""
        ccls = canonical_class(cls)
        th = self.th
        seen: list[Detection] = []
        in_zone = 0
        pose_known = False
        for frame in window or ():
            best = _best_per_frame(frame, ccls, th.conf_min)
            if best is None:
                continue
            seen.append(best)
            p = best.pose()
            if p is not None:
                pose_known = True
                if zone_centre is not None and _point_in_square(p[0], p[1], zone_centre, zone_size):
                    in_zone += 1
        if len(seen) < th.votes_k:
            return "unverified", (seen[-1] if seen else None)
        best = max(seen, key=lambda d: (d.confidence, d.stamp))
        if pose_known and in_zone >= th.votes_k:
            return "verified", best
        if pose_known and in_zone == 0:
            return "misplaced", best
        return "unverified", best


# ---------------------------------------------------------------------------
# Mission definition
# ---------------------------------------------------------------------------

ALLOWED_STEPS = {"goto", "pick", "place", "verify", "read_board", "wait", "fetch"}


@dataclass
class Step:
    kind: str
    params: dict[str, Any] = field(default_factory=dict)


@dataclass
class Mission:
    name: str
    clock_budget_s: float
    steps: list[Step] = field(default_factory=list)
    description: str = ""


def load_mission(path: str) -> Mission:
    import yaml

    with open(path) as f:
        data = yaml.safe_load(f) or {}
    steps: list[Step] = []
    for i, item in enumerate(data.get("steps") or []):
        if not isinstance(item, dict) or len(item) != 1:
            raise ValueError("step %d must be a single-key mapping" % i)
        kind, params = next(iter(item.items()))
        if kind not in ALLOWED_STEPS:
            raise ValueError("step %d: unknown kind %r" % (i, kind))
        steps.append(Step(kind=str(kind), params=dict(params or {})))
    return Mission(
        name=str(data.get("name", "mission")),
        clock_budget_s=float(data.get("clock_budget_s", 540)),
        steps=steps,
        description=str(data.get("description", "")),
    )


# ---------------------------------------------------------------------------
# Clock
# ---------------------------------------------------------------------------


class MissionClock:
    """Internal budget, additionally clipped by the platform's match clock."""

    def __init__(self, budget_s: float, platform: Any = None) -> None:
        self.budget_s = float(budget_s)
        self.platform = platform
        self._t0 = time.monotonic()

    def remaining(self) -> float:
        internal = self.budget_s - (time.monotonic() - self._t0)
        external = internal
        fn = getattr(self.platform, "time_left", None)
        if callable(fn):
            try:
                external = float(fn())
            except Exception:  # a broken clock must not abort the mission by itself
                external = internal
        return max(0.0, min(internal, external))


# ---------------------------------------------------------------------------
# Executor
# ---------------------------------------------------------------------------


class Outcome(str, Enum):
    OK = "ok"
    SKIPPED = "skipped"
    UNVERIFIED = "unverified"
    MISPLACED = "misplaced"
    FAILED = "failed"
    ABORT = "abort"


@dataclass
class StepRecord:
    index: int
    kind: str
    params: dict[str, Any]
    outcome: Outcome
    reason: str = ""
    attempts: int = 0
    details: dict[str, Any] = field(default_factory=dict)
    elapsed_s: Optional[float] = None


@dataclass
class MissionReport:
    state: str
    reason: str
    records: list[StepRecord]
    memory: dict[str, Any]
    mapping: dict[str, str]
    events: list[dict[str, Any]]
    clock_remaining: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "state": self.state,
            "reason": self.reason,
            "clock_remaining": self.clock_remaining,
            "mapping": self.mapping,
            "memory": self.memory,
            "events": self.events,
            "steps": [
                {
                    "index": r.index,
                    "kind": r.kind,
                    "params": r.params,
                    "outcome": r.outcome.value,
                    "reason": r.reason,
                    "attempts": r.attempts,
                    "details": r.details,
                    "elapsed_s": r.elapsed_s,
                }
                for r in self.records
            ],
        }


class MissionRunner:
    """Executes a mission against any platform implementing the protocol above."""

    def __init__(
        self,
        mission: Mission,
        platform: Any,
        world: WorldModel,
        memory: Optional[MissionMemory] = None,
        thresholds: Optional[Thresholds] = None,
        approachable: Optional[Callable[[float, float], bool]] = None,
        on_step: Optional[Callable[["StepRecord"], None]] = None,
    ) -> None:
        self.mission = mission
        self.platform = platform
        self.world = world
        self.memory = memory or MissionMemory.from_inventory(
            world.bricks, world.default_mapping
        )
        if not self.memory.mapping:
            self.memory.mapping = dict(world.default_mapping)
        self.th = thresholds or Thresholds()
        self.gate = Gate(self.th)
        self.clock = MissionClock(mission.clock_budget_s, platform)
        self.approachable = approachable
        self.records: list[StepRecord] = []
        self.events: list[dict[str, Any]] = []
        self.state = "IDLE"
        self.abort_reason = ""
        self.on_step = on_step
        self.current: Optional[Step] = None
        self.current_index = -1

    # -- helpers ------------------------------------------------------------

    def _event(self, name: str, **kw: Any) -> None:
        entry = {"kind": name}
        entry.update(kw)
        self.events.append(entry)

    def _call(self, name: str, *args: Any) -> Any:
        fn = getattr(self.platform, name, None)
        if not callable(fn):
            return None
        return fn(*args)

    def _recovery_ladder(self) -> None:
        for name in ("clear_costmaps", "rotate_scan", "backup"):
            if callable(getattr(self.platform, name, None)):
                self._event("recovery", action=name)
                self._call(name)

    def _stop_requested(self) -> bool:
        fn = getattr(self.platform, "stop_requested", None)
        return bool(fn()) if callable(fn) else False

    # -- entry point --------------------------------------------------------

    def run(self) -> MissionReport:
        self.state = "PRECHECK"
        precheck = getattr(self.platform, "precheck", None)
        if callable(precheck):
            ok, reason = precheck()
            if not ok:
                return self._abort("precheck failed: %s" % reason)

        self.state = "EXECUTE"
        for index, step in enumerate(self.mission.steps):
            if self._stop_requested():
                return self._abort("operator stop")
            if self.clock.remaining() <= 0.0:
                return self._abort("match clock expired")
            self.current_index = index
            self.current = step
            record = self._dispatch(index, step)
            self.records.append(record)
            self._event(
                "step",
                index=index,
                kind=step.kind,
                outcome=record.outcome.value,
                reason=record.reason,
            )
            if self.on_step is not None:
                self.on_step(record)
            if record.outcome == Outcome.ABORT:
                return self._abort(record.reason)

        self.state = "DONE"
        return self._report()

    # -- dispatch -----------------------------------------------------------

    def _dispatch(self, index: int, step: Step) -> StepRecord:
        handlers = {
            "goto": self._do_goto,
            "pick": self._do_pick,
            "place": self._do_place,
            "verify": self._do_verify,
            "read_board": self._do_read_board,
            "wait": self._do_wait,
            "fetch": self._do_fetch,
        }
        fn = handlers.get(step.kind)
        if fn is None:
            return StepRecord(index, step.kind, dict(step.params), Outcome.FAILED,
                              "unknown step %r" % step.kind)
        t0 = self._platform_now()
        outcome, reason, attempts, details = fn(step.params)
        t1 = self._platform_now()
        elapsed = (t1 - t0) if (t0 is not None and t1 is not None) else None
        return StepRecord(index, step.kind, dict(step.params), outcome, reason,
                          attempts, details, elapsed)

    def _platform_now(self) -> Optional[float]:
        fn = getattr(self.platform, "now", None)
        if not callable(fn):
            return None
        try:
            return float(fn())
        except Exception:
            return None

    # -- goto ---------------------------------------------------------------

    def _do_goto(self, params: dict[str, Any]) -> tuple[Outcome, str, int, dict[str, Any]]:
        point = str(params.get("point", ""))
        if not point:
            return Outcome.FAILED, "goto without a point", 0, {}
        for attempt in (1, 2):
            if self._call("goto", point):
                return Outcome.OK, "", attempt, {"point": point}
            self._event("nav_failure", point=point, attempt=attempt)
            if attempt == 1:
                self._recovery_ladder()
        return Outcome.FAILED, "unreachable: %s" % point, 2, {"point": point}

    # -- pick ---------------------------------------------------------------

    def _do_pick(self, params: dict[str, Any]) -> tuple[Outcome, str, int, dict[str, Any]]:
        cls = canonical_class(params.get("class", ""))
        station = params.get("station")
        attempts = int(params.get("attempts", 3))
        target_pose = params.get("target_pose")
        if target_pose is not None:
            target_pose = (float(target_pose[0]), float(target_pose[1]))
        station_pose = self.world.station_pose(station)
        verdicts: list[dict[str, Any]] = []

        for attempt in range(1, attempts + 1):
            window = self._call("scan_window", self.th.votes_n) or []
            verdict = self.gate.pick(
                window, cls, station_pose, self.memory,
                time_left=self.clock.remaining(), approachable=self.approachable,
                target_pose=target_pose,
            )
            verdicts.append({"attempt": attempt, "action": verdict.action.value,
                             "reason": verdict.reason})

            if verdict.action == GateAction.CONFLICT:
                second = self._call("scan_window", self.th.votes_n) or []
                verdict2 = self.gate.pick(
                    second, cls, station_pose, self.memory,
                    time_left=self.clock.remaining(), approachable=self.approachable,
                    check_memory=False, target_pose=target_pose,
                )
                if verdict2.passed:
                    self.memory.revise_delivered(cls)
                    self._event("memory_conflict_revised", cls=cls)
                    verdict = verdict2
                else:
                    self._event("memory_conflict_unresolved", cls=cls)
                    return (Outcome.SKIPPED, "unresolved memory conflict", attempt,
                            {"verdicts": verdicts})

            if not verdict.passed:
                self._event("gate_rejected", cls=cls, action=verdict.action.value,
                            reason=verdict.reason)
                if verdict.action == GateAction.RESCAN:
                    self._call("rotate_scan")
                    continue
                if verdict.action == GateAction.REFRAME:
                    if callable(getattr(self.platform, "reframe", None)):
                        self._call("reframe", station)
                    else:
                        self._call("rotate_scan")
                    continue
                if verdict.action in (GateAction.SKIP, GateAction.REPLAN):
                    if verdict.action == GateAction.SKIP:
                        self.memory.mark_skipped(cls, pose=target_pose)
                    return (Outcome.SKIPPED, verdict.reason, attempt,
                            {"verdicts": verdicts})
                continue

            result = self._call("pick", cls)
            if result is None:
                result = PickResult(ok=False, reason="platform returned no result")
            if result.ok:
                # Only a confirmed grasp changes memory. A failed attempt must leave the
                # cube pending, otherwise the retry would see "already carrying".
                confirmed = canonical_class(result.confirmed_class or cls)
                pick_pose = getattr(result, "confirmed_pose", None)
                if pick_pose is None and verdict.evidence:
                    pick_pose = verdict.evidence.pose()
                self.memory.mark_picked(cls, confirmed=confirmed, pose=pick_pose)
                if confirmed != cls:
                    self._event("confirmed_class_mismatch", requested=cls, confirmed=confirmed)
                return (Outcome.OK, "", attempt,
                        {"verdicts": verdicts, "confirmed_class": confirmed})
            self._event("pick_failed", cls=cls, reason=getattr(result, "reason", ""))
            if not getattr(result, "retryable", True):
                break

        self.memory.mark_skipped(cls, pose=target_pose)
        return (Outcome.SKIPPED, "pick attempts exhausted", attempts,
                {"verdicts": verdicts})

    # -- place --------------------------------------------------------------

    def _do_place(self, params: dict[str, Any]) -> tuple[Outcome, str, int, dict[str, Any]]:
        zone = params.get("zone")
        level = int(params.get("level", 1))
        for attempt in (1, 2):
            result = self._call("place", level)
            if result is not None and getattr(result, "ok", False):
                self.memory.mark_placed(zone=zone, verified=None)
                return Outcome.OK, "", attempt, {"zone": zone, "level": level}
            if result is not None and not getattr(result, "retryable", True):
                break
        return Outcome.FAILED, "place failed", 2, {"zone": zone, "level": level}

    # -- verify -------------------------------------------------------------

    def _do_verify(self, params: dict[str, Any]) -> tuple[Outcome, str, int, dict[str, Any]]:
        cls = canonical_class(params.get("class", ""))
        zone = params.get("zone", "")
        timeout = float(params.get("timeout_s", 3.0))
        centre = self.world.zone_center(zone)
        windows = max(1, int(round(timeout / max(self.th.verify_interval_s, 0.1))))

        for i in range(windows):
            window = self._call("scan_window", self.th.votes_n) or []
            status, evidence = self.gate.verify(window, cls, centre, self.world.zone_size)
            if status == "verified":
                self.memory.confirm_delivered(cls, zone)
                return Outcome.OK, "", i + 1, {"status": status, "zone": zone}
            if status == "misplaced":
                self.memory.mark_misplaced(cls)
                return (Outcome.MISPLACED, "cube outside %s" % zone, i + 1,
                        {"status": status, "zone": zone})
            self._call("wait", self.th.verify_interval_s)
        return Outcome.UNVERIFIED, "no confirmation within %.1fs" % timeout, windows, {
            "zone": zone
        }

    # -- read board ---------------------------------------------------------

    def _do_read_board(self, params: dict[str, Any]) -> tuple[Outcome, str, int, dict[str, Any]]:
        optional = bool(params.get("optional", False))
        board = self._call("read_board")
        if board is None:
            if optional:
                self.memory.mapping = dict(self.world.default_mapping)
                self._event("board_fallback", reason="unreadable")
                return Outcome.OK, "fallback mapping", 1, dict(self.memory.mapping)
            return Outcome.FAILED, "board unreadable", 1, {}

        classes = [canonical_class(c) for c in board]
        expected = sorted({inst.cls for inst in self.memory.instances if inst.cls != "cake"})
        if len(classes) != len(set(classes)) or sorted(classes) != expected:
            if optional:
                self.memory.mapping = dict(self.world.default_mapping)
                self._event("board_fallback", reason="invalid board %r" % (classes,))
                return Outcome.OK, "fallback mapping", 1, dict(self.memory.mapping)
            return Outcome.FAILED, "invalid board %r" % (classes,), 1, {"board": classes}

        mapping = {c: z for c, z in zip(classes, self.world.board_zones)}
        mapping["cake"] = "A"
        self.memory.mapping = mapping
        self._event("board_read", mapping=mapping)
        return Outcome.OK, "", 1, {"mapping": mapping}

    # -- wait ---------------------------------------------------------------

    def _do_wait(self, params: dict[str, Any]) -> tuple[Outcome, str, int, dict[str, Any]]:
        self._call("wait", float(params.get("seconds", 1.0)))
        return Outcome.OK, "", 1, {}

    # -- fetch: adaptive station -> pick -> mapped zone ---------------------

    def _best_station(self, cls: str):
        """Nearest (pending cube, Sorting side) pair for this class."""
        pending = [i for i in self.memory.pending(cls) if i.x is not None and i.y is not None]
        if not pending:
            return None
        best = None  # (distance, instance, side)
        for inst in pending:
            for side in ("N", "S", "E", "W"):
                pose = self.world.station_pose(side)
                if pose is None:
                    continue
                dist = math.hypot(inst.x - pose[0], inst.y - pose[1])
                if best is None or dist < best[0]:
                    best = (dist, inst, side)
        if best is None:
            return None
        return best[1], best[2]

    def _do_fetch(self, params: dict[str, Any]) -> tuple[Outcome, str, int, dict[str, Any]]:
        """Pick `count` cubes of a class from the best station, deliver to the board zone."""
        cls = canonical_class(params.get("class", ""))
        count = int(params.get("count", 1))
        level = int(params.get("level", 1))
        attempts = int(params.get("attempts", 2))
        delivered = 0
        tries = 0
        details: list[dict[str, Any]] = []
        max_tries = count * 2 + 2

        while delivered < count and tries < max_tries:
            choice = self._best_station(cls)
            if choice is None:
                details.append({"reason": "no pending cube"})
                break
            instance, side = choice
            tries += 1

            outcome, reason, _, detail = self._do_goto(
                {"point": "Sorting_" + side, "tol": 0.08})
            if outcome != Outcome.OK:
                details.append({"side": side, "stage": "goto_station", "reason": reason})
                continue

            target_pose = ((instance.x, instance.y)
                           if instance.x is not None and instance.y is not None else None)
            outcome, reason, _, detail = self._do_pick(
                {"class": cls, "station": side, "attempts": attempts,
                 "target_pose": target_pose})
            if outcome != Outcome.OK:
                details.append({"side": side, "instance": instance.name,
                                "stage": "pick", "reason": reason})
                continue

            zone = self.memory.mapping.get(cls) or self.world.default_mapping.get(cls)
            if not zone:
                details.append({"stage": "mapping", "reason": "no zone for %s" % cls})
                break

            point = "Collection_%s_approach" % zone
            outcome, reason, _, detail = self._do_goto({"point": point, "tol": 0.06})
            if outcome != Outcome.OK:
                details.append({"zone": zone, "stage": "goto_zone", "reason": reason})
                break  # cube is carried; stop and let the mission report it

            self._do_place({"zone": zone, "level": level})
            vout, vreason, _, _ = self._do_verify(
                {"class": cls, "zone": zone, "timeout_s": 3.0})
            delivered += 1
            details.append({"side": side, "zone": zone, "instance": instance.name,
                            "verify": vout.value, "verify_reason": vreason})

        if delivered == 0:
            return Outcome.SKIPPED, "no %s delivered" % cls, tries, {
                "class": cls, "delivered": 0, "details": details}
        reason = "" if delivered >= count else "delivered %d/%d" % (delivered, count)
        return Outcome.OK, reason, tries, {
            "class": cls, "delivered": delivered, "details": details}

    # -- reports ------------------------------------------------------------

    def _report(self) -> MissionReport:
        return MissionReport(
            state=self.state,
            reason=self.abort_reason,
            records=list(self.records),
            memory=self.memory.summary(),
            mapping=dict(self.memory.mapping),
            events=list(self.events) + list(self.memory.events),
            clock_remaining=self.clock.remaining(),
        )

    def _abort(self, reason: str) -> MissionReport:
        self.state = "ABORT"
        self.abort_reason = reason
        self._event("abort", reason=reason)
        return self._report()
