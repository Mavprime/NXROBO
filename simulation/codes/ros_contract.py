#!/usr/bin/env python3
"""Pure helpers for the ROS layer: JSON messages, goal resolution, state serialisation.

No ROS imports, so these run in plain unit tests. Both `mission_node.py` and the
simulation use the same goal-resolution rules, which keeps sim and robot aligned.
"""

from __future__ import annotations

import json
import math
from typing import Any, Optional

from mission_logic import Detection, canonical_class


def parse_detections(text: str) -> list[Detection]:
    """Parse `/detections_world` JSON into Detections.

    Accepted shapes:
        {"stamp": 12.3, "objects": [{"class": "cake", "confidence": 0.8, "x": 1.5, "y": 1.5}]}
        [{"class": "cake", "confidence": 0.8, "x": 1.5, "y": 1.5}, ...]

    Objects without a world pose are dropped: the mission gate needs geometry.
    """
    data = json.loads(text)
    if isinstance(data, dict):
        objects = data.get("objects") or data.get("detections") or []
    elif isinstance(data, list):
        objects = data
    else:
        objects = []

    out: list[Detection] = []
    for obj in objects:
        if not isinstance(obj, dict):
            continue
        raw = obj.get("class", obj.get("cls", obj.get("id")))
        if raw is None:
            continue
        try:
            confidence = float(obj.get("confidence", obj.get("conf", 1.0)))
            x = float(obj["x"])
            y = float(obj["y"])
        except (KeyError, TypeError, ValueError):
            continue
        stamp = float(obj.get("stamp", 0.0))
        out.append(Detection(raw, confidence, x, y, stamp))
    return out


def resolve_goal(world: dict, point: str) -> tuple[float, float, float]:
    """Named point -> (x, y, yaw) in the map/arena frame.

    Collection approaches face the zone centre; sorting points face the central
    cluster; the classification point faces the table (west)."""
    points = {k: (float(v[0]), float(v[1])) for k, v in world["points"].items()}
    if point not in points:
        raise KeyError("unknown point: %s" % point)
    x, y = points[point]

    arena = world.get("arena") or {}
    ox, oy = arena.get("origin", [0.0, 0.0])
    sx, sy = arena.get("size", [3.0, 3.0])
    cx, cy = ox + sx / 2.0, oy + sy / 2.0

    if point.startswith("Collection_"):
        zone = point.split("_")[1]
        centre = world["zones"][zone]["collection_center"]
        yaw = math.atan2(centre[1] - y, centre[0] - x)
    elif point.startswith("Sorting_"):
        yaw = math.atan2(cy - y, cx - x)
    elif point == "Classification_area":
        yaw = math.pi
    else:
        yaw = 0.0
    return x, y, yaw


def encode_arm_pick(request_id: str, cls: str, station: Optional[str] = None) -> str:
    return json.dumps({
        "id": request_id,
        "op": "pick",
        "class": canonical_class(cls),
        "station": station or "",
    })


def encode_arm_place(request_id: str, zone: Optional[str] = None, level: int = 1) -> str:
    return json.dumps({
        "id": request_id,
        "op": "place",
        "zone": zone or "",
        "level": int(level),
    })


def parse_arm_result(text: str) -> dict[str, Any]:
    data = json.loads(text)
    return {
        "id": data.get("id"),
        "ok": bool(data.get("ok", False)),
        "confirmed_class": canonical_class(data.get("confirmed_class") or ""),
        "reason": str(data.get("reason", "")),
        "duration_s": float(data.get("duration_s", 0.0) or 0.0),
    }


def parse_board(text: str) -> Optional[list[str]]:
    """Board read: JSON list of three classes left -> right, or a dict with 'classes'."""
    data = json.loads(text)
    if isinstance(data, dict):
        data = data.get("classes") or data.get("board") or []
    if not isinstance(data, list) or not data:
        return None
    return [canonical_class(c) for c in data]


def state_dict(runner: Any, memory: Any, clock_remaining: float) -> dict[str, Any]:
    current = "-"
    if getattr(runner, "current", None) is not None:
        params = runner.current.params or {}
        label = params.get("point") or params.get("class")
        current = runner.current.kind + (" " + str(label) if label else "")
    return {
        "state": runner.state,
        "current": current,
        "steps": [
            {"kind": r.kind, "outcome": r.outcome.value, "reason": r.reason,
             "elapsed_s": r.elapsed_s}
            for r in runner.records
        ],
        "mapping": dict(memory.mapping),
        "memory": memory.summary(),
        "clock": clock_remaining,
    }
