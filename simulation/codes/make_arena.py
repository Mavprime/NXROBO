#!/usr/bin/env python3
"""Generate arena.world (Gazebo 11 / SDF 1.6) and scenario.json for the Spark auto-task field.

Frame: origin = arena centre, +x = east (right in the layout figure), +y = north (top).
Zones: A = top-left, B = top-right, C = bottom-right, D = bottom-left.
The robot starts in the A collection zone (top-left corner).

Usage:  python3 make_arena.py --seed 3 --out .
Same seed -> same brick layout and board. Seed only changes types, never positions.
"""
import argparse
import json
import math
import os
import random

HALF = 1.5                      # arena is 3 m x 3 m
WALL_H = 0.4
BRICK = 0.10
PITCH = BRICK + 0.045           # 4.5 cm gap between bricks
ZONES = {"A": (-1, 1), "B": (1, 1), "C": (1, -1), "D": (-1, -1)}
YELLOW, GREEN, RED = (1.0, 0.75, 0.0), (0.4, 1.0, 0.6), (0.8, 0.0, 0.0)
COLORS = {"doll": (0.55, 0.35, 0.2), "clock": (0.85, 0.85, 0.85),
          "banana": (0.95, 0.85, 0.15), "cake": (0.9, 0.3, 0.4)}
TABLE = {"x": -HALF + 0.165, "y": 0.0, "sx": 0.33, "sy": 0.40, "sz": 0.20}


def rgba(c):
    return "%.3f %.3f %.3f 1" % c


def link(name, size, color, collision=True, mass=None, friction=False):
    sx, sy, sz = size
    inertial = ""
    if mass:
        inertial = (
            "<inertial><mass>%g</mass><inertia>"
            "<ixx>%.6g</ixx><ixy>0</ixy><ixz>0</ixz>"
            "<iyy>%.6g</iyy><iyz>0</iyz><izz>%.6g</izz></inertia></inertial>"
            % (mass, mass / 12 * (sy * sy + sz * sz),
               mass / 12 * (sx * sx + sz * sz), mass / 12 * (sx * sx + sy * sy)))
    geom = "<geometry><box><size>%g %g %g</size></box></geometry>" % size
    surf = ""
    if friction:
        surf = "<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>"
    col = "<collision name='%s_col'>%s%s</collision>" % (name, geom, surf) if collision else ""
    vis = ("<visual name='%s_vis'>%s<material><ambient>%s</ambient><diffuse>%s</diffuse>"
           "</material></visual>" % (name, geom, rgba(color), rgba(color)))
    return "<link name='%s_link'>%s%s%s</link>" % (name, inertial, col, vis)


def model(name, pose, body, static=True):
    x, y, z = pose
    return "<model name='%s'><static>%s</static><pose>%.4f %.4f %.4f 0 0 0</pose>%s</model>" % (
        name, "true" if static else "false", x, y, z, body)


def build(seed):
    rng = random.Random(seed)
    parts = []

    # walls: 3 m x 3 m interior, 0.4 m high
    t = 0.02
    for name, pos, size in (
        ("wall_n", (0, HALF + t / 2, WALL_H / 2), (2 * HALF + 2 * t, t, WALL_H)),
        ("wall_s", (0, -HALF - t / 2, WALL_H / 2), (2 * HALF + 2 * t, t, WALL_H)),
        ("wall_e", (HALF + t / 2, 0, WALL_H / 2), (t, 2 * HALF, WALL_H)),
        ("wall_w", (-HALF - t / 2, 0, WALL_H / 2), (t, 2 * HALF, WALL_H)),
    ):
        parts.append(model(name, pos, link(name, size, (0.95, 0.95, 0.95))))

    # floor zones (visual only): 1 m x 1 m transport, 0.45 m collection in the corner
    zones = {}
    for z, (sx, sy) in ZONES.items():
        tc = YELLOW if z in "AD" else GREEN
        parts.append(model("zone_%s" % z, (sx * 1.0, sy * 1.0, 0.0005),
                           link("zone_%s" % z, (1.0, 1.0, 0.001), tc, collision=False)))
        cc = (sx * (HALF - 0.225), sy * (HALF - 0.225))
        parts.append(model("collect_%s" % z, (cc[0], cc[1], 0.0015),
                           link("collect_%s" % z, (0.45, 0.45, 0.001), RED, collision=False)))
        zones[z] = {"transport_center": [sx * 1.0, sy * 1.0], "collection_center": list(cc)}

    # sorting table on the west wall
    parts.append(model("sort_table", (TABLE["x"], TABLE["y"], TABLE["sz"] / 2),
                       link("sort_table", (TABLE["sx"], TABLE["sy"], TABLE["sz"]), (0.75, 0.75, 0.75))))

    # board: three types read left -> right (assumes robot faces west at the table,
    # so left = south = -y). Left-to-right maps to zones B, C, D.
    board = ["doll", "clock", "banana"]
    rng.shuffle(board)
    for i, ty in enumerate(board):
        parts.append(model("board_%d" % i, (TABLE["x"], (i - 1) * 0.12, TABLE["sz"] + 0.001),
                           link("board_%d" % i, (0.10, 0.10, 0.002), COLORS[ty], collision=False)))

    # centre brick grid: 3 doll, 3 clock, 3 banana; centre cell is a fixed clock; cake on top
    cells = [(r, c) for r in range(3) for c in range(3) if (r, c) != (1, 1)]
    types = ["doll"] * 3 + ["clock"] * 2 + ["banana"] * 3
    rng.shuffle(types)
    bricks = [{"name": "brick_1_1", "type": "clock", "row": 1, "col": 1, "layer": 1,
               "x": 0.0, "y": 0.0}]
    for (r, c), ty in zip(cells, types):
        bricks.append({"name": "brick_%d_%d" % (r, c), "type": ty, "row": r, "col": c,
                       "layer": 1, "x": (c - 1) * PITCH, "y": (1 - r) * PITCH})
    bricks.append({"name": "cake", "type": "cake", "row": 1, "col": 1, "layer": 2,
                   "x": 0.0, "y": 0.0})
    for b in bricks:
        z = BRICK / 2 + (BRICK + 0.0005 if b["layer"] == 2 else 0.0)
        parts.append(model(b["name"], (b["x"], b["y"], z),
                           link(b["name"], (BRICK, BRICK, BRICK), COLORS[b["type"]],
                                mass=0.05, friction=True), static=False))

    world = (
        "<?xml version='1.0' ?>\n<sdf version='1.6'>\n<world name='spark_arena'>\n"
        "<include><uri>model://sun</uri></include>\n"
        "<include><uri>model://ground_plane</uri></include>\n"
        "<gui><camera name='user_camera'><pose>0 0 5 0 1.5708 0</pose></camera></gui>\n"
        + "\n".join(parts) + "\n</world>\n</sdf>\n")

    pts = {
        "Classification_area": [-0.90, 0.0],
        "Sorting_N": [0.0, 0.38], "Sorting_W": [-0.38, 0.0],
        "Sorting_S": [0.0, -0.38], "Sorting_E": [0.38, 0.0],
    }
    for z, (sx, sy) in ZONES.items():
        # Red collection square centre (0.45 x 0.45 m in the corner).
        cx, cy = sx * (HALF - 0.225), sy * (HALF - 0.225)
        pts["Collection_%s" % z] = [cx, cy]
        # Chassis stand-off for the arm: 0.35 m inwards from the square centre.
        # Placing 0.30 m ahead of the robot from here lands inside the red square.
        k = 0.35 / math.sqrt(2.0)
        pts["Collection_%s_approach" % z] = [cx - sx * k, cy - sy * k]
    scenario = {
        "seed": seed,
        "bricks": bricks,
        "board_left_to_right": board,
        "board_zones": ["B", "C", "D"],
        "zones": zones,
        "start_pose": {"x": -HALF + 0.225, "y": HALF - 0.225, "yaw": -1.5708},
        "points": pts,
        "note": "points are approximate stand-ins; real ones come from your mapping step",
    }
    return world, scenario


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=".")
    a = ap.parse_args()
    w, s = build(a.seed)
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "arena.world"), "w") as f:
        f.write(w)
    with open(os.path.join(a.out, "scenario.json"), "w") as f:
        json.dump(s, f, indent=2)
    print("wrote arena.world and scenario.json (seed %d, board %s)" % (a.seed, s["board_left_to_right"]))
