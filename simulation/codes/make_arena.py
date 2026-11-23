#!/usr/bin/env python3
"""Generate arena.world (Gazebo 11 / SDF 1.6) and scenario.json from arena_map.yaml.

`config/arena_map.yaml` is the single source of truth for the arena: 3x3 m,
frame origin at the SW corner, A top-left, B top-right, C bottom-right, D bottom-left,
classification table on the left wall between A and D.

Usage:
    python3 make_arena.py                 # reads ../config/arena_map.yaml
    python3 make_arena.py --out .         # write here
    python3 make_arena.py --seed 3        # accepted for compatibility; map is fixed

The seed no longer changes brick types: the map lists them explicitly so simulation
and field runs describe the same layout. Use `--randomize-types` to shuffle the nine
first-tier types while keeping their counts.
"""

import argparse
import json
import os
import random

import yaml

BRICK = 0.10
YELLOW, GREEN, RED = (1.0, 0.75, 0.0), (0.4, 1.0, 0.6), (0.8, 0.0, 0.0)
COLORS = {"doll": (0.55, 0.35, 0.2), "clock": (0.85, 0.85, 0.85),
          "banana": (0.95, 0.85, 0.15), "cake": (0.9, 0.3, 0.4)}


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


def build(map_data, randomize_types=False):
    arena = map_data["arena"]
    x0, y0 = arena["origin"]
    width, height = arena["size"]
    wall_h = arena.get("wall_height", 0.4)
    parts = []

    # walls just outside the arena rectangle
    t = 0.02
    for name, pos, size in (
        ("wall_n", (x0 + width / 2, y0 + height + t / 2, wall_h / 2), (width + 2 * t, t, wall_h)),
        ("wall_s", (x0 + width / 2, y0 - t / 2, wall_h / 2), (width + 2 * t, t, wall_h)),
        ("wall_e", (x0 + width + t / 2, y0 + height / 2, wall_h / 2), (t, height, wall_h)),
        ("wall_w", (x0 - t / 2, y0 + height / 2, wall_h / 2), (t, height, wall_h)),
    ):
        parts.append(model(name, pos, link(name, size, (0.95, 0.95, 0.95))))

    # 1x1 transport squares and 0.45 m red collection squares
    zones = {}
    for zone, info in map_data["zones"].items():
        tc = info["transport_center"]
        cc = info["collection_center"]
        fill = YELLOW if zone in ("A", "D") else GREEN
        parts.append(model("zone_%s" % zone, (tc[0], tc[1], 0.0005),
                           link("zone_%s" % zone, (1.0, 1.0, 0.001), fill, collision=False)))
        parts.append(model("collect_%s" % zone, (cc[0], cc[1], 0.0015),
                           link("collect_%s" % zone, (0.45, 0.45, 0.001), RED,
                                collision=False)))
        zones[zone] = {"transport_center": list(tc), "collection_center": list(cc)}

    # classification table
    table = map_data["table"]
    tx, ty = table["center"]
    sx, sy = table["size"]
    th = table["height"]
    parts.append(model("sort_table", (tx, ty, th / 2),
                       link("sort_table", (sx, sy, th), (0.75, 0.75, 0.75))))

    # board cards on the table top, left -> right for a robot facing west
    board = list(map_data["board_left_to_right"])
    cards_y = map_data["board"]["cards_y"]
    for i, (ty_card, cy) in enumerate(zip(board, cards_y)):
        parts.append(model("board_%d" % i, (tx, cy, th + 0.001),
                           link("board_%d" % i, (0.10, 0.10, 0.002),
                                COLORS.get(ty_card, (0.5, 0.5, 0.5)), collision=False)))

    # centre bricks (fixed types from the map, or shuffled first-tier types)
    bricks = [dict(b) for b in map_data["bricks"]]
    if randomize_types:
        first = [b for b in bricks if b.get("layer") == 1 and (b["row"], b["col"]) != (1, 1)]
        types = [b["type"] for b in first]
        random.shuffle(types)
        for b, ty in zip(first, types):
            b["type"] = ty

    for b in bricks:
        z = 0.05 + (0.1005 if b.get("layer") == 2 else 0.0)
        parts.append(model(b["name"], (b["x"], b["y"], z),
                           link(b["name"], (BRICK, BRICK, BRICK),
                                COLORS[b["type"]], mass=0.05, friction=True),
                           static=False))

    world = (
        "<?xml version='1.0' ?>\n<sdf version='1.6'>\n<world name='spark_arena'>\n"
        "<include><uri>model://sun</uri></include>\n"
        "<include><uri>model://ground_plane</uri></include>\n"
        "<gui><camera name='user_camera'><pose>%.2f %.2f 5 0 1.5708 0</pose></camera></gui>\n"
        % (x0 + width / 2, y0 + height / 2)
        + "\n".join(parts) + "\n</world>\n</sdf>\n")

    scenario = {
        "frame": map_data.get("frame", "arena"),
        "arena": arena,
        "table": table,
        "bricks": bricks,
        "board_left_to_right": board,
        "board_zones": list(map_data["board_zones"]),
        "cluster_center": list(map_data.get(
            "cluster_center", [x0 + width / 2.0, y0 + height / 2.0])),
        "zones": zones,
        "points": {k: list(v) for k, v in map_data["points"].items()},
        "start_pose": dict(map_data["start_pose"]),
        "note": "generated from config/arena_map.yaml; origin at the arena SW corner",
    }
    return world, scenario


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map", default=os.path.join(here, "..", "config", "arena_map.yaml"))
    ap.add_argument("--seed", type=int, default=3, help="kept for compatibility")
    ap.add_argument("--randomize-types", action="store_true",
                    help="shuffle the nine first-tier brick types")
    ap.add_argument("--out", default=".")
    args = ap.parse_args()

    with open(args.map) as f:
        map_data = yaml.safe_load(f)

    world, scenario = build(map_data, randomize_types=args.randomize_types)
    scenario["seed"] = args.seed
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "arena.world"), "w") as f:
        f.write(world)
    with open(os.path.join(args.out, "scenario.json"), "w") as f:
        json.dump(scenario, f, indent=2)
    print("wrote arena.world and scenario.json from %s" % os.path.basename(args.map))
    print("board L->R: %s   start: %s" % (
        scenario["board_left_to_right"], scenario["start_pose"]))


if __name__ == "__main__":
    main()
