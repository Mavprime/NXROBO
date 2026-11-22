#!/usr/bin/env python3
"""Generate a map_server occupancy grid (PGM + YAML) from arena_map.yaml.

The arena is fixed, so a hand-built 3x3 m static map is more repeatable than gmapping:
the map frame is the arena frame (origin at the SW corner), resolution 0.05 m.

    python3 static_map.py                 # -> maps/arena_map.{pgm,yaml,png}
    python3 static_map.py --include-cubes # also mark the brick cluster (static test)

Walls and the classification table are occupied. Cubes are NOT marked by default: they
are picked up and moved, so the local costmap/lidar should treat them as dynamic.
"""

from __future__ import annotations

import argparse
import os

import yaml

FREE = 254
OCCUPIED = 0


def world_to_cell(map_data: dict, x: float, y: float, resolution: float,
                  width: int, height: int) -> tuple[int, int]:
    """Return (row, col); row 0 is the top of the PGM (max y)."""
    ox, oy = map_data["arena"]["origin"]
    col = int((x - ox) / resolution)
    row = height - 1 - int((y - oy) / resolution)
    return row, col


def build_grid(map_data: dict, resolution: float = 0.05,
               include_cubes: bool = False, wall_cells: int = 1):
    arena = map_data["arena"]
    ox, oy = arena["origin"]
    sx, sy = arena["size"]
    width = int(round(sx / resolution))
    height = int(round(sy / resolution))
    grid = [[FREE] * width for _ in range(height)]

    def fill_rect(x0: float, y0: float, x1: float, y1: float) -> None:
        c0 = max(0, int((x0 - ox) / resolution))
        c1 = min(width - 1, int((x1 - ox) / resolution + 0.999))
        r0 = max(0, height - 1 - int((y1 - oy) / resolution))
        r1 = min(height - 1, height - 1 - int((y0 - oy) / resolution))
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                grid[r][c] = OCCUPIED

    # arena walls (the SDF wall is 2 cm; one 5 cm cell is a safe static footprint)
    for k in range(wall_cells):
        for c in range(width):
            grid[k][c] = OCCUPIED
            grid[height - 1 - k][c] = OCCUPIED
        for r in range(height):
            grid[r][k] = OCCUPIED
            grid[r][width - 1 - k] = OCCUPIED

    table = map_data.get("table")
    if table:
        cx, cy = table["center"]
        tx, ty = table["size"]
        fill_rect(cx - tx / 2, cy - ty / 2, cx + tx / 2, cy + ty / 2)

    if include_cubes:
        for brick in map_data.get("bricks", []):
            fill_rect(brick["x"] - 0.05, brick["y"] - 0.05,
                      brick["x"] + 0.05, brick["y"] + 0.05)

    return grid, width, height


def write_pgm(path: str, grid: list[list[int]], width: int, height: int) -> None:
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (width, height))
        f.write(bytes(value for row in grid for value in row))


def write_yaml(path: str, image_name: str, map_data: dict,
               resolution: float) -> None:
    ox, oy = map_data["arena"]["origin"]
    data = {
        "image": image_name,
        "resolution": resolution,
        "origin": [float(ox), float(oy), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with open(path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def write_preview(path: str, grid: list[list[int]], map_data: dict,
                  resolution: float) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    ox, oy = map_data["arena"]["origin"]
    sx, sy = map_data["arena"]["size"]
    fig, ax = plt.subplots(figsize=(6.4, 6.4))
    ax.imshow(grid, cmap="gray", origin="upper",
              extent=[ox, ox + sx, oy, oy + sy], vmin=0, vmax=255)
    for zone, info in map_data.get("zones", {}).items():
        cc = info["collection_center"]
        ax.add_patch(Rectangle((cc[0] - 0.225, cc[1] - 0.225), 0.45, 0.45,
                               fill=False, edgecolor="#d62728", lw=1.5))
        ax.text(info["transport_center"][0], info["transport_center"][1], "Zone " + zone,
                ha="center", va="center", fontsize=8,
                bbox=dict(fc="white", ec="none", alpha=0.6))
    for name, p in map_data.get("points", {}).items():
        if name.endswith("_approach"):
            continue
        ax.plot(p[0], p[1], "o", ms=3, color="#1f77b4")
        ax.text(p[0], p[1] + 0.05, name, fontsize=6, color="#1f77b4", ha="center")
    sp = map_data.get("start_pose") or {}
    if sp:
        ax.plot(sp["x"], sp["y"], "o", ms=6, color="#ff7f0e", mec="k")
    ax.set_xlim(ox, ox + sx)
    ax.set_ylim(oy, oy + sy)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("arena occupancy map (%.2f m/cell)" % resolution)
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map", default=os.path.join(here, "..", "config", "arena_map.yaml"))
    ap.add_argument("--out", default=os.path.join(here, "..", "maps", "arena_map"))
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--include-cubes", action="store_true")
    ap.add_argument("--no-preview", action="store_true")
    args = ap.parse_args()

    with open(args.map) as f:
        map_data = yaml.safe_load(f)

    grid, width, height = build_grid(map_data, args.resolution, args.include_cubes)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    write_pgm(args.out + ".pgm", grid, width, height)
    write_yaml(args.out + ".yaml", os.path.basename(args.out) + ".pgm",
               map_data, args.resolution)
    print("wrote %s.pgm (%dx%d cells), %s.yaml" % (args.out, width, height, args.out))
    if not args.no_preview:
        write_preview(args.out + ".png", grid, map_data, args.resolution)
        print("wrote %s.png" % args.out)


if __name__ == "__main__":
    main()
