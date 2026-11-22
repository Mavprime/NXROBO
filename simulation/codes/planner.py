#!/usr/bin/env python3
"""Grid A* path planning for the kinematic world (ROS-free).

The arena is 3x3 m; a 0.05 m grid is 60x60 cells. Obstacles are axis-aligned boxes
(cube faces, table) inflated by the robot radius plus a margin. Paths are simplified
by line-of-sight so the go-to controller follows a few waypoints instead of every cell.
"""

from __future__ import annotations

import heapq
import math
from typing import Optional

Point = tuple[float, float]
Box = tuple[float, float, float, float]


def _inflate(box: Box, radius: float) -> Box:
    return (box[0] - radius, box[1] - radius, box[2] + radius, box[3] + radius)


def _inside(x: float, y: float, box: Box) -> bool:
    return box[0] <= x <= box[2] and box[1] <= y <= box[3]


def _clear_segment(a: Point, b: Point, boxes: list[Box],
                   bounds: tuple[float, float, float, float], radius: float) -> bool:
    x0, y0, x1, y1 = bounds
    dist = math.hypot(b[0] - a[0], b[1] - a[1])
    steps = max(2, int(dist / 0.02))
    for i in range(steps + 1):
        t = i / steps
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        if x < x0 + radius or x > x1 - radius or y < y0 + radius or y > y1 - radius:
            return False
        if any(_inside(x, y, box) for box in boxes):
            return False
    return True


def plan_path(start: Point, goal: Point, obstacles: list[Box],
              bounds: tuple[float, float, float, float],
              resolution: float = 0.05, radius: float = 0.15) -> Optional[list[Point]]:
    """Plan from start to goal around inflated AABBs. Returns waypoints or None."""
    x0, y0, x1, y1 = bounds
    width = max(1, int(round((x1 - x0) / resolution)))
    height = max(1, int(round((y1 - y0) / resolution)))
    boxes = [_inflate(b, radius) for b in obstacles]

    def world(c: int, r: int) -> Point:
        return (x0 + (c + 0.5) * resolution, y0 + (r + 0.5) * resolution)

    def cell(p: Point) -> tuple[int, int]:
        c = min(max(int((p[0] - x0) / resolution), 0), width - 1)
        r = min(max(int((p[1] - y0) / resolution), 0), height - 1)
        return c, r

    def free(c: int, r: int) -> bool:
        if not (0 <= c < width and 0 <= r < height):
            return False
        wx, wy = world(c, r)
        if wx < x0 + radius or wx > x1 - radius or wy < y0 + radius or wy > y1 - radius:
            return False
        return not any(_inside(wx, wy, box) for box in boxes)

    def find_free(c0: int, r0: int, limit: int = 14) -> Optional[tuple[int, int]]:
        if free(c0, r0):
            return c0, r0
        best = None
        for d in range(1, limit + 1):
            for c in range(max(0, c0 - d), min(width - 1, c0 + d) + 1):
                for r in (r0 - d, r0 + d):
                    if free(c, r):
                        dist = (c - c0) ** 2 + (r - r0) ** 2
                        if best is None or dist < best[0]:
                            best = (dist, c, r)
            for r in range(max(0, r0 - d), min(height - 1, r0 + d) + 1):
                for c in (c0 - d, c0 + d):
                    if free(c, r):
                        dist = (c - c0) ** 2 + (r - r0) ** 2
                        if best is None or dist < best[0]:
                            best = (dist, c, r)
            if best is not None:
                return best[1], best[2]
        return None

    start_cell = find_free(*cell(start))
    goal_cell = find_free(*cell(goal))
    if start_cell is None or goal_cell is None:
        return None

    neighbors = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                 (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)))

    def heuristic(c: int, r: int) -> float:
        dx, dy = abs(c - goal_cell[0]), abs(r - goal_cell[1])
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

    open_heap = [(heuristic(*start_cell), 0.0, start_cell)]
    counter = 1
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {start_cell: 0.0}

    while open_heap:
        _, g, cur = heapq.heappop(open_heap)
        if cur == goal_cell:
            break
        if g > cost.get(cur, float("inf")):
            continue
        c, r = cur
        for dc, dr, step in neighbors:
            nc, nr = c + dc, r + dr
            if not free(nc, nr):
                continue
            if dc and dr and (not free(c + dc, r) or not free(c, r + dr)):
                continue
            ng = g + step
            nxt = (nc, nr)
            if ng < cost.get(nxt, float("inf")):
                cost[nxt] = ng
                came_from[nxt] = cur
                heapq.heappush(open_heap, (ng + heuristic(nc, nr), ng, nxt))
                counter += 1

    if goal_cell not in cost:
        return None

    cells = [goal_cell]
    while cells[-1] != start_cell:
        cells.append(came_from[cells[-1]])
    cells.reverse()

    raw: list[Point] = [start]
    raw.extend(world(c, r) for c, r in cells[1:])
    goal_is_free = (not any(_inside(goal[0], goal[1], box) for box in boxes)
                    and x0 + radius <= goal[0] <= x1 - radius
                    and y0 + radius <= goal[1] <= y1 - radius)
    if goal_is_free:
        raw[-1] = goal

    simplified: list[Point] = [raw[0]]
    i = 0
    while i < len(raw) - 1:
        j = len(raw) - 1
        while j > i + 1 and not _clear_segment(raw[i], raw[j], boxes, bounds, radius):
            j -= 1
        simplified.append(raw[j])
        i = j
    return simplified
