#!/usr/bin/env python3
"""Level 0B: continuous 2D world with a lidar, camera detections and a mocked arm.

This is the spatial simulation the agreement gate needs: the robot has a continuous
pose, the lidar sees walls/table/bricks, and detections carry world positions. It still
has no ROS and no Gazebo — it is a deterministic physics-lite world for algorithm work.

`SpatialPlatform` implements the mission_logic platform protocol on top of it, so the
same mission file and the same executor run here and against the discrete mock.

Coordinates: `map` frame, metres; arena is 3 x 3 m centred on (0, 0).
"""

from __future__ import annotations

import math
import random
import threading
import time
from dataclasses import dataclass, fields
from typing import Optional

from mission_logic import (
    Detection,
    PickResult,
    PlaceResult,
    WorldModel,
    canonical_class,
)
from planner import plan_path

ROBOT_RADIUS = 0.12
FOV = math.radians(90.0)
CAM_RANGE = 1.60
LIDAR_RANGE = 3.5
STEP_DT = 0.05


@dataclass
class RobotParams:
    """Measured Spark parameters; defaults match config/robot.yaml."""

    max_linear_mps: float = 0.18       # runtask.py drive_base()
    max_angular_rps: float = 0.75      # runtask.py turn_base()
    lin_accel_mps2: float = 3.0        # keypatched2v.py MAX_LIN_ACCEL
    lin_decel_mps2: float = 5.0        # keypatched2v.py MAX_LIN_DECEL
    ang_accel_rps2: float = 3.0        # keypatched2v.py MAX_ANG_ACCEL
    ang_decel_rps2: float = 8.0        # keypatched2v.py MAX_ANG_DECEL
    settle_s: float = 0.10
    arm_reach_m: float = 0.45          # uArm ~0.32 m + mount offset + margin
    pick_s: float = 4.0
    place_s: float = 3.0
    frame_period_s: float = 0.10
    scan_timeout_s: float = 3.0

    @classmethod
    def from_yaml(cls, path: str) -> "RobotParams":
        import yaml

        with open(path) as f:
            data = yaml.safe_load(f) or {}
        flat: dict = {}
        for section in ("chassis", "arm", "perception"):
            flat.update(data.get(section) or {})
        known = {f.name for f in fields(cls)}
        return cls(**{k: float(v) for k, v in flat.items() if k in known})


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def ramp_toward(current: float, target: float, accel: float, decel: float,
                dt: float) -> float:
    """Rate-limit a velocity toward its target, with separate accel/decel."""
    if current * target < 0.0:
        step = decel * dt
        return 0.0 if abs(current) <= step else current - math.copysign(step, current)
    rate = accel if abs(target) >= abs(current) else decel
    diff = target - current
    if abs(diff) <= rate * dt:
        return target
    return current + math.copysign(rate * dt, diff)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def circle_aabb(cx: float, cy: float, r: float, box: tuple[float, float, float, float]) -> bool:
    x0, y0, x1, y1 = box
    nx = clamp(cx, x0, x1)
    ny = clamp(cy, y0, y1)
    return math.hypot(cx - nx, cy - ny) < r


@dataclass
class Cube:
    name: str
    cls: str
    x: float
    y: float
    layer: int = 1
    status: str = "field"        # field | carried
    delivered: bool = False

    def __post_init__(self) -> None:
        self.cls = canonical_class(self.cls)


class FakeWorld:
    """Continuous 2D world: differential drive, raycast lidar, camera, arm mock."""

    def __init__(self, scenario: dict, seed: int = 0, speed: float = 4.0,
                 real_time: bool = True, params: Optional[RobotParams] = None) -> None:
        self.sc = scenario
        self.rng = random.Random(seed)
        self.speed = max(0.1, float(speed))
        self.real_time = real_time
        self.p = params or RobotParams()

        start = scenario["start_pose"]
        self.start = (float(start["x"]), float(start["y"]))
        self.x = self.start[0]
        self.y = self.start[1]
        self.yaw = float(start["yaw"])
        self.v = 0.0
        self.w = 0.0
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.t = 0.0
        self.trail = [(self.x, self.y)]
        self.collisions = 0
        self.left_start = False

        self.points = {k: (float(v[0]), float(v[1])) for k, v in scenario["points"].items()}
        self.zones = scenario["zones"]

        arena = scenario.get("arena") or {}
        self.min_x, self.min_y = (float(v) for v in arena.get("origin", [0.0, 0.0]))
        size = arena.get("size", [3.0, 3.0])
        self.max_x = self.min_x + float(size[0])
        self.max_y = self.min_y + float(size[1])
        cluster = scenario.get("cluster_center")
        self.cluster = (tuple(float(v) for v in cluster) if cluster else
                        ((self.min_x + self.max_x) / 2.0,
                         (self.min_y + self.max_y) / 2.0))

        table = scenario.get("table")
        if table:
            cx, cy = table["center"]
            sx, sy = table["size"]
            self.table = (cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2)
        else:
            self.table = None

        self.cubes = [
            Cube(b["name"], b["type"], float(b["x"]), float(b["y"]), int(b.get("layer", 1)))
            for b in scenario["bricks"]
        ]
        self.carried: Optional[Cube] = None

    # -- state --------------------------------------------------------------

    def set_cmd(self, v: float, w: float) -> None:
        self.v_cmd = clamp(float(v), -self.p.max_linear_mps, self.p.max_linear_mps)
        self.w_cmd = clamp(float(w), -self.p.max_angular_rps, self.p.max_angular_rps)

    def stop(self) -> None:
        """Command zero and let the deceleration ramp bring the robot to rest."""
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        for _ in range(10):
            if abs(self.v) < 1e-3 and abs(self.w) < 1e-3:
                break
            self.advance(STEP_DT)

    def pose(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)

    def front_point(self, distance: float = 0.30) -> tuple[float, float]:
        return (self.x + distance * math.cos(self.yaw),
                self.y + distance * math.sin(self.yaw))

    def advance(self, seconds: float) -> None:
        steps = max(1, int(round(seconds / STEP_DT)))
        dt = seconds / steps
        for _ in range(steps):
            self.step(dt)
            if self.real_time:
                time.sleep(dt / self.speed)

    def step(self, dt: float) -> None:
        self.t += dt
        self.v = ramp_toward(self.v, self.v_cmd, self.p.lin_accel_mps2,
                             self.p.lin_decel_mps2, dt)
        self.w = ramp_toward(self.w, self.w_cmd, self.p.ang_accel_rps2,
                             self.p.ang_decel_rps2, dt)
        nx = self.x + self.v * math.cos(self.yaw) * dt
        ny = self.y + self.v * math.sin(self.yaw) * dt
        nyaw = wrap(self.yaw + self.w * dt)
        nx = clamp(nx, self.min_x + ROBOT_RADIUS, self.max_x - ROBOT_RADIUS)
        ny = clamp(ny, self.min_y + ROBOT_RADIUS, self.max_y - ROBOT_RADIUS)

        for box in self.nav_obstacles():
            if circle_aabb(nx, ny, ROBOT_RADIUS, box):
                self.v = 0.0
                self.w = 0.0
                self.v_cmd = 0.0
                self.w_cmd = 0.0
                self.collisions += 1
                return

        self.x, self.y, self.yaw = nx, ny, nyaw
        if math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 0.025:
            self.trail.append((self.x, self.y))
        if not self.left_start and math.hypot(self.x - self.start[0],
                                              self.y - self.start[1]) > 0.25:
            self.left_start = True

    # -- sensing ------------------------------------------------------------

    def nav_obstacles(self) -> list[tuple[float, float, float, float]]:
        """Physical obstacles for navigation: table + field cubes (not carried)."""
        boxes = []
        if self.table is not None:
            boxes.append(self.table)
        for cube in self.cubes:
            if cube.status == "field":
                boxes.append((cube.x - 0.05, cube.y - 0.05,
                              cube.x + 0.05, cube.y + 0.05))
        return boxes

    def _obstacles(self) -> list[tuple[float, float, float, float]]:
        boxes = self.nav_obstacles()
        boxes += [
            (self.min_x, self.min_y - 0.05, self.max_x, self.min_y),
            (self.min_x, self.max_y, self.max_x, self.max_y + 0.05),
            (self.min_x - 0.05, self.min_y, self.min_x, self.max_y),
            (self.max_x, self.min_y, self.max_x + 0.05, self.max_y),
        ]
        for cube in self.cubes:
            if cube.status == "field" and cube.layer == 1:
                boxes.append((cube.x - 0.05, cube.y - 0.05, cube.x + 0.05, cube.y + 0.05))
        return boxes

    @staticmethod
    def _ray_aabb(ox: float, oy: float, dx: float, dy: float,
                  box: tuple[float, float, float, float]) -> Optional[float]:
        x0, y0, x1, y1 = box
        tmin, tmax = 0.0, float("inf")
        for origin, direction, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
            if abs(direction) < 1e-9:
                if origin < lo or origin > hi:
                    return None
            else:
                t1 = (lo - origin) / direction
                t2 = (hi - origin) / direction
                if t1 > t2:
                    t1, t2 = t2, t1
                tmin = max(tmin, t1)
                tmax = min(tmax, t2)
                if tmin > tmax:
                    return None
        return tmin if tmin > 1e-6 else None

    def lidar_points(self, beams: int = 160) -> list[tuple[float, float]]:
        obstacles = self._obstacles()
        points = []
        for i in range(beams):
            angle = self.yaw + 2.0 * math.pi * i / beams
            dx, dy = math.cos(angle), math.sin(angle)
            best = LIDAR_RANGE
            for box in obstacles:
                t = self._ray_aabb(self.x, self.y, dx, dy, box)
                if t is not None and t < best:
                    best = t
            points.append((self.x + dx * best, self.y + dy * best))
        return points

    def detections(self) -> list[Detection]:
        out = []
        for cube in self.cubes:
            if cube.status != "field":
                continue
            dx, dy = cube.x - self.x, cube.y - self.y
            dist = math.hypot(dx, dy)
            if dist > CAM_RANGE:
                continue
            if abs(wrap(math.atan2(dy, dx) - self.yaw)) > FOV / 2.0:
                continue
            if self.rng.random() < 0.06:          # frame dropout
                continue
            conf = clamp(self.rng.gauss(0.85, 0.07), 0.05, 0.99)
            out.append(Detection(cube.cls, conf,
                                 cube.x + self.rng.gauss(0.0, 0.010),
                                 cube.y + self.rng.gauss(0.0, 0.010)))
        return out

    # -- arm mock -----------------------------------------------------------

    def pick_at_reach(self, cls: str) -> tuple[bool, str, Optional[tuple[float, float]]]:
        ccls = canonical_class(cls)
        candidates = [
            c for c in self.cubes
            if c.status == "field" and c.cls == ccls
            and math.hypot(c.x - self.x, c.y - self.y) <= self.p.arm_reach_m
        ]
        if not candidates:
            return False, "", None
        cube = min(candidates, key=lambda c: math.hypot(c.x - self.x, c.y - self.y))
        pose = (cube.x, cube.y)
        cube.status = "carried"
        self.carried = cube
        return True, cube.cls, pose

    def place_front(self, level: int = 1) -> bool:
        cube = self.carried
        if cube is None:
            return False
        px, py = self.front_point(0.30)
        cube.x = clamp(px, self.min_x + 0.05, self.max_x - 0.05)
        cube.y = clamp(py, self.min_y + 0.05, self.max_y - 0.05)
        cube.layer = max(1, int(level))
        cube.status = "field"
        cube.delivered = True
        self.carried = None
        return True

    # -- scoring ------------------------------------------------------------

    def score(self, mapping: dict[str, str]) -> tuple[int, list[tuple]]:
        points = 1 if self.left_start else 0
        detail: list[tuple] = []
        for cube in self.cubes:
            if not cube.delivered:
                continue
            zone = None
            in_collection = False
            for name, info in self.zones.items():
                cc = info["collection_center"]
                tc = info["transport_center"]
                if abs(cube.x - cc[0]) <= 0.225 and abs(cube.y - cc[1]) <= 0.225:
                    zone, in_collection = name, True
                    break
                if abs(cube.x - tc[0]) <= 0.5 and abs(cube.y - tc[1]) <= 0.5:
                    zone, in_collection = name, False
            if zone is None:
                detail.append((None, cube.cls, 0))
                continue
            correct = mapping.get(cube.cls) == zone
            if cube.cls == "cake" and zone != "A":
                correct = False
            if not correct:
                detail.append((zone, cube.cls, 0))
                continue
            layer = max(1, cube.layer)
            pts = (layer + 2 ** (layer - 1)) if in_collection else 2 ** (layer - 1)
            points += pts
            detail.append((zone, cube.cls, pts))
        return points, detail

    def snapshot(self, mapping: dict[str, str], extra: Optional[dict] = None) -> dict:
        cubes = []
        for cube in self.cubes:
            x, y = cube.x, cube.y
            if cube.status == "carried":
                x, y = self.front_point(0.15)
            cubes.append({"x": x, "y": y, "cls": cube.cls,
                          "status": cube.status, "delivered": cube.delivered})
        score, detail = self.score(mapping)
        state = {
            "t": round(self.t, 2),
            "pose": [self.x, self.y, self.yaw],
            "lidar": self.lidar_points(120),
            "trail": self.trail[-1500:],
            "cubes": cubes,
            "score": score,
            "detail": detail,
            "collisions": self.collisions,
            "left_start": self.left_start,
            "carried": self.carried.name if self.carried else None,
            "speed": self.speed,
            "vel": [self.v, self.w],
            "max_linear": self.p.max_linear_mps,
            "max_angular": self.p.max_angular_rps,
        }
        if extra:
            state.update(extra)
        return state


# ---------------------------------------------------------------------------
# Mission platform
# ---------------------------------------------------------------------------


class SpatialPlatform:
    """mission_logic protocol on FakeWorld: drives, scans, grabs, places."""

    def __init__(self, world: FakeWorld, world_model: WorldModel,
                 stop_event: Optional[threading.Event] = None,
                 total_seconds: float = 600.0) -> None:
        self.world = world
        self.wm = world_model
        self.stop_event = stop_event or threading.Event()
        self.total_seconds = total_seconds

    # -- helpers ------------------------------------------------------------

    def stop_requested(self) -> bool:
        return self.stop_event.is_set()

    def now(self) -> float:
        return self.world.t

    def _sleep_sim(self, seconds: float) -> None:
        remaining = seconds
        while remaining > 0.0 and not self.stop_requested():
            chunk = min(0.1, remaining)
            self.world.set_cmd(0.0, 0.0)
            self.world.advance(chunk)
            remaining -= chunk

    def _target_yaw(self, name: str, target: tuple[float, float]) -> Optional[float]:
        if name.startswith("Collection_"):
            zone = name.split("_")[1]
            centre = self.wm.zone_center(zone)
            if centre is not None:
                return math.atan2(centre[1] - target[1], centre[0] - target[0])
        if name.startswith("Sorting_"):
            cx, cy = self.world.cluster                      # face the bricks
            return math.atan2(cy - target[1], cx - target[0])
        if name == "Classification_area":
            return math.pi                            # face the table (west)
        return None

    # -- navigation ---------------------------------------------------------

    def _drive_to(self, tx: float, ty: float, tol: float, deadline: float,
                  desired_yaw: Optional[float] = None) -> bool:
        """Drive to one waypoint; optionally rotate to the final heading."""
        while self.world.t < deadline and not self.stop_requested():
            dx, dy = tx - self.world.x, ty - self.world.y
            dist = math.hypot(dx, dy)
            if dist <= tol:
                if desired_yaw is None:
                    break
                error = wrap(desired_yaw - self.world.yaw)
                if abs(error) <= 0.10:
                    break
                self.world.set_cmd(0.0, clamp(1.5 * error,
                                              -self.world.p.max_angular_rps,
                                              self.world.p.max_angular_rps))
                self.world.advance(STEP_DT)
                continue
            heading_error = wrap(math.atan2(dy, dx) - self.world.yaw)
            if abs(heading_error) > 0.60:
                v, w = 0.0, clamp(1.5 * heading_error,
                                 -self.world.p.max_angular_rps,
                                 self.world.p.max_angular_rps)
            else:
                v = min(self.world.p.max_linear_mps, max(0.05, 1.2 * dist))
                w = clamp(1.60 * heading_error,
                          -self.world.p.max_angular_rps,
                          self.world.p.max_angular_rps)
            self.world.set_cmd(v, w)
            self.world.advance(STEP_DT)

        self.world.set_cmd(0.0, 0.0)
        if desired_yaw is not None:
            end = min(deadline, self.world.t + 5.0)
            while self.world.t < end and not self.stop_requested():
                error = wrap(desired_yaw - self.world.yaw)
                if abs(error) <= 0.10:
                    break
                self.world.set_cmd(0.0, clamp(1.5 * error,
                                              -self.world.p.max_angular_rps,
                                              self.world.p.max_angular_rps))
                self.world.advance(STEP_DT)
        self.world.stop()
        self.world.advance(self.world.p.settle_s)

        dist = math.hypot(tx - self.world.x, ty - self.world.y)
        yaw_ok = (desired_yaw is None
                  or abs(wrap(desired_yaw - self.world.yaw)) <= 0.15)
        return dist <= tol + 0.03 and yaw_ok

    def goto(self, point: str) -> bool:
        target = self.world.points.get(point)
        if target is None:
            return False
        tx, ty = target
        desired_yaw = self._target_yaw(point, target)
        deadline = self.world.t + 60.0

        path = plan_path(
            (self.world.x, self.world.y), (tx, ty),
            self.world.nav_obstacles(),
            (self.world.min_x, self.world.min_y,
             self.world.max_x, self.world.max_y),
            resolution=0.05, radius=ROBOT_RADIUS + 0.03,
        )
        if not path:
            return False

        for index, (px, py) in enumerate(path[1:], start=1):
            is_last = index == len(path) - 1
            if not self._drive_to(
                    px, py,
                    tol=0.05 if is_last else 0.06,
                    deadline=deadline,
                    desired_yaw=desired_yaw if is_last else None):
                return False
        return math.hypot(tx - self.world.x, ty - self.world.y) <= 0.10

    # -- perception ---------------------------------------------------------

    def scan_window(self, n: int):
        frames = []
        for _ in range(n):
            frames.append(self.world.detections())
            self._sleep_sim(self.world.p.frame_period_s)
        return frames

    def read_board(self):
        return list(self.world.sc.get("board_left_to_right") or [])

    # -- arm ----------------------------------------------------------------

    def pick(self, cls: str) -> PickResult:
        ok, confirmed, pose = self.world.pick_at_reach(cls)
        if ok:
            self._sleep_sim(self.world.p.pick_s)   # aligned with runtask.grasp()
            return PickResult(True, confirmed_class=confirmed, confirmed_pose=pose)
        return PickResult(False, reason="no %s within reach" % cls)

    def place(self, level: int) -> PlaceResult:
        ok = self.world.place_front(level)
        if ok:
            self._sleep_sim(self.world.p.place_s)  # aligned with runtask.drop()
            return PlaceResult(True)
        return PlaceResult(False, reason="nothing carried")

    # -- clock / recovery ---------------------------------------------------

    def time_left(self) -> float:
        return max(0.0, self.total_seconds - self.world.t)

    def wait(self, seconds: float) -> None:
        self._sleep_sim(seconds)

    def precheck(self):
        return True, ""

    def approachable(self, x: float, y: float) -> bool:
        return math.hypot(x - self.world.x, y - self.world.y) <= self.world.p.arm_reach_m

    def clear_costmaps(self) -> None:
        pass

    def rotate_scan(self) -> None:
        end = self.world.t + 1.5
        self.world.set_cmd(0.0, self.world.p.max_angular_rps)
        while self.world.t < end and not self.stop_requested():
            self.world.advance(STEP_DT)
        self.world.stop()

    def backup(self) -> None:
        end = self.world.t + 1.0
        self.world.set_cmd(-self.world.p.max_linear_mps, 0.0)
        while self.world.t < end and not self.stop_requested():
            self.world.advance(STEP_DT)
        self.world.stop()

    def reframe(self, station: Optional[str]) -> None:
        """G4 recovery: face the central bricks and creep 10 cm closer.

        At a sorting stand-off the goto can stop a few centimetres outside the
        arm reach; rotating in place (the old behaviour) turned the cube out of
        the 90 deg camera FOV and the next window had no votes. Facing the
        cluster and closing the last centimetres is what the real reframe does.
        """
        cx, cy = self.world.cluster
        yaw = math.atan2(cy - self.world.y, cx - self.world.x)
        deadline = self.world.t + 4.0
        while (self.world.t < deadline and not self.stop_requested()
               and abs(wrap(yaw - self.world.yaw)) > 0.05):
            err = wrap(yaw - self.world.yaw)
            self.world.set_cmd(0.0, max(-self.world.p.max_angular_rps,
                                        min(self.world.p.max_angular_rps, 1.5 * err)))
            self.world.advance(STEP_DT)
        sx, sy = self.world.x, self.world.y
        while (self.world.t < deadline and not self.stop_requested()
               and math.hypot(self.world.x - sx, self.world.y - sy) < 0.10):
            self.world.set_cmd(0.08, 0.0)
            self.world.advance(STEP_DT)
        self.world.stop()
