#!/usr/bin/env python3
"""Live 2D "simulation" view + recorder for the REAL robot (ROS Noetic).

Same browser viewer as run_live.py (Level 0B), but the state comes from the
robot: /odom for the pose, /scan for the lidar. The map frame is resolved once
from the two nearest wall lines (the nearest-A hypothesis, so start it with the
robot at A), then odometry tracks it - no AMCL.

Every snapshot is also appended to a JSONL playback log, so the session can be
replayed/rendered as a simulation afterwards:

    python3 ~/codes/real_view.py                      # robot at A, port 8091
    python3 ~/codes/real_view.py --port 8091
    python3 ~/codes/real_view.py --anchor 0.225 2.775 -45   # explicit map pose
    python3 ~/codes/real_view.py --anchor 1.09 1.50 0       # e.g. already at W

Browser: http://<robot-ip>:8091/   (bind is 0.0.0.0)
Log:     ~/logs/real_view/view_<timestamp>.jsonl
Stop:    Ctrl+C, or the page's STOP button.

The cube layer is the scenario's initial layout; it does not update when a cube
is picked up. The bag remains the ground truth for the arm and camera.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import threading
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from live_view import ViewerServer

A_XY = (0.225, 2.775)
WALL_OFF = -0.175
MAP_TOP = 3.0 + 0.175


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _peak(P, theta_deg, min_count=30):
    n = np.array([math.cos(math.radians(theta_deg)),
                  math.sin(math.radians(theta_deg))])
    p = P @ n
    h, e = np.histogram(p, bins=np.arange(p.min(), p.max() + 0.01, 0.01))
    i = int(np.argmax(h))
    if h[i] < min_count:
        return None
    p0 = float(e[i] + 0.005)
    bearing = theta_deg if p0 >= 0 else (theta_deg + 180.0) % 360.0
    if bearing > 180.0:
        bearing -= 360.0
    return bearing, p0, int(h[i])


def _span(P, theta_deg, p0):
    n = np.array([math.cos(math.radians(theta_deg)),
                  math.sin(math.radians(theta_deg))])
    sel = np.abs(P @ n - p0) < 0.04
    if not np.any(sel):
        return 0.0
    t = P[sel] @ np.array([-n[1], n[0]])
    return float(t.max() - t.min())


def wall_lines(P):
    cands = []
    for th in np.arange(0.0, 180.0, 0.5):
        r = _peak(P, th)
        if r is not None:
            cands.append((r[2], r[0], r[1], th))
    cands.sort(reverse=True)
    fams = []
    for count, bearing, p0, th in cands:
        if any(abs(th - f[1]) < 8.0 and abs(abs(p0) - f[2]) < 0.08 for f in fams):
            continue
        span = _span(P, th, p0)
        if span < 1.2:
            continue
        fams.append((count, th, abs(p0), bearing, span))
        if len(fams) >= 6:
            break
    if not fams:
        return None
    f1 = fams[0]
    others = [f for f in fams[1:] if 60.0 <= abs(f[1] - f1[1]) <= 120.0]
    if not others:
        return [(f1[3], f1[2], f1[0])]
    f2 = max(others, key=lambda f: f[4])
    return [(f1[3], f1[2], f1[0]), (f2[3], f2[2], f2[0])]


def init_map_pose(P):
    """Two wall lines -> (x, y, heading_deg), disambiguated by nearest A."""
    lines = wall_lines(P)
    if not lines or len(lines) < 2:
        return None, "no pair of wall lines"

    def hyp(role, b, d):
        x = y = None
        if role == "west":
            h, x = 180.0 - b, WALL_OFF + d
        elif role == "east":
            h, x = -b, MAP_TOP - d
        elif role == "south":
            h, y = 270.0 - b, WALL_OFF + d
        else:
            h, y = 90.0 - b, MAP_TOP - d
        return h, x, y

    cands = []
    for rx in ("west", "east"):
        for ry in ("south", "north"):
            for r1, r2 in ((rx, ry), (ry, rx)):
                b1, d1, _ = lines[0]
                b2, d2, _ = lines[1]
                h1, x1, y1 = hyp(r1, b1, d1)
                h2, x2, y2 = hyp(r2, b2, d2)
                cons = abs(((h2 - h1 + 180.0) % 360.0) - 180.0)
                if cons > 20.0:
                    continue
                x = x1 if x1 is not None else x2
                y = y1 if y1 is not None else y2
                err = math.hypot(x - A_XY[0], y - A_XY[1])
                hm = h1 + math.degrees(wrap(math.radians(h2 - h1))) / 2.0
                cands.append((err, x, y, hm % 360.0))
    if not cands:
        return None, "corner hypotheses inconsistent"
    cands.sort()
    return cands[0][1:], "%.1f cm from A" % (100 * cands[0][0])


class RealView:
    def __init__(self, scenario: dict, anchor, port: int, log_path: str) -> None:
        rospy.init_node("real_view")
        self.scenario = scenario
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.snap = {}
        self.trail = []
        self.scan_r = None
        self.scan_a = None
        self.odom = None
        self.yaw0 = None
        self.odom0 = None
        self.map0 = None
        self.yaw0_deg = None
        self.t0 = time.time()
        self.fh = open(log_path, "w") if log_path else None
        if self.fh:
            self.fh.write(json.dumps({"scenario": scenario.get("arena"),
                                      "cubes": scenario.get("bricks", []),
                                      "start": anchor}) + "\n")

        rospy.Subscriber("/scan", LaserScan, self._on_scan)
        rospy.Subscriber("/odom", Odometry, self._on_odom)

        t0 = time.time()
        while (self.scan_r is None or self.odom is None) and time.time() - t0 < 15:
            rospy.sleep(0.1)
        if self.scan_r is None or self.odom is None:
            raise RuntimeError("no /scan or /odom")

        if anchor:
            self.map0 = tuple(float(v) for v in anchor)
            self.log("explicit map pose: (%+.3f, %+.3f) yaw %+.1f"
                     % self.map0)
        else:
            P = self.cloud()
            pose, why = init_map_pose(P)
            if pose is None:
                raise RuntimeError("map frame: %s - restart at A or pass --anchor"
                                   % why)
            self.map0 = (pose[0], pose[1], pose[2])
            self.log("wall fix: map pose (%+.3f, %+.3f) yaw %+.1f (%s)"
                     % (pose[0], pose[1], pose[2], why))
            if math.hypot(pose[0] - A_XY[0], pose[1] - A_XY[1]) > 0.35:
                raise RuntimeError("not near A (%.2f, %.2f) - restart at A or "
                                   "pass --anchor" % (pose[0], pose[1]))

        q = self.odom.pose.pose.orientation
        self.yaw0 = math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z)
        self.yaw0_deg = math.degrees(self.yaw0)
        self.odom0 = (self.odom.pose.pose.position.x,
                      self.odom.pose.pose.position.y)
        self.t0 = time.time()

        self.cubes = [
            {"x": float(b.get("x", 0.0)), "y": float(b.get("y", 0.0)),
             "cls": str(b.get("type", b.get("class", ""))),
             "status": "field", "delivered": False}
            for b in scenario.get("bricks", [])
        ]

        server = ViewerServer(port, scenario, lambda: self.state(),
                              self.stop, host="0.0.0.0")
        server.start()
        self.log("viewer: http://<robot-ip>:%d/   log: %s" % (port, log_path))
        self.server = server

    def log(self, msg: str) -> None:
        print("[REAL_VIEW] " + msg, flush=True)

    def _on_scan(self, m: LaserScan) -> None:
        r = np.array(m.ranges, dtype=float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > 0.10) & (r < 3.2)
        self.scan_r, self.scan_a = r[ok], a[ok]

    def _on_odom(self, m: Odometry) -> None:
        self.odom = m

    def cloud(self):
        return np.column_stack([self.scan_r * np.cos(self.scan_a),
                                self.scan_r * np.sin(self.scan_a)])

    def pose_map(self):
        o = self.odom
        q = o.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z)
        th = math.radians(self.map0[2] - self.yaw0_deg)
        dx = o.pose.pose.position.x - self.odom0[0]
        dy = o.pose.pose.position.y - self.odom0[1]
        x = self.map0[0] + dx * math.cos(th) - dy * math.sin(th)
        y = self.map0[1] + dx * math.sin(th) + dy * math.cos(th)
        h = self.map0[2] + math.degrees(wrap(yaw - self.yaw0))
        return x, y, h

    def lidar_world(self, x: float, y: float, h_deg: float):
        if self.scan_r is None:
            return []
        step = max(1, len(self.scan_r) // 140)
        r = self.scan_r[::step]
        a = self.scan_a[::step]
        c, s = math.cos(math.radians(h_deg)), math.sin(math.radians(h_deg))
        px, py = r * np.cos(a), r * np.sin(a)
        return [[round(x + px[i] * c - py[i] * s, 3),
                 round(y + px[i] * s + py[i] * c, 3)] for i in range(len(r))]

    def state(self):
        with self.lock:
            return dict(self.snap)

    def loop(self) -> None:
        while not rospy.is_shutdown() and not self.stop.is_set():
            x, y, h = self.pose_map()
            pts = self.lidar_world(x, y, h)
            if not self.trail or math.hypot(x - self.trail[-1][0],
                                            y - self.trail[-1][1]) > 0.01:
                self.trail.append([round(x, 3), round(y, 3)])
            tw = self.odom.twist.twist
            t = time.time() - self.t0
            snap = {
                "t": round(t, 2), "pose": [round(x, 3), round(y, 3),
                                           round(math.radians(h), 4)],
                "lidar": pts, "trail": self.trail[-2000:],
                "cubes": self.cubes, "score": 0, "detail": [],
                "collisions": 0, "left_start": False, "carried": None,
                "speed": 1.0, "vel": [round(tw.linear.x, 3),
                                      round(tw.angular.z, 3)],
                "max_linear": 0.20, "max_angular": 0.75,
                "state": "REAL TELEOP", "current": "", "steps": [],
                "mapping": {},
            }
            with self.lock:
                self.snap = snap
            if self.fh:
                self.fh.write(json.dumps({"t": snap["t"], "pose": snap["pose"],
                                          "lidar": pts}) + "\n")
                self.fh.flush()
            time.sleep(0.2)


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    # local tree: codes/ next to config/; robot tree: ~/codes/ with config/ inside
    scenario_default = None
    for cand in (os.path.join(here, "..", "config", "arena_map.yaml"),
                 os.path.join(here, "config", "arena_map.yaml")):
        if os.path.exists(cand):
            scenario_default = os.path.normpath(cand)
            break
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--scenario", default=scenario_default)
    ap.add_argument("--anchor", nargs=3, type=float, metavar=("X", "Y", "YAW_DEG"),
                    help="explicit map pose instead of the wall fix")
    ap.add_argument("--log-dir", default=os.path.expanduser("~/logs/real_view"))
    args = ap.parse_args()

    with open(args.scenario) as f:
        import yaml
        scenario = yaml.safe_load(f)
    os.makedirs(args.log_dir, exist_ok=True)
    log_path = os.path.join(args.log_dir,
                            time.strftime("view_%Y%m%d_%H%M%S.jsonl"))
    view = RealView(scenario, args.anchor, args.port, log_path)
    try:
        view.loop()
    except KeyboardInterrupt:
        pass
    finally:
        if view.fh:
            view.fh.close()
        view.server.stop()
        view.log("stopped; log at %s" % log_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
