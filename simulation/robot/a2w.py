#!/usr/bin/env python3
"""Drive A -> Sorting_W using lidar wall fixes + odometry (no AMCL).

Start: the robot is placed at A collection centre (0.225, 2.775), on the
"45 deg" diagonal (toward the arena centre, map yaw ~-45).
Goal:  Sorting_W (1.090, 1.500), finishing at 0 deg (east, facing the cluster).

Ground truth is the physical wall frame (validated 2026-11-23):
    map_x = -0.175 + d_west
    map_y =  3.175 - d_north
A (0.225, 2.775) therefore means 0.40 m from the west and north walls.
AMCL is ignored: it was lost at 2026-11-23 23:20 (estimated (0.371, 0.887)
at -26 deg while the robot stood still) and is biased near the cluster.

Route (walls first, odometry in between):
  1. east along the north wall from x~0.225 to x=0.95 - the clear lane between
     the classification table (east face x=0.33) and the cube cluster (west
     face x=1.305);
  2. south until the south wall is 1.675 m away, i.e. y = 1.500 exactly;
  3. east 0.14 m into W, facing the cluster.

Corrections at W come from the south wall (y) and from the classification
table east face, or the west wall if the table is out of view (x). Both are
known map surfaces, so either reading pins x to 1.090.

Run on the robot:  python3 -u ~/codes/robot/a2w.py
"""

from __future__ import annotations

import math
import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# --- arena constants (physical wall faces are 0.175 m outside the drawn map) --
WALL_OFF = -0.175                # map x of the physical west/south wall faces
MAP_TOP = 3.0 + 0.175            # map x of the physical east/north wall faces
A_XY = (0.225, 2.775)            # A collection centre


def _map_point(name, default):
    """Named point from the deployed arena map, with a fallback constant.

    The map is the source of truth (re-measured 2026-11-24); read it so this
    script follows map updates without another edit.
    """
    try:
        import os

        import yaml
        here = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(here, "..", "config", "arena_map.yaml")
        with open(path) as f:
            p = (yaml.safe_load(f).get("points") or {}).get(name)
        if p:
            return (float(p[0]), float(p[1]))
    except Exception:
        pass
    return default


W_XY = _map_point("Sorting_W", (1.064, 1.562))
W_HEADING = 0.0                  # face the cube cluster (east)
TRANSIT_X = 0.950                # clear lane between table and cluster
TABLE_EAST_X = 0.330             # classification table east face (0.165 + 0.33/2)
MAX_HOP = 0.06                   # cap on a correction hop toward the cluster


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class A2W:
    def __init__(self, node_name: str = "a2w") -> None:
        rospy.init_node(node_name)
        self.scan = None
        self.odom = None
        rospy.Subscriber("/scan", LaserScan, lambda m: setattr(self, "scan", m))
        rospy.Subscriber("/odom", Odometry, lambda m: setattr(self, "odom", m))
        self.cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        t0 = time.time()
        while (self.scan is None or self.odom is None) and time.time() - t0 < 10:
            rospy.sleep(0.1)
        if self.scan is None or self.odom is None:
            raise RuntimeError("no /scan or /odom")
        self.yaw0 = self.yaw()
        self.odom0 = (self.odom.pose.pose.position.x,
                      self.odom.pose.pose.position.y)
        self.h0 = 0.0

    # ---- plumbing ---------------------------------------------------------
    def yaw(self) -> float:
        q = self.odom.pose.pose.orientation
        return math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z)

    def log(self, msg: str) -> None:
        print("[A2W] " + msg, flush=True)

    def stop(self) -> None:
        for _ in range(8):
            self.cmd.publish(Twist())
            self.rate.sleep()

    # ---- geometry ---------------------------------------------------------
    def cloud(self) -> np.ndarray:
        m = self.scan
        r = np.array(m.ranges)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > 0.10) & (r < 3.2)
        return np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])

    @staticmethod
    def _peak(P: np.ndarray, theta_deg: float, min_count: int = 30):
        """Sharpest line family with normal at theta: (bearing, p0, count).

        p0 is signed: the projection of the surface onto the normal, so it
        tells which side the surface is on.
        """
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
        return (bearing, p0, int(h[i]))

    @staticmethod
    def _span(P: np.ndarray, theta_deg: float, p0: float) -> float:
        """Tangential extent (m) of the points within 4 cm of the line."""
        n = np.array([math.cos(math.radians(theta_deg)),
                      math.sin(math.radians(theta_deg))])
        sel = np.abs(P @ n - p0) < 0.04
        if not np.any(sel):
            return 0.0
        t = P[sel] @ np.array([-n[1], n[0]])
        return float(t.max() - t.min())

    def wall_lines(self):
        """Two perpendicular-ish wall lines: [(bearing, dist, count), ...].

        A wall that is a few degrees off the map axes produces aliased peaks
        (e.g. a chunk of the west wall projected onto a rotated normal). The
        real wall families are long (metres); the aliases are short. Families
        with less than 1.2 m of tangential extent are dropped.
        """
        P = self.cloud()
        if len(P) < 50:
            return None
        cands = []
        for th in np.arange(0.0, 180.0, 0.5):
            r = self._peak(P, th)
            if r is not None:
                cands.append((r[2], r[0], r[1], th))   # count, bearing, p0, theta
        cands.sort(reverse=True)
        fams = []                            # (count, theta, dist, bearing, span)
        for count, bearing, p0, th in cands:
            if any(abs(th - f[1]) < 8.0 and abs(abs(p0) - f[2]) < 0.08
                   for f in fams):
                continue
            span = self._span(P, th, p0)
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
        # of the near-perpendicular families, take the longest wall: aliased
        # chunks of a skewed wall are short, the real wall runs for metres
        f2 = max(others, key=lambda f: f[4])
        return [(f1[3], f1[2], f1[0]), (f2[3], f2[2], f2[0])]

    def init_heading(self) -> None:
        """Resolve which walls the two lines are, and the heading.

        Two lines 90 deg apart admit a 180-deg-rotated alternative (NW corner
        vs SE corner) that fits the scan identically. The hypotheses are
        disambiguated by the known start: the one that puts the robot nearest
        A is taken.
        """
        lines = self.wall_lines()
        if not lines or len(lines) < 2:
            raise RuntimeError("no pair of wall lines found")
        self.log("lines: " + ", ".join("bearing %.1f dist %.3f (n=%d)" % l
                                       for l in lines))

        def hyp(role, b, d):
            """(heading, x, y) implied when the line plays the given role."""
            x = y = None
            if role == "west":
                h, x = 180.0 - b, WALL_OFF + d
            elif role == "east":
                h, x = -b, MAP_TOP - d
            elif role == "south":
                h, y = 270.0 - b, WALL_OFF + d
            else:                                   # north
                h, y = 90.0 - b, MAP_TOP - d
            return h, x, y

        xs = ("west", "east")
        ys = ("south", "north")
        cands = []
        for rx in xs:
            for ry in ys:
                for order, (r1, r2) in ((1, (rx, ry)), (2, (ry, rx))):
                    b1, d1, _ = lines[0]
                    b2, d2, _ = lines[1]
                    h1, x1, y1 = hyp(r1, b1, d1)
                    h2, x2, y2 = hyp(r2, b2, d2)
                    cons = abs(((h2 - h1 + 180.0) % 360.0) - 180.0)
                    # The real NW corner measured 82 deg, not 90 (2026-11-24,
                    # ~8 deg out of square like the known arena/map yaw bias),
                    # and histogram binning adds a few more degrees, so the
                    # pair is accepted up to 20 deg. The nearest-A position
                    # check below still rejects the 180-deg-rotated corner.
                    if cons > 20.0:
                        continue
                    x = x1 if x1 is not None else x2
                    y = y1 if y1 is not None else y2
                    err = math.hypot(x - A_XY[0], y - A_XY[1])
                    hm = h1 + math.degrees(wrap(math.radians(h2 - h1))) / 2.0
                    cands.append((err, cons, hm % 360.0, x, y, r1, r2))
        if not cands:
            raise RuntimeError("corner hypotheses inconsistent")
        cands.sort()
        err, cons, h, x, y, r1, r2 = cands[0]
        self.h0 = h
        self.pos0 = (x, y)
        self.log("hypothesis %s/%.0f + %s/%.0f: heading %.1f, pos (%.3f, %.3f), "
                 "consistency %.1f, %.1f cm from A"
                 % (r1, lines[0][1] * 1000, r2, lines[1][1] * 1000,
                    h, x, y, cons, 100 * err))

    def h(self) -> float:
        return self.h0 + math.degrees(wrap(self.yaw() - self.yaw0))

    def dist(self, phi_map: float, half: float = 6.0) -> float:
        """Median range (m) toward map bearing phi (deg)."""
        m = self.scan
        r = np.array(m.ranges)
        a = np.degrees(m.angle_min + np.arange(len(r)) * m.angle_increment)
        beta = phi_map - self.h()
        diff = np.abs((a - beta + 180.0) % 360.0 - 180.0)
        good = np.isfinite(r) & (r > 0.10) & (r < 3.2)
        for hw in (half, 12.0, 20.0):
            sel = r[(diff <= hw) & good]
            if len(sel) >= 8:
                return float(np.median(sel))
        return float("nan")

    # ---- motions ----------------------------------------------------------
    def turn_to(self, target_deg: float, maxw: float = 0.30) -> None:
        t0 = time.time()
        while time.time() - t0 < 40:
            err = wrap(math.radians(target_deg - self.h()))
            if abs(err) < math.radians(0.5):
                break
            c = Twist()
            c.angular.z = max(-maxw, min(maxw, 1.1 * err))
            self.cmd.publish(c)
            self.rate.sleep()
        self.stop()
        self.log("turn -> h=%.1f (target %.0f)" % (self.h(), target_deg))

    def guard_range(self, half_deg: float = 10.0) -> float:
        """Nearest valid range inside +-half_deg of the front."""
        m = self.scan
        mn = 99.0
        for i, rv in enumerate(m.ranges):
            if not (m.range_min < rv < m.range_max):
                continue
            ang = m.angle_min + i * m.angle_increment
            if abs(wrap(ang)) < math.radians(half_deg) and rv < mn:
                mn = rv
        return mn

    def drive(self, dist: float, vmax: float = 0.05, timeout: float = 60.0,
              guard: float = 0.15) -> float:
        """Drive forward `dist` metres on odometry, holding the heading."""
        sx, sy = self.odom.pose.pose.position.x, self.odom.pose.pose.position.y
        yaw0 = self.yaw()
        moved = 0.0
        t0 = time.time()
        while time.time() - t0 < timeout:
            x = self.odom.pose.pose.position.x
            y = self.odom.pose.pose.position.y
            moved = (x - sx) * math.cos(yaw0) + (y - sy) * math.sin(yaw0)
            if moved >= dist:
                break
            mn = self.guard_range(10.0)
            if mn < guard:
                self.log("GUARD stop at %.2f m" % mn)
                break
            err = wrap(yaw0 - self.yaw())
            c = Twist()
            c.linear.x = max(0.02, min(vmax, 0.9 * (dist - moved)))
            c.angular.z = max(-0.10, min(0.10, 1.2 * err))
            self.cmd.publish(c)
            self.rate.sleep()
        self.stop()
        self.log("drive: %.3f m of %.3f requested, h=%.1f"
                 % (moved, dist, self.h()))
        return moved

    def drive_until_far(self, phi_map: float, target: float, vmax: float = 0.05,
                        timeout: float = 90.0, guard: float = 0.15) -> bool:
        """Drive forward until the surface at map bearing phi is >= target.

        Used to move *away* from a known wall: the range to a flat wall pins
        the coordinate independently of the (few-degree) heading estimate.
        """
        yaw0 = self.yaw()
        d = self.dist(phi_map)
        self.log("leg: d(bearing %.0f)=%.3f -> reach %.3f" % (phi_map, d, target))
        t0 = time.time()
        ok = True
        while time.time() - t0 < timeout:
            d = self.dist(phi_map)
            if not math.isnan(d) and d >= target:
                break
            if self.guard_range(15.0) < guard:
                self.log("GUARD stop at %.2f m" % self.guard_range(15.0))
                ok = False
                break
            err = wrap(yaw0 - self.yaw())
            c = Twist()
            c.linear.x = vmax
            c.angular.z = max(-0.12, min(0.12, 1.2 * err))
            self.cmd.publish(c)
            self.rate.sleep()
        else:
            self.log("TIMEOUT")
            ok = False
        self.stop()
        self.log("leg done: d(bearing %.0f)=%.3f, h=%.1f"
                 % (phi_map, self.dist(phi_map), self.h()))
        return ok

    def drive_until(self, phi_map: float, stop_dist: float, vmax: float = 0.05,
                    timeout: float = 90.0, guard: float = 0.15) -> bool:
        """Drive forward until the surface at map bearing phi is <= stop_dist."""
        yaw0 = self.yaw()
        sx = self.odom.pose.pose.position.x
        sy = self.odom.pose.pose.position.y
        d = self.dist(phi_map)
        self.log("leg: d(bearing %.0f)=%.3f -> stop %.3f" % (phi_map, d, stop_dist))
        t0 = time.time()
        ok = True
        while time.time() - t0 < timeout:
            d = self.dist(phi_map)
            if not math.isnan(d) and d <= stop_dist:
                break
            if self.guard_range(15.0) < guard:
                self.log("GUARD stop at %.2f m" % self.guard_range(15.0))
                ok = False
                break
            err = wrap(yaw0 - self.yaw())
            c = Twist()
            c.linear.x = vmax
            c.angular.z = max(-0.12, min(0.12, 1.2 * err))
            self.cmd.publish(c)
            self.rate.sleep()
        else:
            self.log("TIMEOUT")
            ok = False
        self.stop()
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        self.log("leg done: odom moved %.3f m, d=%.3f, h=%.1f"
                 % (math.hypot(x - sx, y - sy), self.dist(phi_map), self.h()))
        return ok

    # ---- wall-closed-loop goto -------------------------------------------
    def odom_pose_map(self):
        """Map pose from odometry since the wall-fixed start."""
        o = self.odom
        q = o.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z)
        th = math.radians(self.h0 - math.degrees(self.yaw0))
        dx = o.pose.pose.position.x - self.odom0[0]
        dy = o.pose.pose.position.y - self.odom0[1]
        return (self.pos0[0] + dx * math.cos(th) - dy * math.sin(th),
                self.pos0[1] + dx * math.sin(th) + dy * math.cos(th))

    def wall_xy_lines(self):
        """(x, y) from the wall-LINE perpendicular distances.

        A range read in a map-bearing cone (dist()) is stretched when the
        heading estimate is off, which is what drove the diagonal 37 cm south
        on 2026-11-24. The line fits give the perpendicular distance to the
        wall directly, so they do not care about the heading error; the only
        heading-dependent part is assigning each line to an axis, and that is
        robust for the few-degree skew.
        """
        lines = self.wall_lines() or []
        h = self.h()
        sides = {}
        for bearing, dist, _count in lines:
            mb = (bearing + h) % 360.0
            for axis, role in ((0.0, "east"), (180.0, "west"),
                               (90.0, "north"), (270.0, "south")):
                if role in sides:
                    continue
                if abs(((mb - axis + 180.0) % 360.0) - 180.0) < 30.0:
                    sides[role] = dist
                    break
        x = y = None
        if "west" in sides:
            x = WALL_OFF + sides["west"]
        elif "east" in sides:
            x = MAP_TOP - sides["east"]
        if "north" in sides:
            y = MAP_TOP - sides["north"]
        elif "south" in sides:
            y = WALL_OFF + sides["south"]
        return x, y

    def goto_wall(self, target, tol: float = 0.03, vmax: float = 0.14,
                  timeout: float = 120.0, guard: float = 0.15) -> None:
        """Straight leg on odometry, then wall-line corrections to the point."""
        ex, ey = target[0] - self.pos0[0], target[1] - self.pos0[1]
        dist = math.hypot(ex, ey)
        bearing = math.degrees(math.atan2(ey, ex))
        self.log("goto: turn %.1f, drive %.3f m" % (bearing, dist))
        self.turn_to(bearing)
        moved = self.drive(dist, vmax=vmax, timeout=timeout, guard=guard)
        self.log("goto: odom moved %.3f m" % moved)
        for i in range(4):
            x, y = self.wall_xy_lines()
            if x is None or y is None:
                self.log("goto: wall fix incomplete (%s, %s)" % (x, y))
                break
            dx, dy = target[0] - x, target[1] - y
            d = math.hypot(dx, dy)
            self.log("goto fix %d: (%.3f, %.3f) err %.3f m" % (i + 1, x, y, d))
            if d <= tol:
                break
            b = math.degrees(math.atan2(dy, dx))
            self.turn_to(b)
            self.drive(min(d, 0.30), vmax=min(vmax, 0.06), timeout=25,
                       guard=0.12)
        self.stop()

    # ---- W fix ------------------------------------------------------------
    def wall_x(self, d_west: float):
        """Map x from the known west-side surface, or (None, reason)."""
        if math.isnan(d_west):
            return None, "no west reading"
        if 0.55 <= d_west < 0.95:
            return TABLE_EAST_X + d_west, "classification table"
        if 1.02 <= d_west <= 1.45:
            return WALL_OFF + d_west, "west wall"
        return None, "unexpected west range %.3f" % d_west

    def fix_at_W(self):
        """(x, y) at W from the table/west wall and the south wall."""
        d_w = self.dist(180)
        d_s = self.dist(270)
        y = WALL_OFF + d_s if not math.isnan(d_s) else None
        x, src = self.wall_x(d_w)
        self.log("fix at W: d_west=%.3f (%s) d_south=%.3f -> x=%s y=%s"
                 % (d_w, src, d_s,
                    "%.3f" % x if x is not None else "?",
                    "%.3f" % y if y is not None else "?"))
        return x, y

    def correct_at_W(self) -> None:
        x, y = self.fix_at_W()
        if y is not None and abs(y - W_XY[1]) > 0.02:
            dy = W_XY[1] - y
            self.turn_to(90 if dy > 0 else 270)
            step = min(abs(dy), MAX_HOP)
            self.drive(step, vmax=0.03, timeout=20)
            if abs(dy) > MAX_HOP:
                self.log("y correction capped: residual %.1f cm" % (100 * (dy - math.copysign(step, dy))))
        x, y = self.fix_at_W()
        if x is not None and abs(x - W_XY[0]) > 0.02:
            dx = W_XY[0] - x
            self.turn_to(0 if dx > 0 else 180)
            step = min(abs(dx), MAX_HOP)
            self.drive(step, vmax=0.03, timeout=20)
            if abs(dx) > MAX_HOP:
                self.log("x correction capped: residual %.1f cm" % (100 * (dx - math.copysign(step, dx))))
        self.turn_to(W_HEADING)
        x, y = self.fix_at_W()
        if x is not None and y is not None:
            self.log("AT W: pos=(%.3f, %.3f)  W=%s  error %.1f cm, heading %.1f"
                     % (x, y, W_XY, 100 * math.hypot(x - W_XY[0], y - W_XY[1]),
                        self.h()))
        else:
            self.log("AT W: partial fix, heading %.1f" % self.h())

    # ---- scenario ---------------------------------------------------------
    def run(self) -> None:
        self.init_heading()
        x, y = self.pos0
        self.log("start pos = (%.3f, %.3f)  [A is %s]  offset %.1f cm, heading %.1f"
                 % (x, y, A_XY, 100 * math.hypot(x - A_XY[0], y - A_XY[1]),
                    self.h()))
        if math.hypot(x - A_XY[0], y - A_XY[1]) > 0.30:
            raise RuntimeError("not at A - refusing to drive (%.2f, %.2f)" % (x, y))

        # straight diagonal A -> W, closed on the wall distances
        self.log("diagonal A -> W: target (%.3f, %.3f), bearing %.1f deg, %.2f m"
                 % (W_XY[0], W_XY[1],
                    math.degrees(math.atan2(W_XY[1] - y, W_XY[0] - x)),
                    math.hypot(W_XY[0] - x, W_XY[1] - y)))
        self.goto_wall(W_XY, tol=0.04, vmax=0.14, timeout=90)

        self.turn_to(W_HEADING)
        xf, yf = self.wall_xy_lines()
        self.log("AT W: wall pose (%.3f, %.3f)  target (%.3f, %.3f)  error %.1f cm,"
                 " heading %.1f"
                 % (xf if xf is not None else float("nan"),
                    yf if yf is not None else float("nan"),
                    W_XY[0], W_XY[1],
                    100 * math.hypot((xf or W_XY[0]) - W_XY[0],
                                     (yf or W_XY[1]) - W_XY[1]), self.h()))
        self.stop()
        self.log("DONE - at W, arm empty. Check the pose before the pick.")


def main() -> None:
    A2W().run()


if __name__ == "__main__":
    main()
