#!/usr/bin/env python3
"""Drive D -> A using wall distances as ground truth (no AMCL).

Convention (validated 2026-11-23): the physical wall faces sit 0.175 m outside
the drawn map, i.e. at map -0.175 on the west/south and 3.175 on the east/north.
    map_x = -0.175 + d_west
    map_y =  3.175 - d_north
A (0.225, 2.775) therefore means d_west = 0.40 and d_north = 0.40.

The robot is placed at D facing roughly north-east; heading and position are
taken from the two visible walls, then tracked with odometry. AMCL is ignored
(it was lost at 2026-11-23 23:20: estimated (0.371, 0.887) at -26 deg while
the robot stood still).

Route: east to x=0.75 (clear of the west-wall block y 1.30-1.70), north to
y~2.575, then close in on the west and north walls to A's 0.40 m offsets.

Run on the robot:  python3 -u ~/codes/robot/d2a.py
"""

from __future__ import annotations

import math
import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

WALL_OFF = -0.175          # map coordinate of the physical west/south wall faces
MAP_TOP = 3.0 + 0.175      # map coordinate of the physical east/north wall faces
A_XY = (0.225, 2.775)      # A centre (map frame)
D_XY = (0.225, 0.225)


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class D2A:
    def __init__(self) -> None:
        rospy.init_node("d2a")
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
        self.h0 = 0.0

    # ---- plumbing ---------------------------------------------------------
    def yaw(self) -> float:
        q = self.odom.pose.pose.orientation
        return math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z)

    def log(self, msg: str) -> None:
        print("[D2A] " + msg, flush=True)

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
        """Sharpest line family with normal at theta: (bearing, dist, count)."""
        n = np.array([math.cos(math.radians(theta_deg)),
                      math.sin(math.radians(theta_deg))])
        p = P @ n
        h, e = np.histogram(p, bins=np.arange(p.min(), p.max() + 0.01, 0.01))
        i = int(np.argmax(h))
        if h[i] < min_count:
            return None
        p0 = e[i] + 0.005
        bearing = theta_deg if p0 >= 0 else (theta_deg + 180.0) % 360.0
        if bearing > 180.0:
            bearing -= 360.0
        return (bearing, abs(p0), int(h[i]))

    def wall_lines(self):
        """Two perpendicular-ish wall lines: [(bearing, dist), (bearing, dist)]."""
        P = self.cloud()
        best = {}
        for th in np.arange(0.0, 180.0, 0.5):
            r = self._peak(P, th)
            if r is not None:
                best[th] = r
        if not best:
            return None
        th1 = max(best, key=lambda t: best[t][2])
        w1 = best[th1]
        # second family: 60..120 deg away from the first normal
        others = [(t, v) for t, v in best.items()
                  if 60.0 <= abs(t - th1) <= 120.0]
        if not others:
            return [w1]
        th2, w2 = max(others, key=lambda kv: kv[1][2])
        return [w1, w2]

    def init_heading(self) -> None:
        """Resolve which walls the two lines are, and the heading.

        Two lines 90 deg apart admit a 180-deg-rotated alternative (SW corner
        vs NE corner) that fits the scan identically. The hypotheses are
        disambiguated by the known start: the one that puts the robot nearest
        D is taken.
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
                    if cons > 10.0:
                        continue
                    x = x1 if x1 is not None else x2
                    y = y1 if y1 is not None else y2
                    err = math.hypot(x - D_XY[0], y - D_XY[1])
                    hm = h1 + math.degrees(wrap(math.radians(h2 - h1))) / 2.0
                    cands.append((err, cons, hm % 360.0, x, y, r1, r2))
        if not cands:
            raise RuntimeError("corner hypotheses inconsistent")
        cands.sort()
        err, cons, h, x, y, r1, r2 = cands[0]
        self.h0 = h
        self.log("hypothesis %s/%.0f + %s/%.0f: heading %.1f, pos (%.3f, %.3f), "
                 "consistency %.1f, %.1f cm from D"
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

    def drive_until(self, phi_map: float, stop_dist: float, vmax: float = 0.05,
                    timeout: float = 90.0, guard: float = 0.15) -> bool:
        """Drive forward until the wall at map bearing phi is <= stop_dist."""
        h_start = self.yaw()
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
            m = self.scan
            mn = 99.0
            for i, rv in enumerate(m.ranges):
                if not (m.range_min < rv < m.range_max):
                    continue
                ang = m.angle_min + i * m.angle_increment
                if abs(wrap(ang)) < math.radians(15) and rv < mn:
                    mn = rv
            if mn < guard:
                self.log("GUARD stop at %.2f m" % mn)
                ok = False
                break
            err = wrap(h_start - self.yaw())
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

    # ---- scenario ---------------------------------------------------------
    def run(self) -> None:
        self.init_heading()
        dw, ds = self.dist(180), self.dist(270)
        x, y = WALL_OFF + dw, WALL_OFF + ds
        self.log("start pos = (%.3f, %.3f)  [D is %s]  offset %.1f cm"
                 % (x, y, D_XY, 100 * math.hypot(x - D_XY[0], y - D_XY[1])))
        if math.hypot(x - D_XY[0], y - D_XY[1]) > 0.30:
            raise RuntimeError("not at D - refusing to drive (%.2f, %.2f)" % (x, y))

        self.turn_to(90)                                   # face north
        self.turn_to(0)                                    # face east
        self.drive_until(180, 0.925, vmax=0.05, timeout=45)   # x ~ 0.75

        self.turn_to(90)                                   # face north
        self.drive_until(90, 0.60, vmax=0.06, timeout=120)    # y ~ 2.575

        self.turn_to(180)                                  # face west
        self.drive_until(180, 0.40, vmax=0.04, timeout=60)    # x = 0.225

        self.turn_to(90)                                   # face north
        self.drive_until(90, 0.40, vmax=0.03, timeout=60)     # y = 2.775

        dw, dn = self.dist(180), self.dist(90)
        x, y = WALL_OFF + dw, MAP_TOP - dn
        self.log("AT A: pos=(%.3f, %.3f)  A=%s  error %.1f cm"
                 % (x, y, A_XY, 100 * math.hypot(x - A_XY[0], y - A_XY[1])))
        self.turn_to(270)                                  # face south
        self.log("final heading %.1f (south = 270)" % self.h())


def main() -> None:
    D2A().run()


if __name__ == "__main__":
    main()
