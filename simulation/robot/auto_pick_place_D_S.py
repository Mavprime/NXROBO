#!/usr/bin/env python3
"""Automated cake pick-and-place: centre D -> S -> pick cake -> centre D.

Packages the manually validated 2026-11-23 sequence:
  * the physical wall frame (lidar) is ground truth, not AMCL;
  * odometry carries the map frame between wall fixes;
  * AMCL is re-anchored at each wall fix only so arm_real_node's world gate works;
  * pick uses /arm/pick, place uses /arm/place level 1;
  * camera red-pixel fraction verifies the cake is held / placed.

Requires: hardware stack (base, camera, lidar), detector on /objects, swiftpro,
and arm_real_node running with _creep:=false _turn_align:=false _cake_pos_tol:=0.20
(this script starts it if missing).

Run on the robot:  python3 -u ~/codes/robot/auto_pick_place_D_S.py
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import time

import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

# --- arena constants -------------------------------------------------------
WALL = (-0.175, -0.175)          # physical wall faces, map frame (measured)
D_CENTRE = (0.225, 0.225)
S_POINT = (1.500, 1.090)
DROP_POSE = (0.422, 0.422)       # 279 mm from D centre along the diagonal
S_HEADING = 90.0                 # face the cluster
DROP_HEADING = 225.0             # face the D corner
PICK_SETTINGS = "_creep:=false _turn_align:=false _cake_pos_tol:=0.20"

HELD_RED_PX = 2500               # carry frame red-pixel count above this = cake held
PLACED_RED_PX_MIN = 300          # cake visible after placing


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class Auto:
    def __init__(self) -> None:
        rospy.init_node("auto_pick_place")
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf)
        self.cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.initp = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.pick_pub = rospy.Publisher("/arm/pick", String, queue_size=1)
        self.place_pub = rospy.Publisher("/arm/place", String, queue_size=1)
        self.scan = None
        self.result = {}
        rospy.Subscriber("/scan", LaserScan, self._scan_cb)
        rospy.Subscriber("/arm/result", String, self._res_cb)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.odom0 = None
        self.map0 = None
        self.h0 = None
        rospy.sleep(1.5)

    # -- plumbing ----------------------------------------------------------
    def _scan_cb(self, m):
        self.scan = m

    def _res_cb(self, m):
        try:
            d = json.loads(m.data)
            self.result[d.get("id")] = d
        except Exception:
            pass

    def log(self, msg):
        print("[AUTO] %s" % msg, flush=True)

    def pose(self, p, c):
        t = self.tf.lookup_transform(p, c, rospy.Time(0), rospy.Duration(3))
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z))

    # -- wall fix ----------------------------------------------------------
    def _cloud(self):
        m = self.scan
        if m is None:
            m = rospy.wait_for_message("/scan", LaserScan, timeout=5)
        r = np.array(m.ranges)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > max(m.range_min, 0.10)) & (r < 3.2)
        return np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])

    def wall_heading(self):
        """Heading from the wall lines + the SW-corner direction (valid at D,
        where the near scan is the corner; at S the cube cluster dominates)."""
        P = self._cloud()
        near = P[np.hypot(P[:, 0], P[:, 1]) < 1.2]
        if len(near) < 50:
            near = P[np.hypot(P[:, 0], P[:, 1]) < 2.5]
        phi_cc = math.degrees(math.atan2(near[:, 1].mean(), near[:, 0].mean()))
        best_th, best_sc = 0.0, -1
        for th in np.arange(0.0, math.pi / 2, math.radians(0.25)):
            n = np.array([math.cos(th), math.sin(th)])
            p = P @ n
            h, _ = np.histogram(p, bins=np.arange(p.min(), p.max(), 0.01))
            sc = np.sort(h)[-2:].sum()
            if sc > best_sc:
                best_sc, best_th = sc, th
        theta_w = math.degrees(best_th)
        cands = [((s * theta_w + 90.0 * k + 180.0) % 360.0) - 180.0
                 for s in (1.0, -1.0) for k in range(4)]
        # the SW corner is always behind-left/behind-right => map bearing ~ -135;
        # pick the candidate that puts it in (-180, -90) degrees.
        valid = [c for c in cands
                 if -180.0 <= ((c + phi_cc + 180.0) % 360.0) - 180.0 <= -90.0]
        h = valid[0] if valid else min(
            cands, key=lambda c: abs(wrap(math.radians(c + 135.0))))
        self.log("wall heading: corner bearing %.1f -> heading %.1f" % (phi_cc, h))
        return h

    def wall_pos(self, heading_deg):
        """(x, y) from the nearest wall faces along the known map axes."""
        P = self._cloud()
        xh = np.array([math.cos(math.radians(-heading_deg)),
                       math.sin(math.radians(-heading_deg))])
        yh = np.array([math.cos(math.radians(90.0 - heading_deg)),
                       math.sin(math.radians(90.0 - heading_deg))])
        px = np.percentile(P @ xh, 1.0)
        py = np.percentile(P @ yh, 1.0)
        x = WALL[0] - px
        y = WALL[1] - py
        self.log("wall pos: (%.3f, %.3f)  wall dist %.2f/%.2f" % (x, y, -px, -py))
        return x, y

    # -- odom-anchored map frame ------------------------------------------
    def rebase(self, pos, heading_deg):
        ox, oy, oyaw = self.pose("odom", "base_footprint")
        self.odom0 = (ox, oy, oyaw)
        self.map0 = pos
        self.h0 = heading_deg
        self.anchor_amcl(pos, heading_deg)
        self.log("rebase: map=(%.3f, %.3f) heading=%.1f" % (pos[0], pos[1], heading_deg))

    def anchor_amcl(self, pos, heading_deg):
        q = PoseWithCovarianceStamped()
        q.header.frame_id = "map"
        q.pose.pose.position.x = pos[0]
        q.pose.pose.position.y = pos[1]
        yaw = math.radians(heading_deg)
        q.pose.pose.orientation.z = math.sin(yaw / 2)
        q.pose.pose.orientation.w = math.cos(yaw / 2)
        q.pose.covariance[0] = 0.25
        q.pose.covariance[7] = 0.25
        q.pose.covariance[35] = 0.0685
        for _ in range(2):
            self.initp.publish(q)
            rospy.sleep(0.3)

    def map_pose(self):
        ox, oy, oyaw = self.pose("odom", "base_footprint")
        th = math.radians(self.h0 - math.degrees(self.odom0[2]))
        dx = ox - self.odom0[0]
        dy = oy - self.odom0[1]
        x = self.map0[0] + dx * math.cos(th) - dy * math.sin(th)
        y = self.map0[1] + dx * math.sin(th) + dy * math.cos(th)
        heading = self.h0 + math.degrees(oyaw - self.odom0[2])
        return x, y, heading

    # -- motions -----------------------------------------------------------
    def turn_to(self, bearing_deg, maxw=0.35):
        _, _, h = self.map_pose()
        delta = wrap(math.radians(bearing_deg - h))
        target = wrap(self.pose("odom", "base_footprint")[2] + delta)
        t0 = time.time()
        while time.time() - t0 < 30:
            yaw = self.pose("odom", "base_footprint")[2]
            err = wrap(target - yaw)
            if abs(err) < 0.015:
                break
            c = Twist()
            c.angular.z = max(-maxw, min(maxw, 1.2 * err))
            self.cmd.publish(c)
            self.rate.sleep()
        self.stop()

    def drive(self, dist, vmax=0.06, guard=True, timeout=60):
        sx, sy, y1 = self.pose("odom", "base_footprint")
        t0 = time.time()
        while time.time() - t0 < timeout:
            x, y, yaw = self.pose("odom", "base_footprint")
            moved = (x - sx) * math.cos(y1) + (y - sy) * math.sin(y1)
            if moved >= dist:
                break
            if guard and self.scan is not None:
                m = self.scan
                mn = 99.0
                for i, rv in enumerate(m.ranges):
                    if not (m.range_min < rv < m.range_max):
                        continue
                    ang = m.angle_min + i * m.angle_increment
                    if abs(wrap(ang)) < math.radians(10) and rv < mn:
                        mn = rv
                if mn < 0.15:
                    self.log("GUARD stop at %.2f m" % mn)
                    break
            err = wrap(y1 - yaw)
            c = Twist()
            c.linear.x = max(0.02, min(vmax, 0.9 * (dist - moved)))
            c.angular.z = max(-0.10, min(0.10, 1.2 * err))
            self.cmd.publish(c)
            self.rate.sleep()
        self.stop()
        x, y, _ = self.pose("odom", "base_footprint")
        return (x - sx) * math.cos(y1) + (y - sy) * math.sin(y1)

    def stop(self):
        for _ in range(6):
            self.cmd.publish(Twist())
            self.rate.sleep()

    # -- perception --------------------------------------------------------
    def red_pixels(self):
        img = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5)
        cv = self.bridge.imgmsg_to_cv2(img, "bgr8").astype(int)
        r, g, b = cv[:, :, 2], cv[:, :, 1], cv[:, :, 0]
        mask = (r > 120) & (r - g > 60) & (r - b > 60)
        mask[240:, :] = False
        return int(mask.sum())

    def cake_visible(self, seconds=4.0):
        hits = 0
        t_end = time.time() + seconds
        while time.time() < t_end and not rospy.is_shutdown():
            try:
                m = rospy.wait_for_message("/objects", Detection2DArray, timeout=2)
            except Exception:
                break
            for d in m.detections:
                for res in d.results:
                    if str(res.id) in ("2", "20") and float(res.score) >= 0.60:
                        hits += 1
        return hits

    # -- arm ---------------------------------------------------------------
    def arm_call(self, topic, payload, wait_s=90):
        rid = payload["id"]
        self.result.pop(rid, None)
        (self.pick_pub if topic == "pick" else self.place_pub).publish(String(data=json.dumps(payload)))
        t0 = time.time()
        while time.time() - t0 < wait_s and rid not in self.result and not rospy.is_shutdown():
            rospy.sleep(0.5)
        return self.result.get(rid, {"ok": False, "reason": "timeout"})

    # -- sequence ----------------------------------------------------------
    def run(self):
        # 0. fix at D and rebase (corner in view at D -> heading from walls)
        h = self.wall_heading()
        x, y = self.wall_pos(h)
        self.rebase((x, y), h)
        self.log("start offset from D: %.1f cm" % (100 * math.hypot(x - D_CENTRE[0], y - D_CENTRE[1])))

        # 1. drive to S
        d = math.hypot(S_POINT[0] - x, S_POINT[1] - y)
        bearing = math.degrees(math.atan2(S_POINT[1] - y, S_POINT[0] - x))
        self.log("drive to S: bearing %.1f, %.3f m" % (bearing, d))
        self.turn_to(bearing)
        self.log("drove %.3f m" % self.drive(d, 0.06, True))
        self.turn_to(S_HEADING)

        # 2. fix at S, correct (heading from odom tracking; cluster blocks the corner)
        h = self.map_pose()[2]
        x, y = self.wall_pos(h)
        self.rebase((x, y), h)
        err = math.hypot(x - S_POINT[0], y - S_POINT[1])
        self.log("S offset: %.1f cm" % (100 * err))
        if err > 0.03:
            bearing = math.degrees(math.atan2(S_POINT[1] - y, S_POINT[0] - x))
            self.turn_to(bearing)
            self.drive(err, 0.03, False)
            self.turn_to(S_HEADING)
            h = self.map_pose()[2]
            x, y = self.wall_pos(h)
            self.rebase((x, y), h)
            self.log("S after correction: %.1f cm" % (100 * math.hypot(x - S_POINT[0], y - S_POINT[1])))

        # 3. cake visible?
        hits = self.cake_visible()
        self.log("cake detections at S: %d" % hits)

        # 4. pick
        res = self.arm_call("pick", {"id": "auto-pick", "class": "cake"})
        self.log("pick result: %s" % res)
        red = self.red_pixels()
        held = red >= HELD_RED_PX
        self.log("red pixels after pick: %d -> %s" % (red, "HELD" if held else "NOT HELD"))

        # 5. carry to drop pose
        x, y, _ = self.map_pose()
        d = math.hypot(DROP_POSE[0] - x, DROP_POSE[1] - y)
        bearing = math.degrees(math.atan2(DROP_POSE[1] - y, DROP_POSE[0] - x))
        self.log("carry to DROP: bearing %.1f, %.3f m" % (bearing, d))
        self.turn_to(bearing)
        self.log("drove %.3f m" % self.drive(d, 0.06, True))
        self.turn_to(DROP_HEADING)

        # 6. fix and correct at drop pose
        h = self.map_pose()[2]
        x, y = self.wall_pos(h)
        self.rebase((x, y), h)
        err = math.hypot(x - DROP_POSE[0], y - DROP_POSE[1])
        self.log("drop-pose offset: %.1f cm" % (100 * err))
        if err > 0.03:
            bearing = math.degrees(math.atan2(DROP_POSE[1] - y, DROP_POSE[0] - x))
            self.turn_to(bearing)
            self.drive(err, 0.03, False)
            self.turn_to(DROP_HEADING)
            h = self.map_pose()[2]
            x, y = self.wall_pos(h)
            self.rebase((x, y), h)
            self.log("drop pose after correction: %.1f cm from ideal" %
                     (100 * math.hypot(x - DROP_POSE[0], y - DROP_POSE[1])))

        # 7. place
        res = self.arm_call("place", {"id": "auto-place", "level": 1}, wait_s=30)
        self.log("place result: %s" % res)
        red = self.red_pixels()
        self.log("red pixels after place: %d" % red)
        self.log("DONE")


def ensure_arm_node():
    out = subprocess.run(["bash", "-lc", "pgrep -f 'arm[_]real_node'"],
                         capture_output=True, text=True)
    if out.stdout.strip():
        return
    home = os.path.expanduser("~")
    cmd = ("cd %s/auto/codes && setsid nohup python3 -u arm_real_node.py %s "
           "_world:=%s/codes/config/arena_map.yaml "
           "> %s/logs/run/arm_auto.log 2>&1 < /dev/null &"
           % (home, PICK_SETTINGS, home, home))
    subprocess.run(["bash", "-lc", cmd], check=False)
    time.sleep(8)


def main():
    ensure_arm_node()
    Auto().run()


if __name__ == "__main__":
    main()
