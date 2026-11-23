#!/usr/bin/env python3
"""Print a compact lidar diagnostic (no motion): line families and near points."""
import math

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

odom = {"yaw": None}
rospy.init_node("diag_scan")


def _odom(m):
    q = m.pose.pose.orientation
    odom["yaw"] = math.degrees(math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z))


rospy.Subscriber("/odom", Odometry, _odom)
m = rospy.wait_for_message("/scan", LaserScan, timeout=5)
rospy.sleep(0.5)

r = np.array(m.ranges)
a = m.angle_min + np.arange(len(r)) * m.angle_increment
ok = np.isfinite(r) & (r > 0.10) & (r < 3.2)
P = np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])
print("angle_min %.3f angle_max %.3f inc %.5f n=%d valid=%d odom_yaw=%s"
      % (m.angle_min, m.angle_max, m.angle_increment, len(r), int(ok.sum()),
         "%.1f" % odom["yaw"] if odom["yaw"] is not None else "?"))

near = P[np.hypot(P[:, 0], P[:, 1]) < 0.70]
print("near(<0.70m) n=%d x[%.2f %.2f] y[%.2f %.2f]"
      % (len(near), near[:, 0].min(), near[:, 0].max(),
         near[:, 1].min(), near[:, 1].max()))
ang = np.degrees(np.arctan2(near[:, 1], near[:, 0]))
for lo, hi in ((0, 0),):                      # placeholder, keep linters calm
    pass
hist, edges = np.histogram(ang, bins=np.arange(-180, 181, 10))
for i, h in enumerate(hist):
    if h:
        print("sector %+4.0f..%+4.0f: %3d" % (edges[i], edges[i + 1], h))

best = []
for th in np.arange(0.0, 180.0, 0.5):
    n = np.array([math.cos(math.radians(th)), math.sin(math.radians(th))])
    h, e = np.histogram(P @ n, bins=np.arange(P.min(), P.max() + 0.01, 0.01))
    i = int(np.argmax(h))
    if h[i] >= 25:
        best.append((int(h[i]), float(th), float(e[i] + 0.005)))
best.sort(reverse=True)
print("-- line families --")
shown = []
for c, th, p0 in best:
    n = np.array([math.cos(math.radians(th)), math.sin(math.radians(th))])
    if any(abs(th - s[0]) < 8.0 and abs(p0 - s[1]) < 0.08 for s in shown):
        continue
    shown.append((th, p0))
    sel = np.abs(P @ n - p0) < 0.04
    Q = P[sel]
    tang = np.array([-n[1], n[0]])
    t = Q @ tang
    p = Q @ n
    bearing = th if p0 >= 0 else (th + 180.0) % 360.0
    if bearing > 180.0:
        bearing -= 360.0
    print("theta %5.1f p0 %+.3f n=%4d bearing %+6.1f pmean %+.3f pstd %.3f "
          "tangent %+.2f..%+.2f" % (th, p0, c, bearing, p.mean(), p.std(),
                                    t.min(), t.max()))
    if len(shown) >= 8:
        break
