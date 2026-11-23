#!/usr/bin/env python3
"""Save named map anchors from the real robot, wall-fixed (no AMCL).

Use this to re-measure the arena: put the robot at A facing the centre, run
this, then drive with teleop in another terminal. At each spot type a name and
press Enter here; Ctrl+C finishes. The file is written after every record.

    python3 -u ~/codes/robot/anchor_log.py                 # start at A
    python3 -u ~/codes/robot/anchor_log.py --yaw0 -45      # if the wall fix
                                                            # cannot resolve

Each record holds, in the map frame:
  * x, y from the wall distances (west/east and north/south; both when the
    robot can see opposite walls, so the two estimates cross-check)
  * the four map-bearing ranges this tool used
  * the odom pose and the tracked map heading
  * the wall-line families seen (for a sanity check afterwards)

Suggested names:
  A            collection centre (recorded automatically at start)
  class        beside the classification table, where you stand to read it
  W / N / S / E  pick stand-off in front of the cluster on that side
  A_approach   where you place the cake in A
  ...

Output: ~/logs/anchors/anchors_<timestamp>.json
Send that file back for the map rebuild (arena_map.yaml + scenario + anchors).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time

import rospy

try:
    from a2w import A2W, WALL_OFF, MAP_TOP
except ImportError:                      # running from another directory
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from a2w import A2W, WALL_OFF, MAP_TOP


class AnchorLog(A2W):
    def wall_pose(self):
        dw, de = self.dist(180), self.dist(0)
        dn, ds = self.dist(90), self.dist(270)
        xs = []
        if not math.isnan(dw) and 0.10 < dw < 3.10:
            xs.append(("west", WALL_OFF + dw))
        if not math.isnan(de) and 0.10 < de < 3.10:
            xs.append(("east", MAP_TOP - de))
        ys = []
        if not math.isnan(dn) and 0.10 < dn < 3.10:
            ys.append(("north", MAP_TOP - dn))
        if not math.isnan(ds) and 0.10 < ds < 3.10:
            ys.append(("south", WALL_OFF + ds))
        # prefer the near wall on each axis; report the disagreement
        x = xs[0][1] if xs else None
        y = ys[0][1] if ys else None
        x_span = (max(v for _, v in xs) - min(v for _, v in xs)) if len(xs) > 1 else 0.0
        y_span = (max(v for _, v in ys) - min(v for _, v in ys)) if len(ys) > 1 else 0.0
        return {"x": x, "y": y,
                "ranges": {"west": dw, "east": de, "north": dn, "south": ds},
                "sides": {"x": xs, "y": ys},
                "span": {"x": x_span, "y": y_span}}

    def record(self, name):
        o = self.odom.pose.pose
        q = o.orientation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z))
        wp = self.wall_pose()
        rec = {
            "name": name,
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "x": wp["x"], "y": wp["y"],
            "wall_fix": wp,
            "odom": {"x": o.position.x, "y": o.position.y, "yaw": yaw},
            "map_heading": self.h(),
            "lines": [(b, d) for b, d, _ in (self.wall_lines() or [])],
        }
        self.records.append(rec)
        with open(self.out, "w") as f:
            json.dump(self.records, f, indent=1)
        print("[ANCHOR] %-12s x=%s y=%s  ranges W/E/N/S = %.2f/%.2f/%.2f/%.2f  "
              "span x %.2f y %.2f  h=%.0f"
              % (name,
                 "%.3f" % wp["x"] if wp["x"] is not None else "?",
                 "%.3f" % wp["y"] if wp["y"] is not None else "?",
                 wp["ranges"]["west"], wp["ranges"]["east"],
                 wp["ranges"]["north"], wp["ranges"]["south"],
                 wp["span"]["x"], wp["span"]["y"], self.h()), flush=True)

    def run(self, yaw0=None, start=(0.225, 2.775)) -> None:
        if yaw0 is not None:
            self.h0 = yaw0
            self.pos0 = (float(start[0]), float(start[1]))
            self.log("explicit start: (%.3f, %.3f), heading %.1f"
                     % (self.pos0[0], self.pos0[1], self.h0))
        else:
            self.init_heading()
            x, y = self.pos0
            self.log("start pose (wall fix): (%.3f, %.3f), heading %.1f"
                     % (x, y, self.h()))
            if math.hypot(x - 0.225, y - 2.775) > 0.35:
                self.log("WARNING: start is %.0f cm from A - positions will be "
                         "wall-relative anyway, but check the heading"
                         % (100 * math.hypot(x - 0.225, y - 2.775)))
        self.records = []
        self.out = os.path.expanduser(
            "~/logs/anchors/anchors_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(os.path.dirname(self.out), exist_ok=True)
        print("[ANCHOR] file: %s" % self.out, flush=True)
        print("[ANCHOR] drive with teleop; type a name + Enter to record; "
              "Ctrl+C to finish", flush=True)
        self.record("A_start")
        while not rospy.is_shutdown():
            try:
                name = input("anchor name> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not name:
                continue
            if name in ("q", "quit", "exit"):
                break
            self.record(name)
        self.stop()
        print("[ANCHOR] saved %d records to %s" % (len(self.records), self.out),
              flush=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--yaw0", type=float, default=None,
                    help="explicit map heading at the start (skips the wall fit)")
    ap.add_argument("--start", nargs=2, type=float, default=(0.225, 2.775),
                    metavar=("X", "Y"), help="start position for --yaw0")
    args = ap.parse_args()
    AnchorLog().run(yaw0=args.yaw0, start=args.start)


if __name__ == "__main__":
    main()
