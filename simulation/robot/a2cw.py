#!/usr/bin/env python3
"""Stage 1 of the real mission: A -> classification area -> Sorting_W.

Starts at A collection centre (0.225, 2.775), "45 deg" toward the arena centre.
Drives south to beside the classification table, east to the guided
classification anchor (0.323, 1.776), then uses the clear lane between the
table and the cube cluster to reach Sorting_W (1.090, 1.500), finishing at
0 deg facing the cluster - ready for the cake pick.

The table stops the south leg: the forward guard halts 0.22 m short of its
north face (the guided anchor itself is 7.6 cm from the map table corner,
which is inside the chassis footprint). The robot is therefore logged as
"beside the classification area", not on the exact anchor.

Uses a2w's lidar wall fixes + odometry; no AMCL. Stops at W for the operator
check before any arm motion.

Run on the robot:  python3 -u ~/codes/robot/a2cw.py
"""

from __future__ import annotations

import math

import a2w
from a2w import A2W, WALL_OFF, MAP_TOP, W_XY, W_HEADING, TRANSIT_X

CLASS_XY = (0.323, 1.776)        # anchor_classification.json, guided from A
CLASS_HEADING = 0.0              # anchor heading (robot faces east)
CLASS_GUARD = 0.22               # stop this far short of the table north face
# (the table north face is map y = 1.5 + 0.4 / 2 = 1.70)


class A2CW(A2W):
    def run(self) -> None:
        self.init_heading()
        x, y = self.pos0
        self.log("start pos = (%.3f, %.3f)  [A is %s]  offset %.1f cm, heading %.1f"
                 % (x, y, (0.225, 2.775),
                    100 * math.hypot(x - 0.225, y - 2.775), self.h()))
        if math.hypot(x - 0.225, y - 2.775) > 0.30:
            raise RuntimeError("not at A - refusing to drive (%.2f, %.2f)" % (x, y))

        # 1. south until the table guard stops us beside the classification area
        dy = y - CLASS_XY[1]
        if dy > 0.02:
            self.turn_to(270)
            self.drive(dy, vmax=0.05, timeout=60, guard=CLASS_GUARD)
        # table north face is map y=1.70; the median range due south is the gap
        d_tab = self.dist(270)
        y_class = (1.70 + d_tab) if not math.isnan(d_tab) and d_tab < 0.6 else y - 0.95
        self.log("beside classification area: y~%.3f (anchor y=%.3f)" % (y_class, CLASS_XY[1]))

        # 2. east to the classification anchor x (west wall is clear at this y)
        d_w = self.dist(180)
        if not math.isnan(d_w) and 0.2 < d_w < 1.4:
            x = WALL_OFF + d_w
        dx = CLASS_XY[0] - x
        self.log("classification x: measured %.3f -> anchor %.3f (dx %.3f)"
                 % (x, CLASS_XY[0], dx))
        if abs(dx) > 0.02:
            self.turn_to(0 if dx > 0 else 180)
            self.drive(abs(dx), vmax=0.03, timeout=20)
        self.turn_to(CLASS_HEADING)
        self.log("AT CLASSIFICATION AREA (visited; the cake -> A rule needs no board read)")

        # 3. east along the clear lane to x = TRANSIT_X (west-wall range pins x)
        self.turn_to(0)
        self.drive_until_far(180, TRANSIT_X - WALL_OFF, vmax=0.06, timeout=60)

        # 4. south until the south wall is 1.675 m away -> y = 1.500
        self.turn_to(270)
        self.drive_until(270, W_XY[1] - WALL_OFF, vmax=0.05, timeout=120)

        # 5. east into W, measured from the table east face or the west wall
        self.turn_to(0)
        x_now, src = self.wall_x(self.dist(180))
        if x_now is None:
            self.log("no west-side fix before W (%s): assuming x=%.2f"
                     % (src, TRANSIT_X))
            x_now = TRANSIT_X
        dx = W_XY[0] - x_now
        self.log("approach x: %.3f (%s) -> W %.3f (dx %.3f)"
                 % (x_now, src, W_XY[0], dx))
        if abs(dx) > 0.02:
            self.turn_to(0 if dx > 0 else 180)
            self.drive(abs(dx), vmax=0.04, timeout=30)
        self.turn_to(W_HEADING)

        self.correct_at_W()
        self.stop()
        self.log("DONE - at W, arm empty. Check the pose before the pick.")


def main() -> None:
    A2CW().run()


if __name__ == "__main__":
    main()
