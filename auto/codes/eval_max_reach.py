#!/usr/bin/env python3
import match_logic

for x in (330, 340, 345, 350, 352, 355, 358, 360, 365, 370):
    ok = match_logic.ik_reachable(x, 4.5, 38.0)
    print("X=%.1f -> ik_reachable: %s" % (x, ok))
