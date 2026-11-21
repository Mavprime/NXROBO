#!/usr/bin/env python3
import match_logic

for x in (280, 298, 315, 325, 335, 345, 355):
    ok = match_logic.ik_reachable(x, 4.5, 38.0)
    print("X=%.1f, Y=4.5, Z=38.0 -> ik_reachable: %s" % (x, ok))
