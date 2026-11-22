#!/usr/bin/env python3
"""Reproduce stored target (pixel -> xy) pairs with both candidate matrices."""
import json
import sys

import numpy as np

sys.path.insert(0, r"D:\NXROBO\auto\codes")
import match_logic as ml  # noqa: E402

LOG = r"D:\NXROBO\.tmp\grasp_generic.jsonl"
M040 = np.load(r"D:\NXROBO\.tmp\calib_current.dat", allow_pickle=True)
M06 = np.load(r"D:\NXROBO\.tmp\calib_06c7.dat", allow_pickle=True)

rows = []
with open(LOG, encoding="utf-8") as fh:
    for line in fh:
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except Exception:
            pass

print("entries:", len(rows))
print("%-20s %-13s %-10s %-22s %-22s %-22s" % ("ts", "result", "mode", "stored xy", "040c DLT(+20)", "06c7 DLT(+20)"))
for row in rows[-14:]:
    target = row.get("target") or {}
    if not target:
        continue
    uv = target.get("pixel") or target.get("uv")
    xy = target.get("xy")
    if not uv or not xy:
        continue
    a = ml.pixel_to_xy_at_z(M040, uv[0], uv[1], -100.0)
    b = ml.pixel_to_xy_at_z(M06, uv[0], uv[1], -100.0)
    fmt = lambda p: "(%7.1f, %7.1f)" % (p[0] + 20.0, p[1]) if p else "None"
    print("%-20s %-13s %-10s (%7.1f, %7.1f)   %-22s %-22s"
          % (row.get("ts", "?"), row.get("result"), target.get("mode"),
             xy[0], xy[1], fmt(a), fmt(b)))
