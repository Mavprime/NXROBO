#!/usr/bin/env python3
"""Compare the two candidate hand-eye matrices against the 15:38 pad measurement."""
import sys

import numpy as np

sys.path.insert(0, r"D:\NXROBO\auto\codes")
import match_logic as ml  # noqa: E402

POSE = (270.0, 0.0, 30.0)
UV = (338.14, 294.96)

for label, path in (
    ("current 040c", r"D:\NXROBO\.tmp\calib_current.dat"),
    ("bak-1601 06c7", r"D:\NXROBO\.tmp\calib_06c7.dat"),
):
    M = np.load(path, allow_pickle=True)
    uv = ml.project_arm(M, POSE[0], POSE[1], POSE[2])
    err = np.hypot(uv[0] - UV[0], uv[1] - UV[1])
    xy = ml.pixel_to_xy_at_z(M, UV[0], UV[1], POSE[2])
    print("%-15s projects %s -> (%.1f, %.1f)  |err|=%.1f px" % (label, POSE, uv[0], uv[1], err))
    if xy:
        print("%-15s inverse of %s at z=30 -> (%.1f, %.1f) mm" % (label, UV, xy[0], xy[1]))
    ok, problems, stats = ml.check_transform(M)
    print("%-15s check=%s stats=%s" % (label, ok, {k: round(v, 3) if isinstance(v, float) else v for k, v in stats.items()}))
