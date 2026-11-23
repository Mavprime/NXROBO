#!/usr/bin/env python3
"""Predict the runtask active pad check for the shipped 040c matrix.

The calibration sidecar is stale (it belongs to the lost 15:38 refit), but its
``check`` frame is a real measurement: at wrist pose (270, 0, 30) mm the pad
centre was at pixel (338.14, 294.96).  verify_calibration() falls back to
comparing the measured pad against the *projection* of that pose under the
current matrix, so that distance is what the operator would see.
"""
import math
import sys

import numpy as np

sys.path.insert(0, r"D:\NXROBO\auto\codes")
import match_logic  # noqa: E402

M = np.load(r"D:\NXROBO\auto\codes\transform_3d.dat", allow_pickle=True)
print("matrix shape:", M.shape)
print(M)

pose = (270.0, 0.0, 30.0)
uv_measured = (338.14, 294.96)
proj = match_logic.project_arm(M, pose[0], pose[1], pose[2])
print("project_arm(%s) = %s" % (pose, proj))
if proj:
    err = math.hypot(proj[0] - uv_measured[0], proj[1] - uv_measured[1])
    print("distance to stored 15:38 measurement: %.2f px (tolerance 6.0)" % err)
    print("predicted pad check: %s" % ("PASS" if err <= 6.0 else "FAIL"))
