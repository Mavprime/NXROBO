#!/usr/bin/env python3
"""Compare the DLT plane inversion against the legacy Layer-2 cake affine."""
import sys

sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")

import numpy as np

import match_logic

M = np.load("/home/spark/transform_3d.dat")
print("M shape", M.shape)

def show(label, u, v, z):
    xy = match_logic.pixel_to_xy_at_z(M, u, v, z)
    print("%-28s pixel=(%3.0f,%3.0f) z=%+5.0f -> x=%7.1f y=%7.1f"
          % (label, u, v, z, xy[0], xy[1]))

# Observed detections (cake top face, front-row bear cube top face).
for z in (-50.0, 0.0, 40.0, 50.0):
    show("cake top @ z=%+d" % int(z), 362.0, 71.0, z)
for z in (-100.0, -50.0):
    show("front-row bear @ z=%+d" % int(z), 351.0, 311.0, z)

print()
print("legacy cake affine (thefile2 +30mm):",
      match_logic.pixel_to_cake_xy(M, 362.0, 71.0))

# Project a few candidate poses back to pixels to see which lands on (362, 71).
def project(x, y, z):
    cam = np.array([x, y, z, 1.0]) @ M
    if cam[2] <= 1e-6:
        return None
    return (match_logic.CAM_FX * cam[0] / cam[2] + match_logic.CAM_CX,
            match_logic.CAM_FY * cam[1] / cam[2] + match_logic.CAM_CY)

print()
for pose in [(304.7, -16.1, 50.0), (350.0, -3.1, 50.0), (330.0, -10.0, 50.0),
             (340.0, -10.0, 50.0), (320.0, -15.0, 50.0)]:
    print("project %-22s -> %s" % (str(pose), project(*pose)))
