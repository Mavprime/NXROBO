#!/usr/bin/env python3
import numpy as np
import match_logic

M = np.load("/home/spark/transform_3d.dat")

def project(x, y, z):
    cam = np.array([x, y, z, 1.0]) @ M
    if cam[2] <= 1e-6:
        return None
    return (match_logic.CAM_FX * cam[0] / cam[2] + match_logic.CAM_CX,
            match_logic.CAM_FY * cam[1] / cam[2] + match_logic.CAM_CY)

for x in (250, 220, 200, 180, 160, 140):
    uv = project(x, 0.0, -55.0)
    print("X=%d, Z=-55 -> u=%.1f, v=%.1f" % (x, uv[0], uv[1]))
