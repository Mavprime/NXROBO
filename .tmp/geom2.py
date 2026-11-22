import sys
import numpy as np
sys.path.insert(0, r"D:\NXROBO\auto\codes")
import match_logic
M = np.load(r"D:\NXROBO\auto\codes\transform_3d.dat", allow_pickle=True)

print("== project (x, y, z) grid ==")
for z in (-55.0, -80.0, -100.0):
    print("-- z=%.0f --" % z)
    for y in (-100.0, 0.0, 100.0):
        row = []
        for x in (300.0, 350.0, 400.0, 450.0, 500.0, 600.0, 700.0):
            uv = match_logic.project_arm(M, x, y, z)
            row.append("%s" % (None if uv is None else "(%.0f,%.0f)" % uv))
        print("  y=%5.0f: %s" % (y, "  ".join(row)))

print("\n== inverse at z=-100, u=320 ==")
for v in (40, 50, 60, 70, 80, 90, 100, 107, 110, 120, 130, 150, 170, 200, 250):
    xy = match_logic.pixel_to_xy_at_z(M, 320, v, -100.0)
    print("v=%3d -> (%.1f, %.1f)" % (v, xy[0], xy[1]))

print("\n== inverse at z=-55, u=320 ==")
for v in (60, 80, 90, 100, 107, 110, 120, 130, 150, 170):
    xy = match_logic.pixel_to_xy_at_z(M, 320, v, -55.0)
    print("v=%3d -> (%.1f, %.1f)" % (v, xy[0], xy[1]))
