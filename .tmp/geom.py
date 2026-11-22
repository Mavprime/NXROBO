import sys
import numpy as np
sys.path.insert(0, r"D:\NXROBO\auto\codes")
import match_logic

M = np.load(r"D:\NXROBO\auto\codes\transform_3d.dat", allow_pickle=True)
print("M shape", M.shape)
print(M)

print("\n== project floor wrist points (y=0) ==")
for x in (150, 200, 250, 290, 300, 320, 350, 400, 450, 500, 600):
    uv = match_logic.project_arm(M, x, 0.0, -55.0)
    print("x=%4d z=-55 -> u,v = %s" % (x, None if uv is None else tuple(round(v,1) for v in uv)))

print("\n== project cake wrist points (y=0, z=40) ==")
for x in (200, 250, 290, 320, 348, 380, 400, 450, 500, 600):
    uv = match_logic.project_arm(M, x, 0.0, 40.0)
    print("x=%4d z=+40 -> u,v = %s" % (x, None if uv is None else tuple(round(v,1) for v in uv)))

print("\n== inverse: pixel_to_xy_at_z at floor z=-100 ==")
for v in (60, 80, 100, 110, 120, 130, 140, 160, 180, 200, 220, 250, 280, 300, 350, 400):
    xy = match_logic.pixel_to_xy_at_z(M, 320, v, -100.0)
    print("v=%3d -> xy=%s" % (v, None if xy is None else (round(xy[0],1), round(xy[1],1))))

print("\n== cake affine (u=320) ==")
for v in (60, 80, 100, 110, 120, 130, 140, 160, 180, 200, 220, 250):
    x, y = match_logic.pixel_to_cake_xy(M, 320, v)
    raw = -0.8030661953564442 * v + 395.53156305089396 + 125.0
    print("v=%3d -> clamped xy=(%.1f, %.1f) raw_x=%.1f" % (v, x, y, raw))
