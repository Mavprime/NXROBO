#!/usr/bin/env python3
import numpy as np
import match_logic

M = np.load("/home/spark/transform_3d.dat")
u, v = 350.0, 159.0

print("--- thefile2.txt with different cup_offset_x ---")
for off in (30.0, 45.0, 50.0, 55.0, 60.0):
    xy = match_logic.pixel_to_cake_xy(M, u, v, cup_offset_x=off)
    print("offset=+%.1f mm -> x=%.1f, y=%.1f" % (off, xy[0], xy[1]))

print("\n--- DLT pixel_to_xy_at_z from ~/transform_3d.dat ---")
for z in (35.0, 40.0, 45.0, 50.0):
    xy = match_logic.pixel_to_xy_at_z(M, u, v, z)
    print("z=%.1f mm -> x=%.1f, y=%.1f" % (z, xy[0], xy[1]))
