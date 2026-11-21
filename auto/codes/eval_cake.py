#!/usr/bin/env python3
import numpy as np
import match_logic

M = np.load("/home/spark/transform_3d.dat")
print("M loaded, shape:", M.shape)

for v in (71, 248):
    for z in (40, 50, 0, -55):
        xy = match_logic.pixel_to_xy_at_z(M, 344, v, z)
        print("v=%d, z=%d -> xy=%s" % (v, z, xy))
    cake_xy = match_logic.pixel_to_cake_xy(M, 344, v, cup_offset_x=30.0)
    print("v=%d -> pixel_to_cake_xy=%s" % (v, cake_xy))
