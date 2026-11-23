import sys, os
sys.path.insert(0, r"D:\NXROBO\auto\codes")
import numpy as np
import match_logic as ml

M = np.load(r"D:\NXROBO\auto\codes\transform_3d.dat")
print("far floor (345,107):", ml.far_target_xy(M, 345, 107, False))
print("reach floor far:", ml.grasp_reach_state("floor", *ml.far_target_xy(M, 345, 107, False), far=True))
print("reach floor far coords w/o flag:", ml.grasp_reach_state("floor", *ml.far_target_xy(M, 345, 107, False)))
print("far cake (345,107) raw:", ml.far_target_xy(M, 345, 107, True))
print("cake clamped:", ml.pixel_to_cake_xy(M, 345, 107))
print("cake far reach:", ml.grasp_reach_state("cake", *ml.far_target_xy(M, 345, 107, True)))
print("cake clamped reach:", ml.grasp_reach_state("cake", *ml.pixel_to_cake_xy(M, 345, 107)))
print("near floor (346,294):", ml.far_target_xy(M, 346, 294, False))
print("near floor reach:", ml.grasp_reach_state("floor", *ml.far_target_xy(M, 346, 294, False)))
print("height floor:", ml.grasp_heights("floor"))
print("height cake:", ml.grasp_heights("cake", None, 40.0))
print("dist far:", ml.approach_target_distance("floor", *ml.far_target_xy(M, 345, 107, False)))
