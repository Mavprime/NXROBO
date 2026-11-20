#!/usr/bin/env python3
"""
Arm reachability + pixel-to-arm conversion for the Spark uArm.

The IK test is the same mathematics used by swiftpro_rviz_node.cpp (and by
the M-Robots semi-auto grasp code). If the firmware's IK cannot reach a pose
it silently refuses the command: the arm simply does not move. Checking first
avoids "it never moved" surprises.

Pure Python, no ROS imports, so it can be tested anywhere.

Coordinate convention (from every Spark grasp example):
    arm_x = k1 * pixel_row + b1
    arm_y = k2 * pixel_col + b2
i.e. the image row drives x and the image column drives y. Do not "fix" this
by swapping: it is how the calibration file was generated.
"""

import math

MATH_TRANS = 57.2958
MATH_L1 = 106.6
MATH_L2 = 13.2
MATH_LOWER_ARM = 142.07
MATH_UPPER_ARM = 158.81
MATH_UPPER_LOWER = MATH_UPPER_ARM / MATH_LOWER_ARM


def pixel_to_arm(pixel_row, pixel_col, x_kb, y_kb):
    """Return (x, y) in millimetres from an image pixel.

    x_kb = (k1, b1), y_kb = (k2, b2), as stored in ~/thefile.txt.
    """
    return x_kb[0] * pixel_row + x_kb[1], y_kb[0] * pixel_col + y_kb[1]


def ik_reachable(x, y, z):
    """True if the arm can reach (x, y, z) in millimetres."""
    z = z + 74.55
    z_in = (z - MATH_L1) / MATH_LOWER_ARM
    if x < 0.1:
        x = 0.1
    if y == 0:
        rot = 90.0
    elif y < 0:
        rot = -math.atan(x / y) * MATH_TRANS
    else:
        rot = 180.0 - math.atan(x / y) * MATH_TRANS

    sin_rot = math.sin(rot / MATH_TRANS)
    if sin_rot == 0:
        return False

    x_in = (x / sin_rot - MATH_L2 - 56.55) / MATH_LOWER_ARM
    if x_in == 0:
        return False

    sqrt_zx = math.sqrt(z_in * z_in + x_in * x_in)
    if sqrt_zx == 0:
        return False

    upper = MATH_UPPER_LOWER
    right_all = (sqrt_zx * sqrt_zx + upper * upper - 1) / (2 * upper * sqrt_zx)
    left_all = (sqrt_zx * sqrt_zx + 1 - upper * upper) / (2 * sqrt_zx)
    return abs(right_all) <= 1 and abs(left_all) <= 1


def pixel_reachable(pixel_row, pixel_col, z_list, x_kb, y_kb):
    """True if the pixel is reachable at every height in z_list."""
    arm_x, arm_y = pixel_to_arm(pixel_row, pixel_col, x_kb, y_kb)
    return all(ik_reachable(arm_x, arm_y, z) for z in z_list)
