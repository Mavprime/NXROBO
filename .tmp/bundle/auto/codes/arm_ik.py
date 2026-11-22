#!/usr/bin/env python3
"""
Arm reachability + pixel-to-arm conversion for the Spark uArm.

Compatibility shim: re-exports the single source of truth from match_logic.py.
The IK test is the same mathematics used by swiftpro_rviz_node.cpp (and by
the M-Robots semi-auto grasp code). If the firmware's IK cannot reach a pose
it silently refuses the command: the arm simply does not move. Checking first
avoids "it never moved" surprises.
"""

from match_logic import (
    MATH_TRANS,
    MATH_L1,
    MATH_L2,
    MATH_LOWER_ARM,
    MATH_UPPER_ARM,
    MATH_UPPER_LOWER,
    pixel_to_arm,
    ik_reachable,
    pixel_reachable,
)

__all__ = [
    "MATH_TRANS",
    "MATH_L1",
    "MATH_L2",
    "MATH_LOWER_ARM",
    "MATH_UPPER_ARM",
    "MATH_UPPER_LOWER",
    "pixel_to_arm",
    "ik_reachable",
    "pixel_reachable",
]
