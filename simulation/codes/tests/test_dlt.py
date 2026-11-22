#!/usr/bin/env python3
"""Tests for the production DLT path (transform_3d.dat + arm mount)."""

import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
CODE_DIR = os.path.dirname(HERE)
sys.path.insert(0, CODE_DIR)

from projection import (  # noqa: E402
    ARM_MOUNT_X,
    arm_to_base,
    load_dlt_matrix,
    pixel_to_arm_at_z,
    project_arm,
)

MATRIX = os.path.join(CODE_DIR, "..", "config", "transform_3d.dat")


class DLTTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = load_dlt_matrix(MATRIX)

    def test_matrix_shape(self):
        self.assertEqual(self.m.shape, (4, 3))

    def test_documented_calibration_point(self):
        # The 2026-11-21 recalibration documents pixel (330,174) -> (315, 0) mm.
        x, y = pixel_to_arm_at_z(self.m, 330, 174, -100.0)
        self.assertAlmostEqual(x, 314.5, delta=1.5)
        self.assertAlmostEqual(y, -0.9, delta=1.5)

    def test_round_trip(self):
        for u, v, z in ((348, 288, -100), (330, 174, -100),
                        (255, 222, -100), (320, 240, -100),
                        (300, 300, 0)):
            arm = pixel_to_arm_at_z(self.m, u, v, z)
            self.assertIsNotNone(arm)
            uv = project_arm(self.m, arm[0], arm[1], z)
            self.assertIsNotNone(uv)
            self.assertLess(math.hypot(uv[0] - u, uv[1] - v), 0.5)

    def test_arm_to_base(self):
        x, y = arm_to_base(199.2, -9.5)
        self.assertAlmostEqual(x, ARM_MOUNT_X + 0.1992)
        self.assertAlmostEqual(y, -0.0095)


if __name__ == "__main__":
    unittest.main(verbosity=2)
