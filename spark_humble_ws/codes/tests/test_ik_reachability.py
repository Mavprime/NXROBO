import unittest
import math
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import arm_ik


class TestArmIKReachability(unittest.TestCase):
    """Test suite for uArm Swift Pro kinematic reachability and coordinate transforms."""

    def test_standard_grasp_poses_are_reachable(self):
        """Verify that known operational poses in the competition workspace are reachable."""
        # Tier 1 pick point (X ~ 235 mm, Y ~ 0 mm, Z ~ -55 mm)
        self.assertTrue(
            arm_ik.ik_reachable(235.0, 0.0, -55.0),
            "Tier 1 standard pick pose (235, 0, -55) must be reachable"
        )

        # Tier 2 pick point (Cake on central pedestal, Z ~ 45 mm)
        self.assertTrue(
            arm_ik.ik_reachable(235.0, 0.0, 45.0),
            "Tier 2 Cake pick pose (235, 0, 45) must be reachable"
        )

        # Retracted service/ready pose (50, 180, 160)
        self.assertTrue(
            arm_ik.ik_reachable(50.0, 180.0, 160.0),
            "Service/ready pose (50, 180, 160) must be reachable"
        )

        # Symmetric lateral reaching (-100 and +100 mm in Y)
        self.assertTrue(
            arm_ik.ik_reachable(200.0, 100.0, -50.0),
            "Lateral reach (+Y) must be reachable"
        )
        self.assertTrue(
            arm_ik.ik_reachable(200.0, -100.0, -50.0),
            "Lateral reach (-Y) must be reachable"
        )

    def test_out_of_bounds_poses_are_rejected(self):
        """Unreachable positions must return False without throwing exceptions."""
        # Far beyond maximum arm extension (~320 mm)
        self.assertFalse(arm_ik.ik_reachable(600.0, 0.0, 0.0), "X=600 mm is beyond maximum reach")
        self.assertFalse(arm_ik.ik_reachable(0.0, 600.0, 0.0), "Y=600 mm is beyond maximum reach")
        self.assertFalse(arm_ik.ik_reachable(200.0, 0.0, 800.0), "Z=800 mm is physically impossible")

    def test_zero_and_edge_values_do_not_crash(self):
        """Ensure no ZeroDivisionError or domain errors occur on singular inputs."""
        try:
            res_zero = arm_ik.ik_reachable(0.0, 0.0, 0.0)
            self.assertIsInstance(res_zero, bool)

            res_neg_x = arm_ik.ik_reachable(-100.0, 0.0, 0.0)
            self.assertIsInstance(res_neg_x, bool)

            res_nan_check = arm_ik.ik_reachable(0.0001, -0.0001, -55.0)
            self.assertIsInstance(res_nan_check, bool)
        except ZeroDivisionError as e:
            self.fail(f"ik_reachable raised ZeroDivisionError on edge inputs: {e}")

    def test_pixel_to_arm_transform(self):
        """Linear pixel-to-arm mapping must accurately evaluate affine coefficients."""
        # Example calibration: row drives arm_x, col drives arm_y
        x_kb = (0.5, 50.0)   # arm_x = 0.5 * row + 50.0
        y_kb = (-0.4, 20.0)  # arm_y = -0.4 * col + 20.0

        pixel_row = 300
        pixel_col = 200

        arm_x, arm_y = arm_ik.pixel_to_arm(pixel_row, pixel_col, x_kb, y_kb)
        self.assertAlmostEqual(arm_x, 0.5 * 300 + 50.0)
        self.assertAlmostEqual(arm_y, -0.4 * 200 + 20.0)

    def test_pixel_reachable_checks_all_heights(self):
        """pixel_reachable must return True only if all required Z heights are achievable."""
        x_kb = (0.2, 180.0)
        y_kb = (0.1, -20.0)

        # Heights for descent, grasp, and lift
        z_valid = [-55.0, 0.0, 60.0]
        self.assertTrue(arm_ik.pixel_reachable(240, 320, z_valid, x_kb, y_kb))

        # Include an impossible height
        z_invalid = [-55.0, 999.0]
        self.assertFalse(arm_ik.pixel_reachable(240, 320, z_invalid, x_kb, y_kb))


if __name__ == "__main__":
    unittest.main()
