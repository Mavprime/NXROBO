import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _bootstrap import arm_ik, match_logic  # noqa: E402


class TestArmIKReachability(unittest.TestCase):
    """Reachability of the real uArm Swift Pro workspace."""

    def test_operational_poses_are_reachable(self):
        self.assertTrue(arm_ik.ik_reachable(235.0, 0.0, -55.0),
                        "tier-1 pick pose must be reachable")
        self.assertTrue(arm_ik.ik_reachable(235.0, 0.0, 45.0),
                        "tier-2 cake pose must be reachable")
        self.assertTrue(arm_ik.ik_reachable(50.0, 180.0, 160.0),
                        "service/tuck pose must be reachable")
        self.assertTrue(arm_ik.ik_reachable(200.0, 100.0, -50.0))
        self.assertTrue(arm_ik.ik_reachable(200.0, -100.0, -50.0))

    def test_out_of_bounds_poses_are_rejected(self):
        self.assertFalse(arm_ik.ik_reachable(600.0, 0.0, 0.0))
        self.assertFalse(arm_ik.ik_reachable(0.0, 600.0, 0.0))
        self.assertFalse(arm_ik.ik_reachable(200.0, 0.0, 800.0))

    def test_degenerate_inputs_do_not_crash(self):
        for point in ((0.0, 0.0, 0.0), (-100.0, 0.0, 0.0), (0.0001, -0.0001, -55.0)):
            self.assertIsInstance(arm_ik.ik_reachable(*point), bool)

    def test_pixel_to_arm_mapping(self):
        x_kb = (0.5, 50.0)    # arm_x = 0.5 * row + 50
        y_kb = (-0.4, 20.0)   # arm_y = -0.4 * col + 20
        arm_x, arm_y = arm_ik.pixel_to_arm(300, 200, x_kb, y_kb)
        self.assertAlmostEqual(arm_x, 0.5 * 300 + 50.0)
        self.assertAlmostEqual(arm_y, -0.4 * 200 + 20.0)

    def test_pixel_reachable_checks_every_height(self):
        x_kb = (0.2, 180.0)
        y_kb = (0.1, -20.0)
        self.assertTrue(arm_ik.pixel_reachable(240, 320, [-55.0, 0.0, 60.0], x_kb, y_kb))
        self.assertFalse(arm_ik.pixel_reachable(240, 320, [-55.0, 999.0], x_kb, y_kb))


class TestSingleSourceOfKinematics(unittest.TestCase):
    """arm_ik.py is a shim: runtask.py and the shim must share one implementation."""

    def test_shim_reexports_the_same_function_objects(self):
        self.assertIs(arm_ik.ik_reachable, match_logic.ik_reachable)
        self.assertIs(arm_ik.pixel_to_arm, match_logic.pixel_to_arm)
        self.assertIs(arm_ik.pixel_reachable, match_logic.pixel_reachable)

    def test_constants_match(self):
        self.assertEqual(arm_ik.MATH_L1, match_logic.MATH_L1)
        self.assertEqual(arm_ik.MATH_LOWER_ARM, match_logic.MATH_LOWER_ARM)


if __name__ == "__main__":
    unittest.main()
