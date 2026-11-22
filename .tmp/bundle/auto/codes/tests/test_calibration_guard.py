"""Guards around the camera->arm (hand-eye) calibration.

The expensive failure this suite protects against: a stale or broken
``transform_3d.dat`` makes every grasp miss by 5-7 cm while every subsystem
reports "OK".  ``check_transform`` catches the cheap breakages at startup and
``pad_projection_error`` is the number the active pad check compares.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _bootstrap import CODES_DIR, match_logic  # noqa: E402

try:
    import numpy as np
except ImportError:            # the robot has numpy; keep the suite importable
    np = None

CALIB_FILE = os.path.join(CODES_DIR, "transform_3d.dat")
CHECK_POSE = (270.0, 0.0, 40.0)


@unittest.skipUnless(np is not None and os.path.exists(CALIB_FILE),
                     "needs numpy and the shipped calibration file")
class TestTransformSanity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.matrix = np.load(CALIB_FILE).tolist()

    def test_shipped_matrix_passes(self):
        ok, problems, stats = match_logic.check_transform(self.matrix)
        self.assertTrue(ok, problems)
        self.assertGreater(stats["px_per_mm"], 0.5)
        self.assertLess(stats["px_per_mm"], 2.0)
        self.assertLess(stats["roundtrip_mm"], 5.0)

    def test_wrong_shape_is_rejected(self):
        for bad in ([[1, 2, 3], [4, 5, 6], [7, 8, 9]], [[1, 2], [3, 4]],
                    [[1, 2, 3, 4]] * 4):
            ok, problems, _ = match_logic.check_transform(bad)
            self.assertFalse(ok)
            self.assertTrue(problems)

    def test_non_finite_values_are_rejected(self):
        broken = [row[:] for row in self.matrix]
        broken[2][1] = float("nan")
        self.assertFalse(match_logic.check_transform(broken)[0])

    def test_all_zero_matrix_is_rejected(self):
        self.assertFalse(match_logic.check_transform([[0.0] * 3] * 4)[0])

    def test_workspace_outside_the_image_is_rejected(self):
        moved = [row[:] for row in self.matrix]
        moved[3][0] += 5.0          # workspace now metres off to the side
        ok, problems, _ = match_logic.check_transform(moved)
        self.assertFalse(ok)
        self.assertTrue(problems)

    def test_uniform_scale_does_not_change_the_projection(self):
        """The DLT is defined up to scale; a scaled copy must still pass."""
        scaled = [[5.0 * value for value in row] for row in self.matrix]
        ok, problems, _ = match_logic.check_transform(scaled)
        self.assertTrue(ok, problems)
        base = match_logic.project_arm(self.matrix, *CHECK_POSE)
        scaled_uv = match_logic.project_arm(scaled, *CHECK_POSE)
        self.assertAlmostEqual(base[0], scaled_uv[0], places=6)
        self.assertAlmostEqual(base[1], scaled_uv[1], places=6)


@unittest.skipUnless(np is not None and os.path.exists(CALIB_FILE),
                     "needs numpy and the shipped calibration file")
class TestPadCheck(unittest.TestCase):
    """The active check must flag a camera that was physically moved."""

    @classmethod
    def setUpClass(cls):
        cls.matrix = np.load(CALIB_FILE).tolist()
        cls.uv = match_logic.project_arm(cls.matrix, *CHECK_POSE)

    def test_perfect_observation_has_zero_error(self):
        self.assertIsNotNone(self.uv)
        error = match_logic.pad_projection_error(self.matrix, CHECK_POSE, self.uv)
        self.assertLess(error, 1e-6)

    def test_camera_shift_is_detected(self):
        shifted = [row[:] for row in self.matrix]
        shifted[3][1] += 0.03       # camera moved 30 mm sideways
        error = match_logic.pad_projection_error(shifted, CHECK_POSE, self.uv)
        self.assertIsNotNone(error)
        self.assertGreater(error, 6.0, "a moved camera must fail the check")

    def test_point_behind_the_camera_has_no_projection(self):
        flipped = [[-value for value in row] for row in self.matrix]
        self.assertIsNone(match_logic.project_arm(flipped, *CHECK_POSE))


@unittest.skipUnless(np is not None and os.path.exists(CALIB_FILE),
                     "needs numpy and the shipped calibration file")
class TestInverseAcceptsLists(unittest.TestCase):
    """pixel_to_xy_at_z is called with the np array and with plain lists."""

    @classmethod
    def setUpClass(cls):
        cls.matrix = np.load(CALIB_FILE).tolist()

    def test_list_and_array_agree(self):
        u, v = match_logic.project_arm(self.matrix, 270.0, 0.0, -50.0)
        as_list = match_logic.pixel_to_xy_at_z(self.matrix, u, v, -50.0)
        as_array = match_logic.pixel_to_xy_at_z(
            np.array(self.matrix), u, v, -50.0)
        self.assertAlmostEqual(as_list[0], as_array[0], places=9)
        self.assertAlmostEqual(as_list[1], as_array[1], places=9)

    def test_round_trip_returns_the_pose(self):
        for pose in ((270.0, 0.0, -50.0), (240.0, 60.0, -100.0)):
            u, v = match_logic.project_arm(self.matrix, *pose)
            x, y = match_logic.pixel_to_xy_at_z(self.matrix, u, v, pose[2])
            self.assertAlmostEqual(x, pose[0], delta=0.5)
            self.assertAlmostEqual(y, pose[1], delta=0.5)


if __name__ == "__main__":
    unittest.main()
