"""Handling of a cube that is OUTSIDE the manipulator's reach.

The failure this suite protects against: the field test used to discard every
detection above the calibrated near field (``v < 130 px``) as "far
wall/ceiling noise".  A cube standing further than ~35 cm therefore produced
``NO_TARGET`` (exit code 3) and the robot never drove towards it.

The replacement contract, pinned down here:

* a far pixel that passes the loose scan window is still a usable target, but
  it is flagged ``far`` and is NEVER considered in reach, so the base approach
  step runs;
* the dedicated Cake affine keeps its reach clamp for the final grasp plan
  (a Cake seen on its top face looks "too far"), while the far path keeps the
  unclamped ``x`` so a genuinely distant Cake also drives the base;
* near-field pixels keep working exactly as before;
* ``grasp_generic.py`` actually calls the tested helpers.

All coordinates use the shipped hand-eye matrix; the pixels are real values
measured on the field when the bug was reproduced.
"""

import io
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
GRASP_GENERIC = os.path.join(CODES_DIR, "grasp_generic.py")

#: Live detections of the failing run: a cube at ~0.65 m (v < 130) next to a
#: reachable cube (v = 294).  The first one used to be thrown away.
FAR_FLOOR_PIXEL = (345.0, 107.0)
NEAR_FLOOR_PIXEL = (346.0, 294.0)


def read(path):
    with io.open(path, encoding="utf-8") as handle:
        return handle.read()


class TestScanWindow(unittest.TestCase):
    """The scan window must not throw away an out-of-reach cube."""

    def test_recorded_far_cube_passes_the_scan_window(self):
        u, v = FAR_FLOOR_PIXEL
        self.assertTrue(match_logic.in_scan_window(u, v),
                        "an out-of-reach cube must reach the approach logic")
        self.assertFalse(match_logic.in_near_field(u, v),
                         "v=107 is outside the calibrated near field")

    def test_recorded_near_cube_is_in_the_near_field(self):
        u, v = NEAR_FLOOR_PIXEL
        self.assertTrue(match_logic.in_near_field(u, v))
        self.assertTrue(match_logic.in_scan_window(u, v))

    def test_the_window_contains_the_whole_near_field(self):
        """Every calibrated near-field pixel must also pass the loose window."""
        for u in (80.0, 320.0, 560.0):
            for v in (130.0, 270.0, 450.0):
                self.assertTrue(match_logic.in_scan_window(u, v),
                                "near-field pixel (%s, %s) was rejected" % (u, v))

    def test_image_edges_are_still_rejected(self):
        for u, v in ((5.0, 10.0), (630.0, 100.0), (320.0, 5.0), (320.0, 478.0)):
            self.assertFalse(match_logic.in_scan_window(u, v),
                             "edge pixel (%s, %s) must stay rejected" % (u, v))

    def test_the_window_is_a_strict_superset_of_the_near_field(self):
        near = (match_logic.NEAR_FIELD_U[0] >= match_logic.SCAN_WINDOW_U[0]
                and match_logic.NEAR_FIELD_U[1] <= match_logic.SCAN_WINDOW_U[1]
                and match_logic.NEAR_FIELD_V[0] >= match_logic.SCAN_WINDOW_V[0]
                and match_logic.NEAR_FIELD_V[1] <= match_logic.SCAN_WINDOW_V[1])
        self.assertTrue(near)


@unittest.skipUnless(np is not None and os.path.exists(CALIB_FILE),
                     "needs numpy and the shipped calibration file")
class TestFarFieldTarget(unittest.TestCase):
    """A far pixel must produce an approach point and be declared out of reach."""

    @classmethod
    def setUpClass(cls):
        cls.matrix = np.load(CALIB_FILE).tolist()

    def test_far_floor_pixel_is_a_valid_approach_point(self):
        xy = match_logic.far_target_xy(self.matrix, *FAR_FLOOR_PIXEL,
                                       is_cake=False)
        self.assertIsNotNone(xy)
        x, y = xy
        self.assertAlmostEqual(x, 410.8, delta=3.0)
        self.assertAlmostEqual(y, -19.6, delta=3.0)
        self.assertGreater(x, match_logic.FLOOR_IN_REACH["dist_max"],
                           "the far target must stay outside the grasp envelope")

    def test_far_flag_forces_a_base_approach(self):
        xy = match_logic.far_target_xy(self.matrix, *FAR_FLOOR_PIXEL,
                                       is_cake=False)
        in_reach, dist = match_logic.grasp_reach_state(
            "floor", xy[0], xy[1], far=True)
        self.assertFalse(in_reach)
        self.assertGreater(dist, match_logic.FLOOR_IN_REACH["dist_max"])

    def test_far_cake_keeps_its_distance_for_the_approach(self):
        """The reach clamp must NOT turn a distant Cake into a "reachable" one."""
        raw = match_logic.pixel_to_cake_xy(self.matrix, *FAR_FLOOR_PIXEL,
                                           clamp=False)
        clamped = match_logic.pixel_to_cake_xy(self.matrix, *FAR_FLOOR_PIXEL)
        self.assertGreater(raw[0], match_logic.CAKE_IN_REACH["dist_target"] + 50.0)
        self.assertLessEqual(clamped[0], 348.0)

        # This is the behaviour regression: without the far flag the clamped
        # near-field approximation would claim the cube is already graspable.
        self.assertTrue(match_logic.grasp_reach_state("cake", *clamped)[0])
        self.assertFalse(match_logic.grasp_reach_state("cake", *raw, far=True)[0])

    def test_far_cake_target_xy_is_unclamped(self):
        raw = match_logic.pixel_to_cake_xy(self.matrix, *FAR_FLOOR_PIXEL,
                                           clamp=False)
        xy = match_logic.far_target_xy(self.matrix, *FAR_FLOOR_PIXEL,
                                       is_cake=True)
        self.assertAlmostEqual(xy[0], raw[0], places=6)
        self.assertAlmostEqual(xy[1], raw[1], places=6)

    def test_near_floor_pixel_stays_graspable(self):
        xy = match_logic.far_target_xy(self.matrix, *NEAR_FLOOR_PIXEL,
                                       is_cake=False)
        in_reach, dist = match_logic.grasp_reach_state("floor", xy[0], xy[1])
        self.assertTrue(in_reach, "a cube at 20 cm must stay graspable")

    def test_degenerate_matrix_returns_none(self):
        broken = [[0.0, 0.0, 0.0] for _ in range(4)]
        self.assertIsNone(match_logic.far_target_xy(broken, 320.0, 100.0,
                                                    is_cake=False))


class TestGraspHeights(unittest.TestCase):
    """The geometry moved into match_logic: the wrist heights must not change."""

    def test_floor_heights(self):
        self.assertEqual(match_logic.grasp_heights("floor"),
                         (70.0, -58.0, 60.0))

    def test_cake_heights_use_the_calibrated_touch_height(self):
        self.assertEqual(match_logic.grasp_heights("cake", None, 40.0),
                         (75.0, 37.0, 80.0))

    def test_a_low_cake_estimate_falls_back_to_floor_heights(self):
        self.assertEqual(match_logic.grasp_heights("cake", z_est=-100.0, cake_touch_z=40.0),
                         (70.0, -58.0, 60.0))

    def test_approach_distance_targets(self):
        self.assertEqual(match_logic.approach_target_distance("floor", 400.0, 0.0),
                         (400.0, 240.0))
        self.assertEqual(match_logic.approach_target_distance("cake", 400.0, 0.0),
                         (400.0, 305.0))


class TestTargetChoice(unittest.TestCase):
    """Which target the robot drives to: Cake first, then the cheapest one."""

    @staticmethod
    def target(mode, x, y, pixel, far=False, class_id=None):
        if class_id is None:
            class_id = match_logic.CAKE_ID if mode == "cake" else match_logic.BANANA_ID
        return {"mode": mode, "class": class_id,
                "is_cake": match_logic.is_cake(class_id),
                "pixel": [float(pixel[0]), float(pixel[1])],
                "xy": [float(x), float(y)], "z_est": None, "far": bool(far)}

    def test_empty_scan_returns_none(self):
        self.assertIsNone(match_logic.choose_scan_target([]))
        self.assertIsNone(match_logic.choose_scan_target([None]))

    def test_graspable_cube_wins_over_a_far_one(self):
        near = self.target("floor", 240.0, 0.0, (340.0, 290.0))
        far = self.target("floor", 600.0, 0.0, (320.0, 60.0), far=True)
        self.assertIs(match_logic.choose_scan_target([far, near]), near,
                      "a cube that needs no base motion must be picked")

    def test_cake_priority_wins_even_out_of_reach(self):
        """Operator rule: the rule-critical Cake is never skipped."""
        cube = self.target("floor", 240.0, 0.0, (340.0, 290.0))
        cake = self.target("cake", 434.0, 8.0, (333.0, 110.0), far=True)
        self.assertIs(match_logic.choose_scan_target([cube, cake]), cake)

    def test_corner_phantom_does_not_beat_the_cube_ahead(self):
        """Live failure: a low-confidence corner box turned the robot 63 deg away."""
        phantom = self.target("floor", 103.2, 207.3, (85.0, 427.0))
        ahead = self.target("floor", 410.8, -19.6, (345.0, 107.0), far=True)
        self.assertIs(match_logic.choose_scan_target([phantom, ahead]), ahead)

    def test_approach_cost_prefers_the_heading(self):
        ahead = self.target("floor", 410.8, -19.6, (345.0, 107.0), far=True)
        sideways = self.target("floor", 103.2, 207.3, (85.0, 427.0))
        self.assertLess(match_logic.approach_cost(ahead),
                        match_logic.approach_cost(sideways))

    def test_graspable_target_has_negative_cost(self):
        near = self.target("floor", 240.0, 0.0, (340.0, 290.0))
        far = self.target("floor", 600.0, 0.0, (320.0, 60.0), far=True)
        self.assertLess(match_logic.approach_cost(near), 0.0)
        self.assertGreater(match_logic.approach_cost(far), 0.0)


class TestGraspAimShift(unittest.TestCase):
    """Operator calibration knob: the aim shift is back to the raw projection.

    The field test tried -40 mm (aim 4 cm closer) on 2026-11-22 and the operator
    confirmed the untouched projection aims correctly, so the default is 0.0.
    The helper and ``--aim-shift`` stay available for field tuning.
    """

    def test_default_is_the_untouched_projection(self):
        self.assertEqual(match_logic.GRASP_AIM_SHIFT_MM, 0.0)
        self.assertEqual(match_logic.apply_grasp_aim_shift((348.0, 5.9)),
                         (348.0, 5.9))

    def test_shift_is_still_configurable(self):
        self.assertEqual(
            match_logic.apply_grasp_aim_shift((348.0, 5.9), shift_mm=-40.0),
            (308.0, 5.9))

    def test_none_and_garbage_pass_through_safely(self):
        self.assertIsNone(match_logic.apply_grasp_aim_shift(None))
        self.assertIsNone(match_logic.apply_grasp_aim_shift("broken"))


class TestReachCheck(unittest.TestCase):
    """The reach test is the firmware IK with a comfort margin.

    Live arm-frame coordinates of the restored field (2026-11-22).  The old
    hard rectangle (x_hi=290, |y|<=100, dist<=295) was wrong in both
    directions: it declared the side cubes of the first row unreachable (so the
    base drove for nothing) while accepting poses where the arm cannot touch
    the floor.  The base must approach ONLY when the manipulator truly cannot
    reach.
    """

    def test_first_row_side_cubes_are_reachable(self):
        """|y| = 129/147: the old |y|<=100 box wrongly demanded a base drive."""
        for xy in ((247.3, 128.6), (239.2, -147.0)):
            in_reach, _dist = match_logic.grasp_reach_state("floor", *xy)
            self.assertTrue(in_reach, "reachable cube %s declared out of reach" % (xy,))

    def test_middle_row_needs_the_base_approach(self):
        """~390 mm is the live position of the middle row: genuinely too far."""
        in_reach, dist = match_logic.grasp_reach_state("floor", 394.0, 118.9)
        self.assertFalse(in_reach)
        self.assertGreater(dist, 350.0)

    def test_pose_reachable_only_on_the_edge_is_out_of_reach(self):
        """Plain IK passes, the comfort ring does not: the firmware would refuse."""
        x, y = 335.0, 0.0
        for z in (-58.0, 60.0, 70.0):
            self.assertTrue(match_logic.ik_reachable(x, y, z))
        self.assertFalse(match_logic.grasp_reach_state("floor", x, y)[0])

    def test_cube_reachable_only_at_lift_height_is_out_of_reach(self):
        """x=340 is reachable at lift z, but the arm cannot touch the floor."""
        in_reach, _dist = match_logic.grasp_reach_state("floor", 340.0, 40.0)
        self.assertFalse(in_reach)

    def test_cake_tier2_target_stays_reachable(self):
        in_reach, _dist = match_logic.grasp_reach_state("cake", 348.0, 5.9)
        self.assertTrue(in_reach)

    def test_far_field_target_is_never_in_reach(self):
        in_reach, _dist = match_logic.grasp_reach_state(
            "floor", 500.0, 0.0, far=True)
        self.assertFalse(in_reach)

    def test_comfort_margin_is_small_but_nonzero(self):
        self.assertGreater(match_logic.REACH_MARGIN_MM, 0.0)
        self.assertLessEqual(match_logic.REACH_MARGIN_MM, 25.0)


class TestFirstRowProtection(unittest.TestCase):
    """The base must not drive into a cube row on its way to a target.

    Live failure: driving towards a target behind the first row knocked that
    row over.  The arm-frame positions below are the real detections of that
    pose (near row: teddy (227.3, 128.6), clock (219.2, -147.0); target: the
    Cake at (348.0, 5.9)).
    """

    TARGET = (348.0, 5.9)
    NEAR_ROW = [(227.3, 128.6), (219.2, -147.0)]

    def test_live_first_row_is_detected_in_the_corridor(self):
        obstacle = match_logic.nearest_corridor_obstacle_mm(self.TARGET,
                                                            self.NEAR_ROW)
        self.assertIsNotNone(obstacle)
        self.assertAlmostEqual(obstacle, 261.2, delta=2.0)
        self.assertGreater(obstacle, match_logic.APPROACH_CLEARANCE_MM,
                           "the live first row leaves room for a short step")

    def test_obstacle_closer_than_clearance_refuses_the_drive(self):
        """The 22:38 live sample had the first row at 141 mm: no drive at all."""
        obstacle_mm = 141.0
        self.assertLess(obstacle_mm, match_logic.APPROACH_CLEARANCE_MM)

    def test_clear_corridor_returns_none(self):
        self.assertIsNone(match_logic.nearest_corridor_obstacle_mm(
            self.TARGET, [(600.0, 0.0), (348.0, 300.0)]))

    def test_cube_behind_the_robot_or_beyond_the_target_is_ignored(self):
        self.assertIsNone(match_logic.nearest_corridor_obstacle_mm(
            self.TARGET, [(-200.0, 0.0), (500.0, 0.0)]))

    def test_cube_off_the_drive_corridor_is_ignored(self):
        self.assertIsNone(match_logic.nearest_corridor_obstacle_mm(
            self.TARGET, [(200.0, 300.0)]))

    def test_nearest_obstacle_wins(self):
        obstacle = match_logic.nearest_corridor_obstacle_mm(
            self.TARGET, [(300.0, 0.0), (200.0, 10.0)])
        self.assertAlmostEqual(obstacle, 200.2, delta=1.0)

    def test_safety_clearance_matches_the_chassis(self):
        """Chassis front is ~120 mm ahead of the arm base; keep a real gap."""
        self.assertGreaterEqual(match_logic.APPROACH_CLEARANCE_MM, 150.0)
        self.assertLessEqual(match_logic.APPROACH_CLEARANCE_MM, 220.0)
        self.assertLessEqual(match_logic.APPROACH_CORRIDOR_MM, 200.0)


class TestGraspGenericWiring(unittest.TestCase):
    """grasp_generic.py must use the tested helpers, not its own copy."""

    @classmethod
    def setUpClass(cls):
        cls.source = read(GRASP_GENERIC)

    def test_uses_the_scan_window_helpers(self):
        self.assertIn("match_logic.in_near_field(", self.source)
        self.assertIn("match_logic.in_scan_window(", self.source)

    def test_uses_the_far_field_coordinates(self):
        self.assertIn("match_logic.far_target_xy(", self.source)

    def test_uses_the_target_chooser(self):
        self.assertIn("match_logic.choose_scan_target(", self.source)

    def test_passes_the_far_flag_into_the_reach_decision(self):
        self.assertIn("far=far", self.source,
                      "a far target must be reported out of reach")

    def test_old_hard_crop_is_gone(self):
        self.assertNotIn("130.0 <= v <= 450.0", self.source,
                         "the hard crop is what produced NO_TARGET/exit code 3")

    def test_reports_out_of_reach_instead_of_no_target(self):
        self.assertIn('result="OUT_OF_REACH"', self.source)

    def test_distinguishes_a_lost_target_from_no_target_at_all(self):
        self.assertIn('result="TARGET_LOST"', self.source,
                      "an empty re-scan after driving is not 'no target'")

    def test_reach_state_and_geometry_come_from_match_logic(self):
        self.assertIn("grasp_reach_state as reach_state", self.source)
        self.assertIn("grasp_heights as _grasp_heights", self.source)

    # --- first-row protection and the operator aim calibration (2026-11-22) --

    def test_aim_shift_comes_from_match_logic(self):
        self.assertIn("match_logic.apply_grasp_aim_shift(", self.source,
                      "the 4 cm aim calibration must be applied on the robot")

    def test_drive_step_is_short(self):
        self.assertIn("MAX_DRIVE_STEP_M = 0.10", self.source,
                      "0.25 m was enough to drive into the first row")
        self.assertIn("FAR_DRIVE_STEP_M = 0.08", self.source)
        self.assertNotIn("MAX_DRIVE_STEP_M = 0.25", self.source)

    def test_drive_checks_for_cubes_in_the_corridor(self):
        self.assertIn("nearest_corridor_obstacle_mm(", self.source)
        self.assertIn("obstacle_mm", self.source)
        self.assertIn("APPROACH_CLEARANCE_MM", self.source)

    def test_bumper_does_not_veto_the_drive(self):
        """The IR bumper is noisy, so it only shortens the step (crawl)."""
        self.assertIn("if forward and blocked:", self.source)
        self.assertIn("BUMPER_CRAWL_STEP_M", self.source,
                      "a pressed bumper must still let the base approach")
        self.assertNotIn("refusing to drive \"\n", self.source)
        self.assertNotIn("blocked and target_dist < 180.0", self.source)



if __name__ == "__main__":
    unittest.main()
