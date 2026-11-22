#!/usr/bin/env python3
"""Tests for the inflated-obstacle A* planner."""

import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
CODE_DIR = os.path.dirname(HERE)
sys.path.insert(0, CODE_DIR)

from planner import plan_path  # noqa: E402

BOUNDS = (0.0, 0.0, 3.0, 3.0)


class PlannerTests(unittest.TestCase):
    def test_routes_around_centre_block(self):
        obstacle = (1.0, 1.0, 2.0, 2.0)
        radius = 0.15
        path = plan_path((0.5, 1.5), (2.5, 1.5), [obstacle], BOUNDS, 0.05, radius)
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 2)
        self.assertEqual(path[0], (0.5, 1.5))
        self.assertEqual(path[-1], (2.5, 1.5))
        # at least one waypoint must leave the inflated corridor
        inflated_y0 = obstacle[1] - radius
        inflated_y1 = obstacle[3] + radius
        self.assertTrue(any(p[1] < inflated_y0 or p[1] > inflated_y1 for p in path))
        inflated_x0 = obstacle[0] - radius
        inflated_x1 = obstacle[2] + radius
        for p in path:
            inside = (inflated_x0 <= p[0] <= inflated_x1
                      and inflated_y0 <= p[1] <= inflated_y1)
            self.assertFalse(inside, "waypoint inside the inflated obstacle: %s" % (p,))

    def test_direct_path_when_clear(self):
        path = plan_path((0.5, 0.5), (2.5, 0.5), [], BOUNDS, 0.05, 0.15)
        self.assertEqual(path, [(0.5, 0.5), (2.5, 0.5)])

    def test_no_path_through_full_wall(self):
        wall = (-0.1, 1.4, 3.1, 1.6)
        path = plan_path((0.5, 0.5), (2.5, 2.5), [wall], BOUNDS, 0.05, 0.15)
        self.assertIsNone(path)

    def test_path_is_finite_and_inside_bounds(self):
        path = plan_path((0.3, 0.3), (2.7, 2.7), [(1.0, 1.0, 2.0, 2.0)],
                         BOUNDS, 0.05, 0.15)
        self.assertIsNotNone(path)
        for x, y in path:
            self.assertTrue(math.isfinite(x) and math.isfinite(y))
            self.assertGreaterEqual(x, 0.15)
            self.assertLessEqual(x, 2.85)
            self.assertGreaterEqual(y, 0.15)
            self.assertLessEqual(y, 2.85)


if __name__ == "__main__":
    unittest.main(verbosity=2)
