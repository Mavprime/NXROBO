#!/usr/bin/env python3
"""Tests for projection.py: pinhole ray -> plane, inverse projection, arena filter."""

import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
CODE_DIR = os.path.dirname(HERE)
sys.path.insert(0, CODE_DIR)

from projection import (  # noqa: E402
    CameraInfo,
    camera_ray,
    inside_arena,
    pixel_to_plane,
    rotation_matrix,
    world_to_pixel,
    yaw_from_quaternion,
)

K = CameraInfo(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
DOWN = (1.0, 0.0, 0.0, 0.0)     # 180 deg about x: optical +z -> world -z
CAM = (1.5, 1.5, 1.2)
PLANE = 0.05


class ProjectionTests(unittest.TestCase):
    def test_center_pixel_hits_under_camera(self):
        pose = pixel_to_plane(K, 320, 240, CAM, DOWN, PLANE)
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose[0], 1.5)
        self.assertAlmostEqual(pose[1], 1.5)

    def test_right_pixel_moves_east(self):
        pose = pixel_to_plane(K, 420, 240, CAM, DOWN, PLANE)
        self.assertGreater(pose[0], 1.5)
        self.assertAlmostEqual(pose[1], 1.5)

    def test_down_pixel_moves_south(self):
        pose = pixel_to_plane(K, 320, 340, CAM, DOWN, PLANE)
        self.assertLess(pose[1], 1.5)
        self.assertAlmostEqual(pose[0], 1.5)

    def test_round_trip(self):
        point = (1.6, 1.4, PLANE)
        pixel = world_to_pixel(K, point, CAM, DOWN)
        self.assertIsNotNone(pixel)
        back = pixel_to_plane(K, pixel[0], pixel[1], CAM, DOWN, PLANE)
        self.assertAlmostEqual(back[0], point[0], places=6)
        self.assertAlmostEqual(back[1], point[1], places=6)

    def test_horizontal_camera_misses_floor(self):
        # identity rotation: optical +z is world +z, plane is below -> t < 0
        self.assertIsNone(pixel_to_plane(K, 320, 240, (1.5, 1.5, 1.0),
                                         (0.0, 0.0, 0.0, 1.0), PLANE))

    def test_plane_above_camera_is_behind(self):
        self.assertIsNone(pixel_to_plane(K, 320, 240, CAM, DOWN, 1.5))

    def test_arena_filter(self):
        self.assertTrue(inside_arena(1.5, 1.5, {"origin": [0, 0], "size": [3, 3]}))
        self.assertFalse(inside_arena(-0.1, 1.5, {"origin": [0, 0], "size": [3, 3]}))
        self.assertFalse(inside_arena(1.5, 3.1, {"origin": [0, 0], "size": [3, 3]}))

    def test_camera_info_from_k(self):
        info = CameraInfo.from_k([615.0, 0, 320.0, 0, 616.0, 240.0, 0, 0, 1], 640, 480)
        self.assertEqual((info.fx, info.fy, info.cx, info.cy), (615.0, 616.0, 320.0, 240.0))

    def test_yaw_from_quaternion(self):
        # 90 deg about z
        yaw = yaw_from_quaternion((0.0, 0.0, math.sin(math.pi / 4),
                                   math.cos(math.pi / 4)))
        self.assertAlmostEqual(yaw, math.pi / 2)

    def test_rotation_identity(self):
        r = rotation_matrix((0.0, 0.0, 0.0, 1.0))
        self.assertEqual(r, ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
