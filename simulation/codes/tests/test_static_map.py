#!/usr/bin/env python3
"""Tests for static_map.py: geometry, occupancy, map_server files."""

import os
import sys
import tempfile
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CODE_DIR = os.path.dirname(HERE)
sys.path.insert(0, CODE_DIR)

import static_map  # noqa: E402

MAP_PATH = os.path.join(CODE_DIR, "..", "config", "arena_map.yaml")


class StaticMapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(MAP_PATH) as f:
            cls.map_data = yaml.safe_load(f)

    def cell(self, x, y, grid, width, height, res=0.05):
        return static_map.world_to_cell(self.map_data, x, y, res, width, height)

    def test_dimensions(self):
        grid, w, h = static_map.build_grid(self.map_data)
        self.assertEqual((w, h), (60, 60))

    def test_border_is_occupied(self):
        grid, w, h = static_map.build_grid(self.map_data)
        for c in (0, w - 1):
            self.assertEqual(grid[0][c], static_map.OCCUPIED)
            self.assertEqual(grid[h - 1][c], static_map.OCCUPIED)
        for r in (0, h - 1):
            self.assertEqual(grid[r][0], static_map.OCCUPIED)
            self.assertEqual(grid[r][w - 1], static_map.OCCUPIED)

    def test_table_is_occupied(self):
        grid, w, h = static_map.build_grid(self.map_data)
        r, c = self.cell(0.165, 1.50, grid, w, h)
        self.assertEqual(grid[r][c], static_map.OCCUPIED)

    def test_centre_and_approach_are_free(self):
        grid, w, h = static_map.build_grid(self.map_data)
        for x, y in ((1.5, 1.5), (1.5, 1.91), (0.4725, 2.5275), (0.225, 2.775)):
            r, c = self.cell(x, y, grid, w, h)
            self.assertEqual(grid[r][c], static_map.FREE, (x, y))

    def test_include_cubes_marks_centre(self):
        grid, w, h = static_map.build_grid(self.map_data, include_cubes=True)
        r, c = self.cell(1.5, 1.5, grid, w, h)
        self.assertEqual(grid[r][c], static_map.OCCUPIED)

    def test_writes_pgm_and_yaml(self):
        grid, w, h = static_map.build_grid(self.map_data)
        with tempfile.TemporaryDirectory() as tmp:
            stem = os.path.join(tmp, "arena_map")
            static_map.write_pgm(stem + ".pgm", grid, w, h)
            static_map.write_yaml(stem + ".yaml", "arena_map.pgm", self.map_data, 0.05)
            with open(stem + ".pgm", "rb") as f:
                header = f.read(16)
            self.assertTrue(header.startswith(b"P5\n60 60\n255\n"))
            with open(stem + ".yaml") as f:
                meta = yaml.safe_load(f)
            self.assertEqual(meta["resolution"], 0.05)
            self.assertEqual(meta["origin"], [0.0, 0.0, 0.0])
            self.assertEqual(meta["image"], "arena_map.pgm")


if __name__ == "__main__":
    unittest.main(verbosity=2)
