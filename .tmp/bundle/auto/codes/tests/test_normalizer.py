import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _bootstrap import match_logic  # noqa: E402


class TestClassNormalizer(unittest.TestCase):
    """norm_id() maps every raw YOLO id (including *_site faces) to 1..4."""

    def test_canonical_ids(self):
        self.assertEqual(match_logic.norm_id(1), match_logic.TEDDY_ID)
        self.assertEqual(match_logic.norm_id(2), match_logic.CAKE_ID)
        self.assertEqual(match_logic.norm_id(3), match_logic.CLOCK_ID)
        self.assertEqual(match_logic.norm_id(4), match_logic.BANANA_ID)

    def test_site_aliases(self):
        """Each printed <item>_site face maps to its own item."""
        self.assertEqual(match_logic.norm_id(10), match_logic.TEDDY_ID)
        self.assertEqual(match_logic.norm_id(20), match_logic.CAKE_ID)

    def test_clock_aliases(self):
        self.assertEqual(match_logic.norm_id(30), match_logic.CLOCK_ID)

    def test_banana_aliases(self):
        self.assertEqual(match_logic.norm_id(40), match_logic.BANANA_ID)

    def test_string_inputs(self):
        self.assertEqual(match_logic.norm_id("2"), match_logic.CAKE_ID)
        self.assertEqual(match_logic.norm_id("10"), match_logic.TEDDY_ID)
        self.assertEqual(match_logic.norm_id("40"), match_logic.BANANA_ID)

    def test_unknown_and_noise_ids(self):
        self.assertEqual(match_logic.norm_id(99), 99)
        self.assertEqual(match_logic.norm_id(-1), -1)
        self.assertEqual(match_logic.norm_id("invalid"), "invalid")

    def test_is_cake(self):
        self.assertTrue(match_logic.is_cake(2))
        self.assertTrue(match_logic.is_cake(20))
        self.assertFalse(match_logic.is_cake(1))
        self.assertFalse(match_logic.is_cake(None))

    def test_production_map_is_the_one_under_test(self):
        """Guard: this test must read the deployed mapping, not a local copy."""
        self.assertIs(match_logic.ID_MAP[10], 1)
        self.assertEqual(len(match_logic.ID_MAP), 8)


if __name__ == "__main__":
    unittest.main()
