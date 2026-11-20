import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _bootstrap import match_logic  # noqa: E402


class TestScoringMatrix(unittest.TestCase):
    """score_cube() / match_total() implement the official point rules."""

    def test_red_zone_tier_points(self):
        """Red 0.45 x 0.45 m square: N + 2^(N-1)."""
        self.assertEqual(match_logic.score_cube(True, 1), 2)
        self.assertEqual(match_logic.score_cube(True, 2), 4)
        self.assertEqual(match_logic.score_cube(True, 3), 7)

    def test_standard_zone_tier_points(self):
        """Standard transport zone: 2^(N-1)."""
        self.assertEqual(match_logic.score_cube(False, 1), 1)
        self.assertEqual(match_logic.score_cube(False, 2), 2)
        self.assertEqual(match_logic.score_cube(False, 3), 4)

    def test_invalid_tier_raises(self):
        for tier in (0, 4, -1):
            with self.assertRaises(ValueError):
                match_logic.score_cube(True, tier)

    def test_cake_first_delivery_to_zone_a(self):
        deliveries = [
            {"cube_type": "cake", "zone": "Collection_A", "is_red": True, "tier": 1},
            {"cube_type": "bear", "zone": "Collection_B", "is_red": False, "tier": 1},
        ]
        # 1 (leaving the start) + 2 (cake in the red square) + 1 (bear on the floor)
        self.assertEqual(match_logic.match_total(deliveries), 4)

    def test_illegal_first_cube_scores_zero(self):
        deliveries = [
            {"cube_type": "bear", "zone": "Collection_B", "is_red": False, "tier": 1},
            {"cube_type": "cake", "zone": "Collection_A", "is_red": True, "tier": 1},
        ]
        self.assertEqual(match_logic.match_total(deliveries), 0)

    def test_theoretical_maximum(self):
        """1 + 2 (cake in A) + 3 zones x (2 + 4 + 7) = 42."""
        deliveries = [{"cube_type": "cake", "zone": "Collection_A",
                       "is_red": True, "tier": 1}]
        for _zone in ("B", "C", "D"):
            for tier in (1, 2, 3):
                deliveries.append({"cube_type": "other", "zone": _zone,
                                   "is_red": True, "tier": tier})
        self.assertEqual(match_logic.match_total(deliveries), 42)


class TestScoreEstimate(unittest.TestCase):
    """estimate_score() is the end-of-match log line."""

    def test_empty_match_scores_only_the_start(self):
        drop_times = {"Collection_A": 0, "Collection_B": 0,
                      "Collection_C": 0, "Collection_D": 0}
        self.assertEqual(match_logic.estimate_score(drop_times), 1)

    def test_cake_in_zone_a(self):
        drop_times = {"Collection_A": 1, "Collection_B": 0,
                      "Collection_C": 0, "Collection_D": 0}
        self.assertEqual(match_logic.estimate_score(drop_times), 1 + 2)

    def test_full_towers(self):
        """Cake first in A, then 3 tiers in each of B, C, D."""
        drop_times = {"Collection_A": 1, "Collection_B": 3,
                      "Collection_C": 3, "Collection_D": 3}
        # 1 + 2 (A tier 1, red) + 3 x (1 + 2 + 4) for the standard zones
        self.assertEqual(match_logic.estimate_score(drop_times), 1 + 2 + 21)

    def test_does_not_count_more_than_three_tiers(self):
        drop_times = {"Collection_A": 0, "Collection_B": 9,
                      "Collection_C": 0, "Collection_D": 0}
        self.assertEqual(match_logic.estimate_score(drop_times), 1 + 1 + 2 + 4)


if __name__ == "__main__":
    unittest.main()
