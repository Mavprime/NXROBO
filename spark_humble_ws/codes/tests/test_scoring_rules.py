import unittest


def calculate_cube_score(is_red_zone: bool, tier: int) -> int:
    """
    Compute competition points for placing a cube at a given tier (1, 2, or 3).
    Formula from Chinese competition regulations:
      - Red reception square (0.45 x 0.45 m): N + 2^(N-1)
      - Standard transport zone: 2^(N-1)
    """
    if tier < 1 or tier > 3:
        raise ValueError(f"Invalid tier {tier}, must be 1, 2, or 3")

    if is_red_zone:
        return tier + (2 ** (tier - 1))
    else:
        return 2 ** (tier - 1)


def calculate_match_total(delivered_cubes: list, started_from_base: bool = True) -> int:
    """
    Calculate full match score given a sequence of deliveries.
    Each delivery: dict(cube_type="cake"|"bear"|"clock"|"banana", zone="A"|"B"|"C"|"D", is_red=bool, tier=int)
    Rule: Cake from 2nd tier must be delivered to Zone A FIRST.
    """
    if not delivered_cubes:
        return 1 if started_from_base else 0

    total = 1 if started_from_base else 0

    first_cube = delivered_cubes[0]
    if first_cube.get("cube_type") != "cake" or first_cube.get("zone") != "A":
        # Disqualification or zero score for illegal start
        return 0

    for delivery in delivered_cubes:
        points = calculate_cube_score(delivery["is_red"], delivery["tier"])
        total += points

    return total


class TestCompetitionScoring(unittest.TestCase):
    """Test suite for match point calculation and regulatory stacking rules."""

    def test_red_zone_tier_points(self):
        """Red zone scoring: N + 2^(N-1)."""
        self.assertEqual(calculate_cube_score(is_red_zone=True, tier=1), 2)  # 1 + 1 = 2
        self.assertEqual(calculate_cube_score(is_red_zone=True, tier=2), 4)  # 2 + 2 = 4
        self.assertEqual(calculate_cube_score(is_red_zone=True, tier=3), 7)  # 3 + 4 = 7

    def test_standard_zone_tier_points(self):
        """Standard zone scoring: 2^(N-1)."""
        self.assertEqual(calculate_cube_score(is_red_zone=False, tier=1), 1)  # 2^0 = 1
        self.assertEqual(calculate_cube_score(is_red_zone=False, tier=2), 2)  # 2^1 = 2
        self.assertEqual(calculate_cube_score(is_red_zone=False, tier=3), 4)  # 2^2 = 4

    def test_invalid_tier_raises(self):
        with self.assertRaises(ValueError):
            calculate_cube_score(is_red_zone=True, tier=0)
        with self.assertRaises(ValueError):
            calculate_cube_score(is_red_zone=False, tier=4)

    def test_first_cube_cake_rule_enforcement(self):
        """If first cube is Cake -> Zone A, match score accumulates properly."""
        deliveries = [
            {"cube_type": "cake", "zone": "A", "is_red": True, "tier": 1},
            {"cube_type": "bear", "zone": "B", "is_red": False, "tier": 1},
        ]
        score = calculate_match_total(deliveries, started_from_base=True)
        # 1 (base) + 2 (cake in red zone tier 1) + 1 (bear in standard zone tier 1) = 4
        self.assertEqual(score, 4)

    def test_illegal_first_cube_gives_zero(self):
        """If first cube is NOT cake to Zone A, score is invalid (0)."""
        deliveries = [
            {"cube_type": "bear", "zone": "B", "is_red": False, "tier": 1},
            {"cube_type": "cake", "zone": "A", "is_red": True, "tier": 1},
        ]
        self.assertEqual(calculate_match_total(deliveries), 0)

    def test_maximum_possible_score_theoretical(self):
        """
        Theoretical max score:
        - Start from base: +1
        - 1 Cake in Zone A (Red square, Tier 1): 2 pts
        - 9 remaining cubes stacked in red squares (3 zones x 3 tiers: 2 + 4 + 7 = 13 pts per zone -> 39 pts)
        Total max = 1 + 2 + 39 = 42 points
        """
        deliveries = [
            {"cube_type": "cake", "zone": "A", "is_red": True, "tier": 1}
        ]
        for zone in ["B", "C", "D"]:
            for tier in [1, 2, 3]:
                deliveries.append({"cube_type": "other", "zone": zone, "is_red": True, "tier": tier})

        self.assertEqual(calculate_match_total(deliveries), 42)


if __name__ == "__main__":
    unittest.main()
