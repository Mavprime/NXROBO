import unittest
from unittest.mock import MagicMock


def decode_classification_board(detected_slots, default_mapping=None):
    """
    Decodes the 3 items detected on the classification board (slots for B, C, D)
    and produces the navigation mapping dict.
    Cake (2) is strictly pinned to Collection_A.
    """
    if default_mapping is None:
        default_mapping = {
            2: "Collection_A",
            1: "Collection_B",
            3: "Collection_C",
            4: "Collection_D",
        }

    # Normalize aliases
    alias_map = {1: 1, 10: 1, 2: 2, 20: 2, 3: 3, 30: 3, 4: 4, 40: 4}
    canonical = [alias_map.get(s, s) for s in detected_slots if s in alias_map]

    # Validate that we have 3 unique items for B, C, D (non-cake: 1, 3, 4)
    valid_items = [x for x in canonical if x in (1, 3, 4)]
    if len(set(valid_items)) < 3:
        # Fallback if board was partially occluded or misread (SI-14)
        return dict(default_mapping)

    zones = ["Collection_B", "Collection_C", "Collection_D"]
    mapping = {2: "Collection_A"}
    for item_id, zone_name in zip(valid_items[:3], zones):
        mapping[item_id] = zone_name

    return mapping


def get_drop_action(zone_drop_count: int, mock_arm: MagicMock):
    """
    Routes cube drop action depending on current tier count for the zone.
    Tier 1 (count 0) -> drop()
    Tier 2 (count 1) -> drop1()
    Tier 3 (count 2) -> drop2()
    """
    if zone_drop_count == 0:
        return mock_arm.drop()
    elif zone_drop_count == 1:
        return mock_arm.drop1()
    else:
        return mock_arm.drop2()


def safe_extract_drop_coords(cube_list, fallback_x=235, fallback_y=0):
    """
    Defensive extraction of drop coordinates to prevent IndexError (SI-12).
    """
    if not cube_list or len(cube_list) == 0:
        return fallback_x, fallback_y
    try:
        coords = cube_list[0][1]
        return coords[0], coords[1]
    except (IndexError, TypeError):
        return fallback_x, fallback_y


class TestFSMTransitions(unittest.TestCase):
    """Test suite for FSM logic, classification board routing, and defensive drop handling."""

    def test_board_decoding_standard(self):
        """Slots [1, 3, 4] -> Bear to B, Clock to C, Banana to D; Cake always to A."""
        mapping = decode_classification_board([1, 3, 4])
        self.assertEqual(mapping[2], "Collection_A")
        self.assertEqual(mapping[1], "Collection_B")
        self.assertEqual(mapping[3], "Collection_C")
        self.assertEqual(mapping[4], "Collection_D")

    def test_board_decoding_permutation(self):
        """Slots [4, 1, 3] -> Banana to B, Bear to C, Clock to D."""
        mapping = decode_classification_board([4, 1, 3])
        self.assertEqual(mapping[4], "Collection_B")
        self.assertEqual(mapping[1], "Collection_C")
        self.assertEqual(mapping[3], "Collection_D")
        self.assertEqual(mapping[2], "Collection_A")

    def test_board_decoding_with_yolo_aliases(self):
        """Board detected with aliases [30, 40, 1] should normalize cleanly."""
        mapping = decode_classification_board([30, 40, 1])
        self.assertEqual(mapping[3], "Collection_B")
        self.assertEqual(mapping[4], "Collection_C")
        self.assertEqual(mapping[1], "Collection_D")

    def test_board_decoding_fallback_on_corrupt_data(self):
        """If board is corrupted or has duplicates, fall back to safe default mapping."""
        corrupted_slots = [1, 1, 99]
        mapping = decode_classification_board(corrupted_slots)
        self.assertEqual(mapping[2], "Collection_A")
        self.assertEqual(mapping[1], "Collection_B")
        self.assertEqual(mapping[3], "Collection_C")
        self.assertEqual(mapping[4], "Collection_D")

    def test_tier_stacking_dispatch(self):
        """Verify proper sequential dispatch across tiers 1, 2, 3."""
        mock_arm = MagicMock()

        # 1st drop -> drop()
        get_drop_action(0, mock_arm)
        mock_arm.drop.assert_called_once()
        mock_arm.drop1.assert_not_called()
        mock_arm.drop2.assert_not_called()

        mock_arm.reset_mock()

        # 2nd drop -> drop1()
        get_drop_action(1, mock_arm)
        mock_arm.drop1.assert_called_once()
        mock_arm.drop.assert_not_called()
        mock_arm.drop2.assert_not_called()

        mock_arm.reset_mock()

        # 3rd drop -> drop2()
        get_drop_action(2, mock_arm)
        mock_arm.drop2.assert_called_once()
        mock_arm.drop.assert_not_called()
        mock_arm.drop1.assert_not_called()

    def test_safe_drop_coordinate_extraction(self):
        """Verify SI-12 protection: empty vision detection list must fall back to (235, 0)."""
        # Valid list
        valid_cubes = [[2, [210, 15, 0]]]
        x, y = safe_extract_drop_coords(valid_cubes)
        self.assertEqual((x, y), (210, 15))

        # Empty list -> fallback to default
        empty_cubes = []
        x_def, y_def = safe_extract_drop_coords(empty_cubes)
        self.assertEqual((x_def, y_def), (235, 0))

        # Malformed list -> fallback to default
        x_mal, y_mal = safe_extract_drop_coords([[]])
        self.assertEqual((x_mal, y_mal), (235, 0))


if __name__ == "__main__":
    unittest.main()
