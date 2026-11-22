import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _bootstrap import match_logic  # noqa: E402

ZONES = ("Collection_B", "Collection_C", "Collection_D")


class TestClassificationBoard(unittest.TestCase):
    """decode_classification_board() is what run_task() uses for the mapping."""

    def assertFullMapping(self, mapping):
        """Every canonical item has a zone and the cake is pinned to A."""
        for item in (match_logic.TEDDY_ID, match_logic.CLOCK_ID, match_logic.BANANA_ID):
            self.assertIn(item, mapping, f"item {item} has no zone")
            self.assertIn(mapping[item], ZONES)
        self.assertEqual(mapping[match_logic.CAKE_ID], "Collection_A")

    def test_standard_board(self):
        mapping = match_logic.decode_classification_board([1, 3, 4])
        self.assertFullMapping(mapping)
        self.assertEqual(mapping[1], "Collection_B")
        self.assertEqual(mapping[3], "Collection_C")
        self.assertEqual(mapping[4], "Collection_D")

    def test_permutation(self):
        mapping = match_logic.decode_classification_board([4, 1, 3])
        self.assertEqual(mapping[4], "Collection_B")
        self.assertEqual(mapping[1], "Collection_C")
        self.assertEqual(mapping[3], "Collection_D")
        self.assertEqual(mapping[match_logic.CAKE_ID], "Collection_A")

    def test_yolo_aliases(self):
        """Board read as [30, 40, 1] must normalise before it is applied."""
        mapping = match_logic.decode_classification_board([30, 40, 1])
        self.assertEqual(mapping[3], "Collection_B")
        self.assertEqual(mapping[4], "Collection_C")
        self.assertEqual(mapping[1], "Collection_D")

    def test_duplicate_or_corrupt_board_falls_back(self):
        """SI-14: a broken board must never leave the mission without a mapping."""
        for broken in ([1, 1, 99], [None, None, None], [], [1, 3]):
            mapping = match_logic.decode_classification_board(broken)
            self.assertFullMapping(mapping)
            self.assertEqual(mapping, match_logic.normalize_mapping(None))

    def test_alias_keys_are_present(self):
        mapping = match_logic.decode_classification_board([1, 3, 4])
        self.assertEqual(mapping[10], mapping[1])
        self.assertEqual(mapping[20], "Collection_A")
        self.assertEqual(mapping[30], mapping[3])
        self.assertEqual(mapping[40], mapping[4])

    def test_board_is_confident(self):
        self.assertTrue(match_logic.board_is_confident([1, 3, 4]))
        self.assertTrue(match_logic.board_is_confident([30, 40, 1]))
        self.assertFalse(match_logic.board_is_confident([1, 1, 99]))
        self.assertFalse(match_logic.board_is_confident([]))

    def test_cake_is_never_taken_from_the_board(self):
        """The board has three cards only; a cake reading must not steal a zone."""
        mapping = match_logic.decode_classification_board([2, 1, 3])
        self.assertEqual(mapping[match_logic.CAKE_ID], "Collection_A")
        self.assertFullMapping(mapping)


class TestDropPlanning(unittest.TestCase):
    def test_tier_dispatch(self):
        self.assertEqual(match_logic.plan_drop(0), "drop")
        self.assertEqual(match_logic.plan_drop(1), "drop1")
        self.assertEqual(match_logic.plan_drop(2), "drop2")

    def test_full_zone_is_refused(self):
        """A 4th cube must never be dropped at the 3rd tier height into thin air."""
        self.assertIsNone(match_logic.plan_drop(3))
        self.assertIsNone(match_logic.plan_drop(10))

    def test_zone_comes_from_the_board_mapping(self):
        mapping = match_logic.decode_classification_board([1, 3, 4])
        drop_times = {zone: 0 for zone in ("Collection_A",) + ZONES}
        self.assertEqual(
            match_logic.choose_drop_zone(match_logic.CLOCK_ID, mapping, drop_times),
            "Collection_C")

    def test_full_zone_spills_to_a_free_one(self):
        mapping = match_logic.decode_classification_board([1, 3, 4])
        drop_times = {"Collection_A": 0, "Collection_B": 3,
                      "Collection_C": 0, "Collection_D": 0}
        self.assertEqual(
            match_logic.choose_drop_zone(match_logic.TEDDY_ID, mapping, drop_times),
            "Collection_C")

    def test_every_zone_full_returns_none(self):
        mapping = match_logic.decode_classification_board([1, 3, 4])
        drop_times = {"Collection_A": 3, "Collection_B": 3,
                      "Collection_C": 3, "Collection_D": 3}
        self.assertIsNone(
            match_logic.choose_drop_zone(match_logic.TEDDY_ID, mapping, drop_times))

    def test_cake_goes_to_zone_a(self):
        mapping = match_logic.decode_classification_board([1, 3, 4])
        drop_times = {"Collection_A": 0, "Collection_B": 0,
                      "Collection_C": 0, "Collection_D": 0}
        self.assertEqual(
            match_logic.choose_drop_zone(match_logic.CAKE_ID, mapping, drop_times),
            "Collection_A")


class TestStationRouting(unittest.TestCase):
    def test_preferred_station_for_each_zone(self):
        stations = {name: 2 for name in match_logic.STATION_ORDER}
        self.assertEqual(match_logic.choose_station("Collection_B", stations), "Sorting_E")
        self.assertEqual(match_logic.choose_station("Collection_C", stations), "Sorting_S")
        self.assertEqual(match_logic.choose_station("Collection_D", stations), "Sorting_W")

    def test_preference_skips_a_retired_station(self):
        stations = {name: 2 for name in match_logic.STATION_ORDER}
        stations["Sorting_E"] = 0
        self.assertEqual(match_logic.choose_station("Collection_B", stations), "Sorting_N")

    def test_no_preference_falls_back_to_the_first_station(self):
        stations = {name: 2 for name in match_logic.STATION_ORDER}
        # Collection_A has no preference table, so the fixed order decides.
        self.assertEqual(match_logic.choose_station("Collection_A", stations),
                         "Sorting_W")

    def test_all_retired_returns_none(self):
        self.assertIsNone(match_logic.choose_station("Collection_B", {}))

    def test_retire_station_removes_it_once(self):
        stations = {"Sorting_W": 1}
        self.assertTrue(match_logic.retire_station(stations, "Sorting_W"))
        self.assertFalse(match_logic.retire_station(stations, "Sorting_W"))
        self.assertEqual(stations, {})

    def test_approach_waypoints(self):
        self.assertEqual(match_logic.approach_waypoint("Sorting_W", "Collection_B"),
                         "Step_NW")
        self.assertEqual(match_logic.approach_waypoint("Sorting_S", "Collection_B"),
                         "Step_SE")
        self.assertEqual(match_logic.approach_waypoint("Sorting_M", "Collection_C"),
                         "Step_NE")
        self.assertIsNone(match_logic.approach_waypoint("Sorting_W", "Collection_A"))

    def test_exit_manoeuvres(self):
        self.assertEqual(match_logic.exit_maneuver("Sorting_S", "Collection_B"),
                         ("back", match_logic.EXIT_BACK_M))
        self.assertEqual(match_logic.exit_maneuver("Sorting_W", "Collection_B"),
                         ("arrive", "Step_NW"))
        self.assertIsNone(match_logic.exit_maneuver("Sorting_E", "Collection_B"))


class TestLaunchSequence(unittest.TestCase):
    """The mission plan is a plain dict of counters: it must always terminate."""

    def test_plan_is_bounded_by_the_cube_count(self):
        stations = {name: 2 for name in match_logic.STATION_ORDER}
        self.assertEqual(sum(stations.values()), match_logic.TOTAL_CUBES)

    def test_empty_station_is_retired_instead_of_looping(self):
        stations = {name: 2 for name in match_logic.STATION_ORDER}
        order = []
        while stations and len(order) < 20:
            station = match_logic.choose_station("Collection_A", stations)
            if station is None:
                break
            order.append(station)
            # simulate "nothing found twice": the real loop retires the station
            match_logic.retire_station(stations, station)
        self.assertEqual(len(order), len(match_logic.STATION_ORDER))
        self.assertEqual(stations, {})


if __name__ == "__main__":
    unittest.main()
