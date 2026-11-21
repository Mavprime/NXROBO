import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _bootstrap import match_logic  # noqa: E402


class FakeTime(object):
    """Controllable stand-in for time.monotonic."""

    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class TestMatchClock(unittest.TestCase):
    """The match clock is what keeps the run inside the 10-minute limit."""

    def setUp(self):
        self.clock_time = FakeTime()
        self.clock = match_logic.MatchClock(
            limit_s=540.0, warn_s=60.0, time_fn=self.clock_time)

    def test_elapsed_is_zero_before_start(self):
        self.assertEqual(self.clock.elapsed(), 0.0)
        self.assertEqual(self.clock.remaining(), 540.0)
        self.assertFalse(self.clock.expired())

    def test_start_returns_self_and_resets_elapsed(self):
        self.assertIs(self.clock.start(), self.clock)
        self.assertEqual(self.clock.elapsed(), 0.0)

    def test_elapsed_and_remaining_follow_time(self):
        self.clock.start()
        self.clock_time.advance(120.0)
        self.assertEqual(self.clock.elapsed(), 120.0)
        self.assertEqual(self.clock.remaining(), 420.0)

    def test_warning_window(self):
        self.clock.start()
        self.clock_time.advance(500.0)      # 40 s left
        self.assertTrue(self.clock.warn())
        self.assertFalse(self.clock.expired())

    def test_expiry_is_exact_at_the_limit(self):
        self.clock.start()
        self.clock_time.advance(539.9)
        self.assertFalse(self.clock.expired())
        self.clock_time.advance(0.2)
        self.assertTrue(self.clock.expired())
        self.assertEqual(self.clock.remaining(), 0.0)
        self.assertFalse(self.clock.warn(), "warn() is False once expired")

    def test_elapsed_never_goes_negative(self):
        self.clock.start()
        self.clock_time.advance(-50.0)
        self.assertEqual(self.clock.elapsed(), 0.0)

    def test_default_budget_leaves_room_in_a_ten_minute_match(self):
        default = match_logic.MatchClock()
        self.assertGreater(default.limit_s, 0)
        self.assertLess(default.limit_s, match_logic.MATCH_DURATION_S,
                        "the mission must stop before the 600 s match ends")


class TestOperatorCommands(unittest.TestCase):
    """parse_task_command() decides whether a message starts or stops the match."""

    def test_start_payloads(self):
        for payload in ("start", "START", "  start \n", "1", "go", "on", "true"):
            self.assertEqual(match_logic.parse_task_command(payload), "start",
                             f"{payload!r} should start the mission")

    def test_stop_payloads(self):
        for payload in ("stop", "STOP", "0", "halt", "off", "abort", "false", "kill"):
            self.assertEqual(match_logic.parse_task_command(payload), "stop",
                             f"{payload!r} should stop the mission")

    def test_empty_and_unknown_payloads_are_ignored(self):
        """The old code started the mission on ANY message, including 'stop'."""
        for payload in ("", "   ", None, "hello", "2", "restart-now"):
            self.assertIsNone(match_logic.parse_task_command(payload))

    def test_stop_is_never_treated_as_start(self):
        self.assertNotEqual(match_logic.parse_task_command("stop"), "start")


if __name__ == "__main__":
    unittest.main()
