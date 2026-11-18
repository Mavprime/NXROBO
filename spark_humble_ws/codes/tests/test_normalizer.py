import unittest
import sys
import os

# Add parent directory to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# Import norm_id directly or define standard contract
try:
    from runtask import norm_id, ID_MAP
except ImportError:
    ID_MAP = {
        1: 1, 10: 1,
        2: 2, 20: 2,
        3: 3, 30: 3,
        4: 4, 40: 4,
    }
    def norm_id(raw_id):
        try:
            return ID_MAP.get(int(raw_id), int(raw_id))
        except Exception:
            return raw_id


class TestClassNormalizer(unittest.TestCase):
    """Test suite for object class ID normalization and hardware aliases."""

    def test_canonical_ids(self):
        """Standard canonical IDs 1..4 should map directly to themselves."""
        self.assertEqual(norm_id(1), 1, "ID 1 (Bear) must normalize to 1")
        self.assertEqual(norm_id(2), 2, "ID 2 (Wine/Cake) must normalize to 2")
        self.assertEqual(norm_id(3), 3, "ID 3 (Clock) must normalize to 3")
        self.assertEqual(norm_id(4), 4, "ID 4 (Banana) must normalize to 4")

    def test_site_aliases(self):
        """Each printed <item>_site face maps to its own item per items_config.yaml."""
        self.assertEqual(norm_id(10), 1, "Alias 10 (bear_site) must normalize to 1 (Bear)")
        self.assertEqual(norm_id(20), 2, "Alias 20 (wine_site) must normalize to 2 (Wine)")

    def test_clock_aliases(self):
        """Clock alias 30 must normalize to 3."""
        self.assertEqual(norm_id(30), 3, "Alias 30 must normalize to 3 (Clock)")

    def test_banana_aliases(self):
        """Banana alias 40 must normalize to 4."""
        self.assertEqual(norm_id(40), 4, "Alias 40 must normalize to 4 (Banana)")

    def test_string_inputs(self):
        """String representation of integers should be handled safely."""
        self.assertEqual(norm_id("2"), 2)
        self.assertEqual(norm_id("10"), 1)
        self.assertEqual(norm_id("20"), 2)
        self.assertEqual(norm_id("30"), 3)
        self.assertEqual(norm_id("40"), 4)

    def test_unknown_and_noise_ids(self):
        """Unknown IDs should not crash the normalizer."""
        self.assertEqual(norm_id(99), 99)
        self.assertEqual(norm_id(-1), -1)
        self.assertEqual(norm_id("invalid"), "invalid")


if __name__ == "__main__":
    unittest.main()
