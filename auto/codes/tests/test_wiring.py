"""Wiring tests: the production files must actually use the tested logic.

These tests are deliberately about *text/AST*, because the ROS modules cannot be
imported on a machine without ROS.  They catch the failure mode this project
already suffered once: beautiful passing tests against a re-implementation,
while the robot ran something else.
"""

import ast
import glob
import io
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _bootstrap import CODES_DIR  # noqa: E402

RUNTASK = os.path.join(CODES_DIR, "runtask.py")
DETECT_ROS = os.path.join(CODES_DIR, "detect_ros.py")
START_SYSTEM = os.path.join(CODES_DIR, "start_system.sh")
STOP_SYSTEM = os.path.join(CODES_DIR, "stop_system.sh")

#: Logic that must live in match_logic.py and must NOT be re-implemented in a
#: test file (nor privately inside runtask.py).
PRODUCTION_FUNCTIONS = (
    "norm_id",
    "ik_reachable",
    "pixel_to_arm",
    "cluster_detections",
    "decode_classification_board",
    "choose_drop_zone",
    "choose_station",
    "plan_drop",
    "score_cube",
    "match_total",
    "parse_task_command",
)


def read(path):
    with io.open(path, encoding="utf-8") as handle:
        return handle.read()


def module_tree(path):
    return ast.parse(read(path), filename=path)


def defined_functions(path):
    tree = module_tree(path)
    return {node.name for node in ast.walk(tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))}


def imported_modules(path):
    names = set()
    for node in ast.walk(module_tree(path)):
        if isinstance(node, ast.Import):
            names.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module.split(".")[0])
            names.update(alias.name for alias in node.names)
    return names


def describes_only_logic_of(production_function, test_paths):
    """Names a test file defines that shadow production logic."""
    findings = []
    for path in test_paths:
        if production_function in defined_functions(path):
            findings.append(os.path.basename(path))
    return findings


class TestRuntaskUsesMatchLogic(unittest.TestCase):
    def setUp(self):
        self.source = read(RUNTASK)

    def test_imports_match_logic(self):
        self.assertIn("match_logic", imported_modules(RUNTASK))

    def test_uses_the_voting_and_clustering_helper(self):
        self.assertIn("match_logic.cluster_detections(", self.source,
                      "detector() must run the tested clustering")

    def test_uses_the_board_decoder(self):
        self.assertIn("decode_classification_board(", self.source)

    def test_uses_the_match_clock(self):
        self.assertIn("MatchClock(", self.source,
                      "the mission needs a hard time budget")

    def test_uses_the_payload_parser(self):
        self.assertIn("parse_task_command(", self.source,
                      "'stop' must never start a run")

    def test_uses_the_zone_and_tier_planners(self):
        self.assertIn("choose_drop_zone(", self.source)
        self.assertIn("plan_drop(", self.source)

    def test_no_private_copy_of_ik_or_normalizer(self):
        defined = defined_functions(RUNTASK)
        for name in ("ik_reachable", "pixel_to_arm", "norm_id"):
            self.assertNotIn(name, defined,
                             f"runtask.py must not re-implement {name}()")

    def test_no_infinite_empty_station_loop(self):
        """The old body retired a station with `continue` and spun forever."""
        self.assertIn("MAX_EMPTY_ATTEMPTS", self.source)
        self.assertIn("retire_station(", self.source)

    def test_run_task_is_defined(self):
        self.assertIn("run_task", defined_functions(RUNTASK))


class TestDetectorWiring(unittest.TestCase):
    def setUp(self):
        self.source = read(DETECT_ROS)

    def test_deduplicates_the_two_models(self):
        self.assertIn("match_logic.dedupe_boxes(", self.source)

    def test_augmentation_is_not_forced_on(self):
        self.assertNotIn("augment=True", self.source,
                         "test-time augmentation triples CPU inference time")

    def test_publishes_objects_topic(self):
        self.assertIn('"/objects"', self.source)


class TestStartupScripts(unittest.TestCase):
    def test_start_system_records_a_bag(self):
        source = read(START_SYSTEM)
        self.assertIn("rosbag record", source)
        self.assertIn("ROSBAG", source)

    def test_start_system_checks_navigation(self):
        source = read(START_SYSTEM)
        self.assertIn("nav_ready", source)
        self.assertIn("move_base", source)

    def test_stop_system_closes_the_bag(self):
        self.assertIn("rosbag record", read(STOP_SYSTEM))


class TestTestsDoNotReimplementProductionLogic(unittest.TestCase):
    """Regression guard for the defect this suite was rewritten to fix."""

    def test_no_test_file_shadows_production_logic(self):
        test_paths = sorted(glob.glob(os.path.join(os.path.dirname(__file__), "test_*.py")))
        self.assertGreaterEqual(len(test_paths), 5)
        offenders = {}
        for function_name in PRODUCTION_FUNCTIONS:
            shadowed_in = describes_only_logic_of(function_name, test_paths)
            if shadowed_in:
                offenders[function_name] = shadowed_in
        self.assertEqual(
            offenders, {},
            f"test files must import production logic, not copy it: {offenders}")


if __name__ == "__main__":
    unittest.main()
