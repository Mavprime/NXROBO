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
CALIB3D = os.path.join(CODES_DIR, "calib3d.py")
START_SYSTEM = os.path.join(CODES_DIR, "start_system.sh")
STOP_SYSTEM = os.path.join(CODES_DIR, "stop_system.sh")
DASHBOARD = os.path.join(CODES_DIR, "dashboard.sh")
RUN_GRASP_SH = os.path.join(CODES_DIR, "run_grasp_test.sh")
UPLOAD_PS1 = os.path.abspath(os.path.join(CODES_DIR, "..", "launchers", "upload.ps1"))

#: Logic that must live in match_logic.py and must NOT be re-implemented in a
#: test file (nor privately inside runtask.py).
PRODUCTION_FUNCTIONS = (
    "norm_id",
    "ik_reachable",
    "pixel_to_arm",
    "pixel_to_xy_at_z",
    "project_arm",
    "pad_projection_error",
    "check_transform",
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

    def test_uses_the_calibration_guard(self):
        """A stale hand-eye matrix must be caught at startup, not in the log."""
        self.assertIn("check_transform(", self.source)
        self.assertIn("verify_calibration(", self.source)

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

    def test_grasp1_uses_verified_cake_offset_constant(self):
        """SI-37: grasp1 must use match_logic.CAKE_CUP_OFFSET_X, not stale 30.0."""
        self.assertIn("CAKE_CUP_OFFSET_X", self.source)
        self.assertNotIn("cup_offset_x=30.0", self.source)

    def test_frames_buffer_supports_multi_sample_scans(self):
        """SI-27: deque maxlen must be >= 16 so 5/6-sample scans do not stall."""
        self.assertIn("deque(maxlen=max(16,", self.source)

    def test_cancel_publisher_is_precreated(self):
        self.assertIn("self.cancel_pub", self.source)
        self.assertNotIn('rospy.Publisher("move_base/cancel", GoalID, queue_size=1).publish', self.source)

    def test_calib3d_imports_pad_detect(self):
        self.assertIn("from pad_detect import", read(CALIB3D))
        self.assertNotIn("from calib_pad import", read(CALIB3D))


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
        self.assertIn("/pump_topic", read(STOP_SYSTEM))

    def test_startup_scripts_do_not_force_tta_augment(self):
        self.assertNotIn("_augment:=true", read(START_SYSTEM))
        self.assertNotIn("_augment:=true", read(RUN_GRASP_SH))

    def test_dashboard_checks_move_base_node_not_topic(self):
        dash = read(DASHBOARD)
        self.assertIn("rosnode list", dash)
        self.assertNotIn("rostopic list 2>/dev/null | grep -qx '/move_base/status'", dash)

    def test_upload_ps1_does_not_unconditionally_overwrite_calibration(self):
        ps1 = read(UPLOAD_PS1)
        self.assertIn("PushCalib", ps1)
        self.assertIn("pad_detect.py", ps1)


class TestLauncherLayout(unittest.TestCase):
    """The Windows launchers must work from either the repo root or auto\\.

    A full copy of run_grasp_test.bat was once extracted into auto\\ where
    ``%~dp0auto\\codes`` resolves to ``auto\\auto\\codes``: every scp failed and
    the operator saw a wall of SCP ERRORs.  The canonical launcher now resolves
    the repository root itself, and any copy in auto\\ must be a forwarder.
    """

    def setUp(self):
        self.repo_root = os.path.abspath(os.path.join(CODES_DIR, "..", ".."))
        self.root_bat = os.path.join(self.repo_root, "run_grasp_test.bat")
        self.scan_bat = os.path.join(self.repo_root, "run_scan_only.bat")

    def test_root_launcher_exists(self):
        self.assertTrue(os.path.isfile(self.root_bat),
                        "run_grasp_test.bat must live in the repository root")

    def test_root_launcher_resolves_the_repository_root(self):
        source = read(self.root_bat)
        self.assertIn("PROJECT_ROOT", source)
        self.assertIn("%~dp0..", source,
                      "the launcher must fall back to the parent directory "
                      "when it is launched from auto\\")
        self.assertNotIn("set SRC_DIR=%~dp0auto\\codes", source,
                         "%~dp0auto\\codes only works from the repo root")

    def test_root_launcher_does_not_hardcode_the_absolute_path(self):
        self.assertNotIn("d:\\nxrobo", read(self.root_bat).lower(),
                         "the repo must keep working from any checkout path")

    def test_scan_only_launcher_forwards_without_duplicating_logic(self):
        source = read(self.scan_bat)
        self.assertIn("--scan-only", source)
        self.assertNotIn("scp ", source)

    def test_auto_copies_are_forwarders_not_full_copies(self):
        for name in ("run_grasp_test.bat", "run_scan_only.bat"):
            path = os.path.join(self.repo_root, "auto", name)
            if not os.path.isfile(path):
                continue
            source = read(path)
            self.assertIn("..\\run_grasp_test.bat", source,
                          f"auto\\{name} must forward to the root launcher")
            self.assertNotIn("scp ", source,
                             f"auto\\{name} must not duplicate the upload logic")


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
