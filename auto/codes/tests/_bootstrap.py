"""Import bootstrap shared by the unit tests.

Every test does::

    import os, sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from _bootstrap import match_logic

so the tests always exercise the SAME modules that run on the robot.

Hard rule (enforced by ``test_wiring.py``): a test file must not re-implement
production logic.  The previous suite defined its own copies of ``norm_id``,
``decode_classification_board`` and the clustering algorithm inside the test
files, so "27/27 passed" said nothing about the code on the robot.
"""

import os
import sys

CODES_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if CODES_DIR not in sys.path:
    sys.path.insert(0, CODES_DIR)

import match_logic  # noqa: E402,F401  (re-exported for the tests)
import arm_ik       # noqa: E402,F401  (compatibility shim over match_logic)

__all__ = ["CODES_DIR", "match_logic", "arm_ik"]
