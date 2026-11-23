#!/usr/bin/env python3
"""Tests for ros_contract.py: JSON parsing, goal resolution, state serialisation."""

import json
import math
import os
import sys
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CODE_DIR = os.path.dirname(HERE)
sys.path.insert(0, CODE_DIR)

import ros_contract  # noqa: E402
from mission_logic import (  # noqa: E402
    MissionMemory,
    Outcome,
    Step,
    StepRecord,
)

MAP_PATH = os.path.join(CODE_DIR, "..", "config", "arena_map.yaml")


class GoalResolutionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(MAP_PATH) as f:
            cls.world = yaml.safe_load(f)

    def test_sorting_points_face_centre(self):
        x, y, yaw = ros_contract.resolve_goal(self.world, "Sorting_N")
        self.assertEqual((x, y), (1.474, 1.972))
        self.assertAlmostEqual(yaw, -math.pi / 2)
        _, _, yaw_e = ros_contract.resolve_goal(self.world, "Sorting_E")
        self.assertAlmostEqual(abs(yaw_e), math.pi)
        _, _, yaw_s = ros_contract.resolve_goal(self.world, "Sorting_S")
        self.assertAlmostEqual(yaw_s, math.pi / 2)
        _, _, yaw_w = ros_contract.resolve_goal(self.world, "Sorting_W")
        self.assertAlmostEqual(yaw_w, 0.0)

    def test_collection_approach_faces_zone(self):
        x, y, yaw = ros_contract.resolve_goal(self.world, "Collection_A_approach")
        self.assertAlmostEqual(x, 0.4725)
        self.assertAlmostEqual(y, 2.5275)
        self.assertAlmostEqual(yaw, 3 * math.pi / 4)

    def test_classification_faces_west(self):
        _, _, yaw = ros_contract.resolve_goal(self.world, "Classification_area")
        self.assertAlmostEqual(abs(yaw), math.pi)

    def test_unknown_point_raises(self):
        with self.assertRaises(KeyError):
            ros_contract.resolve_goal(self.world, "Nowhere")


class MessageParsingTests(unittest.TestCase):
    def test_parse_detections_dict(self):
        text = json.dumps({"stamp": 1.0, "objects": [
            {"class": "doll", "confidence": 0.7, "x": 1.4, "y": 1.5},
            {"class": "cake", "confidence": 0.9, "x": 1.5, "y": 1.5},
        ]})
        dets = ros_contract.parse_detections(text)
        self.assertEqual([d.cls for d in dets], ["bear", "cake"])
        self.assertAlmostEqual(dets[0].confidence, 0.7)

    def test_parse_detections_drops_objects_without_pose(self):
        text = json.dumps([{"class": "cake", "confidence": 0.9},
                           {"class": "cake", "confidence": 0.9, "x": 1.5, "y": 1.5}])
        dets = ros_contract.parse_detections(text)
        self.assertEqual(len(dets), 1)
        self.assertEqual(dets[0].pose(), (1.5, 1.5))

    def test_arm_pick_roundtrip(self):
        req = json.loads(ros_contract.encode_arm_pick("id-1", "doll", "N"))
        self.assertEqual(req["class"], "bear")
        self.assertEqual(req["station"], "N")
        result = ros_contract.parse_arm_result(json.dumps(
            {"id": "id-1", "ok": True, "confirmed_class": "doll", "reason": ""}))
        self.assertTrue(result["ok"])
        self.assertEqual(result["confirmed_class"], "bear")
        self.assertEqual(result["id"], "id-1")

    def test_arm_place_request(self):
        req = json.loads(ros_contract.encode_arm_place("id-2", "A", 1))
        self.assertEqual(req["op"], "place")
        self.assertEqual(req["zone"], "A")
        self.assertEqual(req["level"], 1)

    def test_parse_board(self):
        self.assertEqual(ros_contract.parse_board('["doll","clock","banana"]'),
                         ["bear", "clock", "banana"])
        self.assertIsNone(ros_contract.parse_board("[]"))
        self.assertEqual(ros_contract.parse_board('{"classes":["40"]}'), ["banana"])

    def test_state_dict(self):
        memory = MissionMemory.from_inventory(
            [{"name": "c", "type": "cake", "x": 1.5, "y": 1.5}],
            {"cake": "A"},
        )
        record = StepRecord(0, "goto", {"point": "Sorting_N"}, Outcome.OK,
                            attempts=1, elapsed_s=11.0)
        runner = type("R", (), {
            "state": "DONE",
            "current": Step("pick", {"class": "cake"}),
            "records": [record],
        })()
        state = ros_contract.state_dict(runner, memory, 500.0)
        self.assertEqual(state["current"], "pick cake")
        self.assertEqual(state["steps"][0]["elapsed_s"], 11.0)
        self.assertEqual(state["memory"]["counts"]["cake"]["pending"], 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
