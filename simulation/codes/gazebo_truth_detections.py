#!/usr/bin/env python3
"""Synthetic world-truth detections for the Gazebo mission test.

Publishes `/detections_world` from the cube list in `arena_map.yaml`, using TF for the
robot pose. It updates the list on arm events:
    pick ok  -> remove the nearest cube of that class from the field
    place ok -> add the carried class 0.30 m in front of the robot

This is a test double for perception while the real camera bridge is being validated.
It does not move Gazebo models; it only feeds the mission gate and verify step.

Run inside the Gazebo container:
    python3 /ws/codes/gazebo_truth_detections.py _world:=/ws/config/arena_map.yaml
"""

from __future__ import annotations

import json
import math
import os
import threading

import rospy
import tf2_ros
import yaml
from std_msgs.msg import String

from mission_logic import canonical_class
from projection import yaw_from_quaternion
from ros_contract import parse_arm_result


class TruthDetections:
    def __init__(self) -> None:
        world_path = rospy.get_param(
            "~world", os.path.expanduser("~/codes/config/arena_map.yaml"))
        self.frame = rospy.get_param("~target_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        self.confidence = float(rospy.get_param("~confidence", 0.9))
        self.front_offset = float(rospy.get_param("~front_offset", 0.30))

        with open(world_path) as f:
            world = yaml.safe_load(f)
        self.cubes = []
        for brick in world.get("bricks", []):
            self.cubes.append({
                "name": brick.get("name", ""),
                "cls": canonical_class(brick.get("type", "")),
                "x": float(brick["x"]),
                "y": float(brick["y"]),
                "layer": int(brick.get("layer", 1)),
                "field": True,
            })
        self.carried = None
        self.pending_op = None
        self.pending_class = None
        self.lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher("/detections_world", String, queue_size=5)
        rospy.Subscriber("/arm/pick", String, self._on_pick, queue_size=5)
        rospy.Subscriber("/arm/place", String, self._on_place, queue_size=5)
        rospy.Subscriber("/arm/result", String, self._on_result, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._publish)
        rospy.loginfo("truth detections: %d cubes from %s" % (len(self.cubes), world_path))

    # -- arm events ---------------------------------------------------------

    def _on_pick(self, msg: String) -> None:
        try:
            request = json.loads(msg.data)
            self.pending_class = canonical_class(request.get("class", ""))
            self.pending_op = "pick"
        except Exception:
            pass

    def _on_place(self, msg: String) -> None:
        self.pending_op = "place"

    def _on_result(self, msg: String) -> None:
        try:
            result = parse_arm_result(msg.data)
        except Exception:
            return
        if not result["ok"]:
            self.pending_op = None
            return
        with self.lock:
            if self.pending_op == "pick":
                cls = result["confirmed_class"] or self.pending_class
                if cls:
                    self._remove_nearest(cls)
                    self.carried = cls
            elif self.pending_op == "place" and self.carried:
                pose = self._robot_pose()
                if pose is not None:
                    x, y, yaw = pose
                    self.cubes.append({
                        "name": "placed_%s_%d" % (self.carried, len(self.cubes)),
                        "cls": self.carried,
                        "x": x + self.front_offset * math.cos(yaw),
                        "y": y + self.front_offset * math.sin(yaw),
                        "layer": 1,
                        "field": True,
                    })
                self.carried = None
            self.pending_op = None

    def _remove_nearest(self, cls: str) -> None:
        pose = self._robot_pose()
        candidates = [c for c in self.cubes if c["field"] and c["cls"] == cls]
        if not candidates:
            return
        if pose is None:
            candidates[0]["field"] = False
            return
        cube = min(candidates, key=lambda c: math.hypot(c["x"] - pose[0], c["y"] - pose[1]))
        cube["field"] = False

    def _robot_pose(self):
        try:
            tr = self.tf_buffer.lookup_transform(
                self.frame, self.base_frame, rospy.Time(0), rospy.Duration(0.2))
        except Exception:
            return None
        t = tr.transform.translation
        q = tr.transform.rotation
        yaw = yaw_from_quaternion((q.x, q.y, q.z, q.w))
        return (t.x, t.y, yaw)

    # -- publish ------------------------------------------------------------

    def _publish(self, _event=None) -> None:
        with self.lock:
            objects = [
                {"class": c["cls"], "confidence": self.confidence,
                 "x": round(c["x"], 3), "y": round(c["y"], 3)}
                for c in self.cubes if c["field"]
            ]
        payload = {"stamp": rospy.Time.now().to_sec(), "objects": objects}
        self.pub.publish(String(data=json.dumps(payload)))


def main() -> None:
    rospy.init_node("gazebo_truth_detections")
    TruthDetections()
    rospy.spin()


if __name__ == "__main__":
    main()
