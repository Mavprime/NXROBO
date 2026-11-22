#!/usr/bin/env python3
"""RViz markers for the mission layer.

Subscribes /mission_state (JSON) and publishes /mission_markers:
    zones, named points, cube memory by status, current target.

Run:
    python3 mission_viz.py _world:=~/codes/config/arena_map.yaml
"""

from __future__ import annotations

import json
import os

import rospy
import yaml
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

STATUS_COLORS = {
    "pending": (1.0, 0.85, 0.2),
    "carried": (0.2, 0.9, 1.0),
    "delivered": (0.4, 0.85, 0.4),
    "delivered_unverified": (0.6, 0.85, 0.6),
    "misplaced": (0.9, 0.2, 0.2),
    "skipped": (0.5, 0.5, 0.5),
    "lost": (0.9, 0.5, 0.1),
}


def _marker(marker_id: int, ns: str, kind: int, frame: str, life: float = 0.5) -> Marker:
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = rospy.Time.now()
    m.ns = ns
    m.id = marker_id
    m.type = kind
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.lifetime = rospy.Duration(life)
    return m


def _square(x0, y0, x1, y1, z=0.01):
    return [
        Point(x0, y0, z), Point(x1, y0, z), Point(x1, y1, z),
        Point(x0, y1, z), Point(x0, y0, z),
    ]


class MissionViz:
    def __init__(self) -> None:
        world_path = rospy.get_param(
            "~world", os.path.expanduser("~/codes/config/arena_map.yaml"))
        self.frame = rospy.get_param("~frame_id", "map")
        with open(world_path) as f:
            self.world = yaml.safe_load(f)
        self.state = {}
        self.pub = rospy.Publisher("/mission_markers", MarkerArray, queue_size=1)
        rospy.Subscriber("/mission_state", String, self._on_state, queue_size=1)
        rospy.Timer(rospy.Duration(0.5), self._publish)

    def _on_state(self, msg: String) -> None:
        try:
            self.state = json.loads(msg.data)
        except Exception:
            pass

    def _publish(self, _event=None) -> None:
        markers = MarkerArray()
        next_id = 0

        # zones: transport squares and red collection squares
        for zone, info in (self.world.get("zones") or {}).items():
            tc = info["transport_center"]
            cc = info["collection_center"]
            for square, color, ns in (
                (_square(tc[0] - 0.5, tc[1] - 0.5, tc[0] + 0.5, tc[1] + 0.5),
                 (0.3, 0.3, 0.3), "zone_transport"),
                (_square(cc[0] - 0.225, cc[1] - 0.225, cc[0] + 0.225, cc[1] + 0.225),
                 (0.8, 0.1, 0.1), "zone_collection"),
            ):
                m = _marker(next_id, ns, Marker.LINE_STRIP, self.frame, life=0)
                next_id += 1
                m.scale.x = 0.02
                m.color.r, m.color.g, m.color.b, m.color.a = (*color, 0.8)
                m.points = square
                markers.markers.append(m)
            label = _marker(next_id, "zone_label", Marker.TEXT_VIEW_FACING, self.frame, life=0)
            next_id += 1
            label.pose.position = Point(tc[0], tc[1], 0.05)
            label.scale.z = 0.12
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 0.9
            label.text = "Zone " + zone
            markers.markers.append(label)

        # named points
        for name, p in (self.world.get("points") or {}).items():
            if name.endswith("_approach"):
                continue
            m = _marker(next_id, "point", Marker.SPHERE, self.frame, life=0)
            next_id += 1
            m.pose.position = Point(p[0], p[1], 0.03)
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.5, 0.9, 0.9
            markers.markers.append(m)
            label = _marker(next_id, "point_label", Marker.TEXT_VIEW_FACING, self.frame, life=0)
            next_id += 1
            label.pose.position = Point(p[0], p[1], 0.10)
            label.scale.z = 0.07
            label.color.r = label.color.g = label.color.b = 0.7
            label.color.a = 0.9
            label.text = name
            markers.markers.append(label)

        # cube memory (positions known for the initial field layout)
        memory = (self.state or {}).get("memory") or {}
        for inst in memory.get("instances", []):
            x, y = inst.get("x"), inst.get("y")
            if x is None or y is None:
                continue
            color = STATUS_COLORS.get(inst.get("status"), (0.7, 0.7, 0.7))
            m = _marker(next_id, "cube_memory", Marker.CUBE, self.frame, life=0.6)
            next_id += 1
            m.pose.position = Point(x, y, 0.05)
            m.scale.x = m.scale.y = 0.10
            m.scale.z = 0.10
            m.color.r, m.color.g, m.color.b, m.color.a = (*color, 0.85)
            markers.markers.append(m)

        self.pub.publish(markers)


def main() -> None:
    rospy.init_node("mission_viz")
    MissionViz()
    rospy.spin()


if __name__ == "__main__":
    main()
