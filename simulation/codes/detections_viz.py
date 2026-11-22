#!/usr/bin/env python3
"""RViz markers for /detections_world (real-time bridge output).

Run alongside detection_bridge.py, then in RViz:
    Fixed Frame: base_footprint (or map, matching the bridge target)
    Add -> MarkerArray -> /detection_markers

    python3 ~/codes/detections_viz.py _frame:=base_footprint
"""

from __future__ import annotations

import json

import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

COLORS = {
    "bear": (0.55, 0.35, 0.20),
    "clock": (0.85, 0.85, 0.85),
    "banana": (0.95, 0.85, 0.15),
    "cake": (0.90, 0.30, 0.40),
}


class DetectionsViz:
    def __init__(self) -> None:
        self.frame = rospy.get_param("~frame", "base_footprint")
        self.topic = rospy.get_param("~topic", "/detections_world")
        self.markers_topic = rospy.get_param("~markers_topic", "/detection_markers")
        self.lifetime = float(rospy.get_param("~lifetime", 0.7))
        self.objects = []
        self.pub = rospy.Publisher(self.markers_topic, MarkerArray, queue_size=1)
        rospy.Subscriber(self.topic, String, self._on_message, queue_size=5)
        rospy.Timer(rospy.Duration(0.1), self._publish)
        rospy.loginfo("detections viz: %s -> %s frame=%s"
                      % (self.topic, self.markers_topic, self.frame))

    def _on_message(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        objects = payload.get("objects", payload if isinstance(payload, list) else [])
        self.objects = [o for o in objects if "x" in o and "y" in o]

    def _publish(self, _event=None) -> None:
        array = MarkerArray()
        for i, obj in enumerate(self.objects):
            cls = str(obj.get("class", "?"))
            conf = float(obj.get("confidence", 0.0))
            x, y = float(obj["x"]), float(obj["y"])

            cube = Marker()
            cube.header.frame_id = self.frame
            cube.header.stamp = rospy.Time.now()
            cube.ns = "detections"
            cube.id = i
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = x
            cube.pose.position.y = y
            cube.pose.position.z = 0.05
            cube.pose.orientation.w = 1.0
            cube.scale.x = cube.scale.y = cube.scale.z = 0.10
            r, g, b = COLORS.get(cls, (0.7, 0.7, 0.7))
            cube.color.r, cube.color.g, cube.color.b = r, g, b
            cube.color.a = 0.85
            cube.lifetime = rospy.Duration(self.lifetime)
            array.markers.append(cube)

            label = Marker()
            label.header = cube.header
            label.ns = "detection_labels"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.16
            label.scale.z = 0.07
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 0.95
            label.text = "%s %.2f" % (cls, conf)
            label.lifetime = rospy.Duration(self.lifetime)
            array.markers.append(label)

        self.pub.publish(array)


def main() -> None:
    rospy.init_node("detections_viz")
    DetectionsViz()
    rospy.spin()


if __name__ == "__main__":
    main()
