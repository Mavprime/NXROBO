#!/usr/bin/env python3
"""Mock arm for ROS integration tests.

Subscribes /arm/pick and /arm/place (std_msgs/String JSON) and replies on
/arm/result after the configured durations. It does not move anything; it exists so the
mission node, RViz and the stop path can be tested before the real arm service lands.

Run:
    python3 arm_mock_node.py _pick_s:=4.0 _place_s:=3.0 _fail_prob:=0.0
"""

from __future__ import annotations

import json
import random
import threading

import rospy
from std_msgs.msg import String


class ArmMock:
    def __init__(self) -> None:
        self.pick_s = float(rospy.get_param("~pick_s", 4.0))
        self.place_s = float(rospy.get_param("~place_s", 3.0))
        self.fail_prob = float(rospy.get_param("~fail_prob", 0.0))
        self.rng = random.Random(int(rospy.get_param("~seed", 0)))

        self.pub = rospy.Publisher("/arm/result", String, queue_size=10)
        rospy.Subscriber("/arm/pick", String, self._on_pick, queue_size=10)
        rospy.Subscriber("/arm/place", String, self._on_place, queue_size=10)
        rospy.loginfo("arm mock ready: pick %.1fs place %.1fs fail %.2f"
                      % (self.pick_s, self.place_s, self.fail_prob))

    def _on_pick(self, msg: String) -> None:
        try:
            request = json.loads(msg.data)
        except Exception:
            rospy.logwarn("arm mock: bad pick request")
            return
        threading.Timer(self.pick_s, self._finish_pick, args=(request,)).start()

    def _on_place(self, msg: String) -> None:
        try:
            request = json.loads(msg.data)
        except Exception:
            rospy.logwarn("arm mock: bad place request")
            return
        threading.Timer(self.place_s, self._finish_place, args=(request,)).start()

    def _ok(self) -> bool:
        return self.rng.random() >= self.fail_prob

    def _publish(self, payload: dict) -> None:
        self.pub.publish(String(data=json.dumps(payload)))

    def _finish_pick(self, request: dict) -> None:
        ok = self._ok()
        self._publish({
            "id": request.get("id"),
            "ok": ok,
            "confirmed_class": request.get("class", "") if ok else "",
            "reason": "" if ok else "mock grasp failure",
            "duration_s": self.pick_s,
        })

    def _finish_place(self, request: dict) -> None:
        ok = self._ok()
        self._publish({
            "id": request.get("id"),
            "ok": ok,
            "confirmed_class": "",
            "reason": "" if ok else "mock place failure",
            "duration_s": self.place_s,
        })


def main() -> None:
    rospy.init_node("arm_mock")
    ArmMock()
    rospy.spin()


if __name__ == "__main__":
    main()
