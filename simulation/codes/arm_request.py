#!/usr/bin/env python3
"""Send one JSON request to the arm contract and print the result.

Usage:
    python3 arm_request.py pick cake
    python3 arm_request.py place 1
"""

from __future__ import annotations

import json
import sys
import uuid

import rospy
from std_msgs.msg import String


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    op = sys.argv[1]
    rid = uuid.uuid4().hex[:10]
    if op == "pick":
        if len(sys.argv) < 3:
            print(__doc__)
            return 2
        payload = {"id": rid, "op": "pick", "class": sys.argv[2], "station": ""}
        topic = "/arm/pick"
    elif op == "place":
        level = int(sys.argv[2]) if len(sys.argv) > 2 else 1
        payload = {"id": rid, "op": "place", "zone": "", "level": level}
        topic = "/arm/place"
    else:
        print(__doc__)
        return 2

    rospy.init_node("arm_request", anonymous=True)
    result = {}

    def on_result(msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if data.get("id") == rid:
            result.update(data)

    rospy.Subscriber("/arm/result", String, on_result, queue_size=10)
    rospy.sleep(0.3)
    pub = rospy.Publisher(topic, String, queue_size=1)
    deadline = rospy.get_time() + 5.0
    while (pub.get_num_connections() == 0 and rospy.get_time() < deadline
           and not rospy.is_shutdown()):
        rospy.sleep(0.1)
    if pub.get_num_connections() == 0:
        print("NO SUBSCRIBER on %s (is arm_real_node running?)" % topic)
        return 1
    rospy.sleep(0.2)
    pub.publish(String(data=json.dumps(payload)))
    print("sent %s %s" % (topic, payload))
    deadline = rospy.get_time() + 60.0
    while not rospy.is_shutdown() and rospy.get_time() < deadline:
        if result:
            print("RESULT", json.dumps(result))
            return 0 if result.get("ok") else 1
        rospy.sleep(0.1)
    print("TIMEOUT waiting for /arm/result")
    return 1


if __name__ == "__main__":
    sys.exit(main())
