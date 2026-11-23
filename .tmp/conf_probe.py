#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""No-motion probe: raw /objects confidence distribution per class.

Used to pick the cake confidence floor from the real field lighting.
"""
import collections

import rospy
from vision_msgs.msg import Detection2DArray

STATS = collections.defaultdict(list)


def callback(msg):
    for detection in msg.detections:
        if not detection.results:
            continue
        hypothesis = detection.results[0]
        STATS[int(hypothesis.id)].append(
            (round(float(hypothesis.score), 3),
             int(detection.bbox.center.x), int(detection.bbox.center.y)))


def main():
    rospy.init_node("conf_probe", anonymous=True)
    rospy.Subscriber("/objects", Detection2DArray, callback, queue_size=10)
    rospy.sleep(6.0)
    print("frames collected: classes=%s" % sorted(STATS.keys()))
    for cls in sorted(STATS):
        values = [item[0] for item in STATS[cls]]
        print("class=%d n=%d min=%.3f max=%.3f"
              % (cls, len(values), min(values), max(values)))
        if cls == match_cake:
            for item in sorted(STATS[cls], reverse=True)[:25]:
                print("  cake_candidate score=%.3f px=(%d,%d)" % item)
    # Also print the top-N of every class to see the confidence gap.
    for cls in sorted(STATS):
        for item in sorted(STATS[cls], reverse=True)[:5]:
            print("class=%d top score=%.3f px=(%d,%d)" % ((cls,) + item))


match_cake = 2

if __name__ == "__main__":
    main()
