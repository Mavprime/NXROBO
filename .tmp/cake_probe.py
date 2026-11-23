#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Temporary measurement probe: collect Cake detections with scores/positions.

Detector must be started with a low __cake_conf (e.g. 0.03) so that ALL cake
hypotheses reach /objects.  Groups detections by position and prints the score
distribution per physical spot, which is what a safe threshold is chosen from.
"""
import json
import math
import sys
import time

import rospy
from vision_msgs.msg import Detection2DArray

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
OUT = sys.argv[2] if len(sys.argv) > 2 else "/home/spark/logs/cake_probe.jsonl"
GROUP_DIST = 40.0

records = []


def callback(msg):
    stamp = time.time()
    for detection in msg.detections:
        if not detection.results:
            continue
        hypothesis = detection.results[0]
        item_id = int(hypothesis.id)
        if item_id not in (2, 20):
            continue
        records.append({
            "ts": round(stamp, 2),
            "id": item_id,
            "score": round(float(hypothesis.score), 3),
            "cx": int(detection.bbox.center.x),
            "cy": int(detection.bbox.center.y),
            "w": int(detection.bbox.size_x),
            "h": int(detection.bbox.size_y),
        })


def median(values):
    ordered = sorted(values)
    n = len(ordered)
    if n == 0:
        return 0.0
    mid = n // 2
    return ordered[mid] if n % 2 else 0.5 * (ordered[mid - 1] + ordered[mid])


def main():
    rospy.init_node("cake_probe", anonymous=True)
    rospy.Subscriber("/objects", Detection2DArray, callback, queue_size=10)
    print("[PROBE] collecting cake detections for %.1f s ..." % DURATION)
    rospy.sleep(DURATION)

    with open(OUT, "w", encoding="utf8") as handle:
        for record in records:
            handle.write(json.dumps(record) + "\n")
    print("[PROBE] %d cake detections written to %s" % (len(records), OUT))

    groups = []
    for record in records:
        best = None
        for group in groups:
            dist = math.hypot(record["cx"] - group["cx"], record["cy"] - group["cy"])
            if dist < GROUP_DIST and (best is None or dist < best[0]):
                best = (dist, group)
        if best is None:
            groups.append({"cx": float(record["cx"]), "cy": float(record["cy"]),
                           "items": [record]})
        else:
            group = best[1]
            group["items"].append(record)
            group["cx"] = sum(item["cx"] for item in group["items"]) / len(group["items"])
            group["cy"] = sum(item["cy"] for item in group["items"]) / len(group["items"])

    print("\n=== CAKE DETECTION GROUPS (all scores, detector cake_conf=low) ===")
    groups.sort(key=lambda group: -len(group["items"]))
    for index, group in enumerate(groups, 1):
        scores = [item["score"] for item in group["items"]]
        widths = [item["w"] for item in group["items"]]
        heights = [item["h"] for item in group["items"]]
        print("group %d: n=%-3d center=(%d,%d)  box~%dx%d"
              % (index, len(scores), int(group["cx"]), int(group["cy"]),
                 int(median(widths)), int(median(heights))))
        print("    score min=%.3f median=%.3f max=%.3f  all=%s"
              % (min(scores), median(scores), max(scores),
                 sorted(set(scores))))
    print("\nTOTAL groups=%d detections=%d" % (len(groups), len(records)))


if __name__ == "__main__":
    main()
