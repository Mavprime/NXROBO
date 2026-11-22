#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Operator view: what the robot's detector actually sees, as one annotated JPEG.

No grasp motion is executed.  The arm is parked in its camera-clear pose (the
same pose the target scan uses) so the picture matches the scan view; pass
``--no-park`` to leave the arm untouched.

The script
  * collects a few ``/objects`` frames and the latest colour image,
  * draws every published detection with class, id and confidence,
  * applies the SAME temporal clustering + anti-glare cake gate that the grasp
    logic uses, prints the resulting target list,
  * saves ``~/logs/latest_view.jpg`` plus a timestamped copy.

Run through ``run_grasp_test.sh --view`` (or the menu in ``run_grasp_test.bat``).
"""

import argparse
import os
import sys
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

import match_logic

CLASS_NAMES = {1: "teddy", 2: "cake", 3: "clock", 4: "banana"}

#: BGR colours per canonical class (cake is drawn red: it is the priority class).
CLASS_COLORS = {
    1: (0, 200, 0),      # teddy  - green
    2: (0, 0, 255),      # cake   - red
    3: (255, 160, 0),    # clock  - blue-ish
    4: (0, 220, 255),    # banana - yellow
}
CLUSTER_COLOR = (255, 0, 255)   # magenta circles = what the grasp logic sees

LATEST_PATH = os.path.expanduser("~/logs/latest_view.jpg")


def log(message):
    print(message)
    sys.stdout.flush()


def class_label(raw_id):
    """Return (canonical_id, human name including the *site alias)."""
    cid = match_logic.norm_id(raw_id)
    name = CLASS_NAMES.get(cid, "id%s" % raw_id)
    if int(raw_id) != cid:
        name = "%s_site(%s)" % (name, raw_id)
    return cid, name


def cake_skip_reason(raw_frame, raw_id, score, cx, cy,
                     cake_conf, cake_site_conf):
    """Why a raw Cake box is NOT a grasp target; '' when it is one.

    Mirrors exactly what ``match_logic.cluster_detections`` rejects, so the
    operator view and the grasp logic always agree:
      * below the per-class anti-glare gate, or
      * the plain white front face of an ordinary cube (its own print sits
        right above it, see ``match_logic.cake_face_owner``).
    """
    if not match_logic.is_cake(raw_id):
        return ""
    is_site = int(raw_id) != match_logic.CAKE_ID
    gate = cake_site_conf if is_site else cake_conf
    if score < gate:
        return "below anti-glare gate (%.2f)" % gate
    owner = match_logic.cake_face_owner(raw_frame, cx, cy)
    if owner is not None:
        _owner_cid, owner_name = class_label(owner[0])
        return "white cube face, %s %.2f above" % (owner_name, owner[1])
    return ""


class ViewCapture(object):
    """Collects raw /objects frames and the latest camera image."""

    def __init__(self):
        self.frames = []
        self.image = None
        self._bridge = CvBridge()
        self._objects_sub = rospy.Subscriber(
            "/objects", Detection2DArray, self._objects_cb, queue_size=5)
        self._image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self._image_cb,
            queue_size=1, buff_size=2 ** 24)

    def _objects_cb(self, msg):
        frame = []
        for detection in msg.detections:
            if not detection.results:
                continue
            hypothesis = detection.results[0]
            frame.append((int(hypothesis.id), float(hypothesis.score),
                          float(detection.bbox.center.x),
                          float(detection.bbox.center.y),
                          int(detection.bbox.size_x),
                          int(detection.bbox.size_y)))
        self.frames.append(frame)

    def _image_cb(self, msg):
        try:
            self.image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # cv_bridge error must not kill the viewer
            log("[VIEW][WARN] camera frame conversion failed: %s" % exc)

    def wait(self, frames=5, timeout=12.0):
        started = time.time()
        while time.time() - started < timeout:
            if len(self.frames) >= frames and self.image is not None:
                return True
            rospy.sleep(0.05)
        return False


def park_arm():
    """Park the arm out of the camera view (same pose the scan uses)."""
    try:
        from runtask import SwiftProInterface
        interface = SwiftProInterface()
        interface.set_status(True)
        rospy.sleep(0.20)
        interface.set_pose(50.0, 180.0, 160.0, delay=2.0)
        log("[VIEW] arm parked at (50, 180, 160) for a clean camera view")
    except Exception as exc:
        log("[VIEW][WARN] could not park the arm: %s" % exc)


def annotate(image, raw_frame, clusters, conf, cake_conf, cake_site_conf):
    """Draw detections (raw) and clustered target markers on a copy of the image."""
    canvas = image.copy()
    height, width = canvas.shape[:2]

    for raw_id, score, cx, cy, size_x, size_y in raw_frame:
        cid, name = class_label(raw_id)
        color = CLASS_COLORS.get(cid, (200, 200, 200))
        x1 = int(max(0, cx - size_x / 2.0))
        y1 = int(max(0, cy - size_y / 2.0))
        x2 = int(min(width - 1, cx + size_x / 2.0))
        y2 = int(min(height - 1, cy + size_y / 2.0))
        if int(raw_id) != cid:
            label = "%s %d %.2f" % (name, raw_id, score)   # e.g. teddy_site(10)
        else:
            label = "%s %.2f" % (name, score)
        if cake_skip_reason(raw_frame, raw_id, score, cx, cy,
                            cake_conf, cake_site_conf):
            # Detector said "cake" but the grasp logic will NOT target it.
            color = (150, 150, 150)
            label += " (ignored)"
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
        cv2.putText(canvas, label, (x1, max(12, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, label, (x1, max(12, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

    # Clustered targets: what grasp_generic.scan_scene() actually receives.
    for cube_id, (cx, cy, _zero) in clusters:
        center = (int(round(cx)), int(round(cy)))
        cv2.circle(canvas, center, 8, CLUSTER_COLOR, 2, cv2.LINE_AA)

    legend = [
        "ordinary conf >= %.2f   cake(id2) >= %.2f   cake_site(id20) >= %.2f"
        % (conf, cake_conf, cake_site_conf),
        "raw detections: %d    clustered grab targets: %d"
        % (len(raw_frame), len(clusters)),
        "boxes = detector output    magenta circles = grab targets",
        "grey boxes = ignored (below gate or white cube face)",
    ]
    panel_h = 12 + 18 * len(legend)
    overlay = canvas.copy()
    cv2.rectangle(overlay, (0, 0), (width, panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.65, canvas, 0.35, 0.0, canvas)
    for index, line in enumerate(legend):
        cv2.putText(canvas, line, (8, 18 + 18 * index),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1,
                    cv2.LINE_AA)
    return canvas


def main():
    parser = argparse.ArgumentParser(
        description="Save an annotated view of what the robot's detector sees.")
    parser.add_argument("--no-park", action="store_true",
                        help="do not move the arm before the snapshot")
    parser.add_argument("--frames", type=int, default=5,
                        help="number of /objects frames to collect (default 5)")
    parser.add_argument("--conf", type=float,
                        default=match_logic.CONF_THRESHOLD,
                        help="ordinary-cube confidence floor used by the grasp "
                             "scan (default %.2f, same as grasp_generic)"
                             % match_logic.CONF_THRESHOLD)
    parser.add_argument("--cake-conf", type=float,
                        default=match_logic.CAKE_CONF_THRESHOLD,
                        help="anti-glare gate for raw cake id 2 (default %.2f)"
                             % match_logic.CAKE_CONF_THRESHOLD)
    parser.add_argument("--cake-site-conf", type=float,
                        default=match_logic.CAKE_SITE_CONF_THRESHOLD,
                        help="anti-glare gate for cake_site id 20, the real "
                             "Cake cube (default %.2f)"
                             % match_logic.CAKE_SITE_CONF_THRESHOLD)
    parser.add_argument("--out", default=LATEST_PATH,
                        help="output JPEG (default ~/logs/latest_view.jpg)")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("robot_view", anonymous=True)

    if not args.no_park:
        park_arm()

    capture = ViewCapture()
    log("[VIEW] waiting for /objects and /camera/color/image_raw ...")
    if not capture.wait(frames=max(1, args.frames)):
        log("[VIEW][ERROR] no fresh /objects or camera frames within timeout. "
            "Check the detector and the camera driver (run mode 1 of "
            "run_grasp_test.bat, option 1 starts them).")
        return 5

    frames = [frame for frame in capture.frames[-max(1, args.frames):]]
    raw_frame = frames[-1]
    clusters = match_logic.cluster_detections(
        frames, conf_threshold=args.conf,
        cake_conf_threshold=args.cake_conf,
        cake_site_conf_threshold=args.cake_site_conf)
    image = capture.image

    log("")
    log("=== WHAT THE ROBOT SEES ===")
    log("raw detections published by the detector on /objects (last frame): %d"
        % len(raw_frame))
    for raw_id, score, cx, cy, size_x, size_y in sorted(
            raw_frame, key=lambda item: item[0]):
        _cid, name = class_label(raw_id)
        reason = cake_skip_reason(raw_frame, raw_id, score, cx, cy,
                                  args.cake_conf, args.cake_site_conf)
        tag = "  <-- %s, ignored" % reason if reason else ""
        log("  %-14s id=%-3d score=%.2f center=(%d, %d) box=%dx%d%s"
            % (name, raw_id, score, cx, cy, size_x, size_y, tag))
    if not raw_frame:
        log("  (nothing detected in this frame)")

    order = sorted(clusters, key=lambda cube: cube[0])
    log("")
    log("=== AFTER TEMPORAL CLUSTERING + CAKE GATES (what the grasp logic uses) ===")
    log("ordinary conf floor: %.2f | cake(id2) gate: %.2f | cake_site(id20) gate: %.2f"
        % (args.conf, args.cake_conf, args.cake_site_conf))
    if order:
        for cube_id, (cx, cy, _zero) in order:
            _cid, name = class_label(cube_id)
            log("  TARGET CANDIDATE: %s id=%d at px=(%d, %d)"
                % (name, cube_id, cx, cy))
    else:
        log("  (no graspable targets)")

    canvas = annotate(image, raw_frame, clusters, args.conf, args.cake_conf,
                      args.cake_site_conf)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    timestamped = os.path.join(os.path.dirname(expand(args.out)),
                               "view_%s.jpg" % stamp)
    ok_latest = cv2.imwrite(expand(args.out), canvas,
                            [int(cv2.IMWRITE_JPEG_QUALITY), 88])
    ok_stamp = cv2.imwrite(timestamped, canvas,
                           [int(cv2.IMWRITE_JPEG_QUALITY), 88])
    log("")
    if ok_latest:
        log("[VIEW] saved: %s" % expand(args.out))
    else:
        log("[VIEW][ERROR] could not write %s" % expand(args.out))
    if ok_stamp:
        log("[VIEW] saved: %s" % timestamped)
    return 0 if ok_latest else 6


def expand(path):
    return os.path.expanduser(path)


if __name__ == "__main__":
    sys.exit(main())
