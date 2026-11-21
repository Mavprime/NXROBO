#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLO object detector for the Spark competition (publishes /objects).

Two YOLOv5 models are run on every frame (the generic model plus a banana
specialist).  They can both report the same physical cube, so their detections
are merged with ``match_logic.dedupe_boxes`` before publishing: one physical
cube = one entry in /objects.  Without this, the FSM could pick a duplicated
box as its target.

Test-time augmentation is OFF by default: it triples inference time on the
on-board NUC and the FSM already votes over several frames
(``runtask.py: GRASP_FRAMES``).  Enable it with ``~augment:=true`` only if the
CPU has headroom.
"""

import os
import pathlib
import platform

import cv2
import yolov5
import rospy
import rospkg
import yaml
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

import match_logic

# Torch checkpoints saved on Windows carry WindowsPath objects; they cannot be
# unpickled on Linux without this alias.
if platform.system() != 'Windows':
    pathlib.WindowsPath = pathlib.PosixPath

# Everything below this confidence is published; the FSM applies its own,
# stricter grasp threshold (match_logic.CONF_THRESHOLD).
PUBLISH_CONF = 0.25

# Only competition classes are meaningful downstream. items_config.yaml also
# declares ``base: 99`` and YOLO happily detects the robot's own base at 0.95,
# which used to fill /objects with an id the FSM ignores anyway.
REPORTED_IDS = frozenset(match_logic.ID_MAP)


class SparkDetect:
    """Dual-model YOLOv5 detector with cross-model de-duplication."""

    def __init__(self, model_path_1, model_path_2, augment=False,
                 dedupe_px=match_logic.DEDUPE_DIST_PX):
        self.augment = bool(augment)
        self.dedupe_px = float(dedupe_px)
        rospy.loginfo(f">> [SPARK-DETECT] Loading model 1: {model_path_1}")
        self.model_1 = yolov5.load(model_path_1)
        self.model_1.conf = 0.03
        rospy.loginfo(f">> [SPARK-DETECT] Loading model 2: {model_path_2}")
        self.model_2 = yolov5.load(model_path_2)
        self.model_2.conf = 0.03
        rospy.loginfo(f">> [SPARK-DETECT] Both models loaded "
                      f"(augment={self.augment}, dedupe={self.dedupe_px:.0f}px).")

    def _collect(self, model, label, image, boxes):
        """Append one model's detections to ``boxes`` (never raises)."""
        try:
            results = model(image, augment=self.augment)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f">> [SPARK-DETECT] {label} inference failed: {e}")
            return
        try:
            predictions = results.xyxy[0]
        except Exception as e:
            rospy.logwarn_throttle(5.0, f">> [SPARK-DETECT] {label} returned no "
                                        f"prediction tensor: {e}")
            return
        for prediction in predictions:
            try:
                x1, y1, x2, y2, conf, cls = [float(v) for v in prediction[:6]]
                boxes.append({
                    "name": model.model.names[int(cls)],
                    "score": conf,
                    "cx": int((x1 + x2) / 2),
                    "cy": int((y1 + y2) / 2),
                    "size_x": int(x2 - x1),
                    "size_y": int(y2 - y1),
                    "x1": int(x1), "y1": int(y1),
                    "x2": int(x2), "y2": int(y2),
                    "model": label,
                })
            except Exception as e:
                rospy.logwarn_throttle(5.0, f">> [SPARK-DETECT] bad box from "
                                            f"{label}: {e}")

    def detect(self, image):
        """Return de-duplicated boxes (dicts) for ``image``; highlights in place."""
        boxes = []
        self._collect(self.model_1, "model_1", image, boxes)
        self._collect(self.model_2, "model_2", image, boxes)
        kept = match_logic.dedupe_boxes(boxes, dist_px=self.dedupe_px)

        for box in kept:
            cv2.rectangle(image, (box["x1"], box["y1"]), (box["x2"], box["y2"]),
                          (0, 255, 0), 3)
            label = f'{box["name"]} {box["score"]:.2f}'
            cv2.putText(image, label, (box["x1"], box["y1"] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.circle(image, (box["cx"], box["cy"]), 5, (255, 0, 0), -1)
        return kept


class Detector:
    def __init__(self):
        self.image_pub = rospy.Publisher("/debug_image", Image, queue_size=1)
        self.object_pub = rospy.Publisher("/objects", Detection2DArray, queue_size=1)
        self.bridge = CvBridge()

        self.publish_conf = float(rospy.get_param("~conf", PUBLISH_CONF))
        augment = bool(rospy.get_param("~augment", False))
        dedupe_px = float(rospy.get_param("~dedupe_px", match_logic.DEDUPE_DIST_PX))

        # Load the competition item configuration.
        try:
            item_file = os.path.join(
                rospkg.RosPack().get_path('auto_match'), 'config', 'items_config.yaml')
        except Exception:
            item_file = os.path.expanduser(
                '~/spark_noetic/src/3rd_app/auto_match_25/auto_match/config/items_config.yaml')
        if not os.path.exists(item_file):
            alt_path = ('/home/spark/spark_noetic/src/3rd_app/auto_match_25/'
                        'auto_match/config/items_config.yaml')
            if os.path.exists(alt_path):
                item_file = alt_path
        with open(item_file, 'r') as handle:
            config_contents = yaml.safe_load(handle)
        self.obj_id = config_contents['items']
        self.items = list(self.obj_id.keys())
        rospy.loginfo(f">> [DETECTOR] item classes from {item_file}: {self.items}")

        # Load the models BEFORE subscribing so the first frame is not delayed
        # and a broken checkpoint fails loudly at startup.
        home_dir = os.environ.get('HOME', '/home/spark')
        model1 = os.path.join(home_dir, "auto.pt")
        model2 = os.path.join(home_dir, "banana.pt")
        rospy.loginfo(f">> [DETECTOR] Initializing models from {home_dir} ...")
        self.detector = SparkDetect(model1, model2, augment=augment,
                                   dedupe_px=dedupe_px)
        rospy.loginfo(">> [DETECTOR] Neural network models loaded and ready.")

        self._frame_count = 0
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.image_cb,
            queue_size=1, buff_size=2 ** 24,
        )
        rospy.loginfo(">> [DETECTOR] Subscribed to /camera/color/image_raw")

    def image_cb(self, data):
        obj_array = Detection2DArray()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f">> [DETECTOR] Bridge error: {e}")
            return
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        obj_array.header = data.header

        try:
            boxes = self.detector.detect(image)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f">> [DETECTOR] inference exception: {e}")
            boxes = []

        published = 0
        for box in boxes:
            if box["name"] not in self.items:
                continue
            item_id = int(self.obj_id[box["name"]])
            if item_id not in REPORTED_IDS:
                continue
            min_conf = 0.03 if match_logic.is_cake(item_id) else self.publish_conf
            if box["score"] < min_conf:
                continue
            obj = Detection2D()
            obj.header = data.header
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = item_id
            hypothesis.score = float(box["score"])
            obj.results.append(hypothesis)
            obj.bbox.size_x = int(box["size_x"])
            obj.bbox.size_y = int(box["size_y"])
            obj.bbox.center.x = int(box["cx"])
            obj.bbox.center.y = int(box["cy"])
            obj_array.detections.append(obj)
            published += 1

        self.object_pub.publish(obj_array)

        self._frame_count += 1
        if self._frame_count % 50 == 1:
            rospy.loginfo(f">> [DETECTOR] frame {self._frame_count}: "
                          f"{published} object(s) published")

        img_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        try:
            image_out = self.bridge.cv2_to_imgmsg(img_bgr, "bgr8")
            image_out.header = data.header
            self.image_pub.publish(image_out)
        except CvBridgeError as e:
            rospy.logwarn_throttle(5.0, f">> [DETECTOR] debug image bridge error: {e}")


if __name__ == '__main__':
    rospy.init_node('detector_node')
    rospy.loginfo(">> [DETECTOR] Node 'detector_node' starting ...")
    obj = Detector()
    rospy.loginfo(">> [DETECTOR] Node running, publishing /objects and /debug_image.")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()
