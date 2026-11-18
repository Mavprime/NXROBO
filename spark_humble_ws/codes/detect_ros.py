#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import yolov5
import rospy
import rospkg
import yaml
from std_msgs.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import platform
import pathlib
plt = platform.system()
if plt != 'Windows':
    pathlib.WindowsPath = pathlib.PosixPath

class SparkDetect:
    class __results__:
        def __init__(self):
            self.name = []
            self.x = []
            self.y = []
            self.size_x = []
            self.size_y = []
            self.confidence = []
            self.image = None

    def __init__(self, model_path_1, model_path_2):
        '''Initialize dual YOLOv5 detector models.'''
        rospy.loginfo(f">> [SPARK-DETECT] Loading Model 1: {model_path_1}")
        self.model_1 = yolov5.load(model_path_1)
        rospy.loginfo(f">> [SPARK-DETECT] Loading Model 2: {model_path_2}")
        self.model_2 = yolov5.load(model_path_2)
        rospy.loginfo(">> [SPARK-DETECT] Both YOLOv5 models loaded successfully.")

    def detect(self, image):
        '''Detect objects in image frame.'''
        results_1 = self.model_1(image, augment=True)
        results_2 = self.model_2(image, augment=True)
        result = self.__results__()

        try:
            for *xyxy, conf, cls in results_1.xyxy[0]:
                center_x = int((xyxy[0] + xyxy[2]) / 2)
                center_y = int((xyxy[1] + xyxy[3]) / 2)
                size_x = int(xyxy[2] - xyxy[0])
                size_y = int(xyxy[3] - xyxy[1])
                label = f'{self.model_1.model.names[int(cls)]} {conf:.2f}'
                cv2.rectangle(image, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 3)
                cv2.putText(image, label, (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.circle(image, (center_x, center_y), 5, (255, 0, 0), -1)

                result.size_x.append(size_x)
                result.size_y.append(size_y)
                result.name.append(self.model_1.model.names[int(cls)])
                result.x.append(center_x)
                result.y.append(center_y)
                result.confidence.append(float(conf))
        except Exception:
            pass

        try:
            for *xyxy, conf, cls in results_2.xyxy[0]:
                center_x = int((xyxy[0] + xyxy[2]) / 2)
                center_y = int((xyxy[1] + xyxy[3]) / 2)
                size_x = int(xyxy[2] - xyxy[0])
                size_y = int(xyxy[3] - xyxy[1])
                label = f'{self.model_2.model.names[int(cls)]} {conf:.2f}'
                cv2.rectangle(image, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 3)
                cv2.putText(image, label, (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.circle(image, (center_x, center_y), 5, (255, 0, 0), -1)

                result.size_x.append(size_x)
                result.size_y.append(size_y)
                result.name.append(self.model_2.model.names[int(cls)])
                result.x.append(center_x)
                result.y.append(center_y)
                result.confidence.append(float(conf))
        except Exception:
            pass

        result.image = image
        return result


class Detector:
    def __init__(self):
        self.image_pub = rospy.Publisher("/debug_image", Image, queue_size=1)
        self.object_pub = rospy.Publisher("/objects", Detection2DArray, queue_size=1)
        self.bridge = CvBridge()

        # Load competition items configuration
        try:
            item_file = os.path.join(rospkg.RosPack().get_path('auto_match'), 'config', 'items_config.yaml')
        except Exception:
            item_file = os.path.expanduser('~/spark_noetic/src/3rd_app/auto_match_25/auto_match/config/items_config.yaml')
        if not os.path.exists(item_file):
            alt_path = '/home/spark/spark_noetic/src/3rd_app/auto_match_25/auto_match/config/items_config.yaml'
            if os.path.exists(alt_path):
                item_file = alt_path
        with open(item_file, 'r') as file:
            config_contents = yaml.safe_load(file)
            self.obj_id = config_contents['items']
        self.items = list(self.obj_id.keys())

        # Load models FIRST before subscribing to camera frames
        home_dir = os.environ.get('HOME', '/home/spark')
        model1 = os.path.join(home_dir, "auto.pt")
        model2 = os.path.join(home_dir, "banana.pt")
        rospy.loginfo(f">> [DETECTOR] Initializing models from {home_dir}...")
        self.detector = SparkDetect(model1, model2)
        rospy.loginfo(">> [DETECTOR] Neural network models loaded and ready.")

        # Subscribe to camera color stream
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.image_cb, queue_size=1, buff_size=2**24
        )
        rospy.loginfo(">> [DETECTOR] Subscribed to /camera/color/image_raw")

    def image_cb(self, data):
        objArray = Detection2DArray()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f">> [DETECTOR] Bridge error: {e}")
            return
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        objArray.header = data.header
        try:
            results = self.detector.detect(image)
            img_bgr = results.image
            for i in range(len(results.name)):
                # Confidence threshold matched to 0.35
                if (results.name[i] not in self.items) or results.confidence[i] < 0.35:
                    continue
                obj = Detection2D()
                obj.header = data.header
                obj_hypothesis = ObjectHypothesisWithPose()
                obj_hypothesis.id = int(self.obj_id[results.name[i]])
                obj_hypothesis.score = results.confidence[i]
                obj.results.append(obj_hypothesis)
                obj.bbox.size_y = int(results.size_y[i])
                obj.bbox.size_x = int(results.size_x[i])
                obj.bbox.center.x = int(results.x[i])
                obj.bbox.center.y = int(results.y[i])
                objArray.detections.append(obj)
        except Exception as e:
            img_bgr = image
            rospy.logwarn_throttle(5.0, f">> [DETECTOR] Inference exception: {e}")

        self.object_pub.publish(objArray)

        img = cv2.cvtColor(img_bgr, cv2.COLOR_RGB2BGR)
        try:
            image_out = self.bridge.cv2_to_imgmsg(img, "bgr8")
            image_out.header = data.header
            self.image_pub.publish(image_out)
        except CvBridgeError as e:
            pass


if __name__ == '__main__':
    rospy.init_node('detector_node')
    rospy.loginfo(">> [DETECTOR] Node 'detector_node' starting...")
    obj = Detector()
    rospy.loginfo(">> [DETECTOR] Node running and actively publishing on /objects and /debug_image.")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()
