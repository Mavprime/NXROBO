#!/usr/bin/env python3.12
"""semi_auto_match_24 的 NPU 物体检测节点（RK3588 + YOLOv5）。

这是原 ``scripts/detect.py``（TensorFlow SSD MobileNet）的 RKNN 替代实现。
**对外话题与消息类型和原来完全一致**，所以 ``nodes/grasp.py`` 与 launch
都不用跟着改：

===========  ==============================================  ========================================
方向          话题                                            类型
===========  ==============================================  ========================================
订阅          ``image``                                       ``sensor_msgs/Image``（launch 里 remap）
发布          ``location``                                    ``semi_auto_match_24/LocationCentre``
发布          ``debug_image``                                 ``sensor_msgs/Image``
===========  ==============================================  ========================================

与原版的三处关键差异
--------------------
1. **推理换成 RK3588 NPU 上的 YOLOv5**（``librknnrt.so`` + ctypes，见
   ``python/rknn``）：不再需要 TensorFlow / object_detection /
   ``mscoco_label_map.pbtxt`` / ``frozen_inference_graph.pb``。
2. **类别 id 仍然按原程序的编号发布。** 原版发的是 tensorflow_object_detector
   那份 ``mscoco_label_map.pbtxt`` 的 90 类编号（西兰花 = 56），而 YOLOv5 输出的
   是 COCO-80 下标（西兰花 = 50）。``config/yolo/coco_labels.txt`` 是
   ``模型下标 → 外部 id`` 的换算表，默认打开，所以 ``grasp.py`` 里的
   ``loc.obj_id == 56`` 不用改；``~publish_tf_ids:=false`` 可以关掉换算。
3. **兴趣区（ROI）裁剪照旧。** 原版只把 640×480 里的 ``[160:480, 160:480]``
   这块送进模型，再把框中心换算回整幅图像坐标（因为机械臂标定
   ``thefile.txt`` 是整幅图像坐标系下的）。这里保留同样的裁剪与换算，
   只是把裁剪框与换算偏移做成了参数（``~roi`` / ``~roi_offset``）。

启动（板上必须走 run 包装，裸 python3 缺 libpython3.12.so）::

    run roslaunch semi_auto_match_24 detect_rknn.launch
    run rosrun semi_auto_match_24 detect_rknn.py _model_path:=/path/yolov5s.rknn
"""

import os
import sys
import time

import rospkg
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from semi_auto_match_24.msg import LocationCentre

# 导入 rknn_yolo 的同时会把 rknn 兼容层挂上 sys.path
# （见该模块的 _ensure_rknn_importable）
from rknn_yolo import RknnYoloDetector, draw_detections, load_id_map
# ROI ↔ 整幅图像坐标的换算单独成模块，好让它能脱离 ROS 单独测
from roi_map import centre_to_source, crop_slice, parse_offset, parse_roi

# 与本板厂商运行时一致的默认位置；换板子/换模型时用 rosparam 覆盖。
DEFAULT_MODEL = "/data/local/release/rknn-demo/model/yolov5s-640-640.rknn"
DEFAULT_CORE_MASK = "0_1_2"
DEFAULT_IMG_SIZE = [640, 640]

# 原程序的兴趣区：image[160:480, 160:480]，即 640×480 画面里的一块 320×320。
# 写成 (x, y, w, h) 便于换成别的相机分辨率。
DEFAULT_ROI = [160, 160, 320, 320]

# 端到端 FPS 的统计窗口（帧数）。只认端到端速率，
# 用 NPU 调用耗时反推出来的"FPS"会虚高数倍。
FPS_WINDOW = 30


class RknnDetectorNode(object):

    def __init__(self):
        model_path = rospy.get_param("~model_path", DEFAULT_MODEL)
        lib_path = rospy.get_param("~lib_path", "") or None
        labels_path = rospy.get_param("~labels_path", "")
        anchors_path = rospy.get_param("~anchors_path", "")
        if not labels_path or not anchors_path:
            # 只有真要读包内默认文件时才去找包路径：显式给了路径的调用
            # （例如直接从别的目录跑脚本）不该因为找不到包而失败。
            yolo_config_dir = os.path.join(
                rospkg.RosPack().get_path("semi_auto_match_24"), "config", "yolo")
            labels_path = labels_path or os.path.join(yolo_config_dir, "coco_labels.txt")
            anchors_path = anchors_path or os.path.join(yolo_config_dir, "anchors_yolov5.txt")

        self.publish_tf_ids = bool(rospy.get_param("~publish_tf_ids", True))
        self.publish_debug_image = bool(rospy.get_param("~publish_debug_image", True))

        self.roi = parse_roi(rospy.get_param("~roi", DEFAULT_ROI))
        self.roi_offset = parse_offset(rospy.get_param("~roi_offset", []), self.roi)

        # 类别名与"模型下标 → 外部 id"映射来自同一份标签文件：
        # 三列格式（下标 外部id 名字）才启用映射，否则按模型下标发布。
        id_map = load_id_map(labels_path) if self.publish_tf_ids else None
        if self.publish_tf_ids and id_map is None:
            rospy.logwarn(
                "[detect_rknn] %s 不是三列格式（模型下标 外部id 名字），"
                "本次直接发布模型类别下标；grasp.py 里的 obj_id 会对不上号",
                labels_path)

        self.detector = RknnYoloDetector(
            model_path,
            labels=labels_path if labels_path else None,
            anchors=anchors_path if anchors_path else None,
            lib_path=lib_path,
            img_size=rospy.get_param("~img_size", DEFAULT_IMG_SIZE) or None,
            conf_thres=rospy.get_param("~conf_thres", 0.25),
            nms_thres=rospy.get_param("~nms_thres", 0.45),
            core_mask=rospy.get_param("~core_mask", DEFAULT_CORE_MASK),
            head=rospy.get_param("~head", "auto"),
            sigmoid=rospy.get_param("~sigmoid", "auto"),
            max_det=rospy.get_param("~max_det", 100),
            id_map=id_map,
            verbose=bool(rospy.get_param("~verbose", False)),
        )
        rospy.loginfo("[detect_rknn] %s", self.detector.describe())
        rospy.loginfo("[detect_rknn] 兴趣区 roi=%s，换算偏移 roi_offset=%s",
                      self.roi, self.roi_offset)
        if id_map:
            rospy.loginfo("[detect_rknn] 类别 id 已按映射表换算（%d 类）", len(id_map))

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("debug_image", Image, queue_size=1)
        self.location_pub = rospy.Publisher("location", LocationCentre, queue_size=10)
        # 只留最新一帧：处理慢时自动丢旧帧，而不是排队造成延迟累积。
        self.image_sub = rospy.Subscriber("image", Image, self.image_cb,
                                          queue_size=1, buff_size=2 ** 24)
        self._stamps = []
        rospy.on_shutdown(self.detector.release)

    # -- 每帧 ----------------------------------------------------------
    def image_cb(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            rospy.logerr_throttle(5.0, "[detect_rknn] 图像转换失败: %s", exc)
            return

        roi_image = self._crop(image)
        if roi_image is None:
            rospy.logerr_throttle(
                5.0, "[detect_rknn] 图像 %s 装不下兴趣区 %s，请改 ~roi",
                image.shape, self.roi)
            return

        try:
            detections = self.detector.infer(roi_image)
        except Exception as exc:            # 单帧异常不该打死节点
            rospy.logerr_throttle(5.0, "[detect_rknn] 推理失败: %s", exc)
            return

        for det in detections:
            self.location_pub.publish(self._build_location(det))

        if self.publish_debug_image:
            draw_detections(roi_image, detections)
            try:
                out = self.bridge.cv2_to_imgmsg(roi_image, "bgr8")
            except CvBridgeError as exc:
                rospy.logerr_throttle(5.0, "[detect_rknn] 调试图转换失败: %s", exc)
            else:
                out.header = msg.header
                self.image_pub.publish(out)

        self._record_fps()

    def _crop(self, image):
        """按 ``~roi`` 裁出兴趣区；画面比 ROI 小就返回 None（不猜、不补边）。"""
        window = crop_slice(image.shape, self.roi)
        if window is None:
            return None
        return image[window]

    def _build_location(self, det):
        """把 ROI 内的框中心换算回整幅图像坐标，发成原程序的 LocationCentre。

        原版是 ``int((x_min + x_max) * roi_w / 2 + roi_x)``，即
        "ROI 内中心 + ROI 原点"；换算细节见 ``nodes/roi_map.py``。
        """
        cx, cy = centre_to_source(det.x1, det.y1, det.x2, det.y2, self.roi_offset)
        loc = LocationCentre()
        loc.obj_id = int(det.class_id)
        loc.loc_centre_x = cx
        loc.loc_centre_y = cy
        return loc

    def _record_fps(self):
        now = time.time()
        self._stamps.append(now)
        if len(self._stamps) > FPS_WINDOW:
            del self._stamps[:-FPS_WINDOW]
        if len(self._stamps) < 2:
            return
        span = self._stamps[-1] - self._stamps[0]
        fps = (len(self._stamps) - 1) / span if span > 0 else 0.0
        rospy.loginfo_throttle(
            2.0, "[detect_rknn] 端到端 %.1f FPS | NPU %.1f ms | 后处理 %.1f ms",
            fps, self.detector.last_infer_ms, self.detector.last_post_ms)


def main(argv):
    rospy.init_node("detect_rknn", argv=argv)
    try:
        RknnDetectorNode()
    except Exception as exc:
        # 模型/运行时版本不匹配这类问题必须立刻暴露，而不是静默发空消息。
        rospy.logfatal("[detect_rknn] 初始化失败: %s", exc)
        return 1
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
