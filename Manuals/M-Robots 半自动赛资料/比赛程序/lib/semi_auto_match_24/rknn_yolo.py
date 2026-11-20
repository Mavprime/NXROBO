"""通用 YOLO 检测运行时（RK3588 NPU，基于 ``rknn`` 兼容层）。

本模块与 ROS 无关，可以单独在板子上跑单测/基准：
``run python3 rknn_yolo.py <model.rknn> <bus.jpg>``。

支持的导出头
------------
* **v5 系**（yolov5 / v6 / v7）：3 个带 anchor 的尺度头，通道数 = na*(5+nc)，
  含 objectness 通道；输出可能是 raw logits（自动识别并补 sigmoid）。
* **v8 系**（yolov8 / v9 / v11）：单头 ``(1, 4+nc, N)`` 或 ``(1, N, 4+nc)``，
  无 objectness、无 anchor，xywh 已经是输入图像像素。

DFL 未解码的 6 输出头（3×box + 3×cls）不做支持，遇到会给出明确报错而不是猜。

解码的两条硬约定
----------------
1. **先过滤后解码**：阈值判断只跑 objectness（v5）或类分数（v8）这一个通道，
   只对过阈值的候选做 sigmoid 与框解码。25200 个 anchor 全量解码是纯浪费，
   而 ``conf = class_max * objectness <= objectness``，所以先按 objectness
   过滤不会漏框（数学等价，不是近似）。
2. **框一律用 xyxy 表示**，NMS 交集直接算 ``min(x2) - max(x1)``；
   不要退回 (x, y, w, h) 那种写法——那里漏写右下角的 ``+w``/``+h``
   会让 IoU 恒为 0，NMS 静默失效（见 SKILL 铁律 8）。
"""

import collections
import os
import sys
import time

import cv2
import numpy as np

__all__ = [
    "Detection",
    "LetterBox",
    "RknnYoloDetector",
    "load_labels",
    "load_id_map",
    "draw_detections",
]

# ------------------------------------------------------- rknn 兼容层定位 ----

def _ensure_rknn_importable():
    """先 import rknn；失败则把本包自带的兼容层目录塞进 sys.path。

    安装后的布局与源码树的布局不同，两条候选路径都要覆盖：

    * 源码树     ``semi_auto_match_24/nodes/rknn_yolo.py`` → ``semi_auto_match_24/python``
    * 安装树     ``usr/lib/semi_auto_match_24/rknn_yolo.py`` → ``usr/share/semi_auto_match_24/python``

    这样调用方只要 ``import rknn_yolo`` 就行，不必自己管 sys.path 顺序。
    """
    try:
        import rknn  # noqa: F401
        return
    except ImportError:
        pass

    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.environ.get("SEMI_AUTO_MATCH_RKNN_PY"),
        os.path.join(here, os.pardir, "python"),
        os.path.join(here, os.pardir, os.pardir, "share", "semi_auto_match_24", "python"),
    ]
    tried = []
    for candidate in candidates:
        if not candidate:
            continue
        candidate = os.path.abspath(candidate)
        tried.append(candidate)
        if os.path.isdir(os.path.join(candidate, "rknn")):
            if candidate not in sys.path:
                sys.path.insert(0, candidate)
            import rknn  # noqa: F401
            return
    raise ImportError(
        "找不到 rknn 兼容层。已尝试: %s；也可以用环境变量 SEMI_AUTO_MATCH_RKNN_PY "
        "指向含 rknn/ 子目录的目录" % (tried,)
    )


_ensure_rknn_importable()

from rknn.api import RKNNLite, core_mask_from_string, error_name  # noqa: E402


# --------------------------------------------------------------- 标签/映射 ----

def _iter_data_lines(path):
    with open(path, "r") as handle:
        for lineno, raw in enumerate(handle, 1):
            line = raw.split("#", 1)[0].strip()
            if line:
                yield lineno, line


def _is_int(text):
    try:
        int(text)
    except ValueError:
        return False
    return True


def _parse_label_line(line):
    """解析一行标签，返回 ``(模型下标|None, 外部id|None, 名称)``。

    下面几种写法都支持，名称允许带空格（取剩余部分，不会把 ``wine glass``
    截成 ``glass``）：

    * ``person``           —— 只有名字
    * ``0 person``         —— 模型类别下标 + 名字
    * ``0 1 person``       —— 模型下标 + 外部 id + 名字（本包配置的格式）
    * ``0 1 wine glass``   —— 名字带空格
    """
    parts = line.split(None, 2)
    if len(parts) == 3 and _is_int(parts[0]) and _is_int(parts[1]):
        return int(parts[0]), int(parts[1]), parts[2]
    if len(parts) >= 2 and _is_int(parts[0]):
        return int(parts[0]), None, line.split(None, 1)[1]
    return None, None, line


def load_labels(path):
    """读类别名列表，一行一个（格式见 :func:`_parse_label_line`）。"""
    if not path:
        return None
    names = []
    for lineno, line in _iter_data_lines(path):
        name = _parse_label_line(line)[2]
        if not name:
            raise ValueError("%s:%d 解析不出类别名" % (path, lineno))
        names.append(name)
    if not names:
        raise ValueError("标签文件是空的: %s" % path)
    return names


def load_id_map(path):
    """读"模型类别下标 → 对外 id"的映射表。

    要求**每行都带下标与外部 id**（``下标 外部id 名字``），否则返回 ``None``
    表示这份标签文件不做映射。返回的列表下标即模型类别下标。
    """
    if not path:
        return None
    rows = []
    for _, line in _iter_data_lines(path):
        index, external, _ = _parse_label_line(line)
        if index is None or external is None:
            return None
        rows.append((index, external))
    if not rows:
        return None
    rows.sort()
    if [index for index, _ in rows] != list(range(len(rows))):
        raise ValueError("%s 的模型类别下标必须是 0..%d 连续" % (path, len(rows) - 1))
    return [external for _, external in rows]


# --------------------------------------------------------------- letterbox ----

class LetterBox(object):
    """等比缩放 + 居中填充，映射规则与 rknn_model_zoo 的 COCO_test_helper 一致。

    ``to_source`` 把解码出来的框从"填充后的输入空间"搬回"原图像素"。
    """

    def __init__(self, new_shape=(640, 640), pad_value=(0, 0, 0)):
        self.new_shape = (int(new_shape[0]), int(new_shape[1]))  # (h, w)
        self.pad_value = pad_value
        self.ratio = 1.0
        self.dw = 0.0
        self.dh = 0.0

    def apply(self, img):
        h, w = img.shape[:2]
        new_h, new_w = self.new_shape
        ratio = min(float(new_h) / h, float(new_w) / w)
        new_unpad = (int(round(w * ratio)), int(round(h * ratio)))
        dw = (new_w - new_unpad[0]) / 2.0
        dh = (new_h - new_unpad[1]) / 2.0
        if (w, h) != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right,
                                 cv2.BORDER_CONSTANT, value=self.pad_value)
        self.ratio, self.dw, self.dh = ratio, dw, dh
        return img

    def to_source(self, boxes):
        out = np.array(boxes, dtype=np.float32, copy=True)
        out[:, [0, 2]] = (out[:, [0, 2]] - self.dw) / self.ratio
        out[:, [1, 3]] = (out[:, [1, 3]] - self.dh) / self.ratio
        return out


Detection = collections.namedtuple(
    "Detection", "class_id label score x1 y1 x2 y2"
)


# ------------------------------------------------------------------ 检测器 ----

class RknnYoloDetector(object):
    """把 ``.rknn`` 模型包成 ``infer(bgr) -> [Detection]``。

    :param model_path: ``.rknn`` 模型路径
    :param labels: 类别名列表，或标签文件路径（见 :func:`load_labels`）
    :param anchors: anchor 列表 ``[[[w,h], ...], ...]``（每个尺度一组），
                    v5 系必需；给文件路径时按每行一个数字读，形状 ``(n_scale, -1, 2)``
    :param img_size: ``(w, h)``；None 表示从模型输入属性里读
    :param id_map: 模型类别下标 → 对外 id 的映射；None 表示不映射
    :param head: ``auto`` / ``v5`` / ``v8``
    :param sigmoid: ``auto`` / ``on`` / ``off``；只作用于 v5 系的解码
    """

    def __init__(self, model_path, labels=None, anchors=None, lib_path=None,
                 img_size=None, conf_thres=0.25, nms_thres=0.45,
                 core_mask="0_1_2", head="auto", sigmoid="auto",
                 max_det=100, id_map=None, verbose=False, warmup=True):
        self.model_path = model_path
        self.conf_thres = float(conf_thres)
        self.nms_thres = float(nms_thres)
        self.head = head
        self.sigmoid = sigmoid
        self.max_det = int(max_det)
        self.verbose = bool(verbose)
        self.lib_path = lib_path

        self.labels = load_labels(labels) if isinstance(labels, str) else labels
        self.id_map = list(id_map) if id_map is not None else None
        self.anchors = self._read_anchors(anchors)

        self.last_infer_ms = 0.0
        self.last_post_ms = 0.0
        self.frames = 0
        self.head_kind = None
        self.num_classes = None

        self._rknn = RKNNLite(verbose=self.verbose, lib_path=lib_path)
        ret = self._rknn.load_rknn(model_path)
        if ret != 0:
            raise RuntimeError("load_rknn(%s) 失败: %s" % (model_path, error_name(ret)))
        self._rknn.init_runtime(core_mask=core_mask_from_string(core_mask))

        if img_size is None:
            img_size = self._detect_input_size()
        self.img_size = (int(img_size[0]), int(img_size[1]))    # (w, h)

        if warmup:
            self._warmup()

    def _warmup(self):
        """用一张空图跑一次完整前向。

        两个作用：把模型能跑通这件事在**节点启动时**就验证掉（而不是等第一帧
        图像才发现模型/运行时不对），以及把 head 类型与类别数定下来，
        好让启动日志里直接看到真实的 nc —— 类别数对不上是最常见的错配。
        """
        blank = np.zeros((self.img_size[1], self.img_size[0], 3), dtype=np.uint8)
        self.infer(blank)
        self.frames = 0
        self.last_infer_ms = 0.0
        self.last_post_ms = 0.0

    # -- 初始化辅助 ----------------------------------------------------
    @staticmethod
    def _read_anchors(anchors):
        if anchors is None:
            return None
        if isinstance(anchors, str):
            values = []
            for _, line in _iter_data_lines(anchors):
                values.append(float(line.split()[0]))
            return np.array(values, dtype=np.float32).reshape(3, -1, 2).tolist()
        return np.array(anchors, dtype=np.float32).tolist()

    def _detect_input_size(self):
        """从模型输入属性里读输入尺寸；读不到就报错让人显式给 img_size。"""
        attrs = getattr(self._rknn, "input_attrs", None)
        if not attrs:
            raise ValueError("拿不到模型输入属性，请显式指定 img_size=(w, h)")
        shape = attrs[0].shape
        if len(shape) != 4:
            raise ValueError("输入维度不是 4 维: %s，请显式指定 img_size" % (shape,))
        # NHWC: (1, H, W, C)；NCHW: (1, C, H, W)
        if attrs[0].is_nhwc or shape[-1] in (1, 3, 4):
            return (int(shape[2]), int(shape[1]))
        return (int(shape[3]), int(shape[2]))

    @property
    def runtime_versions(self):
        return (self._rknn.sdk_version, self._rknn.driver_version)

    def describe(self):
        return ("model=%s api=%s driver=%s input=%dx%d head=%s nc=%s conf=%.2f nms=%.2f"
                % (os.path.basename(self.model_path), self._rknn.sdk_version,
                   self._rknn.driver_version, self.img_size[0], self.img_size[1],
                   self.head_kind or self.head, self.num_classes,
                   self.conf_thres, self.nms_thres))

    # -- 推理 ----------------------------------------------------------
    def infer(self, bgr_image):
        """对一帧 BGR uint8 图像做检测。

        :returns: ``[Detection]``，坐标已反映射回原图像素，按分数降序。
        """
        if bgr_image is None or getattr(bgr_image, "size", 0) == 0:
            return []

        box = LetterBox((self.img_size[1], self.img_size[0]))
        padded = box.apply(bgr_image)
        # 驱动按模型元数据做 mean/std/rgb2bgr，这里只负责把 BGR 转成模型要的 RGB。
        rgb = np.ascontiguousarray(cv2.cvtColor(padded, cv2.COLOR_BGR2RGB))

        start = time.time()
        outputs = self._rknn.inference(inputs=[rgb], data_type="uint8",
                                       data_format="nhwc")
        self.last_infer_ms = (time.time() - start) * 1000.0
        self.frames += 1

        post_start = time.time()
        kind = self._resolve_head_kind(outputs)
        if kind == "v5":
            boxes, classes, scores = self._post_process_v5(outputs)
        else:
            boxes, classes, scores = self._post_process_v8(outputs)

        detections = []
        if boxes is not None and len(boxes):
            real = box.to_source(boxes)
            height, width = bgr_image.shape[:2]
            order = np.argsort(-scores)[:self.max_det]
            for idx in order:
                x1, y1, x2, y2 = real[idx]
                x1 = int(round(min(max(x1, 0), width - 1)))
                y1 = int(round(min(max(y1, 0), height - 1)))
                x2 = int(round(min(max(x2, 0), width - 1)))
                y2 = int(round(min(max(y2, 0), height - 1)))
                if x2 <= x1 or y2 <= y1:
                    continue
                raw_id = int(classes[idx])
                class_id = self.id_map[raw_id] if self.id_map else raw_id
                detections.append(Detection(class_id, self._class_name(raw_id),
                                            float(scores[idx]), x1, y1, x2, y2))
        self.last_post_ms = (time.time() - post_start) * 1000.0
        return detections

    def _class_name(self, raw_id):
        if self.labels and 0 <= raw_id < len(self.labels):
            return self.labels[raw_id]
        return str(raw_id)

    # -- head 判定 -----------------------------------------------------
    def _resolve_head_kind(self, outputs):
        if self.head in ("v5", "v8"):
            kind = self.head
        else:
            shapes = [tuple(o.shape) for o in outputs]
            if len(outputs) == 1 and len(shapes[0]) == 3:
                kind = "v8"
            elif len(outputs) >= 3 and all(len(s) == 4 for s in shapes):
                kind = "v5"
            elif len(outputs) == 6:
                raise NotImplementedError(
                    "这个模型导出的是未做 DFL 解码的 6 输出头 %s；"
                    "请在导出 .rknn 时打开 DFL 解码（rknn_model_zoo 的 "
                    "YOLOv8 导出脚本默认会解码），或改用单头模型" % (shapes,)
                )
            else:
                raise ValueError(
                    "认不出模型输出布局 %s；用 head='v5' / head='v8' 显式指定"
                    % (shapes,)
                )
        self.head_kind = kind
        return kind

    # -- v5 系解码 -----------------------------------------------------
    def _post_process_v5(self, outputs):
        if not self.anchors:
            raise ValueError("v5 系模型需要 anchors，请给 anchors= 文件路径或列表")

        all_boxes, all_classes, all_scores = [], [], []

        for level, out in enumerate(outputs):
            data = self._as_chw(out, "v5 尺度头 %d" % level)
            if level >= len(self.anchors):
                raise ValueError(
                    "模型有 %d 个输出头，但只给了 %d 组 anchors"
                    % (len(outputs), len(self.anchors))
                )
            anchors = np.asarray(self.anchors[level], dtype=np.float32)
            n_anchor = anchors.shape[0]
            if data.shape[0] % n_anchor != 0:
                raise ValueError(
                    "尺度 %d 的通道数 %d 不能被 anchor 数 %d 整除"
                    % (level, data.shape[0], n_anchor)
                )
            channels = data.shape[0] // n_anchor
            n_classes = channels - 5
            if n_classes < 1:
                raise ValueError("尺度 %d 的通道数 %d 太少，不像 YOLO 头"
                                 % (level, data.shape[0]))
            self._remember_classes(n_classes)

            grid_h, grid_w = data.shape[1], data.shape[2]
            data = data.reshape(n_anchor, channels, grid_h, grid_w)

            # sigmoid 约定**每个尺度头判定一次**（只看 objectness 通道），
            # 判定结果对这个头的所有元素一致生效 —— 否则同一个头里
            # "objectness 走原值、类别走 sigmoid"这种混搭会静默出错。
            sig = self._sigmoid_for(data)

            # 只按 objectness 过滤：conf = class_max * obj <= obj，
            # 所以不会漏掉任何最终能过阈值的候选。
            objectness = sig(data[:, 4, :, :])
            a_idx, i_idx, j_idx = np.nonzero(objectness >= self.conf_thres)
            if a_idx.size == 0:
                continue

            probs = sig(data[a_idx, :, i_idx, j_idx])          # (K, channels)
            class_probs = probs[:, 5:]
            class_max = class_probs.max(axis=1)
            class_id = class_probs.argmax(axis=1)
            conf = class_max * probs[:, 4]

            keep = conf >= self.conf_thres
            if not keep.any():
                continue
            probs, class_id, conf = probs[keep], class_id[keep], conf[keep]
            a_k, i_k, j_k = a_idx[keep], i_idx[keep], j_idx[keep]

            stride_w = float(self.img_size[0]) / grid_w
            stride_h = float(self.img_size[1]) / grid_h
            anchor_wh = anchors[a_k]

            cx = (probs[:, 0] * 2.0 - 0.5 + j_k) * stride_w
            cy = (probs[:, 1] * 2.0 - 0.5 + i_k) * stride_h
            bw = np.power(probs[:, 2] * 2.0, 2) * anchor_wh[:, 0]
            bh = np.power(probs[:, 3] * 2.0, 2) * anchor_wh[:, 1]

            all_boxes.append(np.stack([cx - bw / 2.0, cy - bh / 2.0,
                                       cx + bw / 2.0, cy + bh / 2.0], axis=1))
            all_classes.append(class_id.astype(np.int64))
            all_scores.append(conf)

        if not all_boxes:
            return None, None, None
        return self._nms_per_class(np.concatenate(all_boxes),
                                   np.concatenate(all_classes),
                                   np.concatenate(all_scores))

    # -- v8 系解码 -----------------------------------------------------
    def _post_process_v8(self, outputs):
        if len(outputs) != 1:
            raise NotImplementedError(
                "v8 系只支持单头输出，实际有 %d 个头" % len(outputs)
            )
        pred = self._as_head_matrix(outputs[0], "v8 头")
        if pred.ndim != 2:
            raise ValueError("v8 头期望 2 维 (4+nc, N)，实际 %s" % (pred.shape,))
        if pred.shape[0] > pred.shape[1]:
            pred = pred.T                       # (N, 4+nc) → (4+nc, N)
        n_classes = pred.shape[0] - 4
        if n_classes < 1:
            raise ValueError("v8 头通道数 %d 太少" % pred.shape[0])
        self._remember_classes(n_classes)

        xywh = pred[:4].T                       # (N, 4)，输入图像像素
        scores = pred[4:].T                     # (N, nc)
        if self.sigmoid == "on":
            scores = 1.0 / (1.0 + np.exp(-scores))
        elif self.sigmoid == "auto" and (scores.min() < -0.01 or scores.max() > 1.01):
            scores = 1.0 / (1.0 + np.exp(-scores))

        class_id = scores.argmax(axis=1)
        conf = scores[np.arange(scores.shape[0]), class_id]
        keep = conf >= self.conf_thres
        if not keep.any():
            return None, None, None
        xywh, class_id, conf = xywh[keep], class_id[keep], conf[keep]

        cx, cy, bw, bh = xywh[:, 0], xywh[:, 1], xywh[:, 2], xywh[:, 3]
        boxes = np.stack([cx - bw / 2.0, cy - bh / 2.0,
                          cx + bw / 2.0, cy + bh / 2.0], axis=1)
        return self._nms_per_class(boxes, class_id.astype(np.int64), conf)

    # -- 公共小工具 ----------------------------------------------------
    def _sigmoid_for(self, head):
        """决定这个尺度头要不要补 sigmoid，返回对应的逐元素函数。

        只看 objectness 通道（几万个值）而不是整个头（几十万个值）；
        阈值取 [-0.01, 1.01]：概率一定落在这个区间内，日志值几乎不会。
        """
        if self.sigmoid == "on":
            apply = True
        elif self.sigmoid == "off":
            apply = False
        else:
            obj = head[:, 4, :, :]
            apply = bool(obj.min() < -0.01 or obj.max() > 1.01)
        if apply:
            return lambda x: 1.0 / (1.0 + np.exp(-x))
        return lambda x: x

    def _remember_classes(self, n_classes):
        if self.num_classes is None:
            self.num_classes = n_classes
        elif self.num_classes != n_classes:
            raise ValueError("各输出头的类别数不一致: %d vs %d"
                             % (self.num_classes, n_classes))
        if self.labels and len(self.labels) != n_classes and self.verbose:
            print("rknn_yolo: 警告 标签数 %d 与模型类别数 %d 不一致"
                  % (len(self.labels), n_classes))

    @staticmethod
    def _as_chw(out, what):
        """去掉 batch 维：``(1, C, H, W) -> (C, H, W)``；已经是低维就原样返回。"""
        if out.ndim == 4:
            if out.shape[0] != 1:
                raise ValueError("%s 期望 batch=1，实际 %s" % (what, out.shape))
            return out[0]
        if out.ndim in (2, 3):
            return out
        raise ValueError("%s 的维度不支持: %s" % (what, out.shape))

    @staticmethod
    def _as_head_matrix(out, what):
        """v8 头是 ``(1, 4+nc, N)``：去掉 batch 维，得到 2 维矩阵。

        不能和 :meth:`_as_chw` 共用 —— 那边的 3 维表示"(C, H, W) 无 batch"，
        这边的 3 维表示"(1, C, N) 有 batch"，同一个形状两种含义。
        """
        if out.ndim == 3:
            if out.shape[0] != 1:
                raise ValueError("%s 期望 batch=1，实际 %s" % (what, out.shape))
            return out[0]
        if out.ndim == 2:
            return out
        raise ValueError("%s 期望 (1, 4+nc, N) / (4+nc, N)，实际 %s" % (what, out.shape))

    def _nms_per_class(self, boxes, classes, scores):
        keep_boxes, keep_classes, keep_scores = [], [], []
        for cls in np.unique(classes):
            idx = np.where(classes == cls)[0]
            keep = self._nms_boxes(boxes[idx], scores[idx])
            if keep.size:
                keep_boxes.append(boxes[idx][keep])
                keep_classes.append(classes[idx][keep])
                keep_scores.append(scores[idx][keep])
        if not keep_boxes:
            return None, None, None
        return (np.concatenate(keep_boxes),
                np.concatenate(keep_classes),
                np.concatenate(keep_scores))

    def _nms_boxes(self, boxes, scores):
        """标准 IoU NMS；boxes 为 xyxy。"""
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            current = order[0]
            keep.append(current)
            rest = order[1:]
            if rest.size == 0:
                break
            xx1 = np.maximum(x1[current], x1[rest])
            yy1 = np.maximum(y1[current], y1[rest])
            xx2 = np.minimum(x2[current], x2[rest])
            yy2 = np.minimum(y2[current], y2[rest])
            inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            union = areas[current] + areas[rest] - inter
            iou = np.where(union > 0, inter / np.maximum(union, 1e-12), 0.0)
            order = rest[iou <= self.nms_thres]
        return np.asarray(keep, dtype=np.int64)

    # -- 收尾 ----------------------------------------------------------
    def release(self):
        if self._rknn is not None:
            self._rknn.release()

    def __del__(self):
        try:
            self.release()
        except Exception:
            pass


def draw_detections(image, detections, font_scale=0.6, thickness=2):
    """原地画框与标签，返回同一张图（BGR）。"""
    for det in detections:
        cv2.rectangle(image, (det.x1, det.y1), (det.x2, det.y2), (255, 0, 0), thickness)
        cv2.putText(image, "%s %.2f" % (det.label, det.score),
                    (det.x1, max(12, det.y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 255), thickness)
    return image


# ------------------------------------------------------------------ 自测入口 ----

def _main(argv):
    import argparse
    parser = argparse.ArgumentParser(description="rknn_yolo 单文件自测（板上跑）")
    parser.add_argument("model")
    parser.add_argument("image")
    parser.add_argument("--labels", default=None)
    parser.add_argument("--anchors", default=None)
    parser.add_argument("--lib", default=None)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--nms", type=float, default=0.45)
    parser.add_argument("--core-mask", default="0_1_2")
    parser.add_argument("--head", default="auto", choices=["auto", "v5", "v8"])
    parser.add_argument("--repeat", type=int, default=1,
                        help="重复推理次数，>1 时打印端到端 FPS")
    args = parser.parse_args(argv)

    detector = RknnYoloDetector(
        args.model, labels=args.labels, anchors=args.anchors, lib_path=args.lib,
        conf_thres=args.conf, nms_thres=args.nms, core_mask=args.core_mask,
        head=args.head, verbose=True,
    )
    print("detector: %s" % detector.describe())

    image = cv2.imread(args.image)
    if image is None:
        raise SystemExit("读不到图片: %s" % args.image)

    started = time.time()
    detections = []
    for _ in range(max(1, args.repeat)):
        detections = detector.infer(image)
    elapsed = time.time() - started

    for det in detections:
        print("  %-14s %.3f  [%d, %d, %d, %d]"
              % (det.label, det.score, det.x1, det.y1, det.x2, det.y2))
    print("检出 %d 个目标；NPU 调用 %.1f ms，后处理 %.1f ms，端到端 %.1f ms (%.1f FPS)"
          % (len(detections), detector.last_infer_ms, detector.last_post_ms,
             elapsed / max(1, args.repeat) * 1000.0,
             max(1, args.repeat) / elapsed if elapsed > 0 else 0.0))
    detector.release()
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
