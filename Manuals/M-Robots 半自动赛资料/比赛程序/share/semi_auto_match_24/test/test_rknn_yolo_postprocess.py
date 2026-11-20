#!/usr/bin/env python
"""semi_auto_match_24 的离线回归测试 —— 不需要 NPU、不需要板子、不需要 ROS。

分两层，跑法不同：

**第一层（纯标准库，任何机器上都能跑）** —— 钉住本赛项最容易静默出错的两条契约：

    python3 test/test_rknn_yolo_postprocess.py

* 类别表契约：``grasp.py`` 拿 ``obj_id == 56`` 当西兰花，而 YOLOv5（COCO-80）
  给的西兰花下标是 50 —— 标签表的第 2 列必须完成这个换算，否则抓西兰花那条
  分支永远不触发，且**不会报任何错**。
* 坐标契约：ROI 内框中心换算回整幅图像坐标的算法必须与原 ``detect.py`` 一致。

**第二层（需要 numpy + cv2）** —— YOLOv5 解码/NMS/letterbox 的回归：

    # 板上（numpy 是板端运行时依赖，随镜像装好）
    run python3 test/test_rknn_yolo_postprocess.py

    # 开发机上 SDK 里只有 aarch64 的 cv2、且没有 numpy，所以这一层跑不了，
    # 会打印 [SKIP] 并以 0 退出；第一层照常执行。
"""

import os
import random
import shutil
import sys
import tempfile

NODES_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, "nodes"))
sys.path.insert(0, NODES_DIR)

from roi_map import centre_to_source, crop_slice, parse_offset, parse_roi  # noqa: E402

# numpy + cv2 只有板上齐备：SDK 的 target sysroot 里只有 aarch64 的 cv2，
# 而 numpy 根本不在 SDK 里（它是板端运行时依赖）。
try:
    import numpy as np
    from rknn_yolo import LetterBox, RknnYoloDetector, load_id_map, load_labels
except ImportError as _exc:            # pragma: no cover - 取决于跑在哪台机器
    np = None
    RUNTIME_SKIP = str(_exc)
else:
    RUNTIME_SKIP = None

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LABELS_PATH = os.path.join(PACKAGE_DIR, "config", "yolo", "coco_labels.txt")

FAILURES = []


def check(name, condition, detail=""):
    status = "PASS" if condition else "FAIL"
    print("[%s] %s%s" % (status, name, ("  " + detail) if detail else ""))
    if not condition:
        FAILURES.append(name)


# ===========================================================================
# 第一层：纯标准库 —— 本赛项的两条契约
# ===========================================================================

def read_label_table(path):
    """按标签文件自己的格式（``下标 外部id 名字``）读出来，不依赖被测代码。"""
    rows = []
    with open(path, "r") as handle:
        for raw in handle:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            index, external, name = line.split(None, 2)
            rows.append((int(index), int(external), name))
    return rows


def test_label_table_contract():
    """``config/yolo/coco_labels.txt`` 必须把 COCO-80 下标换成原程序的 TF 90 编号。

    原 ``detect.py`` 发的是 ``mscoco_label_map.pbtxt`` 的编号（西兰花 56、
    wine glass 46），``grasp.py`` 里写死了 56。这里直接断言标签表的内容 ——
    改这张表是最容易把程序改哑的操作，所以用数据断言钉住。
    """
    rows = read_label_table(LABELS_PATH)
    check("表/共 80 行", len(rows) == 80, "实际 %d" % len(rows))
    if len(rows) != 80:
        return
    check("表/模型下标 0..79 连续", [r[0] for r in rows] == list(range(80)),
          "首尾=%s" % ([rows[0][0], rows[-1][0]],))

    check("表/下标 50 是 broccoli", rows[50][2] == "broccoli", rows[50][2])
    check("表/西兰花对外 id 是 56（grasp.py 比的就是它）", rows[50][1] == 56,
          "rows[50]=%s" % (rows[50],))
    check("表/下标 40 是 wine glass 且对外 id 46",
          rows[40][1] == 46 and rows[40][2] == "wine glass", "rows[40]=%s" % (rows[40],))

    # 90 类编号里有 12 个空洞，所以对外 id 必须严格递增但不连续
    externals = [r[1] for r in rows]
    check("表/对外 id 严格递增", all(b > a for a, b in zip(externals, externals[1:])),
          "%s..%s" % (externals[0], externals[-1]))


def test_roi_mapping_matches_original():
    """ROI 内框中心 → 整幅图像坐标，必须与原 ``detect.py`` 的算法一致。

    原版::

        image = image[160:480, 160:480]                      # 320×320
        loc_x = int((x_min + x_max) * 320 / 2 + 0.25 * 640)  # 偏移 160
        loc_y = int((y_min + y_max) * 320 / 2 + 0.33 * 480)  # 偏移 158.4

    本实现按真实裁剪原点 (160, 160) 换算，x 方向与原版**逐像素一致**；
    y 方向差 1.6 px（原版的 0.33*480 是近似）。把偏移设成 [160, 158.4]
    即可逐像素复现原版 —— 这几条断言就是防止有人"顺手"改掉默认值。
    """
    roi = parse_roi([160, 160, 320, 320])
    check("roi/解析出 (160,160,320,320)", roi == (160, 160, 320, 320), str(roi))

    offset = parse_offset([], roi)
    check("roi/偏移留空时用裁剪原点", offset == (160.0, 160.0), str(offset))
    check("roi/显式偏移被接受（含小数）",
          parse_offset([160, 158.4], roi) == (160.0, 158.4),
          str(parse_offset([160, 158.4], roi)))
    check("roi/偏移个数不对时明确报错",
          _raises(lambda: parse_offset([1, 2, 3], roi), ValueError))

    # 画面 640×480：ROI 装得下；640×400 就装不下
    check("roi/画面装得下时给切片",
          crop_slice((480, 640, 3), roi) == (slice(160, 480), slice(160, 480)),
          str(crop_slice((480, 640, 3), roi)))
    check("roi/画面装不下时返回 None", crop_slice((400, 640, 3), roi) is None)
    check("roi/宽高非正时明确报错",
          _raises(lambda: parse_roi([0, 0, 0, 10]), ValueError))

    rng = random.Random(20241014)
    mismatched_x = mismatched_y = 0
    for _ in range(200):
        x1, x2 = sorted((rng.uniform(0, 320), rng.uniform(0, 320)))
        y1, y2 = sorted((rng.uniform(0, 320), rng.uniform(0, 320)))
        got_x, got_y = centre_to_source(x1, y1, x2, y2, offset)

        # 原版：归一化坐标 × ROI 边长，再按整幅宽高的比例加偏移
        want_x = int(((x1 / 320.0) + (x2 / 320.0)) * 320 / 2 + 0.25 * 640)
        want_y = int(((y1 / 320.0) + (y2 / 320.0)) * 320 / 2 + 0.33 * 480)
        mismatched_x += got_x != want_x
        mismatched_y += got_y != want_y

    check("roi/x 方向与原版逐像素一致", mismatched_x == 0, "不一致 %d/200" % mismatched_x)
    check("roi/y 方向默认比原版大 1.6px（0.33*480 是近似）", mismatched_y == 200,
          "不一致 %d/200" % mismatched_y)

    legacy = parse_offset([160, 158.4], roi)
    samples = [(10.0, 20.0, 30.0, 40.0), (0.0, 0.0, 319.0, 319.0),
               (5.5, 7.25, 88.0, 200.0)]
    same_y = all(
        centre_to_source(x1, y1, x2, y2, legacy)[1]
        == int(((y1 / 320.0) + (y2 / 320.0)) * 320 / 2 + 0.33 * 480)
        for x1, y1, x2, y2 in samples
    )
    check("roi/显式 [160,158.4] 时 y 方向也逐像素一致", same_y)


def _raises(fn, exc_type):
    try:
        fn()
    except exc_type:
        return True
    except Exception:
        return False
    return False


# ===========================================================================
# 第二层：需要 numpy + cv2 —— 解码 / NMS / letterbox
# ===========================================================================

def make_detector(anchors, img_size=(640, 640), conf_thres=0.25, nms_thres=0.45,
                  sigmoid="off"):
    """绕开 __init__（那会去加载模型）构造一个只用于解码的实例。

    这是刻意的：解码数学与 NPU 无关，测试不该依赖板子上的模型文件。
    """
    detector = object.__new__(RknnYoloDetector)
    detector.anchors = anchors
    detector.img_size = img_size
    detector.conf_thres = conf_thres
    detector.nms_thres = nms_thres
    detector.sigmoid = sigmoid
    detector.id_map = None
    detector.labels = None
    detector.num_classes = None
    detector.max_det = 100
    detector.verbose = False
    return detector


# ------------------------------- 参考实现：rknn_model_zoo yolov5 的全量解码 ----

def reference_box_process(position, anchors, img_size):
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    grid = np.concatenate((col, row), axis=1)
    stride = np.array([img_size[1] // grid_h, img_size[0] // grid_w]).reshape(1, 2, 1, 1)

    col = col.repeat(len(anchors), axis=0)
    row = row.repeat(len(anchors), axis=0)
    anchor_arr = np.array(anchors)
    anchor_arr = anchor_arr.reshape(*anchor_arr.shape, 1, 1)

    box_xy = position[:, :2, :, :] * 2 - 0.5
    box_wh = pow(position[:, 2:4, :, :] * 2, 2) * anchor_arr
    box_xy += grid
    box_xy *= stride
    box = np.concatenate((box_xy, box_wh), axis=1)

    xyxy = np.copy(box)
    xyxy[:, 0, :, :] = box[:, 0, :, :] - box[:, 2, :, :] / 2
    xyxy[:, 1, :, :] = box[:, 1, :, :] - box[:, 3, :, :] / 2
    xyxy[:, 2, :, :] = box[:, 0, :, :] + box[:, 2, :, :] / 2
    xyxy[:, 3, :, :] = box[:, 1, :, :] + box[:, 3, :, :] / 2
    return xyxy


def reference_nms_boxes(boxes, scores, nms_thresh):
    x, y = boxes[:, 0], boxes[:, 1]
    w, h = boxes[:, 2] - boxes[:, 0], boxes[:, 3] - boxes[:, 1]
    areas = w * h
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])
        inter = (np.maximum(0.0, xx2 - xx1 + 1e-5) * np.maximum(0.0, yy2 - yy1 + 1e-5))
        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        order = order[1:][np.where(ovr <= nms_thresh)[0]]
    return np.array(keep)


def reference_post_process_v5(outputs, anchors, img_size, conf_thres, nms_thresh):
    boxes, scores, classes_conf = [], [], []
    for idx, data in enumerate(outputs):
        data = data.reshape([len(anchors[idx]), -1] + list(data.shape[-2:]))
        boxes.append(reference_box_process(data[:, :4, :, :], anchors[idx], img_size))
        scores.append(data[:, 4:5, :, :])
        classes_conf.append(data[:, 5:, :, :])

    def flatten(arr):
        return arr.transpose(0, 2, 3, 1).reshape(-1, arr.shape[1])

    boxes = np.concatenate([flatten(v) for v in boxes])
    classes_conf = np.concatenate([flatten(v) for v in classes_conf])
    scores = np.concatenate([flatten(v) for v in scores])

    class_max = np.max(classes_conf, axis=-1)
    classes = np.argmax(classes_conf, axis=-1)
    pos = np.where(class_max * scores.reshape(-1) >= conf_thres)
    boxes, classes, scores = boxes[pos], classes[pos], (class_max * scores.reshape(-1))[pos]

    out_boxes, out_classes, out_scores = [], [], []
    for cls in set(classes):
        idx = np.where(classes == cls)
        keep = reference_nms_boxes(boxes[idx], scores[idx], nms_thresh)
        if len(keep):
            out_boxes.append(boxes[idx][keep])
            out_classes.append(classes[idx][keep])
            out_scores.append(scores[idx][keep])
    if not out_boxes:
        return None, None, None
    return (np.concatenate(out_boxes), np.concatenate(out_classes),
            np.concatenate(out_scores))


def normalize(rows):
    """把两组框排成可比顺序：按 (类别, x1, y1) 排。"""
    order = np.lexsort((rows[:, 2], rows[:, 1], rows[:, 0]))
    return rows[order]


# ------------------------------------------------------------- 构造合成头 ----

ANCHORS = [[[10.0, 13.0], [16.0, 30.0], [33.0, 23.0]],
           [[30.0, 61.0], [62.0, 45.0], [59.0, 119.0]],
           [[116.0, 90.0], [156.0, 198.0], [373.0, 326.0]]]


def make_synthetic_heads(n_classes=10, conf_value=0.9):
    """按给定 anchor 尺寸在三个尺度上各放一个"目标"，返回 (1,C,H,W) 概率头。"""
    heads = []
    for level, (grid, anchor_idx) in enumerate([(80, 0), (40, 1), (20, 2)]):
        channels = 3 * (5 + n_classes)
        head = np.zeros((1, channels, grid, grid), dtype=np.float32)
        data = head.reshape(3, 5 + n_classes, grid, grid)
        row, col = grid // 2, grid // 3 + level
        data[anchor_idx, 0, row, col] = 0.6      # tx
        data[anchor_idx, 1, row, col] = 0.4      # ty
        data[anchor_idx, 2, row, col] = 0.3      # tw
        data[anchor_idx, 3, row, col] = 0.5      # th
        data[anchor_idx, 4, row, col] = conf_value
        data[anchor_idx, 5 + (level % n_classes), row, col] = 0.95
        # 一堆"有点分数但过不了阈值"的噪声锚点，检验先过滤后解码不漏不误
        data[anchor_idx, 4, 0, :] = 0.30
        data[anchor_idx, 5 + (level % n_classes), 0, :] = 0.4
        heads.append(head)
    return heads


def test_decode_matches_reference():
    detector = make_detector(ANCHORS)
    heads = make_synthetic_heads()
    boxes, classes, scores = detector._post_process_v5(heads)
    ref_boxes, ref_classes, ref_scores = reference_post_process_v5(
        heads, ANCHORS, (640, 640), detector.conf_thres, detector.nms_thres)

    check("dec/参考实现都检出了目标", boxes is not None and ref_boxes is not None,
          "prod=%s ref=%s" % (0 if boxes is None else len(boxes),
                              0 if ref_boxes is None else len(ref_boxes)))
    if boxes is None or ref_boxes is None:
        return
    check("dec/检出数量一致", len(boxes) == len(ref_boxes),
          "prod=%d ref=%d" % (len(boxes), len(ref_boxes)))
    if len(boxes) != len(ref_boxes):
        return

    prod = normalize(np.hstack([classes[:, None], boxes, scores[:, None]]))
    ref = normalize(np.hstack([ref_classes[:, None], ref_boxes, ref_scores[:, None]]))
    check("dec/框与类别逐行一致", np.allclose(prod, ref, atol=1e-3),
          "\n  prod=%s\n  ref =%s" % (prod.tolist(), ref.tolist()))


def test_nms_suppresses_overlap():
    detector = make_detector(ANCHORS, nms_thres=0.45)
    same = np.array([[0.0, 0.0, 100.0, 100.0], [2.0, 2.0, 102.0, 102.0]])
    keep = detector._nms_boxes(same, np.array([0.9, 0.8], dtype=np.float32))
    check("NMS/重叠框被抑制", len(keep) == 1, "keep=%s" % (keep.tolist(),))

    apart = np.array([[0.0, 0.0, 100.0, 100.0], [300.0, 300.0, 400.0, 400.0]])
    keep = detector._nms_boxes(apart, np.array([0.9, 0.8], dtype=np.float32))
    check("NMS/不相交框都保留", len(keep) == 2, "keep=%s" % (keep.tolist(),))

    touching = np.array([[0.0, 0.0, 100.0, 100.0], [99.0, 0.0, 199.0, 100.0]])
    keep = detector._nms_boxes(touching, np.array([0.9, 0.8], dtype=np.float32))
    check("NMS/接近但 IoU 低于阈值都保留", len(keep) == 2, "keep=%s" % (keep.tolist(),))


def test_letterbox_round_trip():
    box = LetterBox((640, 640))
    source = np.zeros((720, 1280, 3), dtype=np.uint8)
    padded = box.apply(source)
    check("letterbox/输出尺寸正确", padded.shape[:2] == (640, 640),
          "shape=%s" % (padded.shape,))
    check("letterbox/缩放比与填充正确",
          abs(box.ratio - 0.5) < 1e-6 and abs(box.dw) < 1e-6 and abs(box.dh - 140) < 1e-6,
          "ratio=%s dw=%s dh=%s" % (box.ratio, box.dw, box.dh))

    # 源图 (100,50)-(300,250) 在比 0.5、dh=140 下 -> (50,165)-(150,265)
    mapped = np.array([[50.0, 165.0, 150.0, 265.0]], dtype=np.float32)
    back = box.to_source(mapped)
    check("letterbox/逆映射闭环", np.allclose(back, [[100.0, 50.0, 300.0, 250.0]], atol=1e-3),
          "back=%s" % (back.tolist(),))


def test_missing_anchors_is_loud():
    detector = make_detector(None)
    try:
        detector._post_process_v5(make_synthetic_heads())
    except ValueError as exc:
        check("v5/缺 anchors 时明确报错", "anchors" in str(exc), str(exc))
    else:
        check("v5/缺 anchors 时明确报错", False, "没有报错，静默返回了")


def test_label_parsing():
    """标签文件的三种写法都要认，且带空格的类名不能被截断。"""
    # 板上 /tmp 只读，临时文件放到本目录下
    workdir = tempfile.mkdtemp(prefix="sam24_labels_", dir=os.path.dirname(
        os.path.abspath(__file__)))
    try:
        three = os.path.join(workdir, "three.txt")
        with open(three, "w") as fh:
            fh.write("0 1 person\n1 46 wine glass\n2 88 teddy bear\n")
        names = load_labels(three)
        id_map = load_id_map(three)
        check("标签/三列格式名字不被截断", names == ["person", "wine glass", "teddy bear"],
              "names=%s" % (names,))
        check("标签/三列格式抽出映射表", id_map == [1, 46, 88], "id_map=%s" % (id_map,))

        plain = os.path.join(workdir, "plain.txt")
        with open(plain, "w") as fh:
            fh.write("# 注释行\nperson\nzebra shark\n")
        check("标签/单列格式只有名字",
              load_labels(plain) == ["person", "zebra shark"],
              "names=%s" % (load_labels(plain),))
        check("标签/单列格式不做映射", load_id_map(plain) is None)
    finally:
        shutil.rmtree(workdir, ignore_errors=True)


def test_runtime_label_table():
    """生产路径（``load_id_map``）读本包标签表，结果必须与第一层的数据断言一致。"""
    id_map = load_id_map(LABELS_PATH)
    labels = load_labels(LABELS_PATH)
    check("运行时/标签表解析出 80 类映射", id_map is not None and len(id_map) == 80,
          "len=%s" % (None if id_map is None else len(id_map),))
    if id_map is None or len(id_map) != 80:
        return
    check("运行时/西兰花经解析后对外 id 仍是 56", id_map[50] == 56 and labels[50] == "broccoli",
          "id_map[50]=%s name=%s" % (id_map[50], labels[50]))


def main():
    print("---- 第一层：契约（纯标准库）----")
    test_label_table_contract()
    test_roi_mapping_matches_original()

    if RUNTIME_SKIP is None:
        print("")
        print("---- 第二层：解码 / NMS / letterbox（numpy + cv2）----")
        test_decode_matches_reference()
        test_nms_suppresses_overlap()
        test_letterbox_round_trip()
        test_missing_anchors_is_loud()
        test_label_parsing()
        test_runtime_label_table()
        skipped = False
    else:
        skipped = True
        print("")
        print("[SKIP] 第二层（解码/NMS/letterbox）需要 numpy + cv2，本机没有：%s"
              % RUNTIME_SKIP)
        print("       板上跑法：run python3 test/test_rknn_yolo_postprocess.py")

    print("")
    if FAILURES:
        print("FAILED %d: %s" % (len(FAILURES), ", ".join(FAILURES)))
        return 1
    print("ALL PASS" + ("（第二层已跳过）" if skipped else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
