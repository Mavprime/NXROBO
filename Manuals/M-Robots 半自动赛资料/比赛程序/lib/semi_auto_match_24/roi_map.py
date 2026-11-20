"""兴趣区（ROI）与整幅图像坐标之间的换算 —— 本赛项的坐标契约。

为什么单独一个模块
------------------
机械臂标定文件 ``thefile.txt`` 给出的是**整幅图像像素坐标**到机械臂坐标的线性映射
（``pos.x = x_kb[0] * 像素x + x_kb[1]``，y 同理）。而检测器只看画面里的一小块
兴趣区，所以每一条发出去的 ``LocationCentre`` 都必须把"ROI 内的框中心"换算回
"整幅图像坐标"。

这层换算是纯数学、与 ROS 无关，但错了不会报错、只会让机械臂少走或多走几厘米。
所以把它从节点里拆出来：只依赖标准库，可以在任何地方单独跑
（见 ``test/test_rknn_yolo_postprocess.py``），不需要 numpy / cv2 / NPU。

与原程序的对应关系
------------------
原 ``scripts/detect.py`` 写的是::

    image = image[160:480, 160:480]                     # ROI 320×320，原点 (160, 160)
    loc_x = int((x_min + x_max) * 320 / 2 + 0.25 * 640) # 0.25*640 = 160  == ROI 原点 x
    loc_y = int((y_min + y_max) * 320 / 2 + 0.33 * 480) # 0.33*480 = 158.4 != ROI 原点 y(160)

x 方向的偏移正好是 ROI 原点，y 方向则比真实裁剪原点少 1.6 px。本模块按**真实裁剪
原点**换算；要逐像素复现原程序，把偏移显式设成 ``[160, 158.4]`` 即可。
"""

__all__ = ["parse_roi", "parse_offset", "centre_to_source", "crop_slice"]


def parse_roi(value):
    """``~roi`` → ``(x, y, w, h)``，单位像素，必须是 4 个整数。"""
    try:
        roi = [int(v) for v in value]
    except (TypeError, ValueError):
        raise ValueError("roi 必须是 4 个整数 (x, y, w, h)，收到 %r" % (value,))
    if len(roi) != 4:
        raise ValueError("roi 必须是 4 个整数 (x, y, w, h)，收到 %r" % (value,))
    if roi[2] <= 0 or roi[3] <= 0:
        raise ValueError("roi 的宽高必须为正，收到 %r" % (value,))
    return tuple(roi)


def parse_offset(value, roi):
    """``~roi_offset`` → ``(dx, dy)``；留空时用 ROI 左上角。

    允许小数：原程序 y 方向的 158.4 就是这么复现的。
    """
    if value in (None, "", [], ()):
        return (float(roi[0]), float(roi[1]))
    try:
        offset = [float(v) for v in value]
    except (TypeError, ValueError):
        raise ValueError("roi_offset 必须是 2 个数 (dx, dy)，收到 %r" % (value,))
    if len(offset) != 2:
        raise ValueError("roi_offset 必须是 2 个数 (dx, dy)，收到 %r" % (value,))
    return (offset[0], offset[1])


def centre_to_source(x1, y1, x2, y2, offset):
    """ROI 内的框（xyxy，像素）→ 整幅图像里的中心点，取整方式与原程序一致。

    原程序是 ``int(中心 + 偏移)`` —— 先加再截断，不是先各自取整。
    """
    cx = int(offset[0] + (x1 + x2) / 2.0)
    cy = int(offset[1] + (y1 + y2) / 2.0)
    return cx, cy


def crop_slice(shape, roi):
    """按 ``roi`` 算出裁剪用的切片；画面装不下就返回 ``None``（不猜、不补边）。

    :param shape: ``image.shape``（至少两维，前两维是 h, w）
    """
    height, width = shape[0], shape[1]
    x, y, w, h = roi
    if width < x + w or height < y + h:
        return None
    return (slice(y, y + h), slice(x, x + w))
