#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""算出"画面里哪些区域机械臂够得到"，用来指导物体摆放。

    python3 test/reachable_area.py                      # 打印可达区域图
    run python3 .../test/reachable_area.py --overlay    # 板上跑，抓一帧相机并把可达区域画上去

为什么需要：抓取位置是"像素 -> 标定 -> 机械臂坐标"，但**机械臂够不到所有位置**。
够不到时固件不接受那个位姿，表现为**机械臂不动**，而 grasp() 后面照样开泵、
照样抬到 (230,0,160) —— 看起来就像"没抓就跑了"，同时 swiftpro_rviz_node 会刷
"Inverse kinematic is wrong"。

实测（640x480，标定用板上 thefile.txt，z=20 与 -45 都要可达）：

* 画面**最上方约 1/4**（行 < ~110）整行都够不到 —— pos.x 超出臂展
* 行 112~300 只有**中间一条横带**可达，越往下越宽
* **行 320~400 整行都可达** ← 物体放这里最稳
* 行 > 410 时**底部中央**（列约 280~440）出现缺口 —— 离底座太近，机械臂折不过来
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(HERE)
for _cand in (os.path.join(PACKAGE_DIR, "nodes"),
              "/data/local/release/usr/lib/semi_auto_match_24"):
    if os.path.isdir(_cand):
        sys.path.insert(0, _cand)

from arm_ik import pixel_to_arm, pixel_reachable   # noqa: E402

# 与 config/grasp_heights.yaml 保持一致
Z_UP, Z_DOWN = 20.0, -45.0
Z_LIST = (Z_UP, Z_DOWN)
W, H = 640, 480


def print_map():
    print("=" * 74)
    print("画面里哪些区域够得到（要求 z=%.0f 和 z=%.0f 都可达）" % Z_LIST)
    print("=" * 74)
    print("# = 可达    . = 够不到     横轴=像素列 0..%d，纵轴=像素行 0..%d" % (W - 1, H - 1))
    print()
    for row in range(0, H, 16):
        cells = []
        for col in range(0, W, 40):
            ok = all(pixel_reachable(r, c, Z_LIST)
                     for r in range(row, min(row + 16, H), 4)
                     for c in range(col, min(col + 40, W), 8))
            cells.append("#" if ok else ".")
        print("行=%-4d %s" % (row, "   ".join(cells)))

    print()
    print("=" * 74)
    print("逐行看可达的列范围")
    print("=" * 74)
    for row in range(0, H, 30):
        ax = pixel_to_arm(row, 0)[0]
        good = [c for c in range(0, W, 5) if pixel_reachable(row, c, Z_LIST)]
        if good:
            print("  行=%3d  pos.x=%7.2f  可达列 %3d ~ %3d  (%d/%d)"
                  % (row, ax, min(good), max(good), len(good), W // 5))
        else:
            print("  行=%3d  pos.x=%7.2f  ★ 整行都够不到" % (row, ax))
    return 0


def live_overlay():
    """订阅相机抓一帧，把可达区域叠上去存 PNG。只订阅，不改任何东西。"""
    import cv2
    import numpy as np
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image

    topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
    out = rospy.get_param("~out", "/data/local/tmp/reachable_area.png")
    rospy.init_node("reachable_area", anonymous=True)
    bridge = CvBridge()
    box = {"img": None}

    def cb(msg):
        if box["img"] is None:
            try:
                box["img"] = bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as exc:                    # noqa: BLE001
                rospy.logwarn("图像转换失败: %s", exc)

    rospy.Subscriber(topic, Image, cb, queue_size=1, buff_size=2 ** 24)
    print("等一帧 %s ……" % topic)
    rate = rospy.Rate(20)
    for _ in range(200):
        if box["img"] is not None or rospy.is_shutdown():
            break
        rate.sleep()
    if box["img"] is None:
        print("没收到图像 —— 相机没在发？")
        return 1

    img = box["img"]
    h, w = img.shape[:2]
    overlay = img.copy()
    step = 4
    reach = np.zeros((h, w), dtype=bool)
    for r in range(0, h, step):
        for c in range(0, w, step):
            if pixel_reachable(r, c, Z_LIST):
                reach[r:r + step, c:c + step] = True
    # 可达区偏绿、不可达区偏红
    overlay[reach] = (overlay[reach] * 0.55 + np.array([0, 200, 0]) * 0.45).astype(np.uint8)
    overlay[~reach] = (overlay[~reach] * 0.55 + np.array([0, 0, 220]) * 0.45).astype(np.uint8)
    blend = cv2.addWeighted(img, 0.45, overlay, 0.55, 0)
    cv2.putText(blend, "green=reachable  red=unreachable", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.imwrite(out, blend)
    print("已存 %s（绿=够得到，红=够不到）" % out)
    return 0


if __name__ == "__main__":
    sys.exit(live_overlay() if "--overlay" in sys.argv else print_map())
