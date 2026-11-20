#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""量一量 grasp.py 的水源（蓝）/红块检测到底有没有命中，以及计数器会不会被互相清零。

完全照抄 grasp.py 的两条 HSV 流水线与"最近轮廓"选法，只是把内部状态打印出来。
**只订阅话题，不改板端任何东西**，可以边跑比赛程序边量。

用法（板上）::

    run python3 test/probe_grasp_colors.py [帧数]

判据：
* 蓝轮廓 0/N 且全图 S 均值很低  -> 画面里没有蓝色目标，把水源放进相机视野
* 蓝轮廓有但中心每帧跳 >4px      -> 目标在动 / 光斑不稳
* 有蓝有轮廓但 S <90             -> 目标不是饱和蓝（白 / 浅蓝 / 过曝），要放宽阈值
"""

import math
import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

TOPIC = "/camera/color/image_raw"
WINDOW = 60


def pipeline(cv_image, lower, upper, thresh_val):
    """照抄 grasp.py：HSV -> inRange -> 取 H 通道 -> blur -> threshold
    -> CLOSE(25x25) -> erode x4 -> dilate x4 -> findContours。"""
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
    chan = cv2.bitwise_and(hsv, hsv, mask=mask)[:, :, 0]
    blurred = cv2.blur(chan, (9, 9))
    _, thresh = cv2.threshold(blurred, thresh_val, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    closed = cv2.erode(closed, None, iterations=4)
    closed = cv2.dilate(closed, None, iterations=4)
    contours = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
    return contours, mask


def nearest_centre(contours):
    """照抄 grasp.py：取离 (320, 480) 最近的轮廓的中心。返回 (x, y) 或 None。"""
    if not contours:
        return None
    dis_min = 640
    best = None
    for c in contours:
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.intp(box)
        x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
        y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4
        d = math.sqrt((x_mid - 320) ** 2 + (480 - y_mid) ** 2)
        if d < dis_min:
            dis_min = d
            best = (x_mid, y_mid)
    return best


class Probe(object):
    def __init__(self):
        self.frames = 0
        self.bridge = CvBridge()
        # 照抄 grasp.py 的两个计数器语义：**共用一个 water_found_count**
        self.shared_count = 0
        self.blue_only_count = 0
        self.red_only_count = 0
        self.found_water_shared = False
        self.found_water_blue_only = False
        self.found_red_only = False
        self.blue_prev = (0.0, 0.0)
        self.red_prev = (0.0, 0.0)
        self.saved = False
        self.blue_hits = 0
        self.red_hits = 0
        self.blue_stable = 0
        self.red_stable = 0
        self.blue_trace = []
        self.red_trace = []
        rospy.Subscriber(TOPIC, Image, self.cb, queue_size=1, buff_size=2 ** 24)

    def cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            print("图像转换失败: %s" % exc)
            return
        self.frames += 1

        blue, blue_mask = pipeline(img, [95, 90, 80], [130, 255, 255], 90)
        red, red_mask = pipeline(img, [146, 80, 46], [180, 255, 255], 20)
        bc = nearest_centre(blue)
        rc = nearest_centre(red)

        # ---- image_cb（蓝）----
        if blue:
            self.blue_hits += 1
            if bc:
                self.blue_trace.append((round(bc[0], 1), round(bc[1], 1)))
                if abs(bc[0] - self.blue_prev[0]) <= 4 and abs(bc[1] - self.blue_prev[1]) <= 4:
                    self.blue_stable += 1
                    self.blue_only_count += 1
                    self.shared_count += 1
                else:
                    self.blue_only_count = 0
                    self.shared_count = 0
                self.blue_prev = bc
        else:
            self.blue_only_count = 0
            self.shared_count = 0

        # ---- image_cb_red（红）—— 它也在动同一个 shared_count ----
        if red:
            self.red_hits += 1
            if rc:
                self.red_trace.append((round(rc[0], 1), round(rc[1], 1)))
                if abs(rc[0] - self.red_prev[0]) <= 4 and abs(rc[1] - self.red_prev[1]) <= 4:
                    self.red_stable += 1
                    self.red_only_count += 1
                    self.shared_count += 1
                else:
                    self.red_only_count = 0
                    self.shared_count = 0
                self.red_prev = rc
        else:
            self.shared_count = 0          # ★ 红没找到 -> 把蓝攒的计数也清了
            self.red_only_count = 0

        if self.blue_only_count >= 30:
            self.found_water_blue_only = True
        if self.red_only_count >= 10:
            self.found_red_only = True
        if self.shared_count >= 30:
            self.found_water_shared = True

        if not self.saved:
            self.saved = True
            cv2.imwrite("/data/local/tmp/probe_frame.png", img)
            cv2.imwrite("/data/local/tmp/probe_blue_mask.png", blue_mask)
            cv2.imwrite("/data/local/tmp/probe_red_mask.png", red_mask)

        if self.frames % 15 == 0:
            print("  帧 %-3d 蓝轮廓=%-3d 蓝中心=%-18s 蓝计数=%-3d 红轮廓=%-3d 红中心=%-18s 共享计数=%d"
                  % (self.frames, len(blue), str(bc), self.blue_only_count,
                     len(red), str(rc), self.shared_count))


def main(argv):
    want = int(argv[0]) if argv else WINDOW
    rospy.init_node("probe_water", anonymous=True)
    probe = Probe()
    print("订阅 %s，收 %d 帧……（相机没在发图会在这里卡住）" % (TOPIC, want))
    rate = rospy.Rate(30)
    while not rospy.is_shutdown() and probe.frames < want:
        rate.sleep()

    print("")
    print("===== 结果 =====")
    print("收到帧数            : %d" % probe.frames)
    print("蓝色有轮廓的帧      : %d / %d  (%.0f%%)"
          % (probe.blue_hits, probe.frames,
             100.0 * probe.blue_hits / max(1, probe.frames)))
    print("蓝色中心稳定(<=4px) : %d" % probe.blue_stable)
    print("红色有轮廓的帧      : %d / %d  (%.0f%%)"
          % (probe.red_hits, probe.frames,
             100.0 * probe.red_hits / max(1, probe.frames)))
    print("红色中心稳定(<=4px) : %d" % probe.red_stable)
    print("")
    print("蓝中心轨迹(前 12)   : %s" % probe.blue_trace[:12])
    print("红中心轨迹(前 12)   : %s" % probe.red_trace[:12])
    print("")
    print("★ 按 grasp.py 现在的写法（两个回调共用 water_found_count）:")
    print("    found_water = %s   （共享计数最终 %d）"
          % (probe.found_water_shared, probe.shared_count))
    print("★ 如果蓝红各用各的计数器:")
    print("    found_water(蓝) = %s   found_red = %s"
          % (probe.found_water_blue_only, probe.found_red_only))
    print("")
    print("已存 /data/local/tmp/probe_{frame,blue_mask,red_mask}.png")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
