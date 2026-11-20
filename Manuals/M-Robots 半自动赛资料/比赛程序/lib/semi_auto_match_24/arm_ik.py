#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""机械臂可达性判定 + 像素↔机械臂坐标换算（纯数学，无 ROS 依赖）。

被两个地方用：
* ``nodes/grasp.py`` —— 抓取前先判一次，够不到就别发（见那边的说明）
* ``test/reachable_area.py`` —— 算出"画面里哪些区域够得到"，指导摆放

数学照抄 ``swiftpro_rviz_node.cpp`` 的 ``swiftpro_ik()``：够不到时内部两个
``acos`` 的参数会超出 [-1, 1]（三角形两边夹不出这个点），那边就报
``Inverse kinematic is wrong``。实测对齐：报错那次的 (67.18, 22.28, 20) 判不可达，
日志里真正抓成功的 (126.42, -8.58) 与 (100.78, -2.77) 判可达。

**别在这里加 ROS 依赖** —— 这个模块要能在没有 roscore 的机器上直接跑。
"""

import math

# swiftpro 的几何常数（swiftpro_rviz_node.cpp 顶部）
MATH_TRANS = 57.2958
MATH_L1 = 106.6
MATH_L2 = 13.2
MATH_LOWER_ARM = 142.07
MATH_UPPER_ARM = 158.81
MATH_UPPER_LOWER = MATH_UPPER_ARM / MATH_LOWER_ARM

# 手眼标定系数（板上 /data/local/release/usr/thefile.txt）
# pos.x 由**像素行**算，pos.y 由**像素列**算 —— 原程序就是这样反的，别"顺手改正"。
DEFAULT_X_KB = (-0.8840816286146583, 448.66407041831445)
DEFAULT_Y_KB = (-0.8946292902364896, 315.7187464490772)


def pixel_to_arm(pixel_row, pixel_col, x_kb=DEFAULT_X_KB, y_kb=DEFAULT_Y_KB):
    """像素 -> 机械臂坐标（毫米），与 grasp() 里的算法逐字一致。"""
    return x_kb[0] * pixel_row + x_kb[1], y_kb[0] * pixel_col + y_kb[1]


def ik_reachable(x, y, z):
    """(x, y, z)（毫米）这个位姿机械臂够得到吗？"""
    z = z + 74.55
    z_in = (z - MATH_L1) / MATH_LOWER_ARM
    if x < 0.1:
        x = 0.1
    if y == 0:
        rot = 90.0
    elif y < 0:
        rot = -math.atan(x / y) * MATH_TRANS
    else:
        rot = 180.0 - math.atan(x / y) * MATH_TRANS

    sin_rot = math.sin(rot / MATH_TRANS)
    if sin_rot == 0:
        return False
    x_in = (x / sin_rot - MATH_L2 - 56.55) / MATH_LOWER_ARM
    if x_in == 0:
        return False
    sqrt_zx = math.sqrt(z_in * z_in + x_in * x_in)
    if sqrt_zx == 0:
        return False
    upper = MATH_UPPER_LOWER
    right_all = (sqrt_zx * sqrt_zx + upper * upper - 1) / (2 * upper * sqrt_zx)
    left_all = (sqrt_zx * sqrt_zx + 1 - upper * upper) / (2 * sqrt_zx)
    return abs(right_all) <= 1 and abs(left_all) <= 1


def pixel_reachable(pixel_row, pixel_col, z_list, x_kb=DEFAULT_X_KB, y_kb=DEFAULT_Y_KB):
    """这个像素对应的位置，在 z_list 里**每个**高度都够得到才算可达。"""
    ax, ay = pixel_to_arm(pixel_row, pixel_col, x_kb, y_kb)
    return all(ik_reachable(ax, ay, z) for z in z_list)
