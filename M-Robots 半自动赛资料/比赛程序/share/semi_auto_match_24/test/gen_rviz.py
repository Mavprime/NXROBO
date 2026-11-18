#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 semi_auto_match_24 自己的 rviz 布局（``rviz/semi_auto.rviz``）。

    python3 test/gen_rviz.py [fixed_frame]      # 默认 odom

为什么不能直接抄现成的
----------------------
* ``spark_gmapping_d435.rviz`` 是 **SLAM 布局** —— Fixed Frame = ``map``，
  还带 /map 上的 Map 显示。而 ``semi_auto.launch`` **不启动 SLAM / map_server**，
  ``map`` 帧根本不存在，rviz 会 ``Fixed Frame [map] does not exist`` 然后
  3D 视图什么都不画（上板实测就是"画面一片空白/发白"）。
* ``auto_match.rviz``（2026 版）同样是 ``map`` 中心 —— 但**它的 launch 会起
  SLAM 或 map_server**，所以那份没问题，抄过来就有问题。

本脚本的做法
------------
拿 ``spark_description/rviz/urdf.rviz`` 当底 —— 那是**官方的"看机器人"布局**，
Fixed Frame 就是本 launch 真实存在的 ``odom``，而且它恰好配对
``joint_state_publisher`` + ``robot_state_publisher`` 这一对节点；
再把 PointCloud2 面板换成雷达 + 两路图像：

    Grid / RobotModel / TF(关) / LaserScan(/scan)
    / Image(/camera/color/image_raw) / Image(/debug_image)      Fixed Frame: odom

LaserScan 与 Image 的字段块直接从 ``auto_match.rviz`` 里**整块搬**（同板同
rviz 版本，字段 schema 一定对），不手写。

本 launch 实测存在的帧：``odom``（spark_base_server）、``base_link`` /
``laser`` / ``camera_*``（robot_state_publisher 与 realsense）。**没有 ``map``。**
"""

import io
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(HERE)
SRC_DIR = os.path.dirname(os.path.dirname(PACKAGE_DIR))
URDF_RVIZ = os.path.join(SRC_DIR, "spark", "spark_description", "rviz", "urdf.rviz")
AUTO_RVIZ = os.path.join(SRC_DIR, "spark_app", "auto_match_25", "auto_match",
                         "rviz", "auto_match.rviz")
OUT = os.path.join(PACKAGE_DIR, "rviz", "semi_auto.rviz")

FRAME = sys.argv[1] if len(sys.argv) > 1 else "odom"


def read_lines(path):
    with io.open(path, "r", encoding="utf-8") as handle:
        return handle.read().split("\n")


def _is_block_end(line):
    """块内字段缩进 6 空格（嵌套键 8 空格）；遇到空行或缩进不足就是块结束。

    注意不能写成 ``line.startswith("  ")`` —— 6 空格的字段也"以两个空格开头"，
    那样每块都只剩第一行（踩过）。
    """
    if not line.strip():
        return False
    return not line.startswith("      ")


def extract_block(lines, class_marker):
    """按缩进切出一个顶层显示项。

    两种写法都要认：
    * ``    - Alpha: 1`` … ``      Class: rviz/X``   （块从上一行开始）
    * ``    - Class: rviz/X``                        （块就从这一行开始）
    """
    start = None
    inline = False
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped == "Class: " + class_marker:
            start, inline = i, False
            break
        if stripped == "- Class: " + class_marker:
            start, inline = i, True
            break
    if start is None:
        raise SystemExit("找不到 %s 的显示项" % class_marker)

    if inline:
        begin = start
    else:
        begin = start
        while begin > 0 and not lines[begin].startswith("    - "):
            begin -= 1
        if not lines[begin].startswith("    - "):
            raise SystemExit("%s 的显示项找不到起始行" % class_marker)

    end = start + 1
    while end < len(lines) and not _is_block_end(lines[end]):
        end += 1
    return lines[begin:end]


def set_key(block, key, value):
    """把块里某个键的值改掉（保留缩进）。"""
    out = []
    for line in block:
        if line.strip().startswith(key + ":"):
            indent = line[:len(line) - len(line.lstrip())]
            out.append("%s%s: %s" % (indent, key, value))
        else:
            out.append(line)
    return out


# 视角与窗口：官方 urdf.rviz 存的是 Distance 6.64m / 窗口 1859x1056，
# 而板上的 Xvfb 是 1100x650 —— 直接照搬会把机器人挤到画面角落，
# 而且相机盯的是 odom 原点（机器人开走后就不在画面里了）。
# 这里改成：相机跟着 base_link 走，距离 2.5m（对 34cm 的底盘刚好），
# 窗口尺寸对齐板上那块虚拟屏。
VIEW_FIXES = [
    ("      Target Frame: <Fixed Frame>", "      Target Frame: base_link"),
    ("      Distance: 6.64356", "      Distance: 2.50000"),
    ("      Pitch: 0.399797", "      Pitch: 0.350000"),
    ("  Width: 1859", "  Width: 1100"),
    ("  Height: 1056", "  Height: 650"),
    ("  X: 61", "  X: 0"),
    ("  Y: 24", "  Y: 0"),
]


def apply_view_fixes(lines):
    text = "\n".join(lines)
    applied = []
    for old, new in VIEW_FIXES:
        if old in text:
            text = text.replace(old, new, 1)
            applied.append(new.strip())
    return text.split("\n"), applied


def find_block_range(lines, class_marker):
    begin = None
    for i, line in enumerate(lines):
        if line.strip() == "Class: " + class_marker:
            begin = i
            break
    if begin is None:
        raise SystemExit("找不到 %s 的显示项" % class_marker)
    while begin > 0 and not lines[begin].startswith("    - "):
        begin -= 1
    end = begin + 1
    while end < len(lines) and not _is_block_end(lines[end]):
        end += 1
    return begin, end


def main():
    urdf = read_lines(URDF_RVIZ)
    auto = read_lines(AUTO_RVIZ)

    laser = set_key(set_key(extract_block(auto, "rviz/LaserScan"),
                            "Name", "LaserScan"), "Topic", "/scan")
    cam = set_key(set_key(extract_block(auto, "rviz/Image"),
                          "Name", "Camera"), "Image Topic", "/camera/color/image_raw")
    det = set_key(set_key(extract_block(auto, "rviz/Image"),
                          "Name", "Detect"), "Image Topic", "/debug_image")

    begin, end = find_block_range(urdf, "rviz/PointCloud2")
    # 不用深度点云：VNC 上渲染慢，本赛项也用不到；要看随时能在面板里加回来
    new = urdf[:begin] + laser + cam + det + urdf[end:]

    for i, line in enumerate(new):
        if line.strip().startswith("Fixed Frame:"):
            indent = line[:len(line) - len(line.lstrip())]
            new[i] = "%sFixed Frame: %s" % (indent, FRAME)

    new, applied = apply_view_fixes(new)
    print("视角/窗口修正: %s" % (", ".join(applied) if applied else "（无）"))

    with io.open(OUT, "w", encoding="utf-8") as handle:
        handle.write("\n".join(new))

    with io.open(OUT, "r", encoding="utf-8") as handle:
        text = handle.read()
    checks = {
        "Fixed Frame 是 %s" % FRAME: ("Fixed Frame: %s" % FRAME) in text,
        "没有把 map 当 Fixed Frame": "Fixed Frame: map" not in text,
        "有 RobotModel": "Class: rviz/RobotModel" in text,
        "有 LaserScan(/scan)": "Class: rviz/LaserScan" in text and "Topic: /scan" in text,
        "有 Image(/camera/color/image_raw)": "Image Topic: /camera/color/image_raw" in text,
        "有 Image(/debug_image)": "Image Topic: /debug_image" in text,
        "没有 PointCloud2": "Class: rviz/PointCloud2" not in text,
        "相机跟着 base_link": "Target Frame: base_link" in text,
        "窗口对齐 1100x650": "  Width: 1100" in text and "  Height: 650" in text,
    }
    for name, ok in checks.items():
        print("  [%s] %s" % ("OK" if ok else "FAIL", name))
    if not all(checks.values()):
        return 1
    print("已生成 %s（%d 行）" % (OUT, len(new)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
