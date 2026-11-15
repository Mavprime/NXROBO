#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""launch 静态引用检查 —— 不需要 ROS、不需要板子。

板端最典型的失败是 `cannot launch node of type [pkg/node]`：launch 里引用了
一个**在 build.sh 的 skiplist 里、因此根本没编出来**的包。这类错误在交叉编译
阶段完全看不出来，只有上了板才炸。本脚本把三件事提前查掉：

1. 每个 ``$(arg X)`` / ``arg('X')`` 都有出处（本文件声明，或 include 传参）；
2. 每个 ``$(find PKG)`` / ``<node pkg=...>`` 指向的包真的存在
   （工作区、或 SDK 预装的系统包，例如 rviz / map_server）；
3. 引用到的包没有落在 skiplist 里。``swiftpro`` 是已知例外：它在 skiplist 里
   （交叉编译不过），但板端是 prebuilt 装好的。

用法::

    python3 test/check_launch_refs.py                      # 查本包 launch/*.launch
    python3 test/check_launch_refs.py path/to/a.launch ... # 查指定文件

环境变量 ``SPARK_SDK_DIR`` 可覆盖 SDK 解压目录（默认 /home/m-robots/sdk-persist）。
"""

import glob
import os
import re
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(HERE)
SRC_DIR = os.path.dirname(os.path.dirname(PACKAGE_DIR))          # <repo>/spark_noetic/src
REPO_DIR = os.path.dirname(os.path.dirname(SRC_DIR))             # <repo>
SDK_ROOT = os.environ.get("SPARK_SDK_DIR", "/home/m-robots/sdk-persist")
SDK_SHARE = os.path.join(SDK_ROOT, "usr", "share")
BUILD_SH = os.path.join(REPO_DIR, "build.sh")

ARG_REF = re.compile(r"\$\(arg ([A-Za-z0-9_]+)\)")
ARG_EVAL = re.compile(r"arg\('([A-Za-z0-9_]+)'\)")
FIND_REF = re.compile(r"\$\(find ([A-Za-z0-9_]+)\)")

# 板上 prebuilt 装好的，即便在 skiplist 里也能 $(find) 到
PREBUILT_ON_BOARD = {"swiftpro", "swift_moveit_config", "pro_moveit_config"}

failures = []
warnings = []


def workspace_packages():
    found = set()
    for root, dirs, files in os.walk(SRC_DIR):
        if "package.xml" in files:
            found.add(os.path.basename(root))
        dirs[:] = [d for d in dirs if d not in (".git", "build", "devel")]
    return found


def sdk_packages():
    """SDK 里预装的系统 ROS 包（rviz / map_server / tf ... 不在工作区里）。"""
    found = set()
    if not os.path.isdir(SDK_SHARE):
        return found
    for name in os.listdir(SDK_SHARE):
        if os.path.exists(os.path.join(SDK_SHARE, name, "package.xml")):
            found.add(name)
    return found


def skiplist():
    try:
        with open(BUILD_SH, "r") as handle:
            text = handle.read()
    except IOError:
        return set()
    block = re.search(r"#skiplist(.*?)#cmake-args", text, re.S)
    if not block:
        return set()
    return set(re.findall(r'config_args\+=\("([^"]+)"\)', block.group(1)))


def node_exists(pkg, typ):
    for pattern in ("*/" + pkg + "/**/" + typ, "*/**/" + pkg + "/**/" + typ):
        if glob.glob(os.path.join(SRC_DIR, pattern), recursive=True):
            return True
    # SDK 预装包的可执行文件在 <sdk>/usr/lib/<pkg>/ 下（例如 rviz）
    if glob.glob(os.path.join(SDK_ROOT, "usr", "lib", pkg, typ)):
        return True
    return bool(glob.glob(os.path.join(SDK_SHARE, pkg, "**", typ), recursive=True))


def check_file(path, packages, skipped):
    root = ET.parse(path).getroot()
    name = os.path.basename(path)

    # 只有 <launch> 的直接子 <arg> 才是"声明"；<include> 里的 <arg> 是传参
    declared = {a.get("name") for a in root.findall("arg") if a.get("name")}
    passed = {a.get("name") for inc in root.iter("include")
              for a in inc.findall("arg") if a.get("name")}
    with open(path, "r") as handle:
        text = handle.read()

    used = set(ARG_REF.findall(text)) | set(ARG_EVAL.findall(text))
    undeclared = sorted(used - declared - passed)
    if undeclared:
        failures.append("%s: 用了没声明的 arg: %s" % (name, undeclared))
    else:
        print("  OK   %s: %d 个 arg 声明、%d 个 include 传参，引用全部有出处"
              % (name, len(declared), len(passed)))

    unused = sorted(declared - used)
    if unused:
        warnings.append("%s: 声明了但没用到的 arg（确认是刻意保留）: %s" % (name, unused))

    for pkg in sorted(set(FIND_REF.findall(text))):
        if pkg not in packages:
            failures.append("%s: $(find %s) 找不到这个包" % (name, pkg))
        elif pkg in skipped and pkg not in PREBUILT_ON_BOARD:
            failures.append("%s: $(find %s) 在 skiplist 里，板端不会有它" % (name, pkg))

    for node in root.iter("node"):
        pkg, typ = node.get("pkg"), node.get("type")
        if not pkg:
            continue
        if pkg not in packages:
            failures.append("%s: <node pkg=%r> 找不到这个包" % (name, pkg))
            continue
        if pkg in skipped and pkg not in PREBUILT_ON_BOARD:
            failures.append("%s: <node pkg=%r> 在 skiplist 里，板端不会有它" % (name, pkg))
        if typ and not node_exists(pkg, typ):
            warnings.append("%s: <node pkg=%r type=%r> 没找到同名可执行文件"
                            % (name, pkg, typ))


def main(argv):
    targets = argv or sorted(glob.glob(os.path.join(PACKAGE_DIR, "launch", "*.launch")))
    if not targets:
        print("没有要检查的 launch 文件")
        return 1

    packages = workspace_packages() | sdk_packages()
    skipped = skiplist()
    print("工作区包 %d 个；SDK 预装包 %d 个；skiplist %d 个；待查文件 %d 个"
          % (len(workspace_packages()), len(sdk_packages()), len(skipped), len(targets)))
    print("")
    for path in targets:
        check_file(path, packages, skipped)

    print("")
    for w in warnings:
        print("WARN %s" % w)
    if failures:
        for f in failures:
            print("FAIL %s" % f)
        return 1
    print("launch 引用检查通过")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
