#!/usr/bin/env python3
"""Cheap pyflakes-style check: report global names used but never bound."""
import builtins
import os
import symtable
import sys

FILES = [
    "auto/codes/match_logic.py",
    "auto/codes/runtask.py",
    "auto/codes/grasp_generic.py",
    "auto/codes/detect_ros.py",
    "auto/codes/robot_view.py",
    "auto/codes/calib3d.py",
    "auto/codes/pad_detect.py",
    "auto/codes/arm_ik.py",
    "auto/codes/patch_swiftpro.py",
    "auto/codes/snap.py",
]

BUILTINS = set(dir(builtins)) | {"__file__", "__name__", "__doc__", "__package__", "__builtins__"}


def collect(table, defined, used, is_module=True):
    for sym in table.get_symbols():
        n = sym.get_name()
        if sym.is_assigned() or sym.is_imported() or sym.is_parameter():
            defined.add(n)
        elif sym.is_namespace():
            defined.add(n)
        if sym.is_global() and not is_module:
            used.add(n)
    for child in table.get_children():
        collect(child, defined, used, is_module=False)


def check(path):
    src = open(path, encoding="utf-8").read()
    top = symtable.symtable(src, path, "exec")
    defined = set()
    for sym in top.get_symbols():
        n = sym.get_name()
        if sym.is_assigned() or sym.is_imported() or sym.is_namespace() or sym.is_parameter():
            defined.add(n)
    used = set()
    for child in top.get_children():
        collect(child, set(), used, is_module=False)
    missing = sorted(n for n in used if n not in defined and n not in BUILTINS)
    print("%-32s globals=%3d  possibly-undefined=%s" % (os.path.basename(path), len(defined), missing or "none"))


for f in FILES:
    p = os.path.join(r"D:\NXROBO", f.replace("/", os.sep))
    if os.path.exists(p):
        check(p)
    else:
        print("MISSING", f)
