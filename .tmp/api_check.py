#!/usr/bin/env python3
"""Verify every match_logic.<name>(...) call in the runners binds to the real API."""
import ast
import inspect
import os
import sys

sys.path.insert(0, r"D:\NXROBO\auto\codes")
import match_logic  # noqa: E402

FILES = ["runtask.py", "grasp_generic.py", "detect_ros.py", "robot_view.py", "arm_ik.py"]
ROOT = r"D:\NXROBO\auto\codes"

problems = 0
checked = 0
for fname in FILES:
    path = os.path.join(ROOT, fname)
    tree = ast.parse(open(path, encoding="utf-8").read(), path)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        fn = node.func
        if not (isinstance(fn, ast.Attribute) and isinstance(fn.value, ast.Name)
                and fn.value.id in ("match_logic", "ml")):
            continue
        name = fn.attr
        checked += 1
        target = getattr(match_logic, name, None)
        if target is None:
            print("MISSING  %s:%d  match_logic.%s" % (fname, node.lineno, name))
            problems += 1
            continue
        if not callable(target):
            continue
        args = []
        kwargs = {}
        bad = False
        for a in node.args:
            if isinstance(a, ast.Starred):
                bad = True
                break
            if isinstance(a, ast.Constant):
                args.append(a.value)
            else:
                args.append(object())
        for kw in node.keywords:
            if kw.arg is None:
                bad = True
                break
            if isinstance(kw.value, ast.Constant):
                kwargs[kw.arg] = kw.value.value
            else:
                kwargs[kw.arg] = object()
        if bad:
            continue
        try:
            inspect.signature(target).bind(*args, **kwargs)
        except TypeError as exc:
            print("SIGNATURE %s:%d  match_logic.%s -> %s"
                  % (fname, node.lineno, name, exc))
            problems += 1

print("checked %d match_logic call(s), %d problem(s)" % (checked, problems))
sys.exit(1 if problems else 0)
