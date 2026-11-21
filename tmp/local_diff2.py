import sys

import numpy as np
from PIL import Image

PAIRS = [
    ("19:29 before", r"D:\NXROBO\tmp\before.jpg", "19:43 snap_now2", r"D:\NXROBO\tmp\snap_now2.jpg"),
    ("19:43 snap_now2", r"D:\NXROBO\tmp\snap_now2.jpg", "19:57 after_probe", r"D:\NXROBO\tmp\snap_after_probe.jpg"),
    ("19:57 after_probe", r"D:\NXROBO\tmp\snap_after_probe.jpg", "20:02 verify_place", r"D:\NXROBO\tmp\07_verify_place.jpg"),
]


def gray(path):
    return np.asarray(Image.open(path).convert("L"), dtype=np.float64)


for name_a, path_a, name_b, path_b in PAIRS:
    a, b = gray(path_a), gray(path_b)
    diff = np.abs(a - b)
    # Tower region only (the only place the scene is allowed to change).
    region = diff[0:260, 270:460]
    print("%s -> %s : full mean %.2f | tower mean %.2f max %.0f"
          % (name_a, name_b, diff.mean(), region.mean(), region.max()))
