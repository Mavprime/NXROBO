import sys

import numpy as np
from PIL import Image, ImageDraw

IMAGES = {
    "before_1929": r"D:\NXROBO\tmp\before.jpg",
    "now_1959": r"D:\NXROBO\tmp\snap_after_probe.jpg",
}
CROP = (270, 0, 450, 380)

tiles = []
for name, path in IMAGES.items():
    img = Image.open(path).convert("RGB")
    crop = img.crop(CROP).resize(((CROP[2] - CROP[0]) * 2, (CROP[3] - CROP[1]) * 2),
                                 Image.LANCZOS)
    draw = ImageDraw.Draw(crop)
    draw.text((6, 6), name, fill=(255, 0, 0))
    tiles.append(crop)

w = sum(t.width for t in tiles)
h = max(t.height for t in tiles)
canvas = Image.new("RGB", (w + 10, h), (20, 20, 20))
x = 0
for t in tiles:
    canvas.paste(t, (x, 0))
    x += t.width + 10
canvas.save(r"D:\NXROBO\tmp\tower_compare.png")
print("saved tower_compare.png", canvas.size)
