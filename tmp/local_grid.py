import sys

from PIL import Image, ImageDraw

SRC = r"D:\NXROBO\tmp\snap_after_probe.jpg"
OUT = r"D:\NXROBO\tmp\snap_gridded.png"

img = Image.open(SRC).convert("RGB")
img = img.resize((1280, 960), Image.LANCZOS)
draw = ImageDraw.Draw(img)
for u in range(0, 640, 40):
    x = u * 2
    draw.line([(x, 0), (x, 960)], fill=(0, 200, 0), width=1)
    draw.text((x + 2, 2), str(u), fill=(0, 120, 0))
for v in range(0, 480, 40):
    y = v * 2
    draw.line([(0, y), (1280, y)], fill=(0, 200, 0), width=1)
    draw.text((2, y + 1), str(v), fill=(0, 120, 0))
img.save(OUT)
print("saved", OUT)
