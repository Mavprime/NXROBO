import os

from PIL import Image, ImageDraw, ImageFont

BASE = r"D:\NXROBO\auto\docs\cake_run_20261121"
RUN1 = BASE
RUN2 = os.path.join(BASE, "run2")

ROWS = [
    ("04_verify_pick.jpg", "Торт снят с башни (кадр проверки)"),
    ("07_verify_place.jpg", "Торт возвращён на место (кадр проверки)"),
]
COLS = [
    ("ПРОГОН 1 — 20:01", RUN1),
    ("ПРОГОН 2 — 20:14", RUN2),
]

font = None
for candidate in (r"C:\Windows\Fonts\segoeui.ttf", r"C:\Windows\Fonts\arial.ttf"):
    if os.path.exists(candidate):
        font = ImageFont.truetype(candidate, 20)
        font_small = ImageFont.truetype(candidate, 16)
        break
else:
    font = ImageFont.load_default()
    font_small = font

TILE_W, TILE_H = 480, 360
HEAD_H = 36
LABEL_H = 30
gap = 10
canvas_w = TILE_W * len(COLS) + gap * (len(COLS) - 1)
canvas_h = HEAD_H + len(ROWS) * (LABEL_H + TILE_H) + gap * (len(ROWS) - 1)
canvas = Image.new("RGB", (canvas_w, canvas_h), (12, 12, 16))
draw = ImageDraw.Draw(canvas)

for c, (title, folder) in enumerate(COLS):
    x = c * (TILE_W + gap)
    draw.text((x + 8, 8), title, font=font, fill=(120, 220, 255))
    for r, (photo, label) in enumerate(ROWS):
        y = HEAD_H + r * (LABEL_H + TILE_H + gap)
        img = Image.open(os.path.join(folder, photo)).convert("RGB")
        scale = min(TILE_W / img.width, TILE_H / img.height)
        img = img.resize((int(img.width * scale), int(img.height * scale)),
                         Image.LANCZOS)
        draw.text((x + 8, y + 5), label, font=font_small, fill=(255, 220, 90))
        canvas.paste(img, (x, y + LABEL_H))

out = os.path.join(BASE, "cake_repeatability_20261121.png")
canvas.save(out)
print("saved", out, canvas.size)
