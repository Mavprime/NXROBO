import os
import shutil

from PIL import Image, ImageDraw, ImageFont

RUN = r"D:\NXROBO\tmp"
OUT_DIR = r"D:\NXROBO\auto\docs\cake_run_20261121"
os.makedirs(OUT_DIR, exist_ok=True)

STEPS = [
    ("01_above.jpg", "1. Скан: торт найден на 2-м ярусе"),
    ("04_verify_pick.jpg", "2. Торт снят (виден слева на присоске)"),
    ("07_verify_place.jpg", "3. Проверка: торт возвращён на место"),
]
ALL_PHOTOS = ["01_above.jpg", "02_descent.jpg", "03_lifted.jpg",
              "04_verify_pick.jpg", "05_above_place.jpg", "06_place.jpg",
              "07_verify_place.jpg", "final_state.jpg"]

font = None
for candidate in (r"C:\Windows\Fonts\segoeui.ttf", r"C:\Windows\Fonts\arial.ttf"):
    if os.path.exists(candidate):
        font = ImageFont.truetype(candidate, 17)
        break
if font is None:
    font = ImageFont.load_default()

TILE_W, TILE_H = 640, 480
LABEL_H = 34
tiles = []
for name, caption in STEPS:
    path = os.path.join(RUN, name)
    img = Image.open(path).convert("RGB")
    scale = min(TILE_W / img.width, TILE_H / img.height)
    img = img.resize((int(img.width * scale), int(img.height * scale)), Image.LANCZOS)
    tile = Image.new("RGB", (TILE_W, TILE_H + LABEL_H), (25, 25, 30))
    tile.paste(img, ((TILE_W - img.width) // 2, LABEL_H))
    draw = ImageDraw.Draw(tile)
    draw.text((8, 8), caption, font=font, fill=(255, 220, 90))
    tiles.append(tile)

for name in ALL_PHOTOS:
    shutil.copyfile(os.path.join(RUN, name), os.path.join(OUT_DIR, name))

gap = 8
w = sum(t.width for t in tiles) + gap * (len(tiles) - 1)
h = max(t.height for t in tiles)
canvas = Image.new("RGB", (w, h), (12, 12, 16))
x = 0
for tile in tiles:
    canvas.paste(tile, (x, 0))
    x += tile.width + gap

out = os.path.join(OUT_DIR, "cake_pick_place_20261121.png")
canvas.save(out)
print("saved", out, canvas.size)
