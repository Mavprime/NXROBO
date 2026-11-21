import os

from PIL import Image, ImageDraw, ImageFont

BASE = r"D:\NXROBO\auto\docs\cake_run_20261121\run2"
PHOTOS = ["01_above.jpg", "02_descent.jpg", "03_lifted.jpg",
          "04_verify_pick.jpg", "06_place.jpg", "07_verify_place.jpg"]
CROP = (250, 0, 500, 330)
SCALE = 2

font = None
for candidate in (r"C:\Windows\Fonts\segoeui.ttf", r"C:\Windows\Fonts\arial.ttf"):
    if os.path.exists(candidate):
        font = ImageFont.truetype(candidate, 22)
        break
if font is None:
    font = ImageFont.load_default()

tiles = []
for name in PHOTOS:
    img = Image.open(os.path.join(BASE, name)).convert("RGB")
    c = img.crop(CROP)
    c = c.resize((c.width * SCALE, c.height * SCALE), Image.LANCZOS)
    tile = Image.new("RGB", (c.width, c.height + 32), (20, 20, 24))
    tile.paste(c, (0, 32))
    ImageDraw.Draw(tile).text((8, 5), name, font=font, fill=(255, 220, 90))
    tiles.append(tile)

gap = 8
W = sum(t.width for t in tiles) + gap * (len(tiles) - 1)
H = max(t.height for t in tiles)
canvas = Image.new("RGB", (W, H), (10, 10, 14))
x = 0
for tile in tiles:
    canvas.paste(tile, (x, 0))
    x += tile.width + gap
out = r"D:\NXROBO\tmp\run2_stage_crops.png"
canvas.save(out)
print("saved", out, canvas.size)
