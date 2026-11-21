import numpy as np
from PIL import Image, ImageDraw, ImageFont

M = np.load(r"D:\NXROBO\auto\codes\transform_3d.dat")
FX, FY, CX, CY = 605.6860961914062, 605.8469848632812, 315.03875732421875, 249.8477020263672

print("M shape", M.shape)
print(np.array2string(M, precision=6))

# Camera centre in arm coordinates: null space of [x,y,z,1] @ M = 0
# P = M.T is 3x4; null space gives homogeneous camera centre.
P = M.T
u_, s_, vh = np.linalg.svd(P)
C = vh[-1]
C = C / C[3]
print("camera centre in arm frame (mm):", np.round(C[:3], 1))

# Optical axis: third row of P maps (x,y,z,1) to Zc; the ray through the image
# centre (u=cx, v=cy) has direction such that projected point = principal point.
# Solve for direction d with (d,0)@M => Xc/Zc = 0 and Yc/Zc = 0.
# [dx,dy,dz,0] @ M rows: find d in null of first two rows.
A = np.array([[M[0][j] for j in range(3)], [M[1][j] for j in range(3)]])
# Actually rows of M.T; we want d such that d0*M[0][j]... Let's solve:
# camera = (d,0) @ M ; require camera[0]=camera[1]=0.
B = np.array([[M[0][0], M[1][0], M[2][0]],
              [M[0][1], M[1][1], M[2][1]]], dtype=float)
u2, s2, vh2 = np.linalg.svd(B)
d = vh2[-1]
print("image-centre ray direction (arm frame):", np.round(d, 4))

def project(x, y, z):
    cam = np.array([x, y, z, 1.0]) @ M
    if cam[2] <= 1e-9:
        return None
    return (FX * cam[0] / cam[2] + CX, FY * cam[1] / cam[2] + CY)

# Points of interest
POINTS = {
    "pad_cmd_(318.8,-17.4,40)": (318.8, -17.4, 40.0),
    "pad_cmd_(318.8,-17.4,60)": (318.8, -17.4, 60.0),
    "cake_top_from_depth": (318.8, -17.4, -24.9),
    "cake_top_if_z32_inv_pixel": None,
}
for name, p in POINTS.items():
    if p is None:
        continue
    print("%-28s -> pixel %s" % (name, project(*p)))

# Inversion of the observed cake pixel at various planes
def invert(u, v, z):
    up = (u - CX) / FX
    vp = (v - CY) / FY
    r0, r1, r2 = M[:, 0], M[:, 1], M[:, 2]
    C0 = r0[2] * z + r0[3]
    C1 = r1[2] * z + r1[3]
    C2 = r2[2] * z + r2[3]
    a11 = r0[0] - up * r2[0]
    a12 = r0[1] - up * r2[1]
    a21 = r1[0] - vp * r2[0]
    a22 = r1[1] - vp * r2[1]
    b1 = up * C2 - C0
    b2 = vp * C2 - C1
    det = a11 * a22 - a12 * a21
    return ((b1 * a22 - a12 * b2) / det, (a11 * b2 - b1 * a21) / det)

for z in (-25, 0, 10, 20, 32, 40):
    print("cake pixel (354,121) @ z=%4d -> x,y = (%.1f, %.1f)" % (z, *invert(354, 121, z)))
for z in (38, 40, 42):
    print("pad  pixel (360.6,61.3) @ z=%4d -> x,y = (%.1f, %.1f)" % (z, *invert(360.6, 61.3, z)))

# Overlay markers on the descent frame
img = Image.open(r"D:\NXROBO\auto\docs\cake_run_20261121\run2\02_descent.jpg").convert("RGB")
img = img.resize((img.width * 2, img.height * 2), Image.LANCZOS)
draw = ImageDraw.Draw(img)
try:
    font = ImageFont.truetype(r"C:\Windows\Fonts\segoeui.ttf", 18)
except Exception:
    font = ImageFont.load_default()

def mark(pixel, color, label, radius=7):
    u, v = pixel
    x, y = u * 2, v * 2
    draw.ellipse([x - radius, y - radius, x + radius, y + radius], outline=color, width=3)
    draw.text((x + 10, y - 10), label, font=font, fill=color)

mark((354, 121), (255, 60, 60), "cake det (354,121)")
mark((360.6, 61.3), (255, 220, 40), "pad obs (360.6,61.3)")
pad_cmd = project(318.8, -17.4, 40.0)
mark(pad_cmd, (60, 200, 255), "pad cmd z40 proj")
cake_depth = project(318.8, -17.4, -24.9)
mark(cake_depth, (60, 255, 120), "cake depth proj")
img.save(r"D:\NXROBO\tmp\descent_marked.png")
print("saved descent_marked.png")
