import sys

import numpy as np
from PIL import Image

sys.path.insert(0, r"C:\Users\user1\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\Lib\site-packages")

BEFORE = r"D:\NXROBO\tmp\before.jpg"
AFTER = r"D:\NXROBO\tmp\snap_after_probe.jpg"


def gray(path):
    return np.asarray(Image.open(path).convert("L"), dtype=np.float64)


def phase_shift(a, b):
    """Return (du, dv) shift that maps a onto b via FFT phase correlation."""
    a = a - a.mean()
    b = b - b.mean()
    fa = np.fft.fft2(a)
    fb = np.fft.fft2(b)
    cps = fa * np.conj(fb)
    cps /= np.abs(cps) + 1e-9
    corr = np.fft.ifft2(cps).real
    idx = np.unravel_index(np.argmax(corr), corr.shape)
    dv, du = idx
    if dv > a.shape[0] // 2:
        dv -= a.shape[0]
    if du > a.shape[1] // 2:
        du -= a.shape[1]
    return int(du), int(dv)


a = gray(BEFORE)
b = gray(AFTER)
print("shapes", a.shape, b.shape)
for name, (y0, y1) in (("full", (0, 480)), ("tower", (0, 240)),
                      ("bear", (240, 400)), ("table", (100, 480))):
    du, dv = phase_shift(a[y0:y1], b[y0:y1])
    print("%-6s shift (du=%+d, dv=%+d)" % (name, du, dv))

diff = np.abs(a - b)
print("mean abs diff %.2f  max %.0f" % (diff.mean(), diff.max()))
hot = (diff > 40).astype(np.uint8) * 255
Image.fromarray(hot).save(r"D:\NXROBO\tmp\diff_mask.png")
Image.fromarray(np.clip(diff * 3, 0, 255).astype(np.uint8)).save(
    r"D:\NXROBO\tmp\diff_vis.png")
print("saved diff_mask.png diff_vis.png")
