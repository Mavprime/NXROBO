import os
import numpy as np
import rospy
from sensor_msgs.msg import Image
import match_logic
from runtask import CamAction

rospy.init_node("test_cake_point", anonymous=True)
cam = CamAction()
M = cam.transform3d
scale = 0.386784

msg = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image, timeout=5)
raw = np.frombuffer(msg.data, dtype=np.uint16)
depth = raw.reshape(msg.height, msg.width).astype(np.float64) / 1000.0

u, v = 346, 202
half = 2
patch = depth[v - half:v + half + 1, u - half:u + half + 1]
valid = patch[patch > 0]
d = float(np.median(valid)) if len(valid) else None

print(f"Pixel ({u}, {v}) -> raw depth d={d}")

def unproject(u, v, d):
    return np.array([(u - match_logic.CAM_CX) / match_logic.CAM_FX * d,
                     (v - match_logic.CAM_CY) / match_logic.CAM_FY * d, d])

def solve_arm_point(M, cam_point, scale):
    a = np.array([[M[0][j], M[1][j], M[2][j]] for j in range(3)])
    b = np.array([cam_point[j] / scale - M[3][j] for j in range(3)])
    return np.linalg.solve(a, b)

if d is not None:
    pt = solve_arm_point(M, unproject(u, v, d), scale)
    print(f"solve_arm_point with scale={scale}: x={pt[0]:.1f}, y={pt[1]:.1f}, z={pt[2]:.1f}")
    pt1 = solve_arm_point(M, unproject(u, v, d), 1.0)
    print(f"solve_arm_point with scale=1.0: x={pt1[0]:.1f}, y={pt1[1]:.1f}, z={pt1[2]:.1f}")

# Also check around the cake top face (different v offsets)
print("\nScanning patch around cake:")
for dv in (-15, -10, -5, 0, 5, 10, 15):
    p_d = depth[v + dv, u]
    if p_d > 0:
        p_pt = solve_arm_point(M, unproject(u, v + dv, p_d), scale)
        print(f"v={v+dv} d={p_d:.3f} -> x={p_pt[0]:.1f}, y={p_pt[1]:.1f}, z={p_pt[2]:.1f}")
