#!/usr/bin/python3

import sys
import os
import yaml
import _thread
import threading
import pickle
import struct

import numpy as np

import rospy
import rospkg
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseResult
import common.msg
import common.srv
from common.msg import MoveStraightDistanceAction, TurnBodyDegreeAction
# import common.action
import swiftpro.msg
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from swiftpro.msg import position
from vision_msgs.msg import Detection2DArray
import tf
import tf.transformations as tf_transformations
from collections import Counter

# Class ID Normalizer (Temporal Voting & ID Mapping).
# YOLO emits both the item class and its printed face ("<item>_site"), so each
# _site alias must map to ITS OWN item (see auto_match/config/items_config.yaml):
#   bear 1, bear_site 10   -> 1
#   wine 2, wine_site 20   -> 2
#   clock 3, clock_site 30 -> 3
#   banana 4, banana_site 40 -> 4
ID_MAP = {
    1: 1, 10: 1,
    2: 2, 20: 2,
    3: 3, 30: 3,
    4: 4, 40: 4,
}

def norm_id(raw_id):
    try:
        return ID_MAP.get(int(raw_id), int(raw_id))
    except Exception:
        return raw_id

# Affine transform from the uArm wrist coordinates (mm) to the RealSense
# camera frame (metres):  cam = [x, y, z, 1] @ TRANSFORM_3D
# Fitted on this robot from 10 suction-cup observations spread over the
# workspace (z from -120 to -20 mm); RMS residual 1.2 mm.
# If ~/transform_3d.dat exists it overrides this matrix, so a fresh
# camera/arm calibration can be dropped in without editing this file.
TRANSFORM_3D = np.array([
    [-4.500e-05, -9.760e-04,  2.840e-04],
    [-1.018e-03,  5.800e-05, -2.100e-05],
    [ 1.200e-05, -2.620e-04, -9.920e-04],
    [ 2.681900e-02, 1.351830e-01, 3.527390e-01],
], dtype=float)

# The orange suction pad is ~8 mm thick: the depth camera sees its top face,
# while the sealing rim is 8 mm lower.  The wrist must therefore stop 8 mm
# HIGHER than the measured surface, otherwise the pad is pressed into the
# object/floor and the open-loop steppers skip steps.
PAD_THICKNESS_MM = 8.0

# Hard software floor.  Below this the linkage/pad presses on the floor and
# the steppers lose position; never command the wrist lower than this.
SAFE_MIN_Z = -120.0

# RealSense D435 color-camera intrinsics (640x480), used to project a wrist
# position back into the image when the depth point is unusable.
CAM_FX = 605.6860961914062
CAM_FY = 605.8469848632812
CAM_CX = 315.03875732421875
CAM_CY = 249.8477020263672

import math

MATH_TRANS = 57.2958
MATH_L1 = 106.6
MATH_L2 = 13.2
MATH_LOWER_ARM = 142.07
MATH_UPPER_ARM = 158.81
MATH_UPPER_LOWER = MATH_UPPER_ARM / MATH_LOWER_ARM

def ik_reachable(x, y, z):
    '''Check geometric IK reachability of point (x, y, z) in mm for uArm Swift Pro.'''
    try:
        z = z + 74.55
        z_in = (z - MATH_L1) / MATH_LOWER_ARM
        if x < 0.1:
            x = 0.1
        if y == 0:
            rot = 90.0
        elif y < 0:
            rot = -math.atan(x / y) * MATH_TRANS
        else:
            rot = 180.0 - math.atan(x / y) * MATH_TRANS

        sin_rot = math.sin(rot / MATH_TRANS)
        if sin_rot == 0:
            return False

        x_in = (x / sin_rot - MATH_L2 - 56.55) / MATH_LOWER_ARM
        if x_in == 0:
            return False

        sqrt_zx = math.sqrt(z_in * z_in + x_in * x_in)
        if sqrt_zx == 0:
            return False

        upper = MATH_UPPER_LOWER
        right_all = (sqrt_zx * sqrt_zx + upper * upper - 1) / (2 * upper * sqrt_zx)
        left_all = (sqrt_zx * sqrt_zx + 1 - upper * upper) / (2 * sqrt_zx)
        return abs(right_all) <= 1 and abs(left_all) <= 1
    except Exception:
        return False



class SwiftProInterface:
    def __init__(self):
        # Arm motion position publisher
        self.arm_position_pub = rospy.Publisher(
            "position_write_topic", swiftpro.msg.position, queue_size=1)
        # Arm vacuum pump status publisher
        self.arm_pump_pub = rospy.Publisher(
            "pump_topic", swiftpro.msg.status, queue_size=1)
        # Arm motor lock/unlock status publisher
        self.arm_status_pub = rospy.Publisher(
            "swiftpro_status_topic", swiftpro.msg.status, queue_size=1)

    def set_pose(self, x, y, z, delay=0.35):
        '''Publish manipulator target position (x, y, z) in mm.'''
        pos = position()
        pos.x = x
        pos.y = y
        pos.z = z
        self.arm_position_pub.publish(pos)
        if delay > 0:
            rospy.sleep(delay)

    def set_pump(self, enable: bool):
        '''Set vacuum pump state (True: suction on, False: off).'''
        rospy.loginfo(f"Setting arm pump state: {enable}")
        if enable:
            self.arm_pump_pub.publish(swiftpro.msg.status(1))
        else:
            self.arm_pump_pub.publish(swiftpro.msg.status(0))

    def set_status(self, lock: bool):
        '''Set arm motor lock/unlock status.'''
        rospy.loginfo(f"Setting arm motor status: {lock}")
        if lock:
            self.arm_status_pub.publish(swiftpro.msg.status(1))
        else:
            self.arm_status_pub.publish(swiftpro.msg.status(0))

class CamAction:
    def __init__(self):
        # Load competition items configuration
        rospack = rospkg.RosPack()
        package_path = os.path.join(rospack.get_path('auto_match'))
        items_path = os.path.join(package_path, 'config', 'items_config.yaml')
        try:
            with open(items_path, "r", encoding="utf8") as f:
                items_content = yaml.load(f.read(), Loader=yaml.FullLoader)
        except Exception:
            rospy.logerr("Cannot open items_config.yaml")
            sys.exit(1)
        if isinstance(items_content, type(None)):
            rospy.logerr("items_config.yaml is empty")
            sys.exit(1)

        # Target item IDs from yaml
        self.search_id = [
            items_content["items"][items_content["objects"]["objects_a"]],
            items_content["items"][items_content["objects"]["objects_b"]],
            items_content["items"][items_content["objects"]["objects_c"]],
            items_content["items"][items_content["objects"]["objects_d"]]
        ]

        self.latest_msg = None
        self.latest_time = rospy.Time(0)
        self.lock = threading.Lock()
        self.sub = rospy.Subscriber("/objects", Detection2DArray, self._objects_cb, queue_size=1)

        # NOTE: the registered point cloud is ~10 MB/frame, so we do NOT keep
        # a 30 Hz subscriber (that ate a whole CPU core).  point3d() pulls one
        # frame only when a grasp actually needs it.
        self.transform3d = self._load_transform()

    def _load_transform(self):
        '''Load a freshly calibrated 4x3 camera<-arm matrix if present.'''
        path = os.path.expanduser("~/transform_3d.dat")
        try:
            t = np.load(path, allow_pickle=True)
            if t.shape == (4, 3):
                rospy.loginfo(f">> [DEPTH] Loaded 3D camera-arm transform from {path}")
                return t
        except Exception:
            pass
        return TRANSFORM_3D

    def _objects_cb(self, msg):
        with self.lock:
            self.latest_msg = msg
            self.latest_time = rospy.Time.now()

    def point3d(self, u, v, win=6):
        '''
        Median 3D point (metres, camera frame) around pixel (u, v).
        Pulls a single registered point cloud frame on demand; returns None
        when no valid depth is available (glossy/occluded surfaces).
        '''
        try:
            cloud = rospy.wait_for_message(
                "/camera/depth_registered/points", PointCloud2, timeout=3.0)
        except Exception:
            return None
        try:
            w, h = cloud.width, cloud.height
            u = int(round(u)); v = int(round(v))
            u0, u1 = max(0, u - win), min(w - 1, u + win)
            v0, v1 = max(0, v - win), min(h - 1, v + win)
            fo = {f.name: f.offset for f in cloud.fields}
            if not all(k in fo for k in ("x", "y", "z")):
                return None
            step = cloud.point_step
            row = cloud.row_step
            data = cloud.data
            vals = []
            for yy in range(v0, v1 + 1):
                base = yy * row
                for xx in range(u0, u1 + 1):
                    off = base + xx * step
                    x = struct.unpack_from("<f", data, off + fo["x"])[0]
                    y = struct.unpack_from("<f", data, off + fo["y"])[0]
                    z = struct.unpack_from("<f", data, off + fo["z"])[0]
                    if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                        vals.append((x, y, z))
            if not vals:
                return None
            return [float(q) for q in np.median(np.array(vals), axis=0)]
        except Exception as e:
            rospy.logwarn_throttle(5.0, f">> [DEPTH] point3d failed: {e}")
            return None

    def point_to_arm(self, p):
        '''Convert a camera-frame point (metres) to uArm wrist coordinates (mm).'''
        try:
            B = self.transform3d
            M = B[:3, :].T
            b = B[3, :]
            arm = np.linalg.solve(M, np.array(p, dtype=float) - b)
            return float(arm[0]), float(arm[1]), float(arm[2])
        except Exception as e:
            rospy.logwarn_throttle(5.0, f">> [DEPTH] point_to_arm failed: {e}")
            return None

    def project_wrist(self, x, y, z):
        '''Project uArm wrist coordinates (mm) into the color image pixel.'''
        try:
            cam = np.array([x, y, z, 1.0], dtype=float) @ self.transform3d
            if cam[2] <= 1e-6:
                return None
            u = CAM_FX * cam[0] / cam[2] + CAM_CX
            v = CAM_FY * cam[1] / cam[2] + CAM_CY
            return float(u), float(v)
        except Exception:
            return None

    def pixel_to_arm_at_z(self, u, v, z, x0=200.0, y0=0.0):
        '''
        Wrist (x, y) whose suction-pad projection lands on pixel (u, v) at a
        known surface height z.  Uses the calibrated 3D transform and camera
        intrinsics, so it needs no depth and cannot be fooled by the arm
        occluding the cube.
        '''
        x, y = float(x0), float(y0)
        for _ in range(30):
            p = self.project_wrist(x, y, z)
            if p is None:
                return None
            ex, ey = u - p[0], v - p[1]
            if math.hypot(ex, ey) < 0.5:
                break
            px = self.project_wrist(x + 1.0, y, z)
            py = self.project_wrist(x, y + 1.0, z)
            if px is None or py is None:
                return None
            J = np.array([[px[0] - p[0], py[0] - p[0]],
                          [px[1] - p[1], py[1] - p[1]]], dtype=float)
            try:
                d = np.linalg.solve(J, np.array([ex, ey], dtype=float))
            except Exception:
                return None
            d = np.clip(d, -60.0, 60.0)
            x += float(d[0])
            y += float(d[1])
            x = min(max(x, 50.0), 350.0)
            y = min(max(y, -210.0), 210.0)
        return x, y

    def detector(self, num_samples=1, timeout=3.0):
        '''
        Reliable object detection:
        1. Checks fresh subscriber cache first (< 1.0s old)
        2. Falls back to wait_for_message with generous 3.0s timeout
        @return: list of [[target_id, [x, y, 0]], ...]
        '''
        msg = None
        with self.lock:
            if self.latest_msg is not None and (rospy.Time.now() - self.latest_time).to_sec() < 1.0:
                msg = self.latest_msg

        if msg is None:
            try:
                msg = rospy.wait_for_message("/objects", Detection2DArray, timeout=timeout)
            except Exception as e:
                rospy.logerr(f">> [DETECTOR] No message on /objects ({e}). Check if detect_ros.py and camera driver are running! (Use option 1 in dashboard)")
                return []

        cube_list = []
        for obj in msg.detections:
            raw_id = obj.results[0].id
            cid = norm_id(raw_id)
            score = obj.results[0].score if hasattr(obj.results[0], 'score') else 1.0

            if score < 0.35:
                continue

            if cid in [1, 2, 3, 4] or raw_id in self.search_id:
                x = int(round(obj.bbox.center.x))
                y = int(round(obj.bbox.center.y))
                target_id = cid if cid in [1, 2, 3, 4] else raw_id
                cube_list.append([target_id, [x, y, 0]])

        return cube_list

class ArmAction:
    def __init__(self):
        self.cam = CamAction()

        # Calibration data for Layer 1
        filename = os.environ['HOME'] + "/thefile.txt"
        with open(filename, 'r') as f:
            s = f.read()
        arr = s.split()
        self.x_kb = [float(arr[0]), float(arr[1])]
        self.y_kb = [float(arr[2]), float(arr[3])]

        # Calibration data for Layer 2
        filename1 = os.environ['HOME'] + "/thefile2.txt"
        with open(filename1, 'r') as f1:
            s1 = f1.read()
        arr1 = s1.split()
        self.x_kb1 = [float(arr1[0]), float(arr1[1])]
        self.y_kb1 = [float(arr1[2]), float(arr1[3])]        
    
        # 创建机械臂控制接口的对象
        self.interface = SwiftProInterface()

        self.grasp_status_pub = rospy.Publisher("/grasp_status", String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.sleep(0.2)

        # swiftpro_read_node sends M2019 (motors detached) when it opens the
        # serial port.  Attach them (M17) so the arm holds position and does
        # not lose steps; otherwise every move can slip under gravity.
        self.interface.set_status(True)
        rospy.sleep(0.3)

    def is_in_arm_limit(self, cx, cy, z):
        '''
        Check expanded physical limits of uArm Swift Pro:
        - Extension X: 50.0 .. 350.0 mm
        - Azimuth Y: -210.0 .. 210.0 mm
        - Reach radius R: up to 355.0 mm
        - Analytical inverse kinematics: ik_reachable(cx, cy, z)
        '''
        if not (50.0 <= cx <= 350.0):
            return False
        if not (-210.0 <= cy <= 210.0):
            return False
        if math.hypot(cx, cy) > 355.0:
            return False
        return ik_reachable(cx, cy, z)

    def target_from_pixel(self, u, v, kb_x, kb_y, expected_z, z_lo, z_hi):
        '''
        Convert a detected object pixel into a uArm wrist target (mm).

        1. Primary: registered RealSense 3D point + calibrated camera->arm
           transform.  The result is accepted only if its height is plausible
           for the layer being grasped; otherwise the depth hit the arm,
           the floor, or another occluder.
        2. Fallback: project the pixel onto the known layer height using the
           same 3D transform and camera intrinsics.  Needs no depth, so an
           occluding arm cannot make the arm drive upwards.
        3. Last resort: legacy 2D calibration.
        '''
        p = self.cam.point3d(u, v)
        tgt = self.cam.point_to_arm(p) if p is not None else None
        if tgt is not None and z_lo <= tgt[2] <= z_hi:
            return tgt[0], tgt[1], tgt[2], "3d"

        x0 = kb_x[0] * v + kb_x[1]
        y0 = kb_y[0] * u + kb_y[1]
        xy = self.cam.pixel_to_arm_at_z(u, v, expected_z, x0=x0, y0=y0)
        if xy is not None:
            return xy[0], xy[1], expected_z, "proj"
        return x0, y0, expected_z, "2d"

    def contact_heights(self, z_surface):
        '''
        Hover / touch / lift wrist heights around a measured object surface.

        The depth camera sees the TOP of the suction pad; the sealing rim is
        PAD_THICKNESS_MM lower.  So the wrist must stop PAD_THICKNESS_MM ABOVE
        the measured surface for the rim to meet it.  SAFE_MIN_Z prevents the
        arm from pushing into the floor (which makes the steppers skip).
        '''
        z_hover = z_surface + 45.0
        z_touch = max(z_surface + PAD_THICKNESS_MM, SAFE_MIN_Z)
        z_lift = min(z_surface + 100.0, 165.0)
        return z_hover, z_touch, z_lift

    def turn_base(self, angle_rad):
        '''
        Turn robot base by angle_rad (radians) in place.
        angle_rad > 0: CCW (left)
        angle_rad < 0: CW (right)
        '''
        if abs(angle_rad) < 0.025:  # Less than ~1.5 deg, no turn needed
            return

        # Fast responsive turn speed (0.75 rad/s)
        ang_speed = 0.75
        ang_vel = ang_speed if angle_rad > 0 else -ang_speed
        turn_time = max(0.08, min(abs(angle_rad) / ang_speed, 2.0))

        rospy.loginfo(f">> [VISUAL-SERVO] Fast base turn: {math.degrees(angle_rad):.1f} deg, duration {turn_time:.2f}s, vel={ang_vel:.2f} rad/s")
        tw = Twist()
        tw.angular.z = ang_vel

        t_end = rospy.Time.now() + rospy.Duration.from_sec(turn_time)
        rate = rospy.Rate(30)
        while rospy.Time.now() < t_end and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(tw)
            rate.sleep()

        # Crisp active braking
        for _ in range(2):
            self.cmd_vel_pub.publish(Twist())
            rospy.sleep(0.02)
        rospy.sleep(0.10)

    def drive_base(self, dist_m):
        '''
        Move robot base forward or backward by dist_m meters.
        dist_m > 0: forward
        dist_m < 0: backward
        '''
        if abs(dist_m) < 0.03:  # Less than 3 cm, no drive needed
            return

        # Safe displacement clamp (max 10 cm per step)
        dist_m = max(-0.10, min(0.10, dist_m))
        lin_speed = 0.18  # Crisp 18 cm/s linear velocity
        lin_vel = lin_speed if dist_m > 0 else -lin_speed
        move_time = abs(dist_m) / lin_speed

        rospy.loginfo(f">> [VISUAL-SERVO] Base distance correction: {dist_m*100:.1f} cm, duration {move_time:.2f}s, vel={lin_vel:.2f} m/s")
        tw = Twist()
        tw.linear.x = lin_vel

        t_end = rospy.Time.now() + rospy.Duration.from_sec(move_time)
        rate = rospy.Rate(30)
        while rospy.Time.now() < t_end and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(tw)
            rate.sleep()

        for _ in range(2):
            self.cmd_vel_pub.publish(Twist())
            rospy.sleep(0.02)
        rospy.sleep(0.10)

    # Grasp floor object (Layer 1)
    def grasp(self):
        '''
        Intelligent, high-precision grasp of Layer 1 cube:
        1. If cube is ALREADY in arm reach workspace (X: 170..315 mm, Y: -180..180 mm):
           Base DOES NOT move, robot does not center, but grasps cube immediately in place!
        2. If cube is out of workspace reach:
           - In-place base rotation towards cube (horizontal centering)
           - Forward drive ONLY if cube is too far away (> 295 mm)
           - CRITICAL: 0.20s pause and RESCAN after motion to recalculate exact cube coords!
        3. Grasp and lift:
           - Heights are computed from the cube's measured 3D position
             (works for floor cubes and raised tiers)
           - Enable pump and hold vacuum (0.40s) to build strong negative pressure
           - Confident vertical lift upwards (+100 mm)
        '''

        # 1. Primary cube detection
        cube_list = self.cam.detector(timeout=3.0)
        rospy.loginfo(f">> [GRASP] Cube detection: {cube_list}")

        if len(cube_list) == 0:
            rospy.logwarn(">> [GRASP] Aborted: No target cubes detected in camera view.")
            self.grasp_status_pub.publish(String("1"))
            return 0

        # Sort: choose cube closest to optical center line and arm workspace
        cube_list.sort(key=lambda item: (abs(item[1][0] - 320) + (260 - item[1][1]) * 1.2))
        target_cube = cube_list[0]
        target_id = target_cube[0]
        initial_target_id = target_id
        u = target_cube[1][0]
        v = target_cube[1][1]

        x, y, z_surface, src = self.target_from_pixel(u, v, self.x_kb, self.y_kb, -127.0, -160.0, 10.0)
        z_hover, z_touch, z_lift = self.contact_heights(z_surface)

        # 2. Check: is cube ALREADY in reach of manipulator?
        # uArm Swift Pro can grasp objects left and right (Y: -190..190 mm)
        in_reach_x = (120.0 <= x <= 320.0)
        in_reach_y = (-190.0 <= y <= 190.0)
        ik_touch = self.is_in_arm_limit(x, y, z_touch)
        ik_lift = self.is_in_arm_limit(x, y, z_lift)
        can_reach_direct = in_reach_x and in_reach_y
        if src != "3d":
            # The analytical IK model is conservative at very low z; trust
            # the measured 3D point when we have one.
            can_reach_direct = can_reach_direct and ik_touch and ik_lift

        rospy.loginfo(f">> [GRASP] Target ID {target_id}: u={u}, v={v} -> x={x:.1f}, y={y:.1f}, z_surface={z_surface:.1f} ({src}) | reach_direct={can_reach_direct}")

        if can_reach_direct:
            rospy.loginfo(f">> [GRASP] Cube already in arm workspace! Base NOT moving, grasping immediately.")
        else:
            # 3. If cube cannot be reached directly - adjust base position
            rospy.loginfo(f">> [GRASP] Cube out of direct reach (x={x:.1f}, y={y:.1f}). Adjusting base...")

            # Rotate base towards cube
            delta_u = u - 320.0
            if abs(delta_u) > 25.0:
                angle_rad = -delta_u * 0.0019
                rospy.loginfo(f">> [VISUAL-SERVO] Rotating base towards cube: {math.degrees(angle_rad):.1f} deg (u={u})...")
                self.turn_base(angle_rad)

            # Distance correction ONLY if strictly necessary (do not push cube!)
            if x > 295.0:
                drive_m = min(0.06, (x - 245.0) / 1000.0)
                rospy.loginfo(f">> [VISUAL-SERVO] Cube far (x={x:.1f} mm). Driving forward {drive_m*100:.1f} cm...")
                self.drive_base(drive_m)
            elif x < 165.0:
                rospy.loginfo(f">> [VISUAL-SERVO] Cube too close (x={x:.1f} mm). Backing up 4 cm...")
                self.drive_base(-0.04)

            # 4. RESCAN after base maneuver!
            rospy.sleep(0.20)
            rescan_list = self.cam.detector(timeout=3.0)
            if rescan_list:
                # Keep the same object class we rotated towards; the YOLO head can
                # flip classes between frames, and switching targets here would
                # send the item to the wrong collection zone.
                same_class = [it for it in rescan_list if it[0] == initial_target_id]
                pool = same_class if same_class else rescan_list
                pool.sort(key=lambda item: (abs(item[1][0] - 320) + (260 - item[1][1]) * 1.2))
                target_cube = pool[0]
                target_id = target_cube[0]
                u = target_cube[1][0]
                v = target_cube[1][1]
                x, y, z_surface, src = self.target_from_pixel(u, v, self.x_kb, self.y_kb, -127.0, -160.0, 10.0)
                z_hover, z_touch, z_lift = self.contact_heights(z_surface)
                rospy.loginfo(f">> [GRASP] Exact target after rotation: u={u}, v={v} -> x={x:.1f}, y={y:.1f}, z_surface={z_surface:.1f} ({src})")
            else:
                rospy.logwarn(">> [GRASP] Cube not detected after maneuver, using calculated coordinates.")

        # 5. Execute physical grasp and lift with manipulator
        # Clamp coordinates to safe operating limits
        x = max(120.0, min(320.0, x))
        y = max(-190.0, min(190.0, y))
        rospy.loginfo(f">> [GRASP] Executing grasp for ID {target_id}: x={x:.1f}, y={y:.1f}, z_touch={z_touch:.1f}, z_lift={z_lift:.1f}")

        # Energise the steppers (M17) before any motion after a driver restart.
        self.interface.set_status(True)
        rospy.sleep(0.15)

        # Rapid approach above cube
        self.interface.set_pose(x, y, z_hover, delay=0.35)

        # Enable vacuum pump early for fast suction
        self.interface.set_pump(True)
        rospy.sleep(0.10)

        # Smooth descent to touch top face of cube
        self.interface.set_pose(x, y, z_touch, delay=0.45)

        # Vacuum dwell time for reliable suction seal
        rospy.sleep(0.40)

        # Vertical lift of cube
        self.interface.set_pose(x, y, z_lift, delay=0.55)
        print("Cube lifted successfully")
        rospy.loginfo(f">> [GRASP] Cube ID {target_id} lifted successfully to height z={z_lift}")

        self.grasp_status_pub.publish(String("0"))
        return target_id

    # Grasp 2nd floor Cake (Layer 2)
    def grasp1(self):
        '''
        Intelligent, high-precision grasp of Layer 2 Cake (z_base=45):
        1. If Cake is ALREADY in reach: grasp immediately without rotating base.
        2. If out of reach: rotate, rescan after rotation for exact coordinates.
        3. Vacuum grasp with dwell time and vertical lift.
        '''
        cube_list = self.cam.detector(timeout=3.0)
        rospy.loginfo(f">> [GRASP1] Layer 2 Cake detection: {cube_list}")

        if len(cube_list) == 0:
            rospy.logwarn(">> [GRASP1] Aborted: No layer 2 target detected in camera view.")
            self.grasp_status_pub.publish(String("1"))
            return 0

        cube_list.sort(key=lambda item: (abs(item[1][0] - 320) + (250 - item[1][1]) * 1.2))
        target_cube = cube_list[0]
        target_id = target_cube[0]
        initial_target_id = target_id
        u = target_cube[1][0]
        v = target_cube[1][1]

        x, y, z_surface, src = self.target_from_pixel(u, v, self.x_kb1, self.y_kb1, 57.0, 35.0, 100.0)
        z_hover, z_touch, z_lift = self.contact_heights(z_surface)

        in_reach_x = (120.0 <= x <= 320.0)
        in_reach_y = (-190.0 <= y <= 190.0)
        ik_touch = self.is_in_arm_limit(x, y, z_touch)
        ik_lift = self.is_in_arm_limit(x, y, z_lift)
        can_reach_direct = in_reach_x and in_reach_y
        if src != "3d":
            # The analytical IK model is conservative at very low z; trust
            # the measured 3D point when we have one.
            can_reach_direct = can_reach_direct and ik_touch and ik_lift

        rospy.loginfo(f">> [GRASP1] Target ID {target_id}: u={u}, v={v} -> x={x:.1f}, y={y:.1f}, z_surface={z_surface:.1f} ({src}) | reach_direct={can_reach_direct}")

        if can_reach_direct:
            rospy.loginfo(f">> [GRASP1] Cake already in reach! Base NOT moving, grasping immediately.")
        else:
            rospy.loginfo(f">> [GRASP1] Cake out of direct reach. Adjusting base...")
            delta_u = u - 320.0
            if abs(delta_u) > 25.0:
                angle_rad = -delta_u * 0.0019
                rospy.loginfo(f">> [VISUAL-SERVO] Rotating base towards cake: {math.degrees(angle_rad):.1f} deg (u={u})...")
                self.turn_base(angle_rad)

            if x > 295.0:
                drive_m = min(0.06, (x - 245.0) / 1000.0)
                rospy.loginfo(f">> [VISUAL-SERVO] Cake far (x={x:.1f} mm). Driving forward {drive_m*100:.1f} cm...")
                self.drive_base(drive_m)
            elif x < 165.0:
                rospy.loginfo(f">> [VISUAL-SERVO] Cake close (x={x:.1f} mm). Backing up 4 cm...")
                self.drive_base(-0.04)

            rospy.sleep(0.20)
            rescan_list = self.cam.detector(timeout=3.0)
            if rescan_list:
                same_class = [it for it in rescan_list if it[0] == initial_target_id]
                pool = same_class if same_class else rescan_list
                pool.sort(key=lambda item: (abs(item[1][0] - 320) + (250 - item[1][1]) * 1.2))
                target_cube = pool[0]
                target_id = target_cube[0]
                u = target_cube[1][0]
                v = target_cube[1][1]
                x, y, z_surface, src = self.target_from_pixel(u, v, self.x_kb1, self.y_kb1, 57.0, 35.0, 100.0)
                z_hover, z_touch, z_lift = self.contact_heights(z_surface)
                rospy.loginfo(f">> [GRASP1] Exact target of cake after rotation: u={u}, v={v} -> x={x:.1f}, y={y:.1f}, z_surface={z_surface:.1f} ({src})")

        x = max(120.0, min(320.0, x))
        y = max(-190.0, min(190.0, y))
        rospy.loginfo(f">> [GRASP1] Executing grasp for Cake ID {target_id}: x={x:.1f}, y={y:.1f}, z_touch={z_touch:.1f}, z_lift={z_lift:.1f}")

        self.interface.set_status(True)
        rospy.sleep(0.15)
        self.interface.set_pose(x, y, z_hover, delay=0.35)
        self.interface.set_pump(True)
        rospy.sleep(0.10)
        self.interface.set_pose(x, y, z_touch, delay=0.45)
        rospy.sleep(0.40)

        print("Layer 2 Cake lifted successfully")
        self.interface.set_pose(x, y, z_lift, delay=0.55)
        rospy.loginfo(f">> [GRASP1] Layer 2 Cake ID {target_id} lifted successfully to height z={z_lift}")

        self.grasp_status_pub.publish(String("0"))
        return target_id
    
    
    def drop(self, check=False):
        '''
        放置方块, 可以先判断是否有方块, 从而调整放置高度
        @param check: 是否判断有无方块, 默认判断
        @return item_id: 执行结果
        '''
        x = 235
        y = 0
        z = -55

        # Lower cube to target drop point
        self.interface.set_pose(x, y, z, delay=0.40)

        # Disable vacuum pump and wait for vacuum relief
        self.interface.set_pump(0)
        rospy.sleep(0.25)

        # SAFE VERTICAL RETRACT: lift suction cup straight up +80 mm,
        # to prevent friction or lateral motion from disturbing the placed cube:
        lift_z = min(z + 80, 160)
        self.interface.set_pose(x, y, lift_z, delay=0.35)

        # Retract to service pose (50, 180, 160):
        self.arm_grasp_ready(block=False)

        self.grasp_status_pub.publish(String("0"))
        return True

    def drop1(self, check=False):
        '''
        Place block, layer 2 height
        '''
        cube_list = self.cam.detector(timeout=3.0)
        if cube_list:
            x = self.x_kb[0] * cube_list[0][1][1] + self.x_kb[1]
            y = self.y_kb[0] * cube_list[0][1][0] + self.y_kb[1]
        else:
            x, y = 235, 0
        z = 60

        # Position arm above placed cube
        self.interface.set_pose(x, y, z, delay=0.40)

        # Disable vacuum pump and wait for vacuum relief
        self.interface.set_pump(0)
        rospy.sleep(0.25)

        # Vertical retract upwards
        self.interface.set_pose(x, y, 140, delay=0.35)

        self.arm_grasp_ready(block=False)
        self.grasp_status_pub.publish(String("0"))
        return True
    
    def drop2(self, check=False):
        '''
        Place block, layer 3 height
        '''
        cube_list = self.cam.detector(timeout=3.0)
        if cube_list:
            x = self.x_kb1[0] * cube_list[0][1][1] + self.x_kb1[1]
            y = self.y_kb1[0] * cube_list[0][1][0] + self.y_kb1[1]
        else:
            x, y = 235, 0
        z = 160

        self.interface.set_pose(x, y, z, delay=0.40)

        self.interface.set_pump(0)
        rospy.sleep(0.25)

        self.interface.set_pose(x, y, z + 20, delay=0.30)

        self.arm_grasp_ready(block=False)
        self.grasp_status_pub.publish(String("0"))
        return True

    def arm_position_reset(self):
        '''
        校准机械臂的坐标系, 机械臂因碰撞导致坐标计算出问题时使用
        '''
        r1 = rospy.Rate(10)
        self.interface.set_status(False)
        r1.sleep()
        self.interface.set_status(True)
        r1.sleep()

    def arm_home(self, block=False):
        '''
        收起机械臂(无物品)
        '''
        self.interface.set_pose(130, 0, 35, delay=0.35 if block else 0.20)

    def arm_grasp_ready(self, block=False):
        '''
        移动机械臂到摄像头看不到的地方，以方便识别与抓取
        '''
        self.interface.set_pose(50, 180, 160, delay=0.35 if block else 0.20)

    def arm_grasp_laser(self, block=False):
        self.interface.set_pose(160, 0, 20, delay=0.35 if block else 0.20)

class RobotMoveAction:
    def __init__(self):
        # 创建控制spark直走的action客户端
        self.move_action_cli = actionlib.SimpleActionClient(
            'move_straight', MoveStraightDistanceAction)
        self.move_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))

        # 创建控制spark旋转的action客户端
        self.turn_action_cli = actionlib.SimpleActionClient(
            'turn_body', TurnBodyDegreeAction)
        self.turn_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))


        # Create front/back distance service client (with timeout to prevent hang on startup)
        try:
            rospy.wait_for_service('/get_distance', timeout=rospy.Duration.from_sec(1.0))
            self.distance_srv = rospy.ServiceProxy(
                'get_distance', common.srv.GetFrontBackDistance)
        except Exception:
            rospy.logwarn("/get_distance service unavailable, skipping")
            self.distance_srv = None

        # 创建导航地点的话题发布者
        self.goto_local_pub = rospy.Publisher(
            "mark_nav", String, queue_size=1)


    def goto_local(self, name):
        '''
        根据目标点名称,发布目标位置到MoveBase服务器,根据返回状态进行判断
        @return: True 为成功到达, False 为失败
        '''

        # 发布目标位置
        self.goto_local_pub.publish("go "+name)
        rospy.loginfo(f"已发送go指令至: {name}")

        # 设定1分钟的时间限制，进行阻塞等待
        try:
            ret_status = rospy.wait_for_message(
                'move_base/result', MoveBaseActionResult, rospy.Duration(60)).status.status
        except Exception:
            rospy.logwarn("nav timeout!!!")
            ret_status = GoalStatus.ABORTED

        # 如果一分钟之内没有到达，放弃目标
        if ret_status != GoalStatus.SUCCEEDED:
            rospy.Publisher("move_base/cancel", GoalID, queue_size=1).publish(
                GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
            try:
                rospy.wait_for_message(
                    'move_base/result', MoveBaseActionResult, rospy.Duration(3))
            except Exception:
                rospy.logwarn("move_base result timeout. this is abnormal.")
            rospy.loginfo("==========Timed out achieving goal==========")
            return False
        else:
            rospy.loginfo("==========Goal succeeded==========")
            return True

    #不记录方向信息   
    def arriveto_local(self, name):
        '''
        根据目标点名称,发布目标位置到MoveBase服务器,根据返回状态进行判断
        @return: True 为成功到达, False 为失败
        '''

        # 发布目标位置
        self.goto_local_pub.publish("arrive "+name)
        rospy.loginfo(f"已发送arrivex指令至: {name}")

        # 设定1分钟的时间限制，进行阻塞等待
        try:
            ret_status = rospy.wait_for_message(
                'move_base/result', MoveBaseActionResult, rospy.Duration(60)).status.status
        except Exception:
            rospy.logwarn("nav timeout!!!")
            ret_status = GoalStatus.ABORTED

        # 如果一分钟之内没有到达，放弃目标
        if ret_status != GoalStatus.SUCCEEDED:
            rospy.Publisher("move_base/cancel", GoalID, queue_size=1).publish(
                GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
            try:
                rospy.wait_for_message(
                    'move_base/result', MoveBaseActionResult, rospy.Duration(3))
            except Exception:
                rospy.logwarn("move_base result timeout. this is abnormal.")
            rospy.loginfo("==========Timed out achieving goal==========")
            return False
        else:
            rospy.loginfo("==========Goal succeeded==========")
            return True
        

    
    def step_back(self,dis):
        '''
        后退, 用于抓取或放置后使用
        @return: True 为调整成功, False 为调整失败
        '''
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=-0.3,
                move_distance=dis,
            ),
            rospy.Duration.from_sec(5)  # 超过5s为超时
        )
        return True
    
    def step_go(self,dis):
        '''
        前进, 用于抓取或放置前使用
        @return: True 为调整成功, False 为调整失败
        '''
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=0.3,
                move_distance=dis,
            ),
            rospy.Duration.from_sec(5)  # 超过5s为超时
        )
        return True


class AutoAction:
    def __init__(self):
        # # 初始化节点
        # if init_node:
        rospy.init_node('spark_auto_match_node', anonymous=True)

        print("========ready to task===== ")

        # 实例化Cam
        try: self.cam = CamAction()
        except Exception as e:  print("except cam:",e)
        print("========实例化Cam===== ")
        # 实例化Arm
        try: self.arm = ArmAction()
        except Exception as e:  print("except arm:",e)
        print("========实例化Arm===== ")
        # 实例化Robot
        try: self.robot = RobotMoveAction()
        except Exception as e:  print("except robot:",e)
        print("========实例化Robot===== ")
        
        # Item ID mapping (Rule-compliant normalization)
        self.BANANA_ID = 4      # banana (4 or banana_site 40)
        self.TEDDY_ID  = 1      # teddy bear (1 or bear_site 10)
        self.CLOCK_ID  = 3      # clock (3 or clock_site 30)
        self.CAKE_ID   = 2      # cake/wine (2 or wine_site 20)
        self.CUP_ID    = 2      # alias for backwards compatibility

        # 订阅任务控制指令的话题
        self.task_cmd_sub = rospy.Subscriber("/task_start_flag", String, self.task_cmd_cb) # 订阅任务开始与否信号
        self.task_run_th = threading.Thread(target=lambda: "pass") # 创建线程对象
        self.stop_flag = False  # 任务的启停标志

        # Subscribe to manual arm control (/grasp)
        self.grasp_sub = rospy.Subscriber("/grasp", String, self.grasp_cb)

        rospy.loginfo("==============================================")
        rospy.loginfo("  spark_auto_match_node is ready!             ")
        rospy.loginfo("  Ready to accept /grasp 1 and /task_start_flag ")
        rospy.loginfo("==============================================")

    # 接收到启动自动赛信号，开始执行任务
    def run_task(self):
        ret = False # 是否导航成功标志
        item_type = 0 
        # self.arm.arm_home()  # 移动机械臂到其他地方

        # ===== 现在开始执行任务 =====
        rospy.loginfo("start task now.")
        self.arm.arm_grasp_ready()  # 移动机械臂到其他地方
        # ==== 离开起始区,避免在膨胀区域中，导致导航失败 =====
        self.robot.step_go(0.4)

        if self.stop_flag: return

        # ==== 移动机械臂 =====
        self.arm.arm_position_reset()  # 重置机械臂坐标系

        # robot_thread = threading.Thread(target=lambda: self.robot.step_go(0.4))
        # robot_thread.start()

        # arm_thread = threading.Thread(target=lambda: [self.arm.arm_position_reset(), self.arm.arm_grasp_ready()])
        # arm_thread.start()

        # robot_thread.join()
        # arm_thread.join()

        # ===== 导航到分类区 =====
        if self.robot.goto_local("Classification_area"):
            rospy.sleep(0.2)
            # 新增：解析分类板
            self.mapping = self.detect_classification_board()
        else:
            rospy.logerr("Navigation to Classification_area failed,please run task again ")
            self.stop_flag = True
        
        # 创建任务安排字典，设定前往的抓取地点与次数
        sorting_status_times = {
            "Sorting_W":2,
            "Sorting_S":2,
            "Sorting_E":2,
            "Sorting_N":2,
            "Sorting_M":2,
        }

        drop_times = {
            "Collection_B":0,
            "Collection_C":0,
            "Collection_D":0,
            "Collection_A":0,
        }

        sorting_name = "Sorting_W"
        # pre_sorting = "Sorting_W"
        at_sorting = False
        drop_zone = 'Collection_A'
        total_drop_times = 0
        # =======开始循环运行前往中心区域抓取与放置任务======
        while True:
            # 根据任务安排逐步执行
            print("readying to sort_areas")

            if total_drop_times == 10:
                break
            else:
                sorting_name = list(sorting_status_times.keys())[0]

            # =====导航到目标地点====
            if(at_sorting == False):
                if(drop_zone == "Collection_B"):
                    if "Sorting_E" in sorting_status_times:
                        if(sorting_status_times["Sorting_E"] > 0):
                            sorting_name = "Sorting_E"
                    elif "Sorting_N" in sorting_status_times:
                        if(sorting_status_times["Sorting_N"] > 0):
                            sorting_name = "Sorting_N"
                if(drop_zone == "Collection_C"):
                    if "Sorting_S" in sorting_status_times:
                        if(sorting_status_times["Sorting_S"] > 0):
                            sorting_name = "Sorting_S"
                    elif "Sorting_E" in sorting_status_times:
                        if(sorting_status_times["Sorting_E"] > 0):
                            sorting_name = "Sorting_E"
                if(drop_zone == "Collection_D"):
                    if "Sorting_W" in sorting_status_times:
                        if(sorting_status_times["Sorting_W"] > 0):
                            sorting_name = "Sorting_W"
                    elif "Sorting_S" in sorting_status_times:
                        if(sorting_status_times["Sorting_S"] > 0):
                            sorting_name = "Sorting_S"


            if(at_sorting == False):
                if(drop_zone == "Collection_B"):
                    if(sorting_name == 'Sorting_W'):
                        ret = self.robot.arriveto_local("Step_NW")
                    if(sorting_name == 'Sorting_S'):
                        ret = self.robot.arriveto_local('Step_SE')
                if(drop_zone == "Collection_C"):
                    if(sorting_name == 'Sorting_W'):
                        ret = self.robot.arriveto_local("Step_SW")
                    if(sorting_name == 'Sorting_N' or sorting_name == 'Sorting_M'):
                        ret = self.robot.arriveto_local('Step_NE')
                if(drop_zone == "Collection_D"):
                    if(sorting_name == 'Sorting_N' or sorting_name == 'Sorting_M'):
                        ret = self.robot.arriveto_local("Step_NW")
                    if(sorting_name == 'Sorting_E'):
                        ret = self.robot.arriveto_local('Step_SE')

                        
            ret = self.robot.goto_local(sorting_name) # 导航到目标点
            at_sorting =True
            rospy.sleep(1) # 停稳

            if self.stop_flag: return

            # =====识别并抓取物体====
            item_type = 0
            if ret: # 判断是否成功到达目标点
                print("========扫描中，准备抓取===== ")
                # REGULATION RULE: FIRST item (total_drop_times == 0) MUST be Cake on tier 2!
                if total_drop_times == 0:
                    rospy.loginfo(">> [RULES] Grasping 1st item: Tier 2 Cake (grasp1)")
                    item_type = self.arm.grasp1()
                else:
                    item_type = self.arm.grasp()  # 抓取物品并返回抓取物品的类型

                if self.stop_flag: return

            if item_type == 0: # 如果没有识别到物体，将该地点的抓取次数归0
                sorting_status_times[sorting_name] = 0
                continue

            
            # ====放置物品====
            self.arm.arm_grasp_ready()
            print("========前往放置区===== ")

            # REGULATION RULE: Cake MUST be delivered to Collection_A first!
            if total_drop_times == 0 or norm_id(item_type) == self.CAKE_ID:
                drop_zone = "Collection_A"
            else:
                canonical = norm_id(item_type)
                drop_zone = self.mapping.get(canonical, self.mapping.get(item_type, "Collection_B")) 
            
            

            rospy.loginfo(f"本次物品ID: {item_type} → 放置区: {drop_zone}")

            # === 用动态放置区导航 ===
            if(drop_zone == 'Collection_B'):
                if(sorting_name == 'Sorting_S'):
                    ret = self.robot.step_back(0.4)
                    rospy.sleep(0.6)
                if(sorting_name == 'Sorting_W'):
                    ret = self.robot.arriveto_local("Step_NW")
            if(drop_zone == 'Collection_C'):
                if(sorting_name == 'Sorting_N'):
                    ret = self.robot.step_back(0.4)
                    rospy.sleep(0.6)
                if(sorting_name == 'Sorting_W'):
                    ret = self.robot.arriveto_local("Step_SW")
            if(drop_zone == 'Collection_D'):
                if(sorting_name == 'Sorting_N'):
                    ret = self.robot.step_back(0.4)
                    rospy.sleep(0.6)
                if(sorting_name == 'Sorting_E'):
                    ret = self.robot.step_back(0.4)
                    rospy.sleep(0.6)

            ret = self.robot.goto_local(drop_zone)
            rospy.sleep(1)  # 停稳
            if self.stop_flag: return

            if ret:
                if drop_times[drop_zone] == 0:
                    self.arm.drop()  # 放下物品
                    drop_times[drop_zone] += 1

                elif drop_times[drop_zone] == 1:
                    self.arm.drop1()  # 放下物品
                    drop_times[drop_zone] += 1

                elif drop_times[drop_zone] == 2:
                    self.arm.drop2()  # 放下物品
                    
                if self.stop_flag: return
            else:
                rospy.logerr("task error: navigation to the drop_place fails")
                self.arm.drop()
            if self.stop_flag: return

            # 下一步
            sorting_status_times[sorting_name] = sorting_status_times[sorting_name] - 1
            if "Sorting_E" in sorting_status_times:
                if(sorting_status_times["Sorting_E"] == 0):
                    sorting_status_times.pop("Sorting_E")
                    # if "Sorting_N" in sorting_status_times:
                    #     sorting_status_times["Sorting_N"] -=1
                    # if "Sorting_S" in sorting_status_times:
                    #     sorting_status_times["Sorting_S"] -=1
            if "Sorting_W" in sorting_status_times:
                if(sorting_status_times["Sorting_W"] == 0):
                    sorting_status_times.pop("Sorting_W")
                    # if "Sorting_N" in sorting_status_times:
                    #     sorting_status_times["Sorting_N"] -=1
                    # if "Sorting_S" in sorting_status_times:
                    #     sorting_status_times["Sorting_S"] -=1
            if "Sorting_N" in sorting_status_times:
                if(sorting_status_times["Sorting_N"] == 0):
                    sorting_status_times.pop("Sorting_N")
                    # if "Sorting_W" in sorting_status_times:
                    #     sorting_status_times["Sorting_W"] -=1
                    # if "Sorting_E" in sorting_status_times:
                    #     sorting_status_times["Sorting_E"] -=1
            if "Sorting_S" in sorting_status_times:
                if(sorting_status_times["Sorting_S"] == 0):
                    sorting_status_times.pop("Sorting_S")
                    # if "Sorting_W" in sorting_status_times:
                    #     sorting_status_times["Sorting_W"] -=1
                    # if "Sorting_E" in sorting_status_times:
                    #     sorting_status_times["Sorting_E"] -=1
            at_sorting = False
            total_drop_times += 1


        self.arm.arm_home()
        # act.goto_local("sp")

        rospy.logwarn("***** task finished *****")
        rospy.logwarn("if you want to run task again. ")
        rospy.logwarn("Re-send a message to hm_task_cmd topic. ")
        rospy.logwarn("Or press Ctrl+C to exit the program")
        
    def detect_classification_board(self): 
        rospy.loginfo("开始检测分类版图案 (Temporal Voting)...")

        zones = ["Collection_B", "Collection_C", "Collection_D"]
        mapping = {}
        slot_votes = {0: [], 1: [], 2: []}
        start_time = rospy.Time.now()
        collected = 0

        while not rospy.is_shutdown() and collected < 12 and (rospy.Time.now() - start_time).to_sec() < 6.0:
            try:
                obj_array = rospy.wait_for_message("/objects", Detection2DArray, timeout=0.5)
            except Exception:
                rospy.logwarn("未检测到分类版物体，继续等待...")
                continue

            objs = []
            for obj in obj_array.detections:
                raw_id = obj.results[0].id
                item_id = norm_id(raw_id)
                # Classification board contains only 3 items: bear (1), clock (3), banana (4). Cake (2) is not on board!
                if item_id == 2 or item_id not in (1, 3, 4):
                    continue
                score = obj.results[0].score if hasattr(obj.results[0], 'score') else 1.0
                if score < 0.45:
                    continue
                item_x = obj.bbox.center.x
                objs.append((item_id, item_x))

            if len(objs) < 3:
                continue

            # Sort left to right by X coordinate
            objs.sort(key=lambda x: x[1])
            slot_votes[0].append(objs[0][0])
            slot_votes[1].append(objs[1][0])
            slot_votes[2].append(objs[2][0])
            collected += 1

        for slot in range(3):
            if slot_votes[slot]:
                winner = Counter(slot_votes[slot]).most_common(1)[0][0]
            else:
                default_order = [self.TEDDY_ID, self.CLOCK_ID, self.BANANA_ID]
                winner = default_order[slot]
            target_zone = zones[slot]
            mapping[winner] = target_zone
            # Map aliases to ensure reliable lookup matches
            if winner == self.CLOCK_ID:
                mapping[30] = target_zone
                mapping[3] = target_zone
            elif winner == self.BANANA_ID:
                mapping[40] = target_zone
                mapping[4] = target_zone
            elif winner == self.TEDDY_ID:
                mapping[1] = target_zone

        # Cake/Wine (2 / 20) ALWAYS goes to Collection_A
        mapping[self.CAKE_ID] = "Collection_A"
        mapping[2] = "Collection_A"
        mapping[20] = "Collection_A"

        rospy.loginfo(f"Classification board detection (Final voted mapping): {mapping}")
        return mapping

    def task_cmd_cb(self,flag):
        if flag :
            if not self.task_run_th.is_alive():
                self.stop_flag = False
                self.task_run_th = threading.Thread(target=self.run_task, args=())
                self.task_run_th.start()
                rospy.loginfo("start task!!!")
            else:
                rospy.logwarn("waiting for thread exit...")
                self.stop_flag = True
                self.task_run_th.join()
                rospy.logwarn("thread exit success")

    def grasp_cb(self, msg):
        rospy.loginfo(f">> [GRASP_CB] Received message on /grasp: '{msg.data}'")
        if not self.task_run_th.is_alive():
            if msg.data == "1":
                rospy.loginfo(">> [GRASP_CB] Starting cube grasp (arm.grasp)...")
                self.arm.grasp()
            elif msg.data == "0":
                rospy.loginfo(">> [GRASP_CB] Releasing cube (arm.drop)...")
                self.arm.drop()
                self.arm.arm_grasp_ready()
            else:
                rospy.logwarn(f"Unknown grasp command: '{msg.data}'")
        else:
            rospy.logwarn("Match task thread already running; /grasp command ignored")


if __name__ == '__main__':
    try:
        AutoAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Mark_move finished.")
