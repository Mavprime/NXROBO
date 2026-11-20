#!/usr/bin/python3

import math
import os
import struct
import threading
import time
from collections import Counter, deque

import numpy as np
import yaml

import rospy
import rospkg
import actionlib
from actionlib_msgs.msg import GoalID, GoalStatus
from move_base_msgs.msg import MoveBaseActionResult
import common.msg
import common.srv
from common.msg import MoveStraightDistanceAction, TurnBodyDegreeAction
import swiftpro.msg
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from swiftpro.msg import position
from vision_msgs.msg import Detection2DArray

import match_logic
from match_logic import (
    CAKE_ID, CONF_THRESHOLD, CLUSTER_DIST_PX, DEFAULT_MAPPING, MatchClock,
    MAX_TIER, TOTAL_CUBES, ZONES_BCD, decode_classification_board,
    expand_aliases, ik_reachable, norm_id, parse_task_command,
)

# --------------------------------------------------------------------------
# Robot-specific constants (everything rule-related lives in match_logic.py)
# --------------------------------------------------------------------------

#: Frames of ``/objects`` used for the grasp-grade majority vote. The detector
#: runs two YOLO models and publishes as fast as the NUC allows, so a four
#: frame window is ~0.3-1 s and still removes one-frame ghosts.
GRASP_FRAMES = 4
#: Frames used while reading the classification board (slow, it can wait).
BOARD_FRAMES = 12
#: Give up on a station after this many empty scans (never spin forever).
MAX_EMPTY_ATTEMPTS = 2
#: Picks the mission plans per sorting station.
STATION_PICKS = 2
#: Sorting station that holds the tier-2 cake pedestal. Verify on the arena.
CAKE_SORTING_STATION = "Sorting_M"
#: Match budget: drop dead at 540 s so the 600 s match is never exceeded.
TASK_TIME_BUDGET_S = 540.0
#: Pause after navigation before scanning, so the chassis stops vibrating.
SETTLE_S = 1.0

ARM_TUCK_POSE = (50.0, 180.0, 160.0)

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
        # Load the competition item configuration.  A missing file is reported
        # but no longer fatal: the FSM works on canonical ids 1..4, and the old
        # sys.exit() killed the node halfway through startup.
        self.search_id = []
        try:
            package_path = rospkg.RosPack().get_path('auto_match')
            items_path = os.path.join(package_path, 'config', 'items_config.yaml')
            with open(items_path, "r", encoding="utf8") as f:
                items_content = yaml.load(f.read(), Loader=yaml.FullLoader)
            objects = items_content["objects"]
            self.search_id = [items_content["items"][objects[name]] for name in
                              ("objects_a", "objects_b", "objects_c", "objects_d")]
            rospy.loginfo(f">> [VISION] items_config.yaml loaded, item ids: {self.search_id}")
        except Exception as e:
            rospy.logwarn(f">> [VISION] items_config.yaml unavailable ({e}); "
                          f"working with canonical ids 1..4")

        # Rolling buffer of (monotonic timestamp, frame) for temporal voting.
        # A frame is a list of (raw_id, score, cx, cy) tuples.
        self._frames = deque(maxlen=max(2, GRASP_FRAMES))
        self._lock = threading.Lock()
        self.sub = rospy.Subscriber(
            "/objects", Detection2DArray, self._objects_cb, queue_size=5)

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
        '''Buffer one detection frame (possibly empty) for the voting window.'''
        frame = []
        for obj in msg.detections:
            try:
                result = obj.results[0]
                frame.append((result.id,
                              float(getattr(result, 'score', 1.0) or 0.0),
                              float(obj.bbox.center.x),
                              float(obj.bbox.center.y)))
            except Exception:
                continue
        with self._lock:
            self._frames.append((time.monotonic(), frame))

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

    def detector(self, num_samples=None, timeout=3.0):
        '''
        Majority-voted, spatially clustered cube list.

        Waits until ``num_samples`` detection frames have arrived *after this
        call* (so a re-scan after a base rotation never uses pre-rotation
        frames), then lets match_logic.cluster_detections() drop one-frame
        ghosts, merge the two YOLO models reporting the same cube, and vote on
        the class.

        @return: list of [[canonical_id, [x, y, 0]], ...]
        '''
        wanted = max(1, int(GRASP_FRAMES if num_samples is None else num_samples))
        started = time.monotonic()
        deadline = started + max(0.0, float(timeout))
        frames = []

        while time.monotonic() < deadline:
            with self._lock:
                frames = [frame for stamp, frame in list(self._frames)
                          if stamp >= started]
            if len(frames) >= wanted:
                break
            rospy.sleep(0.05)

        if not frames:
            try:
                msg = rospy.wait_for_message(
                    "/objects", Detection2DArray, timeout=timeout)
            except Exception as e:
                rospy.logerr(f">> [DETECTOR] No message on /objects ({e}). "
                             f"Check detect_ros.py and the camera driver "
                             f"(dashboard option 1).")
                return []
            self._objects_cb(msg)
            with self._lock:
                frames = [frame for stamp, frame in list(self._frames)
                          if stamp >= started]

        frames = frames[-wanted:]
        cubes = match_logic.cluster_detections(
            frames, conf_threshold=CONF_THRESHOLD,
            cluster_dist_px=CLUSTER_DIST_PX)
        rospy.loginfo_throttle(
            1.0, f">> [VISION] {len(cubes)} cube(s) from {len(frames)} frame(s): {cubes}")
        return cubes

class ArmAction:
    def __init__(self, cam=None):
        # One camera node per process: runtask.py used to create a second
        # CamAction here, doubling the /objects subscriber and the buffers.
        self.cam = cam if cam is not None else CamAction()

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

    def move_z_steps(self, x, y, z_from, z_to, steps=1, delay=0.8):
        '''
        Move vertically to target height. If steps <= 1, commands a single
        smooth G0 motion. Multiple steps are kept as an optional fallback.
        '''
        steps = max(1, int(steps))
        if steps <= 1:
            self.interface.set_pose(x, y, z_to, delay=delay)
            return
        for i in range(1, steps + 1):
            z = z_from + (z_to - z_from) * (float(i) / steps)
            self.interface.set_pose(x, y, z, delay=delay)

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

        # Safe displacement clamp (max 25 cm per step)
        dist_m = max(-0.25, min(0.25, dist_m))
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

        if not cube_list:
            rospy.logwarn(">> [GRASP] Aborted: No target cubes detected in camera view.")
            self.grasp_status_pub.publish(String("1"))
            return 0

        target_cube = match_logic.select_grasp_candidate(cube_list)
        target_id = target_cube[0]
        initial_target_id = target_id
        cu = target_cube[1][0]
        cv = target_cube[1][1]

        # Geometry for floor cube: top=-50 mm, cube height=100 mm -> center=-100 mm
        z_center = -100.0
        z_touch = -50.0
        z_above = -15.0
        z_lift = 60.0

        xy = match_logic.pixel_to_xy_at_z(self.cam.transform3d, cu, cv, z_center)
        if xy is None:
            rospy.logerr(">> [GRASP] Degenerate projective mapping.")
            self.grasp_status_pub.publish(String("1"))
            return 0
        x = xy[0] + 20.0  # +20 mm cup center correction
        y = xy[1]

        # 2. Check: is cube in comfortable reach of the arm?
        # Comfortable sweet-spot envelope: arm is not strained, lands dead center on cube
        TARGET_X = 245.0
        dist_to_cube = math.hypot(x, y)
        in_reach = (160.0 <= x <= 265.0) and (abs(y) <= 85.0) and (dist_to_cube <= 270.0)
        rospy.loginfo(f">> [GRASP] Target ID {target_id}: px=({cu:.0f}, {cv:.0f}) -> x={x:.1f}, y={y:.1f}, dist={dist_to_cube:.1f} | in_reach={in_reach}")

        approach_step = 0
        while approach_step < 3 and not in_reach:
            approach_step += 1
            rospy.loginfo(f">> [GRASP] Cube outside sweet spot (x={x:.1f}, y={y:.1f}, dist={dist_to_cube:.1f}). Approach step #{approach_step}...")

            # (a) Turn base towards cube center
            angle_rad = math.atan2(y, x)
            if abs(angle_rad) > math.radians(2.0):
                rospy.loginfo(f">> [GRASP] Turning base towards cube: {math.degrees(angle_rad):.1f} deg...")
                self.turn_base(angle_rad)
                rospy.sleep(0.3)

            # (b) Drive forward to bring cube into sweet spot X = 245 mm
            drive_m = (dist_to_cube - TARGET_X) / 1000.0
            if abs(drive_m) >= 0.02:
                step_m = max(-0.25, min(0.25, drive_m))
                rospy.loginfo(f">> [GRASP] Driving {step_m*100:.1f} cm to position cube at 24.5 cm...")
                self.drive_base(step_m)
                rospy.sleep(0.4)

            # (c) ЗАТЕМ: запустить алгоритм сканирования позиции кубика еще раз!
            rospy.loginfo(f">> [GRASP] Re-scanning cube position after approach step #{approach_step}...")
            rescan_list = self.cam.detector(timeout=3.0)
            if rescan_list:
                target_cube = match_logic.select_grasp_candidate(
                    rescan_list, preferred_id=initial_target_id)
                target_id = target_cube[0]
                cu = target_cube[1][0]
                cv = target_cube[1][1]
                xy = match_logic.pixel_to_xy_at_z(self.cam.transform3d, cu, cv, z_center)
                if xy is not None:
                    x = xy[0] + 20.0
                    y = xy[1]
                    dist_to_cube = math.hypot(x, y)
                    in_reach = (160.0 <= x <= 265.0) and (abs(y) <= 85.0) and (dist_to_cube <= 270.0)
                    rospy.loginfo(f">> [GRASP] Re-scanned target: px=({cu:.0f}, {cv:.0f}) -> x={x:.1f}, y={y:.1f}, dist={dist_to_cube:.1f} | in_reach={in_reach}")
            else:
                break

        # 4. Execute physical grasp
        x = max(120.0, min(320.0, x))
        y = max(-190.0, min(190.0, y))
        rospy.loginfo(f">> [GRASP] Executing grasp for ID {target_id}: x={x:.1f}, y={y:.1f}, z_touch={z_touch:.1f}, z_lift={z_lift:.1f}")

        self.interface.set_status(True)
        rospy.sleep(0.20)

        # Move above cube at safe height
        self.interface.set_pose(x, y, z_above, delay=0.9)

        # Enable vacuum pump early for fast suction
        self.interface.set_pump(True)
        rospy.sleep(0.10)

        # Single smooth descent straight down to the cube surface
        self.interface.set_pose(x, y, z_touch, delay=0.8)

        # Vacuum dwell time for reliable suction seal
        rospy.sleep(0.40)

        # Single smooth vertical lift
        self.interface.set_pose(x, y, z_lift, delay=0.8)
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

        # Prefer an actual cake: this routine is the cake-first rule's only
        # chance, and a ground cube can easily be in the same frame.
        cake_visible = any(match_logic.is_cake(cube[0]) for cube in cube_list)
        target_cube = match_logic.select_grasp_candidate(
            cube_list, preferred_id=CAKE_ID if cake_visible else None, y_ref=250.0)
        target_id = target_cube[0]
        initial_target_id = target_id
        u = target_cube[1][0]
        v = target_cube[1][1]
        if not match_logic.is_cake(target_id):
            rospy.logwarn(
                f">> [GRASP1] No cake class in view; best candidate is id "
                f"{target_id}. Rule-critical: verify CAKE_SORTING_STATION "
                f"({CAKE_SORTING_STATION}) and the cake detector class.")

        # Geometry for Layer 2 Cake: sits on Layer 1 (top at -50 mm).
        # Cake top surface is at z_touch ~ 50.0 mm; cube height ~ 100 mm -> center is at z_center = 0.0 mm
        z_center = 0.0
        z_touch = 50.0
        z_safe = 100.0   # 150 mm above ground cube tops (-50 mm) - clears any Layer 1 obstacles!
        z_lift = 150.0

        xy = match_logic.pixel_to_xy_at_z(self.cam.transform3d, u, v, z_center)
        if xy is None:
            rospy.logerr(">> [GRASP1] Degenerate projective mapping.")
            self.grasp_status_pub.publish(String("1"))
            return 0
        x = xy[0] + 20.0  # +20 mm cup center correction
        y = xy[1]

        # Comfortable sweet-spot envelope
        TARGET_X = 245.0
        dist_to_cake = math.hypot(x, y)
        in_reach = (160.0 <= x <= 265.0) and (abs(y) <= 85.0) and (dist_to_cake <= 270.0)
        rospy.loginfo(f">> [GRASP1] Target ID {target_id}: u={u:.0f}, v={v:.0f} -> x={x:.1f}, y={y:.1f}, dist={dist_to_cake:.1f} | in_reach={in_reach}")

        approach_step = 0
        while approach_step < 3 and not in_reach:
            approach_step += 1
            rospy.loginfo(f">> [GRASP1] Cake outside sweet spot (x={x:.1f}, y={y:.1f}, dist={dist_to_cake:.1f}). Approach step #{approach_step}...")

            angle_rad = math.atan2(y, x)
            if abs(angle_rad) > math.radians(2.0):
                rospy.loginfo(f">> [GRASP1] Turning base towards cake: {math.degrees(angle_rad):.1f} deg...")
                self.turn_base(angle_rad)
                rospy.sleep(0.3)

            drive_m = (dist_to_cake - TARGET_X) / 1000.0
            if abs(drive_m) >= 0.02:
                step_m = max(-0.25, min(0.25, drive_m))
                rospy.loginfo(f">> [GRASP1] Driving {step_m*100:.1f} cm to position cake at 24.5 cm...")
                self.drive_base(step_m)
                rospy.sleep(0.4)

            # CRITICAL: Re-scan cake position after approach!
            rospy.loginfo(f">> [GRASP1] Re-scanning cake position after approach step #{approach_step}...")
            rescan_list = self.cam.detector(timeout=3.0)
            if rescan_list:
                target_cube = match_logic.select_grasp_candidate(
                    rescan_list, preferred_id=initial_target_id, y_ref=250.0)
                target_id = target_cube[0]
                u = target_cube[1][0]
                v = target_cube[1][1]
                xy = match_logic.pixel_to_xy_at_z(self.cam.transform3d, u, v, z_center)
                if xy is not None:
                    x = xy[0] + 20.0
                    y = xy[1]
                    dist_to_cake = math.hypot(x, y)
                    in_reach = (160.0 <= x <= 265.0) and (abs(y) <= 85.0) and (dist_to_cake <= 270.0)
                    rospy.loginfo(f">> [GRASP1] Re-scanned cake target: u={u:.0f}, v={v:.0f} -> x={x:.1f}, y={y:.1f}, dist={dist_to_cake:.1f} | in_reach={in_reach}")
            else:
                break

        x = max(120.0, min(320.0, x))
        y = max(-190.0, min(190.0, y))
        rospy.loginfo(f">> [GRASP1] Executing grasp for Cake ID {target_id}: x={x:.1f}, y={y:.1f}, z_touch={z_touch:.1f}, z_lift={z_lift:.1f}")

        self.interface.set_status(True)
        rospy.sleep(0.20)

        # Move above cake at safe height (z_safe=100 mm clears any ground cube in front)
        self.interface.set_pose(x, y, z_safe, delay=0.9)

        self.interface.set_pump(True)
        rospy.sleep(0.10)
        self.interface.set_pose(x, y, z_touch, delay=0.9)
        rospy.sleep(0.40)

        print("Layer 2 Cake lifted successfully")
        self.interface.set_pose(x, y, z_lift, delay=0.8)
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

    def _tower_xy(self, x_kb, y_kb, fallback=(235.0, 0.0)):
        '''
        (x, y) of the cube already standing in the drop zone, used to align the
        stacking tower.  Falls back to the calibrated default point when the
        camera sees nothing (SI-12: never index an empty detection list).
        '''
        cube = match_logic.select_grasp_candidate(self.cam.detector(timeout=3.0))
        if cube is None:
            rospy.logwarn(">> [DROP] No cube visible in the zone; "
                          "stacking on the default point.")
            return fallback
        u, v = cube[1][0], cube[1][1]
        # row (v) drives arm x, column (u) drives arm y - see arm_ik.py.
        return match_logic.pixel_to_arm(v, u, x_kb, y_kb)

    def drop1(self, check=False):
        '''
        Place block, layer 2 height
        '''
        x, y = self._tower_xy(self.x_kb, self.y_kb)
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
        x, y = self._tower_xy(self.x_kb1, self.y_kb1)
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
        rospy.init_node('spark_auto_match_node', anonymous=True)

        print("========ready to task===== ")

        # Build the three subsystems one by one and record failures instead of
        # swallowing them: a half-built node used to crash later with a bare
        # AttributeError, which is impossible to diagnose on the field.
        self.cam = None
        self.arm = None
        self.robot = None
        self.init_errors = []

        try:
            self.cam = CamAction()
            print("========实例化Cam===== ")
        except Exception as e:
            self.init_errors.append(f"camera: {e}")
            rospy.logerr(f">> [INIT] CamAction failed: {e}")

        try:
            if self.cam is None:
                raise RuntimeError("camera subsystem unavailable")
            self.arm = ArmAction(cam=self.cam)
            print("========实例化Arm===== ")
        except Exception as e:
            self.init_errors.append(f"arm: {e}")
            rospy.logerr(f">> [INIT] ArmAction failed: {e}")

        try:
            self.robot = RobotMoveAction()
            print("========实例化Robot===== ")
        except Exception as e:
            self.init_errors.append(f"robot: {e}")
            rospy.logerr(f">> [INIT] RobotMoveAction failed: {e}")

        #: True only when the whole chain is up; the FSM refuses to start otherwise.
        self.ready = not self.init_errors

        # Item ID mapping (rule-compliant normalization lives in match_logic).
        self.BANANA_ID = match_logic.BANANA_ID
        self.TEDDY_ID = match_logic.TEDDY_ID
        self.CLOCK_ID = match_logic.CLOCK_ID
        self.CAKE_ID = CAKE_ID
        self.CUP_ID = match_logic.CAKE_ID  # alias for backwards compatibility

        # Zone mapping; always defined so no code path can hit AttributeError.
        self.mapping = expand_aliases(dict(DEFAULT_MAPPING))

        # 订阅任务控制指令的话题
        self.task_cmd_sub = rospy.Subscriber("/task_start_flag", String, self.task_cmd_cb)
        self.task_run_th = threading.Thread(target=lambda: "pass")
        self.stop_flag = False  # 任务的启停标志

        # Subscribe to manual arm control (/grasp)
        self.grasp_sub = rospy.Subscriber("/grasp", String, self.grasp_cb)

        rospy.loginfo("==============================================")
        if self.ready:
            rospy.loginfo("  spark_auto_match_node is ready!             ")
            rospy.loginfo("  /task_start_flag start|stop   /grasp 1|0     ")
        else:
            rospy.logerr("  spark_auto_match_node is NOT ready:          ")
            for problem in self.init_errors:
                rospy.logerr(f"    - {problem}")
            rospy.logerr("  Fix the subsystems above; the match will not start.")
        rospy.loginfo("==============================================")

    # ------------------------------------------------------------------
    # Mission helpers
    # ------------------------------------------------------------------
    def _abort_requested(self, clock=None):
        """True when the operator stopped the run or the match budget is gone."""
        if self.stop_flag:
            rospy.logwarn(">> [TASK] stop requested by the operator")
            return True
        if clock is not None:
            if clock.warn():
                rospy.logwarn(f">> [TASK] only {clock.remaining():.0f} s of the "
                              f"time budget left")
            if clock.expired():
                rospy.logerr(f">> [TASK] time budget exhausted after "
                             f"{clock.elapsed():.0f} s; stopping before overtime")
                return True
        return False

    def _navigate(self, name):
        """goto_local with one retry and a diagnostic hint when nav is absent."""
        if self.robot.goto_local(name):
            return True
        rospy.logwarn(f">> [TASK] navigation to {name} failed, retrying once")
        if self.robot.goto_local(name):
            return True
        rospy.logerr(f">> [TASK] navigation to {name} failed twice. Is the "
                     f"navigation stack running? start_system.sh only launches "
                     f"the drivers; /move_base and mark_nav must come from the "
                     f"arena navigation launch file (Phase 2).")
        return False

    def _execute_exit_maneuver(self, station, drop_zone):
        """Leave the station without dragging the cube through the centre board."""
        maneuver = match_logic.exit_maneuver(station, drop_zone)
        if not maneuver:
            return
        kind, value = maneuver
        if kind == "back":
            self.robot.step_back(value)
            rospy.sleep(0.6)
        elif kind == "arrive":
            self.robot.arriveto_local(value)
            rospy.sleep(0.6)

    def _deliver(self, item_id, station, drop_times, force_zone=None):
        """Drive to the mapped zone and stack the held cube; returns the zone."""
        zone = force_zone
        if zone is None or drop_times.get(zone, 0) >= MAX_TIER:
            zone = match_logic.choose_drop_zone(item_id, self.mapping, drop_times)
        if zone is None:
            rospy.logerr(">> [DROP] every zone already holds a 3-cube tower; "
                         "releasing the cube here so the arm is free")
            self.arm.drop()
            return None

        self._execute_exit_maneuver(station, zone)
        self.arm.arm_grasp_ready()

        if not self._navigate(zone):
            rospy.logerr(f">> [DROP] releasing the cube at the current position "
                         f"instead of {zone}")
            self.arm.drop()
            return None

        rospy.sleep(SETTLE_S)
        routine = match_logic.plan_drop(drop_times.get(zone, 0)) or "drop"
        getattr(self.arm, routine)()
        drop_times[zone] = drop_times.get(zone, 0) + 1
        rospy.loginfo(f">> [DROP] item {item_id} -> {zone}, "
                      f"tier {drop_times[zone]} via {routine}()")
        return zone

    def _finish(self, clock, drop_times, total_drop_times, cake_delivered):
        """Park the arm and print a match summary that can be scored by hand."""
        self.arm.arm_home()
        estimate = match_logic.estimate_score(
            drop_times, red_zones=match_logic.RED_ZONES, started_from_base=True)
        rospy.logwarn("***** task finished *****")
        rospy.logwarn(f"  cubes delivered: {total_drop_times} / {TOTAL_CUBES}")
        rospy.logwarn(f"  tiers per zone:  {drop_times}")
        rospy.logwarn(f"  estimated score: {estimate}")
        if not cake_delivered:
            rospy.logwarn("  NOTE: the cake-first rule was not satisfied, so the "
                          "real score is lower than this estimate.")
        rospy.logwarn(f"  elapsed: {clock.elapsed():.0f} s of a "
                      f"{clock.limit_s:.0f} s budget")
        rospy.logwarn("Re-send 'start' on /task_start_flag to run again, or Ctrl+C.")

    # 接收到启动自动赛信号，开始执行任务
    def run_task(self):
        """Full 10-cube mission: cake first, then board-driven sorting."""
        if not self.ready:
            rospy.logerr(">> [TASK] refusing to start: " + "; ".join(self.init_errors))
            return

        clock = MatchClock(limit_s=TASK_TIME_BUDGET_S,
                           warn_s=match_logic.TIME_WARN_S).start()
        self.stop_flag = False
        self.mapping = expand_aliases(dict(DEFAULT_MAPPING))

        stations = {name: STATION_PICKS for name in match_logic.STATION_ORDER}
        drop_times = {zone: 0 for zone in ("Collection_A",) + ZONES_BCD}
        empty_attempts = {name: 0 for name in match_logic.STATION_ORDER}
        total_drop_times = 0
        cake_delivered = False
        last_zone = "Collection_A"

        rospy.loginfo("start task now.")
        self.arm.arm_grasp_ready()
        # ==== 离开起始区,避免在膨胀区域中，导致导航失败 =====
        self.robot.step_go(0.4)
        if self._abort_requested():
            return
        self.arm.arm_position_reset()  # 重置机械臂坐标系

        # ===== classification board (SI-14: fall back instead of aborting) =====
        if self._navigate("Classification_area"):
            rospy.sleep(0.2)
            self.mapping = self.detect_classification_board()
        else:
            rospy.logerr(">> [TASK] board area unreachable: running with the "
                         "DEFAULT mapping (cake->A, bear->B, clock->C, banana->D)")

        # ===== cake rush: the first cube of the match MUST be the cake =====
        if not self._abort_requested(clock):
            rospy.loginfo(f">> [RULES] first delivery from {CAKE_SORTING_STATION} "
                          f"goes to Collection_A")
            if self._navigate(CAKE_SORTING_STATION):
                rospy.sleep(SETTLE_S)
                first_id = self.arm.grasp1()
                if first_id == 0:
                    rospy.logerr(">> [RULES] no cube seen at the cake station: "
                                 "the cake-first rule cannot be guaranteed")
                else:
                    cake_delivered = match_logic.is_cake(first_id)
                    if not cake_delivered:
                        rospy.logerr(f">> [RULES] first grasp was item {first_id}, "
                                     f"not the cake: cake points are forfeited")
                    if self._deliver(first_id, CAKE_SORTING_STATION, drop_times,
                                     force_zone="Collection_A"):
                        total_drop_times += 1
                        last_zone = "Collection_A"
                        stations[CAKE_SORTING_STATION] = max(
                            0, stations.get(CAKE_SORTING_STATION, 1) - 1)
                        if stations.get(CAKE_SORTING_STATION, 0) <= 0:
                            match_logic.retire_station(stations,
                                                       CAKE_SORTING_STATION)
            else:
                rospy.logerr(f">> [RULES] cake station {CAKE_SORTING_STATION} "
                             f"unreachable; continuing with the sorting loop")

        # ===== sorting loop =====
        while not self._abort_requested(clock):
            if total_drop_times >= TOTAL_CUBES:
                break
            if not stations:
                rospy.logwarn(">> [TASK] every station is retired; finishing early")
                break

            station = match_logic.choose_station(last_zone, stations)
            if station is None:
                break
            rospy.loginfo(f">> [TASK] cycle {total_drop_times + 1}/{TOTAL_CUBES}: "
                          f"station {station} "
                          f"({stations.get(station, 0)} pick(s) left)")

            waypoint = match_logic.approach_waypoint(station, last_zone)
            if waypoint:
                self.robot.arriveto_local(waypoint)

            if not self._navigate(station):
                match_logic.retire_station(stations, station)
                rospy.logerr(f">> [TASK] {station} unreachable; retired")
                continue

            rospy.sleep(SETTLE_S)
            # Floor routine everywhere in the loop: the tier-2 cake is handled
            # once, up front, by the cake rush.
            item_id = self.arm.grasp()
            if item_id == 0:
                empty_attempts[station] = empty_attempts.get(station, 0) + 1
                rospy.logwarn(f">> [TASK] nothing grasped at {station} "
                              f"({empty_attempts[station]}/{MAX_EMPTY_ATTEMPTS})")
                if empty_attempts[station] >= MAX_EMPTY_ATTEMPTS:
                    match_logic.retire_station(stations, station)
                    rospy.logerr(f">> [TASK] {station} retired after "
                                 f"{MAX_EMPTY_ATTEMPTS} empty scans")
                continue

            if self._abort_requested(clock):
                rospy.logwarn(">> [TASK] abort while holding a cube: parking it here")
                self.arm.drop()
                break

            delivered_zone = self._deliver(item_id, station, drop_times)
            if delivered_zone:
                last_zone = delivered_zone
            total_drop_times += 1
            empty_attempts[station] = 0
            stations[station] = max(0, stations.get(station, 1) - 1)
            if stations.get(station, 0) <= 0:
                match_logic.retire_station(stations, station)

        self._finish(clock, drop_times, total_drop_times, cake_delivered)

    def detect_classification_board(self):
        '''
        Read the three-slot classification board and build the zone mapping.

        Slots are voted over up to BOARD_FRAMES frames (left to right in image
        order).  The vote result goes through
        match_logic.decode_classification_board(), which keeps the DEFAULT
        mapping when the board was not read cleanly (SI-14: never leave the
        mission without a mapping).
        '''
        rospy.loginfo(">> [BOARD] reading the classification board ...")
        slot_votes = {0: [], 1: [], 2: []}
        collected = 0
        deadline = rospy.Time.now() + rospy.Duration(6.0)
        slots = len(ZONES_BCD)

        while (not rospy.is_shutdown() and collected < BOARD_FRAMES
               and rospy.Time.now() < deadline):
            try:
                obj_array = rospy.wait_for_message(
                    "/objects", Detection2DArray, timeout=0.5)
            except Exception:
                rospy.logwarn_throttle(2.0, ">> [BOARD] no /objects message, waiting ...")
                continue

            objs = []
            for obj in obj_array.detections:
                try:
                    result = obj.results[0]
                    item_id = norm_id(result.id)
                    score = float(getattr(result, 'score', 1.0) or 0.0)
                except Exception:
                    continue
                # The board carries only the bear/clock/banana cards.
                if item_id not in match_logic.BOARD_ITEMS or score < CONF_THRESHOLD:
                    continue
                objs.append((item_id, obj.bbox.center.x))

            if len(objs) < slots:
                continue

            objs.sort(key=lambda pair: pair[1])
            for slot in range(slots):
                slot_votes[slot].append(objs[slot][0])
            collected += 1

        winners = []
        for slot in range(slots):
            votes = slot_votes[slot]
            winners.append(Counter(votes).most_common(1)[0][0] if votes else None)

        mapping = decode_classification_board(winners)
        if match_logic.board_is_confident(winners):
            rospy.loginfo(f">> [BOARD] {collected} frame(s) voted; mapping = {mapping}")
        else:
            rospy.logerr(f">> [BOARD] board not read cleanly (winners={winners}, "
                         f"{collected} frame(s)); using DEFAULT mapping {DEFAULT_MAPPING}")
        return mapping

    def task_cmd_cb(self, msg):
        '''Start/stop the mission from /task_start_flag, honouring the payload.'''
        command = parse_task_command(getattr(msg, 'data', ''))
        if command == "start":
            if not self.ready:
                rospy.logerr(">> [TASK] cannot start: " + "; ".join(self.init_errors))
                return
            if self.task_run_th.is_alive():
                rospy.logwarn(">> [TASK] a mission is already running; send 'stop' first")
                return
            self.stop_flag = False
            self.task_run_th = threading.Thread(
                target=self.run_task, args=(), daemon=True)
            self.task_run_th.start()
            rospy.loginfo(">> [TASK] mission started by /task_start_flag")
        elif command == "stop":
            if self.task_run_th.is_alive():
                self.stop_flag = True
                rospy.logwarn(">> [TASK] stop requested; the mission exits at its "
                              "next checkpoint (the thread is not joined because it "
                              "may be mid-motion)")
            else:
                rospy.loginfo(">> [TASK] nothing to stop")
        else:
            rospy.logwarn(f">> [TASK] ignoring unknown /task_start_flag payload "
                          f"'{getattr(msg, 'data', '')}' (use 'start' or 'stop')")

    def grasp_cb(self, msg):
        rospy.loginfo(f">> [GRASP_CB] Received message on /grasp: '{msg.data}'")
        if not self.ready:
            rospy.logerr(">> [GRASP_CB] manual grasp unavailable: "
                         + "; ".join(self.init_errors))
            return
        if self.task_run_th.is_alive():
            rospy.logwarn("Match task thread already running; /grasp command ignored")
            return
        if msg.data == "1":
            rospy.loginfo(">> [GRASP_CB] Starting cube grasp (arm.grasp)...")
            self.arm.grasp()
        elif msg.data == "0":
            rospy.loginfo(">> [GRASP_CB] Releasing cube (arm.drop)...")
            self.arm.drop()
            self.arm.arm_grasp_ready()
        else:
            rospy.logwarn(f"Unknown grasp command: '{msg.data}'")


if __name__ == '__main__':
    try:
        AutoAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Mark_move finished.")
