#!/usr/bin/env python3
"""
Keyboard control for the Spark robot (base + uArm + vacuum pump).

Base controls:
    w / s      forward / backward (or Up / Down arrows)
    a / d      turn left / right  (or Left / Right arrows)
    q / e      arc turn forward-left / forward-right
    (combined: hold w/s + steer with a/d for smooth arc turns)
    Space      EMERGENCY STOP (instantly stops base & halts arm in place; preserves pump)
    f / Tab / m toggle speed mode (FAST: arena drive / SLOW: fine box manipulation)

Arm controls (step-jogging on keypress/hold):
    8 / 2      up / down (z)
    7 / 9      forward / backward (x)
    4 / 6      swing CCW / CW around base (y)
    1 / 3      wrist CCW / CW (end-effector spin)

Arm presets (direct smooth single-target moves):
    v          Hover / Safe Height (x=280, y=0, z=30, angle=0)
    t          Touch / Pick Floor  (x=280, y=0, z=-50, angle=0)
    c          Carry / Lift High   (x=230, y=0, z=90, angle=0)

Pump, Motors & System:
    Enter      toggle vacuum pump: grab / release
    r          relax / lock arm motors (syncs pose from encoders on lock)
    0          release box (pump off), stop base, and return arm to home pose
    Ctrl+C     stop everything and exit

Topics:
    /cmd_vel                geometry_msgs/Twist       base motion
    /position_write_topic   swiftpro/position         arm x,y,z in mm (absolute)
    /angle4th_topic         swiftpro/angle4th         wrist angle in degrees
    /pump_topic             swiftpro/status           vacuum 1/0
    /swiftpro_status_topic  swiftpro/status           arm motors lock/unlock (1/0)
    /SwiftproState_topic    swiftpro/SwiftproState    current pose feedback
"""

import os
import select
import sys
import termios
import time
import tty

# Автоматическое подключение окружения ROS Noetic, если скрипт запущен напрямую без source
for _ros_path in [
    '/opt/ros/noetic/lib/python3/dist-packages',
    os.path.expanduser('~/spark_noetic/devel/lib/python3/dist-packages'),
]:
    if os.path.exists(_ros_path) and _ros_path not in sys.path:
        sys.path.insert(0, _ros_path)

import rospy
from geometry_msgs.msg import Twist
from swiftpro.msg import SwiftproState, angle4th, position, status

# Speed profiles: FAST for field transit, SLOW for fine manipulation
SPEED_MODES = {
    "FAST": {
        "lin_speed": float(os.environ.get("LIN_SPEED_FAST", "0.45")),     # m/s
        "ang_speed": float(os.environ.get("ANG_SPEED_FAST", "0.75")),     # rad/s
        "arm_step": float(os.environ.get("ARM_STEP_FAST", "30.0")),       # mm per jog step
        "wrist_step": float(os.environ.get("WRIST_STEP_FAST", "12.0")),   # deg per jog step
    },
    "SLOW": {
        "lin_speed": float(os.environ.get("LIN_SPEED_SLOW", "0.07")),     # m/s
        "ang_speed": float(os.environ.get("ANG_SPEED_SLOW", "0.25")),     # rad/s
        "arm_step": float(os.environ.get("ARM_STEP_SLOW", "4.0")),        # mm per jog step (precision)
        "wrist_step": float(os.environ.get("WRIST_STEP_SLOW", "5.0")),    # deg per jog step
    },
}

# Responsive acceleration and fast braking limits for base
MAX_LIN_ACCEL = float(os.environ.get("MAX_LIN_ACCEL", "3.0"))   # m/s^2 (forward/backward accel)
MAX_LIN_DECEL = float(os.environ.get("MAX_LIN_DECEL", "5.0"))   # m/s^2 (crisp instant braking)
MAX_ANG_ACCEL = float(os.environ.get("MAX_ANG_ACCEL", "3.0"))   # rad/s^2 (turning accel)
MAX_ANG_DECEL = float(os.environ.get("MAX_ANG_DECEL", "8.0"))   # rad/s^2 (turning braking)

# Configurable timeouts (used directly in main control loop)
KEY_TIMEOUT = float(os.environ.get("KEY_TIMEOUT", "0.30"))             # s (straight driving timeout)
KEY_GRACE_TIMEOUT = float(os.environ.get("KEY_GRACE_TIMEOUT", "0.80")) # s (grace timeout after steering)
STEER_TIMEOUT = float(os.environ.get("STEER_TIMEOUT", "0.30"))         # s (steering auto-center timeout)

HOME = (120.0, 0.0, 35.0)
HOME_ANGLE = 0.0
WRIST_LIMITS = (0.0, 180.0)

# Protective barrier limits:
# - Z >= -85.0mm prevents driving into the table while allowing solid cube grip (-80.0mm)
# - X in [90.0, 320.0] prevents colliding with LiDAR / RealSense or over-extending cables
# - Y in [-160.0, 160.0] allows full left/right workspace swing
SAFE_LIMITS = {
    "x": (90.0, 320.0),
    "y": (-160.0, 160.0),
    "z": (-85.0, 180.0),
}

ARM_PRESETS = {
    "v": ("Hover / Safe Height", (280.0, 0.0, 30.0), 0.0),
    "t": ("Touch / Pick Floor", (280.0, 0.0, -50.0), 0.0),
    "c": ("Carry / Lift High", (230.0, 0.0, 90.0), 0.0),
}

AXIS_CONFIG = {
    "8": ("z", 1),
    "2": ("z", -1),
    "7": ("x", 1),
    "9": ("x", -1),
    "4": ("y", 1),
    "6": ("y", -1),
    "1": ("angle", 1),
    "3": ("angle", -1),
}

ARROW_MAP = {
    "A": "w",
    "B": "s",
    "C": "d",
    "D": "a",
}

HELP = """
  === Spark Keyboard Teleop ===
  Base:
    w/s: forward / backward     a/d: turn left / right (or Arrow keys)
    q/e: arc turn forward-left / forward-right (single-key smooth arc!)
    (combined: while driving w/s, tap/hold a/d to curve in an arc!)
    Space: EMERGENCY STOP       f / Tab / m: toggle speed mode (FAST / SLOW)

  Arm Jogging (smooth jog step on tap/hold):
    8 / 2: up / down (z)        7 / 9: forward / backward (x)
    4 / 6: swing CCW / CW (y)   1 / 3: wrist CCW / CW

  Arm Presets:
    v: Hover / Safe Height (280, 0, 30)    t: Touch / Pick Floor (280, 0, -50)
    c: Carry / Lift High (230, 0, 90)

  Pump, Motors & Safety:
    Enter: grab / release (pump toggle)   r: relax / lock motors (free-hand align)
    0: release box + park home            Ctrl+C: stop everything and exit
"""


def safe_print(*args, **kwargs):
    """Print with carriage returns and clear HUD line in terminal raw mode."""
    sep = kwargs.get("sep", " ")
    end = kwargs.get("end", "\r\n")
    text = sep.join(str(a) for a in args)
    text = text.replace("\r\n", "\n").replace("\n", "\r\n")
    sys.stdout.write(f"\r\033[K{text}{end}")
    sys.stdout.flush()


def clamp(value, limits):
    return min(max(value, limits[0]), limits[1])


def smooth_velocity(current, target, accel, decel, dt):
    """Smooth velocity with separate acceleration, fast braking, and zero-crossing guard."""
    if current * target < 0.0:
        # Cross through 0.0 cleanly using braking deceleration before reversing
        max_brake = decel * dt
        if abs(current) <= max_brake:
            return 0.0
        return current - max_brake if current > 0.0 else current + max_brake

    diff = target - current
    is_braking = (current > 0.0 and target < current) or (current < 0.0 and target > current)
    rate = decel if is_braking else accel
    max_change = rate * dt
    if abs(diff) <= max_change:
        return target
    return current + (max_change if diff > 0.0 else -max_change)


class ArmState:
    """Tracks both commanded target pose and live hardware feedback from SwiftproState_topic."""

    def __init__(self):
        self.x, self.y, self.z = HOME
        self.angle = HOME_ANGLE
        self.actual_x, self.actual_y, self.actual_z = HOME
        self.actual_angle = HOME_ANGLE
        self.initialised = False

    def callback(self, msg):
        plausible = abs(msg.x) > 1.0 or abs(msg.y) > 1.0 or abs(msg.z) > 1.0
        if plausible:
            self.actual_x = clamp(msg.x, SAFE_LIMITS["x"])
            self.actual_y = clamp(msg.y, SAFE_LIMITS["y"])
            self.actual_z = clamp(msg.z, SAFE_LIMITS["z"])
            self.actual_angle = clamp(msg.motor_angle4, WRIST_LIMITS)
            if not self.initialised:
                self.sync_to_actual()
                self.initialised = True

    def sync_to_actual(self):
        """Sync target coordinates to the latest hardware feedback to prevent sudden jumps."""
        self.x = self.actual_x
        self.y = self.actual_y
        self.z = self.actual_z
        self.angle = self.actual_angle


class SparkTeleopNode:
    def __init__(self):
        rospy.init_node("keyboard_teleop", anonymous=True, argv=[sys.argv[0]])

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pos_pub = rospy.Publisher("/position_write_topic", position, queue_size=1)
        self.angle_pub = rospy.Publisher("/angle4th_topic", angle4th, queue_size=1)
        self.pump_pub = rospy.Publisher("/pump_topic", status, queue_size=1)
        self.arm_status_pub = rospy.Publisher("/swiftpro_status_topic", status, queue_size=1)

        self.arm = ArmState()
        rospy.Subscriber("/SwiftproState_topic", SwiftproState, self.arm.callback, queue_size=1)
        rospy.sleep(0.3)

        self._check_connections()
        rospy.on_shutdown(self.shutdown_hook)

        # Control states
        self.current_mode = "FAST"
        self.motors_attached = True
        self.pump_on = False

        self.dir_linear = 0
        self.dir_angular = 0
        self.last_linear_key = 0.0
        self.last_steer_key = 0.0
        self.last_steer_while_moving = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_arm_pub = 0.0

    def _check_connections(self):
        deadline = time.time() + 2.0
        while time.time() < deadline:
            if self.cmd_pub.get_num_connections() > 0 and self.pos_pub.get_num_connections() > 0:
                break
            rospy.sleep(0.05)

        if self.cmd_pub.get_num_connections() == 0:
            safe_print("WARNING: no subscriber on /cmd_vel.")
            safe_print("         Start base driver: roslaunch spark_bringup driver_bringup.launch")

        if self.pos_pub.get_num_connections() == 0:
            safe_print("WARNING: no subscriber on /position_write_topic.")
            safe_print("         Start arm driver: roslaunch swiftpro pro_control_nomoveit.launch")

        if not self.arm.initialised:
            safe_print("  arm pose unknown - press 0 first to home/sync the arm.")

    def set_pump(self, on):
        self.pump_on = bool(on)
        msg = status()
        msg.status = 1 if self.pump_on else 0
        self.pump_pub.publish(msg)

    def set_arm_status(self, attached):
        self.motors_attached = bool(attached)
        msg = status()
        msg.status = 1 if self.motors_attached else 0
        self.arm_status_pub.publish(msg)

    def set_base(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    def stop_base_immediately(self):
        self.dir_linear = 0
        self.dir_angular = 0
        self.last_linear_key = 0.0
        self.last_steer_key = 0.0
        self.last_steer_while_moving = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.set_base(0.0, 0.0)

    def halt_arm_in_place(self):
        """Sync target to live hardware pose and publish immediately to abort ongoing arm moves."""
        if self.arm.initialised:
            self.arm.sync_to_actual()
            self.publish_pose()
            self.publish_angle()

    def publish_pose(self):
        msg = position()
        msg.x, msg.y, msg.z = self.arm.x, self.arm.y, self.arm.z
        self.pos_pub.publish(msg)

    def publish_angle(self):
        msg = angle4th()
        msg.angle4th = self.arm.angle
        self.angle_pub.publish(msg)

    def go_home(self):
        self.arm.x, self.arm.y, self.arm.z = HOME
        self.arm.angle = HOME_ANGLE
        self.publish_pose()
        self.publish_angle()

    def apply_preset(self, name, coords, angle=0.0):
        target_x, target_y, target_z = coords
        self.arm.x = clamp(target_x, SAFE_LIMITS["x"])
        self.arm.y = clamp(target_y, SAFE_LIMITS["y"])
        self.arm.z = clamp(target_z, SAFE_LIMITS["z"])
        self.arm.angle = clamp(angle, WRIST_LIMITS)
        self.publish_pose()
        self.publish_angle()
        safe_print(
            f">> PRESET: {name} (X={self.arm.x:.1f}, Y={self.arm.y:.1f}, "
            f"Z={self.arm.z:.1f}, A={self.arm.angle:.0f}°)"
        )

    def step_arm(self, key_char, step_size, wrist_step):
        field, direction = AXIS_CONFIG[key_char]
        if field in ("x", "y", "z"):
            step = step_size * direction
            setattr(self.arm, field, clamp(getattr(self.arm, field) + step, SAFE_LIMITS[field]))
            self.publish_pose()
        elif field == "angle":
            self.arm.angle = clamp(self.arm.angle + wrist_step * direction, WRIST_LIMITS)
            self.publish_angle()

    def update_hud(self):
        pump_str = "ON " if self.pump_on else "OFF"
        hud_line = (
            f"\r\033[K[MODE: {self.current_mode:4s}] "
            f"[BASE: {self.current_linear:+.2f} m/s, {self.current_angular:+.2f} rad/s] "
            f"[ARM: X={self.arm.x:5.1f} Y={self.arm.y:5.1f} Z={self.arm.z:5.1f} A={self.arm.angle:3.0f}°] "
            f"[PUMP: {pump_str}]"
        )
        sys.stdout.write(hud_line)
        sys.stdout.flush()

    def shutdown_hook(self):
        try:
            self.set_base(0.0, 0.0)
            self.set_pump(False)
        except Exception:
            pass

    @staticmethod
    def read_key_nonblocking(fd):
        """Read single key or ANSI escape sequence without blocking the 20Hz loop."""
        raw = os.read(fd, 3).decode(errors="ignore")
        if not raw:
            return None
        if raw.startswith("\x1b"):
            if len(raw) >= 3 and raw[1] == "[":
                return ARROW_MAP.get(raw[2], None)
            return None
        return raw[0]

    def handle_key(self, char, now, profile):
        lower_char = char.lower()

        if char == "\x03":  # Ctrl+C
            return False

        elif char == " ":  # Space: Emergency Stop (Base + Arm)
            self.stop_base_immediately()
            self.halt_arm_in_place()
            safe_print(">> STOP (Base & Arm halted in place)")

        elif lower_char in ("f", "m") or char == "\t":  # Speed mode toggle
            self.current_mode = "SLOW" if self.current_mode == "FAST" else "FAST"
            safe_print(f">> SPEED MODE: [{self.current_mode}]")

        elif lower_char == "q":  # Arc turn forward-left
            self.dir_linear = 1
            self.dir_angular = 1
            self.last_linear_key = now
            self.last_steer_key = now
            self.last_steer_while_moving = now

        elif lower_char == "e":  # Arc turn forward-right
            self.dir_linear = 1
            self.dir_angular = -1
            self.last_linear_key = now
            self.last_steer_key = now
            self.last_steer_while_moving = now

        elif char in ("\r", "\n"):  # Enter: Toggle pump
            self.set_pump(not self.pump_on)
            safe_print("PUMP ON" if self.pump_on else "PUMP OFF")

        elif lower_char in ("w", "s"):  # Linear motion
            new_dir = 1 if lower_char == "w" else -1
            if self.dir_linear != 0 and self.dir_linear != new_dir:
                # Reset steering on sudden direction reversal
                self.dir_angular = 0
                self.last_steer_while_moving = 0.0
            self.dir_linear = new_dir
            self.last_linear_key = now
            if self.dir_angular != 0:
                self.last_steer_key = now

        elif lower_char in ("a", "d"):  # Steering motion
            self.dir_angular = 1 if lower_char == "a" else -1
            self.last_steer_key = now
            if self.dir_linear != 0:
                self.last_linear_key = now
                self.last_steer_while_moving = now

        elif char in AXIS_CONFIG:  # Arm jogging
            if now - self.last_arm_pub >= 0.08:
                self.last_arm_pub = now
                self.step_arm(char, profile["arm_step"], profile["wrist_step"])

        elif lower_char in ARM_PRESETS:  # Arm presets
            preset_name, preset_coords, preset_angle = ARM_PRESETS[lower_char]
            self.apply_preset(preset_name, preset_coords, preset_angle)

        elif lower_char == "r":  # Relax / lock motors
            new_state = not self.motors_attached
            self.set_arm_status(new_state)
            if new_state:
                self.arm.sync_to_actual()
                self.publish_pose()
                self.publish_angle()
                safe_print(
                    f">> MOTORS LOCKED (M17) - синхронизировано с текущим положением "
                    f"(X={self.arm.x:.1f}, Y={self.arm.y:.1f}, Z={self.arm.z:.1f})"
                )
            else:
                safe_print(">> MOTORS RELAXED (M2019) - выровняйте рукой и нажмите 'r' для фиксации!")

        elif char == "0":  # Release + Home
            self.set_pump(False)
            self.stop_base_immediately()
            self.go_home()
            safe_print("RELEASE + HOME")

        return True

    def run(self):
        safe_print(HELP)
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        rate = rospy.Rate(20)
        last_time = time.time()
        last_hud_time = 0.0

        try:
            tty.setraw(fd)
            while not rospy.is_shutdown():
                now = time.time()
                dt = max(0.01, min(0.1, now - last_time))
                last_time = now
                profile = SPEED_MODES[self.current_mode]

                while select.select([sys.stdin], [], [], 0)[0]:
                    key = self.read_key_nonblocking(fd)
                    if key is None:
                        continue
                    if not self.handle_key(key, now, profile):
                        return

                # Steering timeout
                if self.dir_angular != 0 and (now - self.last_steer_key > STEER_TIMEOUT):
                    self.dir_angular = 0

                # Linear timeout (with grace period after combined steering)
                linear_timeout = (
                    KEY_GRACE_TIMEOUT
                    if (now - self.last_steer_while_moving < 1.0)
                    else KEY_TIMEOUT
                )
                if self.dir_linear != 0 and (now - self.last_linear_key > linear_timeout):
                    self.dir_linear = 0

                # Update profile in case speed mode changed during keypress handling
                profile = SPEED_MODES[self.current_mode]
                target_linear = self.dir_linear * profile["lin_speed"]
                target_angular = self.dir_angular * profile["ang_speed"]

                self.current_linear = smooth_velocity(
                    self.current_linear, target_linear, MAX_LIN_ACCEL, MAX_LIN_DECEL, dt
                )
                self.current_angular = smooth_velocity(
                    self.current_angular, target_angular, MAX_ANG_ACCEL, MAX_ANG_DECEL, dt
                )

                self.set_base(self.current_linear, self.current_angular)

                if now - last_hud_time >= 0.1:
                    self.update_hud()
                    last_hud_time = now

                rate.sleep()

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.stop_base_immediately()
            self.set_pump(False)
            sys.stdout.write("\r\033[K")
            safe_print("\nstopped - base stopped, pump off")


if __name__ == "__main__":
    node = SparkTeleopNode()
    node.run()