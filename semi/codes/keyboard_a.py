#!/usr/bin/env python3
"""
Keyboard control for the Spark robot (base + uArm + vacuum pump).

Base controls:
    w / s      forward / backward (or Up / Down arrows)
    a / d      turn left / right  (or Left / Right arrows)
    (combined: hold w/s + steer with a/d for smooth arc turns)
    Space      EMERGENCY STOP (instantly stops base & arm; preserves pump & box)
    q          toggle speed mode (FAST: arena drive / SLOW: fine box manipulation)

Arm controls (manual jogging):
    8 / 2      up / down (z)
    7 / 9      forward / backward (x)
    4 / 6      swing CCW / CW around base (y)
    1 / 3      wrist CCW / CW (end-effector spin)

Arm presets (extended +80mm forward to clear nearby objects):
    h          Home pose        (x=120, y=0, z=35, angle=0) - parked
    v          Hover / Ready    (x=280, y=0, z=20, angle=0) - 2cm above box
    p          Pick / Touch     (x=280, y=0, z=-50, angle=0) - table level
    c          Carry / Lift     (x=230, y=0, z=100, angle=0) - safe driving height

Pump & System:
    Enter      toggle vacuum pump: grab / release (one toggle per press)
    0          release box (pump off) and return arm to resting home pose
    Ctrl+C     stop everything and exit

Topics:
    /cmd_vel              geometry_msgs/Twist       base motion
    /position_write_topic swiftpro/position         arm x,y,z in mm (absolute)
    /angle4th_topic       swiftpro/angle4th         wrist angle in degrees
    /pump_topic           swiftpro/status           vacuum 1/0
    /SwiftproState_topic  swiftpro/SwiftproState    current pose feedback

Prerequisites:
    base: roslaunch spark_bringup driver_bringup.launch camera_type_tel:=d435 lidar_type_tel:=ydlidar_g6
    arm:  roslaunch swiftpro pro_control_nomoveit.launch

Usage:
    ssh spark@10.25.141.8
    source ~/spark_noetic/devel/setup.bash
    python3 ~/semi/keyboard_teleop.py
"""

import os
import select
import sys
import termios
import time
import tty

import rospy
from geometry_msgs.msg import Twist
from swiftpro.msg import SwiftproState, angle4th, position, status

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_ik import ik_reachable

# Speed profiles: FAST for field transit, SLOW for fine manipulation
SPEED_MODES = {
    "FAST": {
        "lin_speed": float(os.environ.get("LIN_SPEED_FAST", "0.20")),     # m/s
        "ang_speed": float(os.environ.get("ANG_SPEED_FAST", "0.75")),     # rad/s
        "arm_speed": float(os.environ.get("ARM_SPEED_FAST", "35.0")),     # mm/s
        "wrist_speed": float(os.environ.get("WRIST_SPEED_FAST", "35.0")), # deg/s
    },
    "SLOW": {
        "lin_speed": float(os.environ.get("LIN_SPEED_SLOW", "0.07")),     # m/s
        "ang_speed": float(os.environ.get("ANG_SPEED_SLOW", "0.25")),     # rad/s
        "arm_speed": float(os.environ.get("ARM_SPEED_SLOW", "12.0")),     # mm/s
        "wrist_speed": float(os.environ.get("WRIST_SPEED_SLOW", "15.0")), # deg/s
    },
}

# Responsive acceleration and fast braking limits
MAX_LIN_ACCEL = float(os.environ.get("MAX_LIN_ACCEL", "1.0"))   # m/s^2 (forward/backward accel)
MAX_LIN_DECEL = float(os.environ.get("MAX_LIN_DECEL", "3.0"))   # m/s^2 (crisp braking)
MAX_ANG_ACCEL = float(os.environ.get("MAX_ANG_ACCEL", "3.0"))   # rad/s^2 (turning accel)
MAX_ANG_DECEL = float(os.environ.get("MAX_ANG_DECEL", "6.0"))   # rad/s^2 (turning braking)

# Timeouts: 0.22s stops wheels rapidly after key release
KEY_TIMEOUT = float(os.environ.get("KEY_TIMEOUT", "0.22"))       # s
STEER_TIMEOUT = float(os.environ.get("STEER_TIMEOUT", "0.18"))   # s

# Arm publishing period: 0.05s (20 Hz) streams smooth micro-steps without start-stop stutter
ARM_PUB_PERIOD = float(os.environ.get("ARM_PUB_PERIOD", "0.05")) # s

HOME = (120.0, 0.0, 35.0)
HOME_ANGLE = 0.0
ARM_LIMITS = {"x": (0.0, 350.0), "y": (-160.0, 160.0), "z": (-85.0, 170.0)}
WRIST_LIMITS = (0.0, 180.0)

# Presets extended forward with corrected Pick height (z=-68mm) to firmly touch cubes
ARM_PRESETS = {
    "h": ("Home", (120.0, 0.0, 35.0), 0.0),
    "v": ("Hover / Ready", (280.0, 0.0, 15.0), 0.0),
    "p": ("Pick / Touch", (280.0, 0.0, -68.0), 0.0),
    "c": ("Carry / Lift", (230.0, 0.0, 120.0), 0.0),
}

HELP = """
  === Spark Keyboard Teleop ===
  Base:
    w/s: forward / backward     a/d: turn left / right (or Arrow keys)
    (combined turns: hold w/s + a/d to drive in an arc)
    Space: EMERGENCY STOP (instant halt, keeps pump & box safe)
    q:     toggle speed mode (FAST / SLOW)

  Arm Jogging:
    8 / 2: up / down (z)        7 / 9: forward / backward (x)
    4 / 6: swing CCW / CW (y)   1 / 3: wrist CCW / CW (hold)

  Arm Presets:
    h: Home (120, 0, 35)        v: Hover/Ready (280, 0, 15)
    p: Pick/Touch (280, 0, -68) c: Carry (230, 0, 120)

  Pump & Safety:
    Enter: grab / release (pump toggle)   0: release box + home
    Ctrl+C: stop everything and exit
"""


def safe_print(*args, **kwargs):
    """Print with carriage returns and clear HUD line in terminal raw mode."""
    sep = kwargs.get("sep", " ")
    end = kwargs.get("end", "\r\n")
    text = sep.join(str(a) for a in args)
    text = text.replace("\r\n", "\n").replace("\n", "\r\n")
    sys.stdout.write(f"\r\033[K{text}{end}")
    sys.stdout.flush()


def update_hud(mode, v_lin, v_ang, arm_state, pump_on):
    """Render a compact live status HUD bar on the bottom terminal line."""
    pump_str = "ON " if pump_on else "OFF"
    hud_line = (
        f"\r\033[K[MODE: {mode:4s}] "
        f"[BASE: {v_lin:+.2f} m/s, {v_ang:+.2f} rad/s] "
        f"[ARM: X={arm_state.x:5.1f} Y={arm_state.y:5.1f} Z={arm_state.z:5.1f} A={arm_state.angle:3.0f}°] "
        f"[PUMP: {pump_str}]"
    )
    sys.stdout.write(hud_line)
    sys.stdout.flush()


def clamp(value, limits):
    return min(max(value, limits[0]), limits[1])


def smooth_velocity(current, target, accel, decel, dt):
    """Smooth velocity with separate acceleration and fast braking deceleration."""
    diff = target - current
    is_braking = (current > 0 and target < current) or (current < 0 and target > current)
    rate = decel if is_braking else accel
    max_change = rate * dt
    if abs(diff) <= max_change:
        return target
    return current + (max_change if diff > 0 else -max_change)


class ArmState:
    """Target pose tracked locally, initialized from SwiftproState_topic."""

    def __init__(self):
        self.x, self.y, self.z = HOME
        self.angle = HOME_ANGLE
        self.initialised = False

    def callback(self, msg):
        # Initialize initial target from first valid hardware feedback
        plausible = abs(msg.x) > 1 or abs(msg.y) > 1 or abs(msg.z) > 1
        if not self.initialised and plausible:
            self.x, self.y, self.z = msg.x, msg.y, msg.z
            self.angle = msg.motor_angle4
            self.initialised = True


def main():
    rospy.init_node("keyboard_teleop", anonymous=True, argv=[sys.argv[0]])

    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    pos_pub = rospy.Publisher("/position_write_topic", position, queue_size=1)
    angle_pub = rospy.Publisher("/angle4th_topic", angle4th, queue_size=1)
    pump_pub = rospy.Publisher("/pump_topic", status, queue_size=1)

    arm = ArmState()
    rospy.Subscriber("/SwiftproState_topic", SwiftproState, arm.callback, queue_size=1)
    rospy.sleep(0.3)

    # Check connection to topics with combined deadline
    deadline = time.time() + 2.0
    while time.time() < deadline:
        if cmd_pub.get_num_connections() > 0 and pos_pub.get_num_connections() > 0:
            break
        rospy.sleep(0.05)

    if cmd_pub.get_num_connections() == 0:
        safe_print("WARNING: no subscriber on /cmd_vel.")
        safe_print("         Start base driver: roslaunch spark_bringup driver_bringup.launch")

    if pos_pub.get_num_connections() == 0:
        safe_print("WARNING: no subscriber on /position_write_topic.")
        safe_print("         Start arm driver: roslaunch swiftpro pro_control_nomoveit.launch")

    if not arm.initialised:
        safe_print("  arm pose unknown - press 0 first to home/sync the arm.")

    def set_pump(on):
        message = status()
        message.status = 1 if on else 0
        pump_pub.publish(message)

    def set_base(linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        cmd_pub.publish(twist)

    def shutdown_hook():
        try:
            set_base(0.0, 0.0)
            set_pump(False)
        except Exception:
            pass

    rospy.on_shutdown(shutdown_hook)

    def publish_pose():
        message = position()
        message.x, message.y, message.z = arm.x, arm.y, arm.z
        pos_pub.publish(message)

    def publish_angle():
        message = angle4th()
        message.angle4th = arm.angle
        angle_pub.publish(message)

    def go_home():
        arm.x, arm.y, arm.z = HOME
        arm.angle = HOME_ANGLE
        publish_pose()
        publish_angle()

    def apply_preset(name, coords, angle=0.0):
        target_x, target_y, target_z = coords
        if ik_reachable(target_x, target_y, target_z):
            arm.x, arm.y, arm.z = target_x, target_y, target_z
            arm.angle = angle
            publish_pose()
            publish_angle()
            safe_print(f">> PRESET: {name} (X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f})")
        else:
            safe_print(f"WARNING: preset {name} unreachable by IK!")

    # Generic arm axis step handler
    axis_config = {
        "x+": ("x", 1), "x-": ("x", -1),
        "y+": ("y", 1), "y-": ("y", -1),
        "z+": ("z", 1), "z-": ("z", -1),
        "angle+": ("angle", 1), "angle-": ("angle", -1),
    }
    last_warn_time = 0.0

    def step_arm(axis, dt, arm_speed, wrist_speed):
        nonlocal last_warn_time
        field, direction = axis_config[axis]

        if field in ("x", "y", "z"):
            step = arm_speed * dt * direction
            target = {"x": arm.x, "y": arm.y, "z": arm.z}
            target[field] = clamp(target[field] + step, ARM_LIMITS[field])

            if ik_reachable(target["x"], target["y"], target["z"]):
                arm.x, arm.y, arm.z = target["x"], target["y"], target["z"]
                publish_pose()
            else:
                now_warn = time.time()
                if now_warn - last_warn_time > 0.5:
                    safe_print("arm limit:", field)
                    last_warn_time = now_warn
        elif field == "angle":
            step = wrist_speed * dt * direction
            cand = clamp(arm.angle + step, WRIST_LIMITS)
            if cand != arm.angle:
                arm.angle = cand
                publish_angle()
            else:
                now_warn = time.time()
                if now_warn - last_warn_time > 0.5:
                    safe_print("arm limit: wrist")
                    last_warn_time = now_warn

    safe_print(HELP)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    # Active speed mode
    current_mode = "FAST"

    # Base motion states
    target_linear = target_angular = 0.0
    current_linear = current_angular = 0.0
    last_linear_key = 0.0
    last_angular_key = 0.0

    # Arm motion states
    arm_axis = None
    last_arm_key = 0.0
    last_arm_pub = 0.0

    pump_on = False

    # 20 Hz loop: perfect match for Spark base serial driver (/dev/sparkBase)
    rate = rospy.Rate(20)
    last_time = time.time()
    last_hud_time = 0.0

    try:
        tty.setraw(fd)
        while not rospy.is_shutdown():
            now = time.time()
            dt = max(0.01, min(0.1, now - last_time))
            last_time = now
            profile = SPEED_MODES[current_mode]

            # Read all pending keys from stdin
            while select.select([sys.stdin], [], [], 0)[0]:
                char = os.read(fd, 1).decode(errors="ignore")
                if not char:
                    break

                # Parse multi-byte ANSI escape sequence for arrow keys
                if char == "\x1b":
                    if select.select([sys.stdin], [], [], 0.02)[0]:
                        seq1 = os.read(fd, 1).decode(errors="ignore")
                        if seq1 == "[":
                            if select.select([sys.stdin], [], [], 0.02)[0]:
                                seq2 = os.read(fd, 1).decode(errors="ignore")
                                if seq2 == "A":   char = "w"
                                elif seq2 == "B": char = "s"
                                elif seq2 == "C": char = "d"
                                elif seq2 == "D": char = "a"
                                else: continue
                            else: continue
                        else: continue
                    else: continue

                lower_char = char.lower()

                if char == "\x03":                       # Ctrl+C
                    return
                elif char == " ":                        # Space: Emergency Stop
                    target_linear = target_angular = 0.0
                    current_linear = current_angular = 0.0
                    set_base(0.0, 0.0)
                    arm_axis = None
                    safe_print(">> STOP (Base & Arm halted)")
                elif lower_char == "q":                  # Toggle speed mode
                    current_mode = "SLOW" if current_mode == "FAST" else "FAST"
                    safe_print(f">> SPEED MODE: [{current_mode}]")
                elif char in ("\r", "\n"):               # Enter
                    pump_on = not pump_on
                    set_pump(pump_on)
                    safe_print("PUMP ON" if pump_on else "PUMP OFF")
                elif lower_char in ("w", "s"):           # Linear motion
                    target_linear = profile["lin_speed"] if lower_char == "w" else -profile["lin_speed"]
                    last_linear_key = now
                elif lower_char in ("a", "d"):           # Angular steering motion
                    target_angular = profile["ang_speed"] if lower_char == "a" else -profile["ang_speed"]
                    last_angular_key = now
                    # Crucial: if already driving, keep linear drive alive while steering!
                    if target_linear != 0.0:
                        last_linear_key = now
                elif char in "82794613":                 # Arm manual jogging
                    arm_axis = {
                        "8": "z+", "2": "z-",
                        "7": "x+", "9": "x-",
                        "4": "y+", "6": "y-",
                        "1": "angle+", "3": "angle-"
                    }[char]
                    last_arm_key = now
                elif lower_char in ARM_PRESETS:          # Arm presets (+30mm forward)
                    arm_axis = None
                    preset_name, preset_coords, preset_angle = ARM_PRESETS[lower_char]
                    apply_preset(preset_name, preset_coords, preset_angle)
                elif char == "0":                        # Release + Home
                    pump_on = False
                    set_pump(False)
                    target_linear = target_angular = 0.0
                    current_linear = current_angular = 0.0
                    set_base(0.0, 0.0)
                    go_home()
                    arm_axis = None
                    safe_print("RELEASE + HOME")

            # Update target speeds based on active profile and timeouts
            profile = SPEED_MODES[current_mode]

            # Timeout check for linear motion
            if now - last_linear_key > KEY_TIMEOUT:
                target_linear = 0.0
            else:
                target_linear = profile["lin_speed"] if target_linear > 0 else -profile["lin_speed"]

            # Timeout check for steering motion
            if now - last_angular_key > STEER_TIMEOUT:
                target_angular = 0.0
            else:
                target_angular = profile["ang_speed"] if target_angular > 0 else -profile["ang_speed"]

            # Smooth velocity ramping (acceleration & fast braking)
            current_linear = smooth_velocity(current_linear, target_linear, MAX_LIN_ACCEL, MAX_LIN_DECEL, dt)
            current_angular = smooth_velocity(current_angular, target_angular, MAX_ANG_ACCEL, MAX_ANG_DECEL, dt)

            # Continuously stream base commands at 20 Hz
            set_base(current_linear, current_angular)

            # Arm: check key repeat timeout (stops immediately after key release)
            if arm_axis is not None and now - last_arm_key > 0.22:
                arm_axis = None

            # Arm: stream smooth micro-steps at 20 Hz
            if arm_axis is not None and now - last_arm_pub >= ARM_PUB_PERIOD:
                step_dt = now - last_arm_pub if last_arm_pub > 0.0 else ARM_PUB_PERIOD
                last_arm_pub = now
                step_arm(arm_axis, step_dt, profile["arm_speed"], profile["wrist_speed"])

            # Update Live Status HUD at 10 Hz
            if now - last_hud_time >= 0.1:
                update_hud(current_mode, current_linear, current_angular, arm, pump_on)
                last_hud_time = now

            rate.sleep()

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        set_base(0.0, 0.0)
        set_pump(False)
        sys.stdout.write("\r\033[K")
        safe_print("\nstopped - base stopped, pump off")


if __name__ == "__main__":
    main()