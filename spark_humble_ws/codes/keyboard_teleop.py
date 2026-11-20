#!/usr/bin/env python3
"""
Keyboard control for the Spark robot (base + uArm + vacuum pump).

Base controls:
    w / s      forward / backward (or Up / Down arrows)
    a / d      turn left / right  (or Left / Right arrows)
    (combined: hold w/s + steer with a/d for smooth arc turns)
    Space      EMERGENCY STOP (instantly stops base & arm; preserves pump & box)
    q          toggle speed mode (FAST: arena drive / SLOW: fine box manipulation)

Arm controls (step-jogging on keypress/hold):
    8 / 2      up / down (z)
    7 / 9      forward / backward (x)
    4 / 6      swing CCW / CW around base (y)
    1 / 3      wrist CCW / CW (end-effector spin)

Arm presets (direct smooth single-target moves):
    v          Hover / Ready    (x=280, y=0, z=10, angle=0) - ready over cube
    p          Pick / Touch     (x=280, y=0, z=-90, angle=0) - table level
    c          Carry / Lift     (x=230, y=0, z=120, angle=0) - safe driving height

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
    ssh spark@10.24.170.0
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

# Speed profiles: FAST for field transit, SLOW for fine manipulation
SPEED_MODES = {
    "FAST": {
        "lin_speed": float(os.environ.get("LIN_SPEED_FAST", "0.20")),     # m/s
        "ang_speed": float(os.environ.get("ANG_SPEED_FAST", "0.75")),     # rad/s
        "arm_step": float(os.environ.get("ARM_STEP_FAST", "15.0")),       # mm per jog step
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
MAX_LIN_ACCEL = float(os.environ.get("MAX_LIN_ACCEL", "1.0"))   # m/s^2 (forward/backward accel)
MAX_LIN_DECEL = float(os.environ.get("MAX_LIN_DECEL", "5.0"))   # m/s^2 (crisp instant braking)
MAX_ANG_ACCEL = float(os.environ.get("MAX_ANG_ACCEL", "3.0"))   # rad/s^2 (turning accel)
MAX_ANG_DECEL = float(os.environ.get("MAX_ANG_DECEL", "8.0"))   # rad/s^2 (turning braking)

# Timeouts: 0.22s stops wheels rapidly after key release
KEY_TIMEOUT = float(os.environ.get("KEY_TIMEOUT", "0.22"))       # s
STEER_TIMEOUT = float(os.environ.get("STEER_TIMEOUT", "0.18"))   # s

HOME = (120.0, 0.0, 35.0)
HOME_ANGLE = 0.0
WRIST_LIMITS = (0.0, 180.0)

# Protective barrier limits:
# - Z >= -85.0mm prevents driving into the table while allowing solid cube grip (-80.0mm)
# - X >= 90.0mm prevents colliding with LiDAR / RealSense; X <= 320.0mm prevents cable strain
# - Y in [-160.0, 160.0] allows full left/right workspace swing
SAFE_LIMITS = {
    "x": (90.0, 320.0),
    "y": (-160.0, 160.0),
    "z": (-95.0, 180.0),
}

# Presets:
# - 'v': Hover / Safe height (z=30mm)
# - 't': Touch / Pick floor cubes (z=-50mm)
# - 'c': Carry / Lift high (z=100mm)
ARM_PRESETS = {
    "v": ("Hover / Safe Height", (280.0, 0.0, 30.0), 0.0),
    "t": ("Touch / Pick Floor", (280.0, 0.0, -50.0), 0.0),
    "c": ("Carry / Lift High", (230.0, 0.0, 90.0), 0.0),
}

HELP = """
  === Spark Keyboard Teleop ===
  Base:
    w/s: forward / backward     a/d: turn left / right (or Arrow keys)
    q/e: arc turn forward-left / forward-right (single-key smooth arc!)
    (combined: while driving w/s, tap/hold a/d to curve in an arc!)
    Space: EMERGENCY STOP       f: toggle speed mode (FAST / SLOW)

  Arm Jogging (smooth jog step on tap/hold):
    8 / 2: up / down (z)        7 / 9: forward / backward (x)
    4 / 6: swing CCW / CW (y)   1 / 3: wrist CCW / CW

  Arm Presets:
    v: Hover / Safe Height (280, 0, 30)    t: Touch / Pick Floor (-50)
    c: Carry / Lift High (230, 0, 100)

  Pump, Motors & Safety:
    Enter: grab / release (pump toggle)   r: relax motors (free-hand re-align)
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
    arm_status_pub = rospy.Publisher("/swiftpro_status_topic", status, queue_size=1)

    motors_attached = [True]

    def set_arm_status(attached):
        motors_attached[0] = attached
        msg = status()
        msg.status = 1 if attached else 0
        arm_status_pub.publish(msg)

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
        arm.x = clamp(target_x, SAFE_LIMITS["x"])
        arm.y = clamp(target_y, SAFE_LIMITS["y"])
        arm.z = clamp(target_z, SAFE_LIMITS["z"])
        arm.angle = angle
        publish_pose()
        publish_angle()
        safe_print(f">> PRESET: {name} (X={arm.x:.1f}, Y={arm.y:.1f}, Z={arm.z:.1f})")

    axis_config = {
        "x+": ("x", 1), "x-": ("x", -1),
        "y+": ("y", 1), "y-": ("y", -1),
        "z+": ("z", 1), "z-": ("z", -1),
        "angle+": ("angle", 1), "angle-": ("angle", -1),
    }

    # Step arm with protective soft barrier (table and body safety)
    def step_arm(axis, step_size, wrist_step):
        field, direction = axis_config[axis]
        if field in ("x", "y", "z"):
            step = step_size * direction
            if field == "x":
                arm.x = clamp(arm.x + step, SAFE_LIMITS["x"])
            elif field == "y":
                arm.y = clamp(arm.y + step, SAFE_LIMITS["y"])
            elif field == "z":
                arm.z = clamp(arm.z + step, SAFE_LIMITS["z"])
            publish_pose()
        elif field == "angle":
            arm.angle = clamp(arm.angle + wrist_step * direction, WRIST_LIMITS)
            publish_angle()

    safe_print(HELP)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    # Active speed mode
    current_mode = "FAST"

    # Base motion states (state-driven: enables simultaneous forward drive + steering)
    dir_linear = 0   # +1: forward, -1: backward, 0: stop
    dir_angular = 0  # +1: left, -1: right, 0: straight
    last_linear_key = 0.0
    last_steer_key = 0.0
    last_steer_while_moving = 0.0
    target_linear = target_angular = 0.0
    current_linear = current_angular = 0.0

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
                    dir_linear = dir_angular = 0
                    last_linear_key = last_steer_key = last_steer_while_moving = 0.0
                    target_linear = target_angular = 0.0
                    current_linear = current_angular = 0.0
                    set_base(0.0, 0.0)
                    safe_print(">> STOP (Base & Arm halted)")
                elif lower_char == "f" or char in ("\t",) or lower_char == "m": # Speed mode toggle (f / Tab / m)
                    current_mode = "SLOW" if current_mode == "FAST" else "FAST"
                    safe_print(f">> SPEED MODE: [{current_mode}]")
                elif lower_char == "q":                  # Arc turn forward-left
                    dir_linear = 1
                    dir_angular = 1
                    last_linear_key = now
                    last_steer_key = now
                    last_steer_while_moving = now
                elif lower_char == "e":                  # Arc turn forward-right
                    dir_linear = 1
                    dir_angular = -1
                    last_linear_key = now
                    last_steer_key = now
                    last_steer_while_moving = now
                elif char in ("\r", "\n"):               # Enter
                    pump_on = not pump_on
                    set_pump(pump_on)
                    safe_print("PUMP ON" if pump_on else "PUMP OFF")
                elif lower_char in ("w", "s"):           # Linear motion (preserves active steering!)
                    new_dir = 1 if lower_char == "w" else -1
                    if dir_linear != 0 and dir_linear != new_dir:
                        last_steer_while_moving = 0.0
                    dir_linear = new_dir
                    last_linear_key = now
                    if dir_angular != 0:
                        last_steer_key = now
                elif lower_char in ("a", "d"):           # Steering motion (maintains linear drive if active!)
                    dir_angular = 1 if lower_char == "a" else -1
                    last_steer_key = now
                    if dir_linear != 0:
                        last_linear_key = now
                        last_steer_while_moving = now
                elif char in "82794613":                 # Arm jogging (immediate clean step)
                    axis = {
                        "8": "z+", "2": "z-",
                        "7": "x+", "9": "x-",
                        "4": "y+", "6": "y-",
                        "1": "angle+", "3": "angle-"
                    }[char]
                    if now - last_arm_pub >= 0.08:        # 80ms throttle: fast but no buffer buildup
                        last_arm_pub = now
                        step_arm(axis, profile["arm_step"], profile["wrist_step"])
                elif lower_char in ARM_PRESETS:          # Arm presets
                    preset_name, preset_coords, preset_angle = ARM_PRESETS[lower_char]
                    apply_preset(preset_name, preset_coords, preset_angle)
                elif lower_char == "r":                  # Relax / free motors for manual realignment
                    new_state = not motors_attached[0]
                    set_arm_status(new_state)
                    if new_state:
                        arm.x, arm.y, arm.z = ARM_PRESETS["v"][1]
                        arm.angle = ARM_PRESETS["v"][2]
                        safe_print(">> MOTORS LOCKED (M17) - position locked.")
                    else:
                        safe_print(">> MOTORS RELAXED (M2019) - align arm straight manually, then press 'r'!")
                elif char == "0":                        # Release + Home
                    pump_on = False
                    set_pump(False)
                    dir_linear = dir_angular = 0
                    last_linear_key = last_steer_key = last_steer_while_moving = 0.0
                    target_linear = target_angular = 0.0
                    current_linear = current_angular = 0.0
                    set_base(0.0, 0.0)
                    go_home()
                    safe_print("RELEASE + HOME")

            # Steering timeout: auto-center wheels after 0.30s of no steer key
            if dir_angular != 0 and (now - last_steer_key > 0.30):
                dir_angular = 0

            # Linear timeout:
            # - If user was steering while moving (e.g. held W and pressed D),
            #   grant 0.80s grace period when D is released so forward motion
            #   does not abruptly die due to Windows typematic repeat suppression.
            # - If just driving straight (W only), stop cleanly in 0.30s.
            linear_timeout = 0.80 if (now - last_steer_while_moving < 1.0) else 0.30
            if dir_linear != 0 and (now - last_linear_key > linear_timeout):
                dir_linear = 0

            # Target velocities from active profile and directions
            profile = SPEED_MODES[current_mode]
            target_linear = dir_linear * profile["lin_speed"]
            target_angular = dir_angular * profile["ang_speed"]

            # Smooth velocity ramping (acceleration & fast braking)
            current_linear = smooth_velocity(current_linear, target_linear, MAX_LIN_ACCEL, MAX_LIN_DECEL, dt)
            current_angular = smooth_velocity(current_angular, target_angular, MAX_ANG_ACCEL, MAX_ANG_DECEL, dt)

            # Continuously stream base commands at 20 Hz
            set_base(current_linear, current_angular)

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
