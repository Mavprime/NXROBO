#!/usr/bin/env python3
"""Spark ROS controller: BASE + ARM + WRIST + VACUUM PUMP.

Low-latency stop update: browser/PC key-up immediately sends zero linear
AND angular velocity to /cmd_vel, bypassing the ramp for released axes only.
Acceleration on key-down remains smooth. A zero ROS velocity command cannot
force physical instant braking if the wheel controller or inertia coasts.

Run body.py in an interactive SSH/MobaXterm terminal to use the keyboard
without installing anything on Windows. Click/focus that terminal, then press
W/A/S/D or arrows to drive; use the original arm and pump keys as well.

Optional: the existing Windows keyboard_a.py can still connect over an SSH
port-forward for more accurate key-up detection and multi-key combinations.
No additional robot-side files are needed.

IMPORTANT: SSH terminals do not report true key releases; hold detection uses
key repeats and a timeout. Releasing a key cannot stop as precisely as the
Windows key-state controller. Do not run another /cmd_vel publisher at once.
"""

import json
import math
import os
import select
import shutil
import socket
import sys
import termios
import time
import tty

import rospy
from geometry_msgs.msg import Twist
from swiftpro.msg import SwiftproState, angle4th, position, status

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_ik import ik_reachable

HOST = "127.0.0.1"
PORT = int(os.environ.get("BODY_PORT", "8765"))
WATCHDOG = float(os.environ.get("BODY_WATCHDOG", "0.25"))
# SSH terminals send character repeats but no actual key-up events.
# Allow the OS initial-repeat delay before switching to a shorter timeout.
TERM_FIRST_REPEAT = float(os.environ.get("TERM_FIRST_REPEAT", "0.75"))
TERM_REPEAT_TIMEOUT = float(os.environ.get("TERM_REPEAT_TIMEOUT", "0.32"))
TERM_REARM_PAUSE = float(os.environ.get("TERM_REARM_PAUSE", "0.90"))
TERM_ACTION_GAP = float(os.environ.get("TERM_ACTION_GAP", "0.90"))

# Speed values preserved from the original full-body keyboard controller.
SPEED_MODES = {
    "FAST": {
        "lin_speed": float(os.environ.get("LIN_SPEED_FAST", "0.20")),
        "ang_speed": float(os.environ.get("ANG_SPEED_FAST", "0.75")),
        "arm_speed": float(os.environ.get("ARM_SPEED_FAST", "35.0")),
        "wrist_speed": float(os.environ.get("WRIST_SPEED_FAST", "35.0")),
    },
    "SLOW": {
        "lin_speed": float(os.environ.get("LIN_SPEED_SLOW", "0.07")),
        "ang_speed": float(os.environ.get("ANG_SPEED_SLOW", "0.25")),
        "arm_speed": float(os.environ.get("ARM_SPEED_SLOW", "12.0")),
        "wrist_speed": float(os.environ.get("WRIST_SPEED_SLOW", "15.0")),
    },
}

MAX_LIN_ACCEL = float(os.environ.get("MAX_LIN_ACCEL", "1.0"))
MAX_LIN_DECEL = float(os.environ.get("MAX_LIN_DECEL", "3.0"))
# Key-up is a stop request on that axis: don't ramp to zero, publish it now.
# Each option can be disabled to recover the older soft braking behavior.
INSTANT_LIN_RELEASE = os.environ.get("INSTANT_LIN_RELEASE", "1").lower() not in ("0", "false", "no")
INSTANT_ANG_RELEASE = os.environ.get("INSTANT_ANG_RELEASE", "1").lower() not in ("0", "false", "no")
MAX_ANG_ACCEL = float(os.environ.get("MAX_ANG_ACCEL", "3.0"))
MAX_ANG_DECEL = float(os.environ.get("MAX_ANG_DECEL", "6.0"))
ARM_PUB_PERIOD = float(os.environ.get("ARM_PUB_PERIOD", "0.05"))

HOME = (120.0, 0.0, 35.0)
HOME_ANGLE = 0.0
ARM_LIMITS = {"x": (0.0, 350.0), "y": (-160.0, 160.0), "z": (-85.0, 170.0)}
WRIST_LIMITS = (0.0, 180.0)

# Actual implemented coordinates in the OLD code (not outdated docstring).
ARM_PRESETS = {
    "h": ("Home", (120.0, 0.0, 35.0), 0.0),
    "v": ("Hover / Ready", (280.0, 0.0, 15.0), 0.0),
    "p": ("Pick / Touch", (280.0, 0.0, -68.0), 0.0),
    "c": ("Carry / Lift", (230.0, 0.0, 120.0), 0.0),
}

AXIS_CONFIG = {
    "x+": ("x", +1), "x-": ("x", -1),
    "y+": ("y", +1), "y-": ("y", -1),
    "z+": ("z", +1), "z-": ("z", -1),
    "angle+": ("angle", +1), "angle-": ("angle", -1),
}


def clamp(number, limits):
    return min(max(number, limits[0]), limits[1])


def toward(current, target, accel, decel, dt):
    """Rate-limit starts and stops; slow enough to avoid harsh starts."""
    braking = (target == 0.0 or current * target < 0.0 or
               abs(target) < abs(current))
    max_change = (decel if braking else accel) * dt
    diff = target - current
    if abs(diff) <= max_change:
        return target
    return current + math.copysign(max_change, diff)


def safe_print(*args):
    message = " ".join(str(a) for a in args)
    sys.stdout.write("\r\033[K" + message + "\r\n")
    sys.stdout.flush()


class ArmState:
    """Preserve original once-only initialization from SwiftproState feedback."""

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
    rospy.init_node("body_full_smooth", anonymous=True, argv=[sys.argv[0]])
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    pos_pub = rospy.Publisher("/position_write_topic", position, queue_size=1)
    angle_pub = rospy.Publisher("/angle4th_topic", angle4th, queue_size=1)
    pump_pub = rospy.Publisher("/pump_topic", status, queue_size=1)

    arm = ArmState()
    rospy.Subscriber("/SwiftproState_topic", SwiftproState, arm.callback, queue_size=1)
    rospy.sleep(0.3)

    deadline = time.monotonic() + 2.0
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if cmd_pub.get_num_connections() and pos_pub.get_num_connections():
            break
        rospy.sleep(0.05)
    if not cmd_pub.get_num_connections():
        safe_print("WARNING: no subscriber on /cmd_vel. Start Spark base driver.")
    if not pos_pub.get_num_connections():
        safe_print("WARNING: no subscriber on /position_write_topic. Start Swiftpro driver.")
    if not arm.initialised:
        safe_print("Arm pose unknown: use 0 to release and home/sync if appropriate.")

    def set_base(lin, ang):
        msg = Twist()
        msg.linear.x, msg.angular.z = lin, ang
        cmd_pub.publish(msg)

    def set_pump(on):
        msg = status()
        msg.status = 1 if on else 0
        pump_pub.publish(msg)

    def publish_pose():
        msg = position()
        msg.x, msg.y, msg.z = arm.x, arm.y, arm.z
        pos_pub.publish(msg)

    def publish_angle():
        msg = angle4th()
        msg.angle4th = arm.angle
        angle_pub.publish(msg)

    def go_home():
        arm.x, arm.y, arm.z = HOME
        arm.angle = HOME_ANGLE
        publish_pose()
        publish_angle()

    def apply_preset(key):
        label, coords, wrist_angle = ARM_PRESETS[key]
        x, y, z = coords
        if ik_reachable(x, y, z):
            arm.x, arm.y, arm.z = x, y, z
            arm.angle = wrist_angle
            publish_pose()
            publish_angle()
            safe_print(">> PRESET:", label, "(%.1f, %.1f, %.1f)" % coords)
        else:
            safe_print("WARNING: preset", label, "unreachable by IK")

    last_warn = 0.0

    def step_arm(axis, seconds, profile):
        nonlocal last_warn
        field, direction = AXIS_CONFIG[axis]
        if field == "angle":
            goal = clamp(arm.angle + profile["wrist_speed"] * seconds * direction,
                         WRIST_LIMITS)
            if goal != arm.angle:
                arm.angle = goal
                publish_angle()
                return
        else:
            pose = {"x": arm.x, "y": arm.y, "z": arm.z}
            pose[field] = clamp(pose[field] + profile["arm_speed"] * seconds * direction,
                                ARM_LIMITS[field])
            if ik_reachable(pose["x"], pose["y"], pose["z"]):
                arm.x, arm.y, arm.z = pose["x"], pose["y"], pose["z"]
                publish_pose()
                return
        if time.monotonic() - last_warn >= 0.5:
            safe_print("Arm limit:", field)
            last_warn = time.monotonic()

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((HOST, PORT))
        listener.listen(1)
        listener.setblocking(False)
    except Exception:
        listener.close()
        raise

    client = None
    buffer = b""
    current_mode = "FAST"
    pump_on = False
    input_linear = input_turn = 0
    input_arm = None
    emergency_held = False
    interlock = False  # after emergency/0, require all movement keys released
    actual_linear = actual_turn = 0.0
    last_state = 0.0
    last_loop = time.monotonic()
    last_arm_pub = last_hud = 0.0
    exit_requested = False
    rate = rospy.Rate(20)  # original Spark base command rate

    # Existing SSH terminal can be used as a local keyboard. With raw TTY input,
    # Ctrl+C is a literal byte which we detect and clean up ourselves.
    fd = sys.stdin.fileno()
    has_terminal = os.isatty(fd)
    terminal_settings = None
    pending_keys = bytearray()
    term_lin = term_turn = 0
    term_lin_until = term_turn_until = 0.0
    term_axis = None
    term_axis_until = 0.0
    term_last_key = {}     # distinguish first press from auto-repeat
    term_last_action = {}
    term_last_movement = 0.0
    term_interlock = False

    def stop_motion():
        nonlocal actual_linear, actual_turn, input_linear, input_turn, input_arm
        input_linear = input_turn = 0
        input_arm = None
        actual_linear = actual_turn = 0.0
        set_base(0.0, 0.0)  # bypass smoothing for emergency and disconnect

    def close_client():
        nonlocal client, buffer, last_state, interlock, emergency_held
        if client is not None:
            try:
                client.close()
            except OSError:
                pass
            client = None
        buffer = b""
        last_state = 0.0
        emergency_held = False
        interlock = True
        stop_motion()

    def process_action(key):
        nonlocal current_mode, pump_on, interlock, exit_requested
        if key == "space":
            interlock = True
            stop_motion()
            safe_print(">> EMERGENCY STOP (base and arm halted; pump unchanged)")
        elif key == "q":
            current_mode = "SLOW" if current_mode == "FAST" else "FAST"
            safe_print(">> SPEED MODE:", current_mode)
        elif key == "enter":
            pump_on = not pump_on
            set_pump(pump_on)
            safe_print("PUMP ON" if pump_on else "PUMP OFF")
        elif key in ARM_PRESETS:
            input_arm_stopper()  # cancel jog before preset
            apply_preset(key)
        elif key == "0":
            interlock = True
            stop_motion()
            pump_on = False
            set_pump(False)
            go_home()
            safe_print(">> RELEASE + HOME")
        elif key == "exit":
            exit_requested = True
            stop_motion()

    def input_arm_stopper():
        nonlocal input_arm
        input_arm = None

    def process_line(raw):
        nonlocal input_linear, input_turn, input_arm, last_state, emergency_held, interlock
        nonlocal actual_linear, actual_turn
        if len(raw) > 1024:
            return
        try:
            msg = json.loads(raw.decode("ascii"))
        except (ValueError, UnicodeError):
            return
        if not isinstance(msg, dict):
            return
        if msg.get("kind") == "action":
            key = msg.get("key")
            if isinstance(key, str) and key in ("space", "q", "enter", "h", "v", "p", "c", "0", "exit"):
                process_action(key)
            return
        if msg.get("kind") != "state":
            return
        lin, turn, axis, stop = (msg.get("linear"), msg.get("turn"),
                                 msg.get("arm"), msg.get("stop"))
        if (type(lin) is not int or lin not in (-1, 0, 1) or
                type(turn) is not int or turn not in (-1, 0, 1) or
                (axis is not None and (type(axis) is not str or axis not in AXIS_CONFIG)) or
                type(stop) is not bool):
            return
        last_state = time.monotonic()
        emergency_held = stop
        if stop:
            interlock = True
            stop_motion()
            return
        if interlock:
            if lin == 0 and turn == 0 and axis is None:
                interlock = False
            stop_motion()
            return
        # A browser key-up can arrive between ROS loop ticks. Apply the STOP
        # directly here so both wheel axes are zeroed without ramp or timer.
        released_linear = input_linear != 0 and lin == 0
        released_turn = input_turn != 0 and turn == 0
        input_linear, input_turn, input_arm = lin, turn, axis
        changed = False
        if released_linear and INSTANT_LIN_RELEASE:
            actual_linear = 0.0
            changed = True
        if released_turn and INSTANT_ANG_RELEASE:
            actual_turn = 0.0
            changed = True
        if changed:
            # Preserve whichever axis is still held; do not interrupt arc driving.
            set_base(actual_linear, actual_turn)

    safe_print("Spark combined base + arm + pump controller on", HOST, PORT)
    safe_print("Keyboard:", "TERMINAL (no Windows Python)" if has_terminal else "NONE (no TTY)")
    safe_print("Focus this terminal to drive; Ctrl+C exits. Optional PC keyboard_a.py is supported.")
    safe_print("WASD/arrows: base | 8 2 7 9 4 6 1 3: arm/wrist")
    safe_print("Q: FAST/SLOW | Space: emergency stop | Enter: pump | H V P C: presets | 0: release/home")
    safe_print("Emergency stop stays active until ALL movement keys are released.")
    safe_print("NOTE: SSH key-release detection is approximate; first release may take ~0.75s.")

    # The terminal owns input when no remote PC client is connected.
    # Key-down repeats may be absent or late; never treat this as precise key-up.
    def read_terminal(now):
        nonlocal term_lin, term_turn, term_axis, term_lin_until, term_turn_until
        nonlocal term_axis_until, term_last_movement, term_interlock, exit_requested

        if not has_terminal:
            return
        # Bound work per tick so serial/ROS base updates stay at 20 Hz.
        for _ in range(4):
            if not select.select([fd], [], [], 0)[0]:
                break
            chunk = os.read(fd, 256)
            if not chunk:
                break
            pending_keys.extend(chunk)
            if len(pending_keys) > 2048:
                pending_keys.clear()
                break

        while pending_keys:
            if pending_keys[0] == 27:  # arrows: ESC [ A/B/C/D or ESC O A/B/C/D
                if len(pending_keys) < 2:
                    break
                if pending_keys[1] not in (ord('['), ord('O')):
                    del pending_keys[0]
                    continue
                if len(pending_keys) < 3:
                    break
                key_byte = pending_keys[2]
                del pending_keys[:3]
                key = {ord('A'): 'w', ord('B'): 's',
                       ord('C'): 'd', ord('D'): 'a'}.get(key_byte)
                if key is None:
                    continue
            else:
                key_byte = pending_keys.pop(0)
                if key_byte == 3:  # Ctrl+C in raw terminal mode
                    exit_requested = True
                    stop_motion()
                    return
                if key_byte in (10, 13):
                    key = 'enter'
                else:
                    key = chr(key_byte).lower()

            if key == ' ':
                if not term_interlock:
                    process_action('space')
                else:
                    stop_motion()  # repeated Space stays stopped; no log flood
                term_interlock = True
                term_last_movement = now
                term_lin = term_turn = 0
                term_axis = None
                term_lin_until = term_turn_until = term_axis_until = 0.0
                continue

            # Single-shot actions would otherwise auto-repeat and rapidly
            # toggle the pump/mode, or spam arm preset requests.
            if key in ('q', 'enter', 'h', 'v', 'p', 'c', '0'):
                # Update last-seen even for discarded repeats: a held Enter
                # must never flip the pump on/off every repeat interval.
                prior_action = term_last_action.get(key, -100.0)
                term_last_action[key] = now
                if now - prior_action >= TERM_ACTION_GAP:
                    process_action(key)
                    if key in ARM_PRESETS or key == '0':
                        term_axis = None
                        term_axis_until = 0.0
                    if key == '0':
                        term_interlock = True
                        term_last_movement = now
                        term_lin = term_turn = 0
                        term_lin_until = term_turn_until = 0.0
                continue

            direction = {
                'w': ('linear', 1), 's': ('linear', -1),
                'a': ('turn', 1), 'd': ('turn', -1),
                '8': ('arm', 'z+'), '2': ('arm', 'z-'),
                '7': ('arm', 'x+'), '9': ('arm', 'x-'),
                '4': ('arm', 'y+'), '6': ('arm', 'y-'),
                '1': ('arm', 'angle+'), '3': ('arm', 'angle-'),
            }.get(key)
            if direction is None:
                continue
            term_last_movement = now
            prior = term_last_key.get(key, -100.0)
            term_last_key[key] = now
            if term_interlock or client is not None:
                continue
            # First repeat often arrives 0.5-0.7s after the initial key.
            # Once repeats flow, use a shorter key-release approximation.
            ttl = (TERM_REPEAT_TIMEOUT if now - prior <= TERM_FIRST_REPEAT
                   else TERM_FIRST_REPEAT)
            axis, value = direction
            if axis == 'linear':
                term_lin = value
                term_lin_until = now + ttl
            elif axis == 'turn':
                term_turn = value
                term_turn_until = now + ttl
            else:
                term_axis = value
                term_axis_until = now + ttl

        if term_interlock:
            # Don't resume if a key is still held (repeat stream continues).
            if now - term_last_movement > TERM_REARM_PAUSE:
                term_interlock = False
                safe_print("Keyboard rearmed: movement enabled")

    try:
        if has_terminal:
            terminal_settings = termios.tcgetattr(fd)
            tty.setraw(fd)
        set_base(0.0, 0.0)
        while not rospy.is_shutdown() and not exit_requested:
            now = time.monotonic()
            dt = min(max(now - last_loop, 0.0), 0.1)
            last_loop = now

            read_terminal(now)
            if exit_requested:
                break

            watched = [listener] + ([client] if client is not None else [])
            readable, _, _ = select.select(watched, [], [], 0.0)
            if listener in readable:
                new_client, _ = listener.accept()
                if client is not None:
                    new_client.close()  # one operator at a time
                else:
                    client = new_client
                    client.setblocking(False)
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    buffer = b""
                    last_state = 0.0
                    interlock = True
                    term_interlock = True
                    term_last_movement = now
                    stop_motion()
                    safe_print("Windows keyboard connected")

            if client is not None and client in readable:
                disconnected = False
                try:
                    data = client.recv(8192)
                    if not data:
                        disconnected = True
                    else:
                        buffer += data
                        if len(buffer) > 8192:
                            disconnected = True
                        else:
                            *messages, buffer = buffer.split(b"\n")
                            for raw in messages:
                                process_line(raw)
                except (OSError, ConnectionError):
                    disconnected = True
                if disconnected:
                    close_client()
                    term_interlock = True
                    term_last_movement = now
                    safe_print("Keyboard disconnected: base and arm stopped; pump unchanged")

            if client is not None:
                # Network mode: only the existing Windows key-state controller
                # can produce precise multi-key held states.
                if time.monotonic() - last_state > WATCHDOG:
                    if input_linear or input_turn or input_arm is not None:
                        stop_motion()
                        interlock = True
            else:
                # Local TTY mode: approximate key-up from repeats. Independent
                # linear/turn timeouts do not artificially keep motion going.
                if term_interlock:
                    input_linear = input_turn = 0
                    input_arm = None
                else:
                    input_linear = term_lin if now < term_lin_until else 0
                    input_turn = term_turn if now < term_turn_until else 0
                    input_arm = term_axis if now < term_axis_until else None

            if not exit_requested:
                profile = SPEED_MODES[current_mode]
                target_linear = input_linear * profile["lin_speed"]
                target_turn = input_turn * profile["ang_speed"]
                # Acceleration stays smooth on key-down. Both axes immediately
                # go to zero after key-up or a lost-input watchdog timeout.
                if target_linear == 0.0 and INSTANT_LIN_RELEASE:
                    actual_linear = 0.0
                else:
                    actual_linear = toward(actual_linear, target_linear, MAX_LIN_ACCEL, MAX_LIN_DECEL, dt)
                if target_turn == 0.0 and INSTANT_ANG_RELEASE:
                    actual_turn = 0.0
                else:
                    actual_turn = toward(actual_turn, target_turn, MAX_ANG_ACCEL, MAX_ANG_DECEL, dt)
                set_base(actual_linear, actual_turn)

                if input_arm is not None and now - last_arm_pub >= ARM_PUB_PERIOD:
                    step_dt = min(now - last_arm_pub, 0.1) if last_arm_pub else ARM_PUB_PERIOD
                    last_arm_pub = now
                    step_arm(input_arm, step_dt, profile)
                elif input_arm is None:
                    last_arm_pub = 0.0

            # Compact, terminal-width-aware status HUD. Reserving the last
            # column prevents terminal auto-wrap and the flood of repeated
            # lines seen in narrow MobaXterm/SSH windows.
            if now - last_hud >= 0.1:
                hud = (
                    f"{current_mode} "
                    f"V:{actual_linear:+.2f} "
                    f"W:{actual_turn:+.2f} "
                    f"ARM:{arm.x:.0f},{arm.y:.0f},{arm.z:.0f},{arm.angle:.0f} "
                    f"P:{'ON' if pump_on else 'OFF'} "
                    f"KB:{'PC' if client else ('ON' if has_terminal else 'OFF')}"
                )
                if sys.stdout.isatty():
                    width = shutil.get_terminal_size(fallback=(80, 24)).columns
                    if width > 1:
                        sys.stdout.write("\r\033[2K" + hud[:width - 1])
                        sys.stdout.flush()
                last_hud = now

            rate.sleep()
    finally:
        if terminal_settings is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, terminal_settings)
        for _ in range(5):
            set_base(0.0, 0.0)
            time.sleep(0.02)
        # Preserve the ORIGINAL Ctrl+C / exit behavior: pump OFF on full exit.
        set_pump(False)
        if client is not None:
            client.close()
        listener.close()
        safe_print("\nStopped: base stopped, arm jogging halted, pump off")


if __name__ == "__main__":
    main()
