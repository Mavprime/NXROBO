#!/usr/bin/env python3
"""Windows-side keyboard for the Spark BASE + ARM + WRIST + PUMP.

This replaces the contents of the existing Windows-side keyboard_a.py.
It uses actual press/release states, not SSH terminal auto-repeat.

On Windows (local PowerShell, NOT inside SSH):
    py -m pip install pygame
    py keyboard_a.py

Have an SSH terminal running in parallel with port forwarding:
    ssh -L 8765:127.0.0.1:8765 spark@10.25.141.8
    source ~/spark_noetic/devel/setup.bash
    python3 ~/semi/body.py

Click the pygame controller window to focus it. Unfocusing stops base/arm.
Full exit (Esc, Ctrl+C, or close window) commands robot script to exit and
turn pump OFF, matching the original teleop's full shutdown behavior.
A network disconnect stops base/arm without releasing the pump.
"""

import json
import socket
import sys

try:
    import pygame
except ImportError:
    raise SystemExit("pygame missing. In WINDOWS PowerShell: py -m pip install pygame")

HOST = "127.0.0.1"
PORT = 8765
FPS = 30

# Same directions as in the original keyboard code.
ARM_BUTTONS = (
    ((pygame.K_8, pygame.K_KP8), "z+"),
    ((pygame.K_2, pygame.K_KP2), "z-"),
    ((pygame.K_7, pygame.K_KP7), "x+"),
    ((pygame.K_9, pygame.K_KP9), "x-"),
    ((pygame.K_4, pygame.K_KP4), "y+"),
    ((pygame.K_6, pygame.K_KP6), "y-"),
    ((pygame.K_1, pygame.K_KP1), "angle+"),
    ((pygame.K_3, pygame.K_KP3), "angle-"),
)

ACTION_BUTTONS = {
    pygame.K_q: "q", pygame.K_SPACE: "space",
    pygame.K_RETURN: "enter", pygame.K_KP_ENTER: "enter",
    pygame.K_h: "h", pygame.K_v: "v", pygame.K_p: "p", pygame.K_c: "c",
    pygame.K_0: "0", pygame.K_KP0: "0",
}


def send(sock, kind, **fields):
    packet = json.dumps({"kind": kind, **fields}, separators=(",", ":")) + "\n"
    sock.sendall(packet.encode("ascii"))


def held(keys, *codes):
    return any(keys[code] for code in codes)


def read_state(keys, focused):
    if not focused:
        return 0, 0, None, False
    linear = int(held(keys, pygame.K_w, pygame.K_UP)) - int(held(keys, pygame.K_s, pygame.K_DOWN))
    turn = int(held(keys, pygame.K_a, pygame.K_LEFT)) - int(held(keys, pygame.K_d, pygame.K_RIGHT))
    arm = next((axis for codes, axis in ARM_BUTTONS if held(keys, *codes)), None)
    emergency = bool(keys[pygame.K_SPACE])
    return linear, turn, arm, emergency


def main():
    pygame.init()
    screen = pygame.display.set_mode((710, 365))
    pygame.display.set_caption("Spark: Base + Arm + Wrist + Vacuum Controller")
    font = pygame.font.SysFont(None, 27)
    small = pygame.font.SysFont(None, 23)
    clock = pygame.time.Clock()

    try:
        sock = socket.create_connection((HOST, PORT), timeout=3)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(0.3)
    except OSError as exc:
        pygame.quit()
        raise SystemExit("Cannot reach Spark through SSH port forward 8765: %s" % exc)

    print("Connected. Click controller window. NumLock ON for numpad arm jogging.")
    print("ESC / Ctrl+C / window close: exit and switch pump off.")
    mode = "FAST"
    running = True
    held_action_keys = set()
    lost_connection = False
    try:
        while running:
            events = pygame.event.get()
            focused = bool(pygame.key.get_focused())
            for event in events:
                if event.type == pygame.QUIT:
                    running = False
                    break
                if event.type == pygame.WINDOWFOCUSLOST:
                    held_action_keys.clear()
                if event.type == pygame.KEYUP:
                    held_action_keys.discard(event.key)
                if event.type != pygame.KEYDOWN or not focused:
                    continue
                if event.key == pygame.K_ESCAPE or (event.key == pygame.K_c and event.mod & pygame.KMOD_CTRL):
                    running = False
                    break
                if event.key in held_action_keys:
                    continue  # repeat KEYDOWN must never repeat Enter/preset/q/0
                held_action_keys.add(event.key)
                action = ACTION_BUTTONS.get(event.key)
                if action is not None:
                    send(sock, "action", key=action)
                    if action == "q":
                        mode = "SLOW" if mode == "FAST" else "FAST"

            keys = pygame.key.get_pressed()
            linear, turn, arm, stop = read_state(keys, focused and running)
            send(sock, "state", linear=linear, turn=turn, arm=arm, stop=stop)

            screen.fill((21, 26, 36))
            rows = [
                ("Spark Robot: Full Keyboard Control", (237, 242, 250), font),
                ("W/S or Up/Down: forward/back    A/D or Left/Right: steering", (199, 214, 235), small),
                ("Hold W+A or W+D: smooth arc turns", (199, 214, 235), small),
                ("Arm: 8/2 Z, 7/9 X, 4/6 Y, 1/3 wrist   (NumLock ON)", (199, 214, 235), small),
                ("H: Home   V: Hover   P: Pick   C: Carry", (199, 214, 235), small),
                ("Enter: pump on/off   0: release + home   Q: FAST/SLOW", (199, 214, 235), small),
                ("Space: emergency stop (pump stays ON); release all keys before resuming", (246, 205, 137), small),
                ("Esc / Ctrl+C: exit (pump OFF); losing focus stops movement", (246, 205, 137), small),
                ("MODE: %s | Drive: %s | Turn: %s | Arm: %s" %
                 (mode, linear, turn, arm or "-"), (142, 232, 174), small),
            ]
            for i, (text, color, text_font) in enumerate(rows):
                screen.blit(text_font.render(text, True, color), (24, 18 + 37 * i))
            pygame.display.flip()
            clock.tick(FPS)
    except (OSError, KeyboardInterrupt) as exc:
        lost_connection = isinstance(exc, OSError)
        print("Controller ended:", exc)
    finally:
        try:
            # Explicit clean exit -> robot disables pump, as old Ctrl+C did.
            if not lost_connection:
                send(sock, "action", key="exit")
            else:
                send(sock, "state", linear=0, turn=0, arm=None, stop=False)
        except OSError:
            pass
        sock.close()
        pygame.quit()
        print("Keyboard controller stopped")


if __name__ == "__main__":
    main()
