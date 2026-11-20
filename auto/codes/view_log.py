#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive log viewer for Spark robot.
Follows log in real-time. Exits instantly on 'q', 'Q', 'Enter', 'Esc', or Ctrl+C.
"""

import os
import sys
import time
import select
import termios
import tty

LOG_PATH = os.path.expanduser("~/logs/runtask.log")


def getch_nonblocking():
    dr, _, _ = select.select([sys.stdin], [], [], 0.05)
    if dr:
        return sys.stdin.read(1)
    return None


def main():
    if not os.path.exists(LOG_PATH):
        os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
        with open(LOG_PATH, "a") as f:
            f.write("=== LOG CREATED ===\n")

    print("\033[96m======================================================\033[0m")
    print("\033[93m  LIVE RUNTASK LOG. Press 'q' or Ctrl+C to exit\033[0m")
    print("\033[96m======================================================\033[0m")

    with open(LOG_PATH, "r", errors="ignore") as f:
        lines = f.readlines()
        for line in lines[-25:]:
            sys.stdout.write(line)
        sys.stdout.flush()
        f.seek(0, os.SEEK_END)

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                line = f.readline()
                if line:
                    sys.stdout.write(line)
                    sys.stdout.flush()
                else:
                    time.sleep(0.08)

                ch = getch_nonblocking()
                if ch in ('q', 'Q', '\x1b', '\n', '\r', '\x03'):
                    break
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    print("\n\033[92m>> Returning to main menu...\033[0m")
    time.sleep(0.3)


if __name__ == "__main__":
    main()
