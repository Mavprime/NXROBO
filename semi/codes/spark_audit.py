#!/usr/bin/env python3
"""Read-only ROS diagnostic for Spark semi-auto robot.

Does not publish commands, move hardware, change ROS parameters, or create files.

Usage (on robot):
    source ~/spark_noetic/devel/setup.bash
    python3 spark_audit.py --seconds 25

With the current controller already running, briefly test W/S, A/D, then release.
Compare CMD zero messages to odometry. Odometry is an estimate, not a physical
wheel sensor verification. Never test near people, props, or arena edges.
"""

import argparse
import collections
import threading
import time

import rospy
from geometry_msgs.msg import Twist
try:
    from nav_msgs.msg import Odometry
except ImportError:
    Odometry = None

TOPICS = ("/cmd_vel", "/position_write_topic", "/angle4th_topic", "/pump_topic", "/swiftpro_status_topic")


def publishers():
    try:
        code, message, state = rospy.get_master().getSystemState()
        if code != 1:
            raise RuntimeError(message)
        return dict(state[0])
    except Exception as exc:
        print("Could not query ROS master publishers:", exc)
        return {}


def display_publishers():
    pubs = publishers()
    print("\n=== ACTIVE ROS PUBLISHERS ===")
    for topic in TOPICS:
        names = pubs.get(topic, [])
        print("%-26s %s" % (topic + ":", ", ".join(names) if names else "(none)"))
        if topic in ("/cmd_vel", "/position_write_topic") and len(names) > 1:
            print("  !!! CONFLICT RISK: several nodes are publishing to the same actuator topic")
        if topic == "/angle4th_topic" and names:
            print("  !!! COMPETITION CHECK: wrist commands may be prohibited")


class Monitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.cmd = None
        self.odom = None
        self.cmd_received = 0
        self.odom_received = 0
        self.rate_times = collections.deque(maxlen=200)
        self.release_events = []
        self.pending = {}

    def on_cmd(self, msg):
        now = time.monotonic()
        lin, ang = msg.linear.x, msg.angular.z
        with self.lock:
            self.cmd_received += 1
            self.rate_times.append(now)
            if self.cmd is not None:
                for name, old, new in (("LINEAR", self.cmd[0], lin), ("TURN", self.cmd[1], ang)):
                    if abs(old) > 0.001 and abs(new) < 0.001:
                        self.release_events.append((name, now))
                        self.pending[name] = now
                        print("\n[%6.2fs] %s: /cmd_vel became ZERO" % (now - self.start, name), flush=True)
            self.cmd = (lin, ang, now)

    def on_odom(self, msg):
        now = time.monotonic()
        with self.lock:
            self.odom_received += 1
            self.odom = (msg.twist.twist.linear.x, msg.twist.twist.angular.z, now)
            if self.pending:
                for name, instant in list(self.pending.items()):
                    velocity = self.odom[0] if name == "LINEAR" else self.odom[1]
                    threshold = 0.015 if name == "LINEAR" else 0.04
                    elapsed = now - instant
                    if abs(velocity) <= threshold:
                        print("[%6.2fs] %s: /odom estimated near zero after %.3fs" %
                              (now - self.start, name, elapsed), flush=True)
                        del self.pending[name]
                    elif elapsed >= 2.0:
                        print("[%6.2fs] %s: /odom still shows %.3f after %.1fs (CHECK driver/other publishers)" %
                              (now - self.start, name, velocity, elapsed), flush=True)
                        del self.pending[name]

    def run(self, seconds):
        self.start = time.monotonic()
        rospy.Subscriber("/cmd_vel", Twist, self.on_cmd, queue_size=30)
        if Odometry:
            rospy.Subscriber("/odom", Odometry, self.on_odom, queue_size=30)
        print("\n=== READ-ONLY MONITOR (%.0fs) ===" % seconds)
        print("Drive briefly in SLOW mode in an empty area, then release W/S and A/D.")
        print("No /odom? The script can still check ROS commands and publishers.")
        rate = rospy.Rate(5)
        next_print = 0.0
        while not rospy.is_shutdown() and time.monotonic() - self.start < seconds:
            now = time.monotonic()
            if now >= next_print:
                with self.lock:
                    c = self.cmd
                    o = self.odom
                    r = list(self.rate_times)
                hz = ((len(r) - 1) / (r[-1] - r[0])) if len(r) >= 2 and r[-1] > r[0] else 0.0
                ctext = "%+.3f m/s, %+.3f rad/s" % c[:2] if c else "NO DATA"
                if o is None or now - o[2] > 2.0:
                    otext = "NO RECENT DATA"
                else:
                    otext = "%+.3f m/s, %+.3f rad/s" % o[:2]
                age = "%.2fs" % (now - c[2]) if c else "unknown"
                print("CMD [%s] rate %.1fHz age %s | ODOM [%s]" % (ctext, hz, age, otext), flush=True)
                next_print = now + 1.0
            rate.sleep()
        print("\n=== SUMMARY ===")
        print("Received /cmd_vel messages:", self.cmd_received)
        print("Received /odom messages:", self.odom_received)
        if not self.cmd_received:
            print("WARNING: no /cmd_vel detected: check active ROS master and base controller.")
        if not self.odom_received:
            print("No /odom feedback. Check 'rostopic list | grep odom' to find its actual topic.")
        print("Zero-velocity transitions observed:", len(self.release_events))
        display_publishers()
        print("NOTE: A zero /cmd_vel does NOT prove the wheels physically stopped.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=25)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 600:
        parser.error("--seconds must be between 1 and 600")
    rospy.init_node("spark_readonly_audit", anonymous=True)
    print("Spark semi-auto audit: READ ONLY; zero commands will NOT be sent by this script.")
    display_publishers()
    Monitor().run(args.seconds)


if __name__ == "__main__":
    main()
