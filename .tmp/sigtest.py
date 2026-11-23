#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""No-motion rospy signal test: compare default handlers vs disable_signals.

Usage: python3 sigtest.py {default|custom}
Fires SIGINT at itself 0.4 s into a 2 s rospy.sleep, then checks:
  - was an exception raised into the sleeping code?
  - how long did the sleep actually last?
  - can a publisher created BEFORE the signal still publish afterwards?
"""
import os
import signal
import sys
import threading
import time

import rospy
from std_msgs.msg import String

MODE = sys.argv[1] if len(sys.argv) > 1 else "default"


def main():
    disable = (MODE == "custom")
    rospy.init_node("sigtest_%s" % MODE, anonymous=True, disable_signals=disable)
    if disable:
        def _raise_interrupt(signum, _frame):
            raise KeyboardInterrupt("signal %d" % signum)
        signal.signal(signal.SIGINT, _raise_interrupt)
        signal.signal(signal.SIGTERM, _raise_interrupt)

    pub = rospy.Publisher("/grasp_sigtest", String, queue_size=1)

    def fire():
        time.sleep(0.4)
        os.kill(os.getpid(), signal.SIGINT)

    threading.Thread(target=fire, daemon=True).start()

    started = time.time()
    raised = None
    try:
        rospy.sleep(2.0)
    except BaseException as exc:  # noqa: BLE001 - we want to see anything
        raised = "%s: %s" % (type(exc).__name__, exc)
    elapsed = time.time() - started

    try:
        impl_closed = pub.impl.closed if pub.impl is not None else "no_impl"
    except Exception as exc:
        impl_closed = "err:%s" % exc

    print("mode=%s raised=%s elapsed=%.2f is_shutdown=%s pub_closed=%s"
          % (MODE, raised, elapsed, rospy.is_shutdown(), impl_closed))

    published = None
    try:
        pub.publish(String(data="after-signal"))
        published = "OK (no exception)"
    except BaseException as exc:  # noqa: BLE001
        published = "EXC %s: %s" % (type(exc).__name__, exc)
    print("publish_after_signal=%s" % published)

    try:
        rospy.signal_shutdown("sigtest done")
    except BaseException:
        pass
    time.sleep(0.3)


if __name__ == "__main__":
    main()
