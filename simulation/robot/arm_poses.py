#!/usr/bin/env python3
"""Named arm poses.

Rule (user, 2026-11-24): the arm rests at HOME (120, 0, 35) mm. The earlier
"scanning position" (50, 180, 160) rule is deleted - the base drives with the
arm at home; nothing parks it on the side before a leg.

Publishing is best-effort: if swiftpro messages are unavailable (e.g. a
simulation image without the vendor stack) the helpers silently no-op, so the
same mission code runs in sim and on hardware.
"""

from __future__ import annotations

import rospy

HOME_POSE = (120.0, 0.0, 35.0)     # x, y, z in mm - home / resting pose

_pub = None


def _publisher():
    global _pub
    if _pub is not None:
        return _pub
    try:
        from swiftpro.msg import position as SwPosition  # noqa: F401
    except Exception:
        return None
    from swiftpro.msg import position as SwPosition
    _pub = rospy.Publisher("/position_write_topic", SwPosition, queue_size=1)
    rospy.sleep(0.2)
    return _pub


def send_pose(pose, repeats: int = 6, settle: float = 0.6) -> bool:
    """Publish a Cartesian arm pose (mm). Returns False if the arm is absent."""
    pub = _publisher()
    if pub is None:
        return False
    from swiftpro.msg import position as SwPosition
    for _ in range(repeats):
        pub.publish(SwPosition(x=float(pose[0]), y=float(pose[1]), z=float(pose[2])))
        rospy.sleep(0.12)
    if settle:
        rospy.sleep(settle)
    return True


def to_home(settle: float = 0.6) -> bool:
    """Send the arm to the home pose."""
    return send_pose(HOME_POSE, settle=settle)
