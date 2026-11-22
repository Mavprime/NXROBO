#!/usr/bin/env python3
"""One-shot active pad check, exactly as runtask.py performs it at startup."""
import sys

import rospy

sys.path.insert(0, "/home/spark/codes")

import match_logic  # noqa: E402
import runtask  # noqa: E402


def main():
    rospy.init_node("pad_selfcheck", anonymous=True, disable_signals=True)
    cam = runtask.CamAction()
    arm = runtask.ArmAction(cam=cam)

    pose = runtask.ArmAction.CALIB_CHECK_POSE
    projection = match_logic.project_arm(
        cam.transform3d, pose[0], pose[1], pose[2])
    print("[CHECK] pose=%s" % (pose,))
    print("[CHECK] matrix projection: %s" % (projection,))

    ok, detail = arm.verify_calibration()
    print("[CHECK] measured/compared: %s" % detail)
    print("[CHECK] verify_calibration -> ok=%s" % ok)
    print("[CHECK] meta_stale=%s calib_md5=%s"
          % (getattr(cam, "calib_meta_stale", None),
             (cam.calib_md5 or "")[:8]))

    try:
        arm.arm_home(block=True)
        print("[CHECK] arm parked")
    except Exception as exc:
        print("[CHECK] could not park: %s" % exc)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
