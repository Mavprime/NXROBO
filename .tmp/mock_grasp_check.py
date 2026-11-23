#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Local (no robot, no ROS) behavioural check of the rewritten grasp_generic.py.

Stubs rospy/runtask/sensor_msgs, then executes the real production sequence
against a recording fake arm.  Verifies the operator-requested algorithm and
the "never release the cube in the air" invariant, including the Abort path.
"""
import importlib.util
import os
import sys
import time
import types

CODES = r"D:\NXROBO\auto\codes"
sys.path.insert(0, CODES)

FAILURES = []


def check(name, condition, detail=""):
    if condition:
        print("  [PASS] %s" % name)
    else:
        print("  [FAIL] %s %s" % (name, detail))
        FAILURES.append(name)


# ---------------------------------------------------------------------------
# ROS stubs
# ---------------------------------------------------------------------------
EVENTS = []

rospy_stub = types.ModuleType("rospy")


def _stub_ros_sleep(seconds):
    EVENTS.append(("sleep", round(float(seconds), 3)))
    hook = rospy_stub.sleep_hook
    if hook is not None:
        exc = hook(seconds)
        if exc is not None:
            raise exc


rospy_stub.sleep = _stub_ros_sleep
rospy_stub.sleep_hook = None
rospy_stub.is_shutdown = lambda: False
rospy_stub.myargv = lambda argv=None: list(argv if argv is not None else sys.argv)
rospy_stub.wait_for_message = lambda *a, **k: (_ for _ in ()).throw(
    RuntimeError("wait_for_message must not be called by the sequence"))
rospy_stub.init_node = lambda *a, **k: None
rospy_stub.loginfo = lambda *a, **k: None
rospy_stub.logwarn = lambda *a, **k: None
rospy_stub.logerr = lambda *a, **k: None
rospy_stub.loginfo_throttle = lambda *a, **k: None
rospy_stub.Rate = object
sys.modules["rospy"] = rospy_stub

sensor_msgs = types.ModuleType("sensor_msgs")
sensor_msgs.__path__ = []
sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")


class _Image(object):
    pass


sensor_msgs_msg.Image = _Image
sensor_msgs.msg = sensor_msgs_msg
sys.modules["sensor_msgs"] = sensor_msgs
sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

runtask_stub = types.ModuleType("runtask")


class _RuntaskArmAction(object):
    pass


class _RuntaskCamAction(object):
    pass


runtask_stub.ArmAction = _RuntaskArmAction
runtask_stub.CamAction = _RuntaskCamAction
sys.modules["runtask"] = runtask_stub

spec = importlib.util.spec_from_file_location(
    "grasp_generic", os.path.join(CODES, "grasp_generic.py"))
gg = importlib.util.module_from_spec(spec)
sys.modules["grasp_generic"] = gg
spec.loader.exec_module(gg)


# ---------------------------------------------------------------------------
# Recording fake arm
# ---------------------------------------------------------------------------
class FakePublisher(object):
    def get_num_connections(self):
        return 1


class FakeInterface(object):
    def get_pose_events(self):
        return [(i, e) for i, e in enumerate(EVENTS) if e[0] == "pose"]

    def __init__(self):
        self.arm_position_pub = FakePublisher()

    def set_pose(self, x, y, z, delay=0.35):
        EVENTS.append(("pose", round(float(x), 1), round(float(y), 1),
                       round(float(z), 1)))
        if delay and delay > 0:
            rospy_stub.sleep(delay)

    def set_pump(self, enable):
        EVENTS.append(("pump", bool(enable)))

    def set_status(self, lock):
        EVENTS.append(("status", bool(lock)))


class FakeArm(object):
    def __init__(self):
        self.interface = FakeInterface()


def poses():
    return [e for e in EVENTS if e[0] == "pose"]


def pumps():
    return [(i, e) for i, e in enumerate(EVENTS) if e[0] == "pump"]


def index_of_predicate(predicate):
    for i, event in enumerate(EVENTS):
        if predicate(event):
            return i
    return -1


def last_pose_before(event_index):
    """Closest wrist command issued before the given event index."""
    for event in reversed(EVENTS[:event_index]):
        if event[0] == "pose":
            return event
    return None


def floor_plan():
    target = {"mode": "floor", "class": 3, "xy": [221.0, -12.0],
              "pixel": [300.0, 250.0], "z_est": -115.0}
    return gg.plan_grasp(target)


def no_hold_hook(seconds):
    return None


def interrupt_in_hold(seconds):
    # The operator presses Ctrl-C during the 5 s hold.
    if abs(seconds - 5.0) < 1e-6:
        return RuntimeError("simulated Ctrl-C during hold")
    return None


def reset(events_hook=None):
    del EVENTS[:]
    rospy_stub.sleep_hook = events_hook
    return FakeArm()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def test_plan_heights():
    print("\n[1] plan_grasp heights")
    floor = floor_plan()
    check("floor approach z=70", floor.z_approach == 70.0, floor)
    check("floor touch z=-58", floor.z_touch == -58.0, floor)
    check("floor lift z=60", floor.z_lift == 60.0, floor)
    check("floor X/Y kept", (floor.x, floor.y) == (221.0, -12.0), floor)

    cake = gg.plan_grasp({"mode": "cake", "class": 2, "xy": [300.0, 5.0],
                          "pixel": [320.0, 240.0], "z_est": 20.0}, 35.0)
    check("elevated cake approach z=75", cake.z_approach == 75.0, cake)
    check("elevated cake touch z=32", cake.z_touch == 32.0, cake)
    check("elevated cake lift z=80", cake.z_lift == 80.0, cake)

    cake_floor = gg.plan_grasp({"mode": "cake", "class": 2, "xy": [250.0, 0.0],
                                "pixel": [320.0, 240.0], "z_est": -120.0}, 35.0)
    check("cake on floor uses floor heights", cake_floor.z_touch == -58.0, cake_floor)

    # The firmware IK refuses this pose silently: the plan must be rejected
    # instead of driving the arm to an unreachable point and reporting DONE.
    try:
        gg.plan_grasp({"mode": "floor", "class": 1, "xy": [140.0, 0.0],
                       "pixel": [320.0, 270.0], "z_est": -115.0})
        rejected = None
    except gg.GraspError as exc:
        rejected = exc
    check("unreachable floor target rejected before motion",
          rejected is not None and rejected.result == "UNREACHABLE", rejected)
    check("reach_state marks the same pose out of reach",
          gg.reach_state("floor", 140.0, 0.0, -115.0)[0] is False)
    check("reach_state marks a normal pose in reach",
          gg.reach_state("floor", 240.0, 0.0, -115.0)[0] is True)


def test_full_sequence():
    print("\n[2] full sequence: lower -> suction -> lift -> hold 5 s -> return -> park")
    arm = reset()
    plan = floor_plan()
    gg.execute_sequence(arm, plan, hold_seconds=5.0, return_cube=True)

    pose_events = poses()
    pose_values = [(e[1], e[2], e[3]) for e in pose_events]
    pump_events = pumps()
    pump_on_idx = [i for i, e in pump_events if e[1]]
    pump_off_idx = [i for i, e in pump_events if not e[1]]

    check("manipulator lowered first (first pose is the high transit)",
          pose_values[0] == (140.0, round(plan.y * 0.4, 1), 70.0), pose_values[0])
    check("then above the cube", pose_values[1] == (plan.x, plan.y, 70.0),
          pose_values[1])
    check("pump switched ON exactly once", len(pump_on_idx) == 1, pump_events)
    check("pump switched OFF exactly once (normal flow)", len(pump_off_idx) == 1,
          pump_events)
    check("suction starts before the descent",
          pump_on_idx[0] < index_of_predicate(
              lambda e: e[0] == "pose" and e[3] == plan.z_touch), EVENTS[:5])
    check("vertical descent to contact", pose_values[2] == (plan.x, plan.y, plan.z_touch),
          pose_values[2])
    check("vertical lift to z=60", pose_values[3] == (plan.x, plan.y, plan.z_lift),
          pose_values[3])
    check("5 s hold happened after the lift",
          ("sleep", 5.0) in EVENTS
          and EVENTS.index(("sleep", 5.0)) > index_of_predicate(
              lambda e: e[0] == "pose" and e[3] == plan.z_lift),
          [e for e in EVENTS if e[0] == "sleep"])
    check("no side move while holding (return descent is vertical)",
          pose_values[4] == (plan.x, plan.y, plan.z_touch), pose_values[4])
    check("pump released only on the ground",
          last_pose_before(pump_off_idx[0]) ==
          ("pose", plan.x, plan.y, plan.z_touch),
          last_pose_before(pump_off_idx[0]))
    check("vertical retract before leaving", pose_values[5] == (plan.x, plan.y, 70.0),
          pose_values[5])
    check("arm parked in the initial pose", pose_values[-1] == gg.PARK_POSE,
          pose_values[-1])
    check("exactly 8 wrist commands", len(pose_values) == 8, pose_values)


def test_custom_landmarks():
    print("\n[3] integration landmarks: park_pose / transit_xy are overridable")
    arm = reset()
    plan = floor_plan()
    gg.execute_sequence(arm, plan, hold_seconds=0.01, return_cube=True,
                        park_pose=(10.0, 20.0, 30.0), transit_xy=(120.0, 0.5))
    pose_values = [(e[1], e[2], e[3]) for e in poses()]
    check("custom transit used for the first waypoint",
          pose_values[0] == (120.0, round(plan.y * 0.5, 1), 70.0), pose_values[0])
    check("custom transit used on the way back",
          pose_values[-2] == (120.0, 0.0, 70.0), pose_values[-2])
    check("custom park pose used", pose_values[-1] == (10.0, 20.0, 30.0),
          pose_values[-1])


def test_hold_only_mode():
    print("\n[4] --no-place preview: cube stays attached, no put-back, no park")
    arm = reset()
    plan = floor_plan()
    gg.execute_sequence(arm, plan, hold_seconds=5.0, return_cube=False)

    pose_values = [(e[1], e[2], e[3]) for e in poses()]
    pump_events = pumps()
    check("only 4 wrist commands (lower, above, touch, lift)", len(pose_values) == 4,
          pose_values)
    check("last pose is the raised lift pose",
          pose_values[-1] == (plan.x, plan.y, plan.z_lift), pose_values[-1])
    check("pump never released", all(e[1] for e in pump_events), pump_events)
    check("hold of 5 s still executed", ("sleep", 5.0) in EVENTS, EVENTS)


def test_abort_scenarios():
    print("\n[5] interrupts at lift / hold / return-descent: cube returned, "
          "never released in the air")

    def hold_hook(counter):
        def hook(seconds):
            # Ctrl-C during the 5 s hold (our own SIGINT handler raises this).
            if abs(seconds - 5.0) < 1e-6:
                return KeyboardInterrupt("simulated Ctrl-C during hold")
            return None
        return hook

    def lift_hook(counter):
        def hook(seconds):
            # Ctrl-C while the arm is raising the cube.
            if abs(seconds - gg.LIFT_TIME) < 1e-6:
                return KeyboardInterrupt("simulated Ctrl-C during lift")
            return None
        return hook

    def return_descent_hook(counter):
        def hook(seconds):
            # Ctrl-C while the cube is being lowered back (2nd use of 1.75 s).
            if abs(seconds - gg.DESCENT_TIME) < 1e-6:
                counter[0] += 1
                if counter[0] == 2:
                    return KeyboardInterrupt(
                        "simulated Ctrl-C during return descent")
            return None
        return hook

    def run_abort(hook_factory, label):
        counter = [0]
        arm = reset(hook_factory(counter))
        plan = floor_plan()
        real_sleep = time.sleep
        raw_waits = []

        def fake_raw_sleep(seconds):
            raw_waits.append(round(float(seconds), 3))

        time.sleep = fake_raw_sleep
        raised = None
        try:
            gg.execute_sequence(arm, plan, hold_seconds=5.0, return_cube=True)
        except KeyboardInterrupt as exc:
            raised = exc
        finally:
            time.sleep = real_sleep

        pose_values = [(e[1], e[2], e[3]) for e in poses()]
        pump_events = pumps()
        pump_on_idx = index_of_predicate(lambda e: e[0] == "pump" and e[1] is True)
        pump_off_idx = [i for i, e in pump_events if not e[1]]
        check("%s: interruption propagated" % label, raised is not None)
        check("%s: pump OFF happened at least once" % label,
              len(pump_off_idx) >= 1, pump_events)
        check("%s: pump OFF only at contact height, never in the air" % label,
              last_pose_before(pump_off_idx[0]) ==
              ("pose", plan.x, plan.y, plan.z_touch),
              last_pose_before(pump_off_idx[0]))
        check("%s: cube kept exactly the same X/Y while held" % label,
              all((e[1], e[2]) == (plan.x, plan.y)
                  for i, e in enumerate(EVENTS)
                  if e[0] == "pose" and pump_on_idx < i < pump_off_idx[0]),
              pose_values)
        check("%s: arm retracted and parked" % label,
              pose_values[-1] == gg.PARK_POSE, pose_values[-1])
        check("%s: abort used raw wall-clock waits (works during shutdown)" % label,
              len(raw_waits) >= 3, raw_waits)

    run_abort(hold_hook, "hold")
    run_abort(lift_hook, "lift")
    run_abort(return_descent_hook, "return descent")


def test_no_verification_left():
    print("\n[6] no camera photos / success checks left; safety wiring present")
    path = os.path.join(CODES, "grasp_generic.py")
    with open(path, encoding="utf-8") as handle:
        source = handle.read()
    for forbidden in ("object_near", "cube_still_there", "back_in_place",
                      "CvBridge", "pad_detect", "import cv2"):
        check("removed: %s" % forbidden, forbidden not in source)
    check("sequence API has no camera argument",
          "def execute_sequence(arm, plan," in source)
    check("put-back can be skipped for integration",
          "return_cube=True" in source and "return_cube=False" in source)
    check("integration landmarks are overridable",
          "park_pose=PARK_POSE" in source
          and "transit_xy=(TRANSIT_X, TRANSIT_Y_FACTOR)" in source)
    check("safe abort helper present", "_emergency_put_back" in source)
    check("Ctrl-C keeps the node alive for the abort path",
          "disable_signals=True" in source
          and "_install_signal_handlers" in source)
    check("pump is NOT blindly switched off at startup",
          "Reset the actuators" not in source
          and "NEVER blindly switch the pump off" in source)
    check("grasp plan is validated against the firmware IK",
          "ik_reachable" in source and "UNREACHABLE" in source)


def main():
    test_plan_heights()
    test_full_sequence()
    test_custom_landmarks()
    test_hold_only_mode()
    test_abort_scenarios()
    test_no_verification_left()

    print("\n" + "=" * 60)
    if FAILURES:
        print("FAILED CHECKS: %s" % FAILURES)
        return 1
    print("ALL LOCAL MOCK CHECKS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
