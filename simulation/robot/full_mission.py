#!/usr/bin/env python3
"""One-shot hardware mission: A -> Sorting_W (diagonal) -> pick cake -> A -> place.

Prerequisite (already up): base bring-up on ROS master 11311 -
    roslaunch spark_bringup driver_bringup.launch ...
    roslaunch lidar_driver_transfer ydlidar_g6.launch
    roslaunch swiftpro pro_control_nomoveit.launch

Everything else is handled here:
  1. resolve the map pose from the wall lines (robot placed at A, facing centre)
  2. drive the straight diagonal A -> W, closed-loop on the wall distances
  3. start AMCL (+move_base) at the W pose, the detector and arm_real_node
  4. pick the cake with ONE request per attempt; if the arm says "out of reach"
     creep 4 cm closer (capped at 6 cm) and retry
  5. carry to Collection_A_approach via a waypoint north-east of the
     classification table (its costmap inflation is what stalled the direct
     A goal on 2026-11-24)
  6. place the cake, park the arm at home, stop the nodes it started

Run on the robot:  python3 -u ~/codes/robot/full_mission.py
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import time

import actionlib
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_poses                                   # noqa: E402
from a2w import A2W, A_XY, W_XY, W_HEADING         # noqa: E402

HOME = os.path.expanduser("~")
PICK_SETTINGS = ["_creep:=false", "_turn_align:=false", "_cake_pos_tol:=0.20"]
ARM_WORLD = os.path.join(HOME, "codes", "config", "arena_map.yaml")
# Drive target: the measured Sorting_W plus the 4 cm east that the arm's reach
# gate needed in the first successful pick (2026-11-24). The cake search after
# that is rotation-only - never translate.
W_TARGET = (W_XY[0] + 0.04, W_XY[1])
SEARCH_DEG = (0.0, 10.0, -10.0, 15.0, -15.0)
CARRY_WAYPOINT = (0.75, 2.30, 90.0)      # north-east of the table inflation
A_APPROACH = (0.4725, 2.5275, 135.0)
MAX_CREEP = 0.06


class FullMission(A2W):
    def __init__(self) -> None:
        super().__init__(node_name="full_mission")
        self.procs = []
        self.results = {}
        self.arm_pick = rospy.Publisher("/arm/pick", String, queue_size=1)
        self.arm_place = rospy.Publisher("/arm/place", String, queue_size=1)
        rospy.Subscriber("/arm/result", String, self._on_arm_result)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf)
        self.mb = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    # -- plumbing ----------------------------------------------------------
    def _on_arm_result(self, msg) -> None:
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        if d.get("id"):
            self.results[d["id"]] = d
            print("[MISSION] arm result %s" % json.dumps(d), flush=True)

    def _popen(self, cmd, cwd=None, log=None, env=None):
        os.makedirs(os.path.join(HOME, "logs", "manual"), exist_ok=True)
        path = os.path.join(HOME, "logs", "manual", log) if log else None
        fh = open(path, "w") if path else None
        p = subprocess.Popen(cmd, cwd=cwd, stdout=fh or subprocess.DEVNULL,
                             stderr=subprocess.STDOUT, start_new_session=True,
                             env=env)
        self.procs.append((p, fh))
        return p

    def _kill_nodes(self, keys) -> None:
        try:
            out = subprocess.run(["rosnode", "list"], capture_output=True,
                                 text=True, timeout=5).stdout
        except Exception:
            return
        names = [n for n in out.split()
                 if n.strip("/") != "full_mission" and any(k in n for k in keys)]
        if names:
            print("[MISSION] stopping %s" % " ".join(names), flush=True)
            subprocess.run(["rosnode", "kill"] + names, capture_output=True,
                           timeout=10)

    def _wait_tf(self, timeout: float = 25.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            try:
                tr = self.tf.lookup_transform("map", "base_footprint",
                                              rospy.Time(0), rospy.Duration(0.5))
                p = tr.transform.translation
                print("[MISSION] TF map->base (%.3f, %.3f)" % (p.x, p.y), flush=True)
                return True
            except Exception:
                rospy.sleep(0.5)
        return False

    def _send_arm(self, pub, req, timeout: float):
        rid = req["id"]
        self.results.pop(rid, None)
        t0 = time.time()
        while pub.get_num_connections() < 1 and time.time() - t0 < 10:
            rospy.sleep(0.1)
        pub.publish(String(data=json.dumps(req)))
        print("[MISSION] sent %s (subscribers %d)"
              % (json.dumps(req), pub.get_num_connections()), flush=True)
        t0 = time.time()
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            if rid in self.results:
                return self.results[rid]
            rospy.sleep(0.3)
        return None

    def _goto(self, x: float, y: float, yaw_deg: float,
              timeout: float = 60.0) -> bool:
        if not self.mb.wait_for_server(rospy.Duration(10.0)):
            print("[MISSION] move_base action server missing", flush=True)
            return False
        self.mb.cancel_all_goals()
        rospy.sleep(0.3)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        th = math.radians(yaw_deg)
        goal.target_pose.pose.orientation.z = math.sin(th / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(th / 2.0)
        self.mb.send_goal(goal)
        ok = self.mb.wait_for_result(rospy.Duration(timeout))
        state = self.mb.get_state()
        print("[MISSION] move_base (%.3f, %.3f, %.0f): ok=%s state=%s"
              % (x, y, yaw_deg, ok, state), flush=True)
        if not ok or state != GoalStatus.SUCCEEDED:
            self.mb.cancel_all_goals()
            return False
        return True

    # -- stack -------------------------------------------------------------
    def _start_nav(self, x: float, y: float) -> bool:
        env = os.environ.copy()
        env.update(INIT_X="%.3f" % x, INIT_Y="%.3f" % y, INIT_A="0.0")
        self._popen(["bash", os.path.join(HOME, "codes", "nav_bringup.sh")],
                    log="nav_full.log", env=env)
        return self._wait_tf(25.0)

    def _start_perception(self) -> bool:
        auto = os.path.join(HOME, "auto", "codes")
        self._popen([sys.executable, "-u", "detect_ros.py"], cwd=auto,
                    log="detector_full.log")
        self._popen([sys.executable, "-u", "arm_real_node.py"] + PICK_SETTINGS
                    + ["_world:=%s" % ARM_WORLD], cwd=auto, log="arm_full.log")
        t0 = time.time()
        while time.time() - t0 < 70 and not rospy.is_shutdown():
            if self.arm_pick.get_num_connections() > 0:
                try:
                    rospy.wait_for_message("/objects", Detection2DArray,
                                           timeout=2.0)
                    print("[MISSION] detector + arm node up", flush=True)
                    return True
                except Exception:
                    pass
            rospy.sleep(0.5)
        return False

    # -- mission -----------------------------------------------------------
    def _pick(self) -> bool:
        """Pick the cake, searching by ROTATION ONLY (never translate).

        Attempt at the arrive heading, then +-10 deg, then +-15 deg. Any pick
        failure other than a found-but-out-of-reach target advances the sweep.
        """
        for i, offset in enumerate(SEARCH_DEG):
            if abs(offset) > 0.1:
                self.turn_to(W_HEADING + offset)
                print("[MISSION] search: rotated to %+0.0f deg" % offset,
                      flush=True)
            res = self._send_arm(self.arm_pick,
                                 {"id": "pick-%d" % (i + 1), "op": "pick",
                                  "class": "cake"}, timeout=120.0)
            if res and res.get("ok"):
                print("[MISSION] cake picked at %+0.0f deg" % offset, flush=True)
                return True
            print("[MISSION] attempt %+0.0f deg failed: %s"
                  % (offset, (res or {}).get("reason", "timeout")), flush=True)
        return False

    def run(self) -> None:
        print("[MISSION] start", flush=True)
        self.init_heading()
        x, y = self.pos0
        print("[MISSION] start pose (%.3f, %.3f) heading %.1f"
              % (x, y, self.h()), flush=True)
        if math.hypot(x - A_XY[0], y - A_XY[1]) > 0.30:
            raise RuntimeError("not at A (%.2f, %.2f)" % (x, y))

        # start from a known, compact arm state (the mission begins empty)
        arm_poses.send_pose(arm_poses.HOME_POSE, repeats=4, settle=0.4)
        print("[MISSION] diagonal A -> W target %s (W %s + 4 cm east)"
              % (W_TARGET, W_XY), flush=True)
        self.goto_wall(W_TARGET, tol=0.04, vmax=0.14, timeout=90)
        self.turn_to(W_HEADING)
        wx, wy = self.wall_xy_lines()
        wx = wx if wx is not None else W_XY[0]
        wy = wy if wy is not None else W_XY[1]
        print("[MISSION] at W (%.3f, %.3f), %.1f cm from target"
              % (wx, wy, 100 * math.hypot(wx - W_TARGET[0], wy - W_TARGET[1])),
              flush=True)

        if not self._start_nav(wx, wy):
            raise RuntimeError("no map->base TF after nav start")
        if not self._start_perception():
            raise RuntimeError("detector / arm node did not come up")

        if not self._pick():
            raise RuntimeError("cake not picked")

        if not self._goto(*CARRY_WAYPOINT, timeout=60):
            raise RuntimeError("carry waypoint failed")
        if not self._goto(*A_APPROACH, timeout=60):
            raise RuntimeError("A approach failed")

        res = self._send_arm(self.arm_place,
                             {"id": "place-1", "op": "place", "level": 1},
                             timeout=60.0)
        if not (res and res.get("ok")):
            raise RuntimeError("place failed: %s" % (res,))
        print("[MISSION] placed at A", flush=True)

        arm_poses.send_pose(arm_poses.HOME_POSE, repeats=6, settle=0.8)
        print("[MISSION] arm home", flush=True)
        print("[MISSION] DONE", flush=True)

    def cleanup(self) -> None:
        self.stop()
        self._kill_nodes(("amcl", "move_base", "map_server",
                          "velocity_smoother", "detector_node",
                          "arm_real_node"))
        for p, fh in self.procs:
            try:
                p.terminate()
            except Exception:
                pass
            if fh:
                fh.close()


def main() -> int:
    mission = FullMission()
    rc = 0
    try:
        mission.run()
    except Exception as exc:
        print("[MISSION] FAILED: %s" % exc, flush=True)
        rc = 1
    finally:
        mission.cleanup()
    return rc


if __name__ == "__main__":
    sys.exit(main())
