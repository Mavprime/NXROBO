#!/usr/bin/env python3
"""ROS 1 (Noetic) adapter: run mission_logic on the real robot.

Subscribes:
    /task_start_flag        std_msgs/String   start|1|go|on|true  /  stop|0|...
    /detections_world       std_msgs/String   JSON objects with world x,y
    /classification_board   std_msgs/String   JSON list of classes left -> right
    /arm/result             std_msgs/String   JSON {"id","ok","confirmed_class","reason"}

Publishes:
    /mission_state          std_msgs/String   JSON snapshot (for the operator/RViz)
    /arm/pick               std_msgs/String   JSON {"id","op":"pick","class","station"}
    /arm/place              std_msgs/String   JSON {"id","op":"place","zone","level"}
    /cmd_vel, /move_base/clear_costmaps      recovery actions

Uses the `move_base` action with goals in the arena/map frame (the static arena map
has origin [0,0,0], so `map` == `arena`). No mark_nav / mark_map.pkl dependency.

Run:
    python3 mission_node.py _world:=~/codes/config/arena_map.yaml _mission:=~/codes/config/mission_demo.yaml
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
import uuid

import actionlib
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Empty, String

from mission_logic import (
    MissionMemory,
    MissionRunner,
    PickResult,
    PlaceResult,
    Thresholds,
    WorldModel,
    load_mission,
)
from ros_contract import (
    encode_arm_pick,
    encode_arm_place,
    parse_arm_result,
    parse_board,
    parse_detections,
    resolve_goal,
    state_dict,
)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class RosPlatform:
    """mission_logic platform protocol over ROS topics/action."""

    def __init__(self, world: dict, thresholds: Thresholds) -> None:
        self.world = world
        self.th = thresholds
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.detections_topic = rospy.get_param("~detections_topic", "/detections_world")
        self.board_topic = rospy.get_param("~board_topic", "/classification_board")
        self.arm_result_topic = rospy.get_param("~arm_result_topic", "/arm/result")
        self.arm_pick_topic = rospy.get_param("~arm_pick_topic", "/arm/pick")
        self.arm_place_topic = rospy.get_param("~arm_place_topic", "/arm/place")
        self.move_base_action = rospy.get_param("~move_base_action", "move_base")
        self.arm_reach = float(rospy.get_param("~arm_reach", 0.45))
        self.total_seconds = float(rospy.get_param("~total_seconds", 600.0))
        self.frame_period = float(rospy.get_param("~frame_period", 0.1))

        self.stop = threading.Event()
        self._start_time = None
        self._lock = threading.Lock()
        self._detections: list = []
        self._det_stamp = None
        self._arm_results: dict[str, dict] = {}
        self._board = None

        self._det_sub = rospy.Subscriber(self.detections_topic, String,
                                         self._on_detections, queue_size=10)
        self._board_sub = rospy.Subscriber(self.board_topic, String,
                                           self._on_board, queue_size=1)
        self._arm_sub = rospy.Subscriber(self.arm_result_topic, String,
                                         self._on_arm_result, queue_size=20)
        self._pick_pub = rospy.Publisher(self.arm_pick_topic, String, queue_size=5)
        self._place_pub = rospy.Publisher(self.arm_place_topic, String, queue_size=5)
        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self._clear_pub = rospy.Publisher("/move_base/clear_costmaps", Empty, queue_size=1)

        self.client = actionlib.SimpleActionClient(self.move_base_action, MoveBaseAction)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self._tf_warned = False

    # -- lifecycle ----------------------------------------------------------

    def reset(self) -> None:
        self.stop.clear()
        self._start_time = rospy.Time.now()
        with self._lock:
            self._detections = []
            self._arm_results.clear()

    def now(self) -> float:
        if self._start_time is None:
            return 0.0
        return (rospy.Time.now() - self._start_time).to_sec()

    def time_left(self) -> float:
        return max(0.0, self.total_seconds - self.now())

    def stop_requested(self) -> bool:
        return self.stop.is_set() or rospy.is_shutdown()

    # -- callbacks ----------------------------------------------------------

    def _on_detections(self, msg: String) -> None:
        try:
            dets = parse_detections(msg.data)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "bad detections JSON: %s" % exc)
            return
        with self._lock:
            self._detections = dets
            self._det_stamp = rospy.Time.now()

    def _on_board(self, msg: String) -> None:
        try:
            self._board = parse_board(msg.data)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "bad board JSON: %s" % exc)

    def _on_arm_result(self, msg: String) -> None:
        try:
            result = parse_arm_result(msg.data)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "bad arm result JSON: %s" % exc)
            return
        with self._lock:
            self._arm_results[str(result.get("id"))] = result

    # -- navigation ---------------------------------------------------------

    def goto(self, point: str) -> bool:
        try:
            x, y, yaw = resolve_goal(self.world, point)
        except KeyError:
            rospy.logerr("unknown named point: %s" % point)
            return False

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw

        self.client.send_goal(goal)
        started = time.time()
        while not self.client.wait_for_result(rospy.Duration(0.2)):
            if self.stop_requested():
                self.client.cancel_goal()
                return False
            if time.time() - started > 60.0:
                rospy.logwarn("navigation timeout to %s (60 s)" % point)
                self.client.cancel_goal()
                return False

        if self.client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logwarn("navigation failed to %s: state %s" %
                          (point, self.client.get_state()))
            self.client.cancel_goal()
            return False
        return True

    # -- perception ---------------------------------------------------------

    def scan_window(self, n: int):
        frames = []
        for _ in range(n):
            if self.stop_requested():
                frames.append([])
                continue
            with self._lock:
                frames.append(list(self._detections))
            self.wait(self.frame_period)
        return frames

    def read_board(self):
        return self._board

    # -- arm ----------------------------------------------------------------

    def _call_arm(self, topic, payload: str, timeout_s: float = 25.0) -> dict | None:
        request_id = json.loads(payload)["id"]
        topic.publish(String(data=payload))
        deadline = time.time() + timeout_s
        while time.time() < deadline and not self.stop_requested():
            with self._lock:
                result = self._arm_results.pop(request_id, None)
            if result is not None:
                return result
            rospy.sleep(0.1)
        rospy.logwarn("arm request %s timed out" % request_id)
        return None

    def pick(self, cls: str) -> PickResult:
        if self.stop_requested():
            return PickResult(False, reason="stopped")
        request_id = uuid.uuid4().hex[:10]
        result = self._call_arm(
            self._pick_pub, encode_arm_pick(request_id, cls, None))
        if result is None:
            return PickResult(False, reason="arm timeout")
        return PickResult(
            ok=result["ok"],
            confirmed_class=result["confirmed_class"],
            reason=result["reason"],
            duration_s=result["duration_s"],
        )

    def place(self, level: int) -> PlaceResult:
        if self.stop_requested():
            return PlaceResult(False, reason="stopped")
        request_id = uuid.uuid4().hex[:10]
        result = self._call_arm(
            self._place_pub, encode_arm_place(request_id, None, level))
        if result is None:
            return PlaceResult(False, reason="arm timeout")
        return PlaceResult(ok=result["ok"], reason=result["reason"])

    # -- clock / recovery ---------------------------------------------------

    def wait(self, seconds: float) -> None:
        end = time.time() + max(0.0, seconds)
        while time.time() < end and not self.stop_requested():
            rospy.sleep(min(0.1, max(0.0, end - time.time())))

    def precheck(self):
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            return False, "move_base action server is not available"
        if not self.tf_buffer.can_transform(self.frame_id, self.base_frame,
                                            rospy.Time(0), rospy.Duration(2.0)):
            rospy.logwarn("TF %s -> %s not available yet; G4 will be permissive"
                          % (self.frame_id, self.base_frame))
        return True, ""

    def robot_pose(self):
        try:
            tr = self.tf_buffer.lookup_transform(self.frame_id, self.base_frame,
                                                 rospy.Time(0), rospy.Duration(0.2))
            return tr.transform.translation.x, tr.transform.translation.y
        except Exception:
            if not self._tf_warned:
                rospy.logwarn("robot TF unavailable; G4 falls back to permissive")
                self._tf_warned = True
            return None

    def approachable(self, x: float, y: float) -> bool:
        pose = self.robot_pose()
        if pose is None:
            return True
        return math.hypot(x - pose[0], y - pose[1]) <= self.arm_reach

    def clear_costmaps(self) -> None:
        self._clear_pub.publish(Empty())

    def rotate_scan(self) -> None:
        twist = Twist()
        twist.angular.z = 0.75
        end = time.time() + 1.5
        while time.time() < end and not self.stop_requested():
            self._cmd_pub.publish(twist)
            rospy.sleep(0.05)
        self._cmd_pub.publish(Twist())

    def backup(self) -> None:
        twist = Twist()
        twist.linear.x = -0.10
        end = time.time() + 1.0
        while time.time() < end and not self.stop_requested():
            self._cmd_pub.publish(twist)
            rospy.sleep(0.05)
        self._cmd_pub.publish(Twist())

    def reframe(self, station) -> None:
        self.rotate_scan()


class MissionNode:
    def __init__(self) -> None:
        default_world = os.path.expanduser("~/codes/config/arena_map.yaml")
        default_mission = os.path.expanduser("~/codes/config/mission_demo.yaml")
        default_gate = os.path.expanduser("~/codes/config/gate.yaml")

        world_path = rospy.get_param("~world", default_world)
        mission_path = rospy.get_param("~mission", default_mission)
        gate_path = rospy.get_param("~gate", default_gate)

        import yaml

        with open(world_path) as f:
            self.world = yaml.safe_load(f)
        self.world_model = WorldModel.from_scenario(self.world)
        self.mission = load_mission(mission_path)
        try:
            self.thresholds = Thresholds.from_yaml(gate_path)
        except Exception:
            self.thresholds = Thresholds()

        self.platform = RosPlatform(self.world, self.thresholds)
        self.memory = MissionMemory.from_inventory(
            self.world.get("bricks", []), self.world_model.default_mapping)
        self.runner = None
        self.thread = None
        self._lock = threading.Lock()

        self.state_pub = rospy.Publisher("/mission_state", String, queue_size=1)
        rospy.Subscriber("/task_start_flag", String, self.on_flag, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.2), self.publish_state)
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("mission node ready: world=%s mission=%s"
                      % (world_path, self.mission.name))
        if rospy.get_param("~auto_start", False):
            rospy.Timer(rospy.Duration(1.0), self._auto_start, oneshot=True)

    def _auto_start(self, _event=None) -> None:
        self.start()

    def on_flag(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command in ("start", "1", "go", "on", "true"):
            self.start()
        elif command in ("stop", "0", "halt", "off", "abort"):
            self.platform.stop.set()
            rospy.logwarn("stop requested")

    def start(self) -> None:
        with self._lock:
            if self.thread is not None and self.thread.is_alive():
                rospy.logwarn("mission already running")
                return
            self.platform.reset()
            self.memory = MissionMemory.from_inventory(
                self.world.get("bricks", []), self.world_model.default_mapping)
            self.runner = MissionRunner(
                self.mission, self.platform, self.world_model,
                memory=self.memory, thresholds=self.thresholds,
                approachable=self.platform.approachable,
                on_step=self.on_step,
            )
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            rospy.loginfo("mission started")

    def _run(self) -> None:
        try:
            report = self.runner.run()
            rospy.loginfo("mission %s: %s" % (report.state, report.reason))
        except Exception as exc:
            rospy.logerr("mission crashed: %s" % exc)
            self.platform.stop.set()
        finally:
            self.publish_state(None)

    def on_step(self, record) -> None:
        rospy.loginfo("step %d %s -> %s %s" % (
            record.index + 1, record.kind, record.outcome.value, record.reason))
        self.publish_state(None)

    def publish_state(self, _event) -> None:
        if self.runner is None:
            payload = {"state": "IDLE", "current": "-", "steps": [],
                       "mapping": dict(self.memory.mapping),
                       "memory": self.memory.summary(),
                       "clock": self.platform.time_left()}
        else:
            payload = state_dict(self.runner, self.memory, self.platform.time_left())
        self.state_pub.publish(String(data=json.dumps(payload)))

    def on_shutdown(self) -> None:
        self.platform.stop.set()


def main() -> None:
    rospy.init_node("spark_mission")
    MissionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
