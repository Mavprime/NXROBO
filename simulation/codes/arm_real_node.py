#!/usr/bin/env python3
"""Real arm service for the mission JSON contract.

Subscribes /arm/pick and /arm/place (std_msgs/String JSON) and publishes
/arm/result, reusing the production arm pipeline:

  pick  -> grasp_generic.scan target + reach_state + drive_approach_step +
           grasp_cycle(no_place=True)  (vision-verified pick)
  place -> runtask.ArmAction.drop / drop1 / drop2 at the collection approach

The mission sends only the class (pick) and level (place); the robot's current
pose determines the target, exactly as with arm_mock_node.py.

Run in the ROOT namespace: the swiftpro topics are relative names.

Params:
  ~dry_run     false  run the motion sequence with the vacuum pump disabled
  ~attempts    3      scan / approach retries per pick
  ~cake_offset 125    suction-cup forward offset for the elevated cake (mm)

Requires: detector on /objects, D435 depth topics, swiftpro driver.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time

import rospy
import tf2_ros
import yaml
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist

import grasp_generic
import match_logic
from runtask import ArmAction, CamAction

CLASS_ID = {
    "bear": match_logic.TEDDY_ID,
    "cake": match_logic.CAKE_ID,
    "clock": match_logic.CLOCK_ID,
    "banana": match_logic.BANANA_ID,
}
CARRY_POSE = (230.0, 0.0, 120.0)


class ArmReal:
    def __init__(self) -> None:
        rospy.init_node("arm_real_node")
        self.dry_run = bool(rospy.get_param("~dry_run", False))
        self.raw_detect = bool(rospy.get_param("~raw_detect", False))
        self.conf = float(rospy.get_param("~conf", 0.35))
        self.creep_m = float(rospy.get_param("~creep_m", 0.03))
        self._creeped = 0
        self._turns = 0
        self.creep = bool(rospy.get_param("~creep", True))
        self.turn_align = bool(rospy.get_param("~turn_align", True))
        self.exit_back_m = float(rospy.get_param("~exit_back_m", 0.45))
        self.attempts = int(rospy.get_param("~attempts", grasp_generic.MAX_APPROACH_ATTEMPTS))
        cake_offset = float(rospy.get_param("~cake_offset", grasp_generic.CAKE_CUP_OFFSET_X))

        self.cam = CamAction()
        self.arm = ArmAction(cam=self.cam)
        import cv_bridge
        self.bridge = cv_bridge.CvBridge()
        self.scale, self.cake_touch = grasp_generic.load_memory()
        self.cake_offset = cake_offset
        self.cake_conf = float(rospy.get_param("~cake_conf", grasp_generic.DEFAULT_CAKE_CONF))
        self.cake_site_conf = float(rospy.get_param(
            "~cake_site_conf", grasp_generic.DEFAULT_CAKE_SITE_CONF))
        self.lock = threading.Lock()
        self._raw_lock = threading.Lock()
        self._raw = []
        self.parked = False

        # World-model cake gate: the real cake has a known map position; reject
        # hypotheses that project far from it.
        self.cake_xy = None
        world_path = rospy.get_param("~world", os.path.expanduser("~/codes/config/arena_map.yaml"))
        try:
            with open(world_path) as fh:
                world = yaml.safe_load(fh)
            for brick in world.get("bricks", []):
                if str(brick.get("type", "")).lower() == "cake":
                    self.cake_xy = (float(brick["x"]), float(brick["y"]))
                    break
        except Exception as exc:
            rospy.logwarn("arm real: cannot read world model (%s)", exc)
        self.cake_pos_tol = float(rospy.get_param("~cake_pos_tol", 0.10))
        self.contact_detect = bool(rospy.get_param("~contact_detect", True))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        if self.dry_run:
            rospy.logwarn("arm_real_node: DRY RUN - vacuum pump is disabled")
            self.arm.interface.set_pump = lambda enable: rospy.loginfo(
                "[DRY] pump %s", "ON" if enable else "OFF")

        self.pub = rospy.Publisher("/arm/result", String, queue_size=10)
        rospy.Subscriber("/arm/pick", String, self._on_pick, queue_size=10)
        rospy.Subscriber("/arm/place", String, self._on_place, queue_size=10)
        rospy.Subscriber("/objects", Detection2DArray, self._on_raw_objects, queue_size=5)
        rospy.loginfo("arm real ready: dry_run=%s creep=%s raw_detect=%s contact=%s scale=%s "
                      "cake_touch=%.1f cake_conf=%.2f cake_site_conf=%.2f"
                      % (self.dry_run, self.creep, self.raw_detect, self.contact_detect, self.scale,
                         self.cake_touch, self.cake_conf, self.cake_site_conf))

    # -- contract ----------------------------------------------------------

    def _publish(self, request_id, ok, confirmed, reason, t0) -> None:
        payload = {
            "id": request_id,
            "ok": bool(ok),
            "confirmed_class": confirmed,
            "reason": reason,
            "duration_s": round(time.time() - t0, 1),
        }
        self.pub.publish(String(data=json.dumps(payload)))
        rospy.loginfo("arm result: %s", payload)

    def _on_pick(self, msg: String) -> None:
        t0 = time.time()
        try:
            req = json.loads(msg.data)
        except Exception:
            rospy.logwarn("arm_real: bad pick request")
            return
        cls = str(req.get("class", "")).strip().lower()
        cid = CLASS_ID.get(cls)
        if cid is None:
            self._publish(req.get("id"), False, "", "unsupported class %r" % cls, t0)
            return
        with self.lock:
            ok, reason = self._pick(cid)
        self._publish(req.get("id"), ok, cls if ok else "", reason, t0)

    def _on_place(self, msg: String) -> None:
        t0 = time.time()
        try:
            req = json.loads(msg.data)
        except Exception:
            rospy.logwarn("arm_real: bad place request")
            return
        level = int(req.get("level", 1))
        with self.lock:
            ok, reason = self._place(level)
        self._publish(req.get("id"), ok, "", reason, t0)

    # -- raw detection (test mode) ----------------------------------------

    def _on_raw_objects(self, msg: Detection2DArray) -> None:
        items = []
        for det in msg.detections:
            if not det.results:
                continue
            best = max(det.results, key=lambda r: float(r.score))
            items.append((str(getattr(best, "id", "")), float(best.score),
                          float(det.bbox.center.x), float(det.bbox.center.y)))
        with self._raw_lock:
            self._raw = items

    def _raw_need(self, raw_id: int) -> float:
        if match_logic.norm_id(raw_id) == match_logic.CAKE_ID:
            return self.cake_conf if raw_id == 2 else self.cake_site_conf
        return self.conf

    def _raw_candidates(self, cid: int):
        with self._raw_lock:
            dets = list(self._raw)
        found = []
        for rid, score, u, v in dets:
            try:
                raw_id = int(float(rid))
            except Exception:
                continue
            if match_logic.norm_id(raw_id) != cid:
                continue
            if score < self._raw_need(raw_id):
                continue
            found.append((raw_id, score, u, v))
        return found

    def _raw_scan(self, cid: int):
        """Bypass the production clustering: use /objects directly (test mode)."""
        if not self.parked:
            self.arm.interface.set_pose(*grasp_generic.PARK_POSE, delay=0.35)
            rospy.sleep(0.15)
            self.parked = True
        is_cake = cid == match_logic.CAKE_ID
        deadline = time.time() + 4.0
        valid = []
        while time.time() < deadline:
            valid = self._raw_candidates(cid)
            if valid:
                break
            rospy.sleep(0.15)
        if not valid:
            return None
        ordered = sorted(valid, key=lambda c: abs(c[2] - 320.0))
        for raw_id, score, u, v in ordered:
            xy, z_est = grasp_generic.compute_target_coords(
                self.cam, u, v, is_cake, self.scale, self.cake_offset)
            if xy is None:
                continue
            return {
                "mode": "cake" if is_cake else "floor",
                "is_cake": is_cake,
                "class": raw_id,
                "pixel": [round(u, 1), round(v, 1)],
                "xy": [round(xy[0], 1), round(xy[1], 1)],
                "z_est": round(z_est, 1) if z_est is not None else None,
            }
        return None

    def _raw_still_near(self, u0, v0, cid, radius=45.0, tries=2) -> bool:
        for _ in range(max(1, tries)):
            for _raw_id, _score, u, v in self._raw_candidates(cid):
                if math.hypot(u - u0, v - v0) <= radius:
                    return True
            rospy.sleep(0.2)
        return False

    # -- motions -----------------------------------------------------------

    def _creep_forward(self, distance_m: float) -> None:
        """Timed open-loop base move; allows steps below drive_base's 3 cm floor."""
        distance_m = max(-0.50, min(0.50, float(distance_m)))
        if abs(distance_m) < 0.005:
            return
        speed = 0.15 if distance_m > 0 else -0.15
        duration = abs(distance_m) / abs(speed)
        rate = 30.0
        ros_rate = rospy.Rate(rate)
        end = rospy.get_time() + duration
        while rospy.get_time() < end and not rospy.is_shutdown():
            msg = Twist(); msg.linear.x = speed
            self.arm.cmd_vel_pub.publish(msg)
            ros_rate.sleep()
        for _ in range(2):
            self.arm.cmd_vel_pub.publish(Twist())
            rospy.sleep(0.05)
        rospy.loginfo("creep: %+.0f mm at %.2f m/s" % (distance_m * 1000, speed))

    def _target_map_xy(self, arm_x: float, arm_y: float):
        tr = self.tf_buffer.lookup_transform(
            "map", "base_footprint", rospy.Time(0), rospy.Duration(0.3))
        p = tr.transform.translation
        q = tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        bx = (arm_x + 44.0) / 1000.0
        by = arm_y / 1000.0
        return (p.x + bx * math.cos(yaw) - by * math.sin(yaw),
                p.y + bx * math.sin(yaw) + by * math.cos(yaw))

    def _safe_retract(self) -> None:
        """Pump off and home the arm after a failed or aborted pick."""
        try:
            self.arm.interface.set_pump(False)
            rospy.sleep(0.1)
            self.arm.arm_home()
            rospy.sleep(0.2)
        except Exception:
            pass

    def _pick(self, cid: int):
        cls = [k for k, v in CLASS_ID.items() if v == cid][0]
        self._turns = 0
        self._creeped = 0
        try:
            self.arm.interface.set_status(True)
            rospy.sleep(0.2)
            first = True
            for attempt in range(self.attempts):
                if self.raw_detect:
                    target = self._raw_scan(cid)
                else:
                    target = grasp_generic.scan_scene(
                        self.cam, self.arm, self.scale, fast=not first,
                        tag="_arm%d" % attempt, cake_offset=self.cake_offset,
                        cake_conf_threshold=self.cake_conf,
                        cake_site_conf_threshold=self.cake_site_conf)
                first = False
                if target is None:
                    if attempt + 1 < self.attempts:
                        rospy.sleep(0.6)
                        continue
                    return False, "no target detected"
                if match_logic.norm_id(target["class"]) != cid:
                    return False, "target class %s, requested %s" % (target["class"], cls)
                x, y = target["xy"]
                far = bool(target.get("far"))
                in_reach, dist = grasp_generic.reach_state(
                    target["mode"], x, y, z_est=target.get("z_est"), far=far)
                rospy.loginfo("arm pick %s: target(%s)=(%.1f,%.1f) far=%s dist=%.1f in_reach=%s"
                              % (cls, attempt + 1, x, y, far, dist, in_reach))
                if cid == match_logic.CAKE_ID and self.cake_xy is not None:
                    try:
                        mx, my = self._target_map_xy(x, y)
                        d_cake = math.hypot(mx - self.cake_xy[0], my - self.cake_xy[1])
                        if d_cake > self.cake_pos_tol:
                            rospy.logwarn(
                                "arm pick: cake hypothesis %.3f m from the expected "
                                "cake (%.2f, %.2f) - ignored", d_cake, mx, my)
                            if attempt + 1 < self.attempts:
                                rospy.sleep(0.4)
                                continue
                            return False, "cake not at expected position"
                    except Exception as exc:
                        rospy.logwarn("arm pick: cake position check failed (%s)", exc)
                bearing = math.atan2(y, x)
                if in_reach and self.turn_align and abs(bearing) > math.radians(4.0) \
                        and self._turns < 2:
                    rospy.loginfo("arm pick: target bearing %+.1f deg; aligning base"
                                  % math.degrees(bearing))
                    self.arm.turn_base(bearing)
                    self._turns += 1
                    rospy.sleep(0.5)
                    continue
                if self.creep and in_reach and x > 342.0 and self._creeped < 2:
                    rospy.loginfo("arm pick: cake at the arm limit (x=%.1f); "
                                  "creeping %.0f mm forward to centre the cup"
                                  % (x, self.creep_m * 1000))
                    self._creep_forward(self.creep_m)
                    self._creeped += 1
                    self.parked = True
                    rospy.sleep(0.6)
                    continue
                if in_reach:
                    try:
                        plan = grasp_generic.plan_grasp(target, self.cake_touch)
                    except Exception as exc:
                        self._safe_retract()
                        return False, "plan: %s" % exc
                    if cid == match_logic.CAKE_ID and self.contact_detect:
                        try:
                            import cake_final
                            z_contact, _trace = cake_final.descend_until_contact(
                                self.arm, self.bridge, cake_final.M,
                                plan.x, plan.y, plan.z_approach,
                                max(-60.0, plan.z_touch - 40.0))
                            if z_contact is not None:
                                plan.z_touch = max(z_contact - 2.0, -60.0)
                                plan.z_lift = min(plan.z_touch + 45.0, 100.0)
                                rospy.loginfo(
                                    "arm pick: contact at z=%.1f -> touch z=%.1f"
                                    % (z_contact, plan.z_touch))
                            else:
                                rospy.logwarn("arm pick: contact not found; using memory z")
                        except Exception as exc:
                            rospy.logwarn("arm pick: contact detection failed (%s)", exc)
                    # return_cube=False leaves the cube attached and the pump ON.
                    grasp_generic.execute_sequence(
                        self.arm, plan, hold_seconds=0.5, return_cube=False)
                    u0, v0 = target["pixel"]
                    if self._raw_still_near(u0, v0, cid):
                        # Nothing attached: lower to the pickup point before release.
                        self.arm.interface.set_pose(
                            plan.x, plan.y, plan.z_touch, delay=1.0)
                        rospy.sleep(0.3)
                        self._safe_retract()
                        return False, "grasp MISS"
                    self.arm.interface.set_pose(*CARRY_POSE, delay=0.9)
                    if self.exit_back_m > 0.01:
                        rospy.loginfo("arm pick: exit maneuver, backing out %.2f m",
                                      self.exit_back_m)
                        self._creep_forward(-abs(self.exit_back_m))
                        rospy.sleep(0.5)
                    return True, ""
                if not self.creep:
                    self._safe_retract()
                    return False, "out of reach (creep disabled)"
                if not grasp_generic.drive_approach_step(self.arm, target):
                    self._safe_retract()
                    return False, "base approach blocked"
                rospy.sleep(0.3)
            self._safe_retract()
            return False, "out of reach after %d attempts" % self.attempts
        except Exception as exc:
            rospy.logerr("arm pick exception: %s", exc)
            self._safe_retract()
            return False, "exception: %s" % exc

    def _place(self, level: int):
        try:
            routine = {1: self.arm.drop, 2: self.arm.drop1, 3: self.arm.drop2}.get(
                level, self.arm.drop)
            ok = bool(routine())
            return ok, "" if ok else "drop failed"
        except Exception as exc:
            rospy.logerr("arm place exception: %s", exc)
            return False, "exception: %s" % exc


def main() -> None:
    node = ArmReal()
    rospy.on_shutdown(lambda: node.arm.interface.set_pump(False))
    rospy.spin()


if __name__ == "__main__":
    main()
