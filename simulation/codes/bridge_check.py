#!/usr/bin/env python3
"""Live validation of /objects -> world positions on the real robot.

Two modes:

  ~mode:=dlt       (default, real robot)  production transform_3d.dat + arm mount
  ~mode:=pinhole   (simulation)           camera_info + camera TF + floor plane

The real D435 image is top-down, while the URDF camera TF is horizontal, so the
pinhole mode is wrong on the robot. The DLT mode uses the calibrated 4x3 matrix that
the production pick pipeline uses, then maps arm millimetres to base_footprint with
the URDF mount (base_link -> Base at 0.044, 0, 0.138, no rotation).

Examples:

    # real robot, base frame, a cube measured 0.30 m ahead
    python3 -u ~/codes/bridge_check.py \
        _mode:=dlt _matrix:=$HOME/codes/transform_3d.dat \
        _target_frame:=base_footprint \
        _expected_class:=bear _expected_x:=0.30 _expected_y:=0.00

    # map frame (AMCL running), cube at a measured arena point
    python3 -u ~/codes/bridge_check.py \
        _mode:=dlt _target_frame:=map \
        _expected_class:=cake _expected_x:=1.5 _expected_y:=1.5

The node moves nothing.
"""

from __future__ import annotations

import math
import os
import statistics
import time

import rospy
import tf2_ros
import yaml
from sensor_msgs.msg import CameraInfo as CameraInfoMsg
from vision_msgs.msg import Detection2DArray

from mission_logic import canonical_class
from projection import (
    ARM_MOUNT_X,
    ARM_MOUNT_Y,
    DLT_CX,
    DLT_CY,
    DLT_FX,
    DLT_FY,
    CameraInfo,
    arm_to_base,
    load_dlt_matrix,
    pixel_to_arm_at_z,
    pixel_to_plane,
    rotation_matrix,
)


class BridgeCheck:
    def __init__(self) -> None:
        self.mode = rospy.get_param("~mode", "dlt")
        self.objects_topic = rospy.get_param("~objects_topic", "/objects")
        self.info_topic = rospy.get_param("~camera_info_topic",
                                          "/camera/color/camera_info")
        self.target_frame = rospy.get_param("~target_frame", "base_footprint")
        self.camera_frame = rospy.get_param("~camera_frame",
                                            "camera_color_optical_frame")
        self.conf_min = float(rospy.get_param("~conf_min", 0.2))
        self.samples = int(rospy.get_param("~samples", 20))
        self.print_period = float(rospy.get_param("~print_period", 0.5))

        # pinhole mode
        self.plane_default = float(rospy.get_param("~plane_z_default", 0.05))
        self.plane_by_class = dict(rospy.get_param("~plane_z_by_class", {"cake": 0.15}))

        # dlt mode
        self.matrix_path = rospy.get_param(
            "~matrix", os.path.expanduser("~/codes/transform_3d.dat"))
        self.mount_x = float(rospy.get_param("~mount_x", ARM_MOUNT_X))
        self.mount_y = float(rospy.get_param("~mount_y", ARM_MOUNT_Y))
        self.z_default = float(rospy.get_param("~z_center_default", -100.0))
        self.z_by_class = dict(rospy.get_param("~z_center_by_class", {"cake": 0.0}))
        self.fix_dx = float(rospy.get_param("~fix_dx", 0.0))
        self.fix_dy = float(rospy.get_param("~fix_dy", 0.0))
        self.matrix = None
        if self.mode == "dlt":
            try:
                self.matrix = load_dlt_matrix(self.matrix_path)
                rospy.loginfo("DLT matrix %s shape %s" %
                              (self.matrix_path, self.matrix.shape))
            except Exception as exc:
                rospy.logerr("cannot load DLT matrix %s: %s" % (self.matrix_path, exc))

        expected_class = canonical_class(rospy.get_param("~expected_class", ""))
        ex = rospy.get_param("~expected_x", None)
        ey = rospy.get_param("~expected_y", None)
        self.expected = (float(ex), float(ey)) if ex is not None and ey is not None else None
        self.expected_class = expected_class

        world_path = rospy.get_param(
            "~world", os.path.expanduser("~/codes/config/arena_map.yaml"))
        self.cubes = []
        try:
            with open(world_path) as f:
                world = yaml.safe_load(f)
            for brick in world.get("bricks", []):
                self.cubes.append({
                    "name": brick.get("name", ""),
                    "cls": canonical_class(brick.get("type", "")),
                    "x": float(brick["x"]),
                    "y": float(brick["y"]),
                })
        except Exception as exc:
            rospy.logwarn("could not load world map (%s); no reference distances" % exc)

        self.camera = None
        self.metrics: list[float] = []
        self.last_print = 0.0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(self.info_topic, CameraInfoMsg, self._on_info, queue_size=1)
        rospy.Subscriber(self.objects_topic, Detection2DArray, self._on_objects,
                         queue_size=5)
        rospy.loginfo("bridge check: mode=%s target=%s expected=%s samples=%d"
                      % (self.mode, self.target_frame, self.expected, self.samples))

    # -- camera -------------------------------------------------------------

    def _on_info(self, msg: CameraInfoMsg) -> None:
        if not msg.K or len(msg.K) < 6 or msg.K[0] <= 0.0:
            rospy.logwarn_throttle(5.0, "camera_info has no valid K")
            return
        if self.camera is None:
            self.camera = CameraInfo.from_k(list(msg.K), msg.width, msg.height)
            rospy.loginfo("camera: fx=%.1f fy=%.1f cx=%.1f cy=%.1f %dx%d"
                          % (self.camera.fx, self.camera.fy, self.camera.cx,
                             self.camera.cy, self.camera.width, self.camera.height))

    def _intrinsics(self):
        if self.camera is not None:
            return (self.camera.fx, self.camera.fy, self.camera.cx, self.camera.cy)
        return (DLT_FX, DLT_FY, DLT_CX, DLT_CY)

    # -- dispatch -----------------------------------------------------------

    def _on_objects(self, msg: Detection2DArray) -> None:
        if self.mode == "dlt":
            self._handle_dlt(msg)
        else:
            self._handle_pinhole(msg)

    def _detections(self, msg):
        for detection in msg.detections:
            if not detection.results:
                continue
            best = max(detection.results, key=lambda r: float(r.score))
            cls = canonical_class(getattr(best, "id", ""))
            score = float(best.score)
            if not cls or score < self.conf_min:
                continue
            yield cls, float(detection.bbox.center.x), float(detection.bbox.center.y), score

    # -- dlt ----------------------------------------------------------------

    def _handle_dlt(self, msg: Detection2DArray) -> None:
        if self.matrix is None:
            rospy.logerr_throttle(5.0, "no DLT matrix loaded")
            return
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
        transform = None
        if self.target_frame not in ("base_footprint", "base_link"):
            for query_time in (stamp, rospy.Time(0)):
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.target_frame, "base_footprint", query_time,
                        rospy.Duration(0.10))
                    break
                except Exception:
                    continue
            if transform is None:
                rospy.logwarn_throttle(2.0, "TF %s <- base_footprint unavailable"
                                       % self.target_frame)
                return

        fx, fy, cx, cy = self._intrinsics()
        for cls, u, v, score in self._detections(msg):
            z = float(self.z_by_class.get(cls, self.z_default))
            arm = pixel_to_arm_at_z(self.matrix, u, v, z, fx, fy, cx, cy)
            if arm is None:
                continue
            x_mm = arm[0] + self.fix_dx
            y_mm = arm[1] + self.fix_dy
            bx, by = arm_to_base(x_mm, y_mm, self.mount_x, self.mount_y)
            if transform is not None:
                t = transform.transform.translation
                q = transform.transform.rotation
                r = rotation_matrix((q.x, q.y, q.z, q.w))
                px = (r[0][0] * bx + r[0][1] * by + t.x)
                py = (r[1][0] * bx + r[1][1] * by + t.y)
            else:
                px, py = bx, by
            extra = "arm=(%.1f,%.1f)mm z=%+.0fmm base=(%.3f,%.3f)" % (
                x_mm, y_mm, z, bx, by)
            self._report(cls, u, v, score, (px, py), extra)

    # -- pinhole ------------------------------------------------------------

    def _handle_pinhole(self, msg: Detection2DArray) -> None:
        if self.camera is None:
            rospy.logwarn_throttle(5.0, "no camera_info yet")
            return
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
        transform = None
        for query_time in (stamp, rospy.Time(0)):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame, self.camera_frame, query_time,
                    rospy.Duration(0.10))
                break
            except Exception:
                continue
        if transform is None:
            rospy.logwarn_throttle(2.0, "TF %s <- %s unavailable"
                                   % (self.target_frame, self.camera_frame))
            return
        t = transform.transform.translation
        q = transform.transform.rotation
        camera_xyz = (t.x, t.y, t.z)
        camera_quat = (q.x, q.y, q.z, q.w)

        for cls, u, v, score in self._detections(msg):
            plane_z = float(self.plane_by_class.get(cls, self.plane_default))
            pose = pixel_to_plane(self.camera, u, v, camera_xyz, camera_quat, plane_z)
            if pose is None:
                continue
            self._report(cls, u, v, score, pose, "z=%.2f" % plane_z)

    # -- report -------------------------------------------------------------

    def _report(self, cls, u, v, score, pose, extra) -> None:
        line = "[%s] px=(%.0f,%.0f) conf=%.2f -> (%.3f, %.3f) %s" % (
            cls, u, v, score, pose[0], pose[1], extra)
        metric = None
        if self.expected is not None and (not self.expected_class
                                          or cls == self.expected_class):
            metric = math.hypot(pose[0] - self.expected[0],
                                pose[1] - self.expected[1])
            line += " | expected (%.3f, %.3f) err=%.3f m" % (
                self.expected[0], self.expected[1], metric)
        elif self.target_frame == "map":
            nearest = self._nearest(cls, pose[0], pose[1])
            if nearest is not None:
                line += " | nearest %s (%.3f, %.3f) d=%.3f m" % (
                    nearest[0], nearest[1], nearest[2], nearest[3])
                metric = nearest[3]
        if metric is not None:
            self.metrics.append(metric)

        now = time.time()
        if now - self.last_print > self.print_period:
            print(line, flush=True)
            self.last_print = now
        if len(self.metrics) >= self.samples:
            self._summary()

    def _nearest(self, cls: str, x: float, y: float):
        best = None  # (distance, name, x, y)
        for cube in self.cubes:
            if cube["cls"] != cls:
                continue
            d = math.hypot(cube["x"] - x, cube["y"] - y)
            if best is None or d < best[0]:
                best = (d, cube["name"], cube["x"], cube["y"])
        if best is None:
            return None
        return (best[1], best[2], best[3], best[0])

    def _summary(self) -> None:
        if not self.metrics:
            return
        median = statistics.median(self.metrics)
        print("SUMMARY n=%d median=%.3f m max=%.3f m  (%s)" % (
            len(self.metrics), median, max(self.metrics),
            "vs expected" if self.expected is not None else "to nearest mapped cube"),
            flush=True)
        self.metrics = []


def main() -> None:
    rospy.init_node("bridge_check")
    BridgeCheck()
    rospy.spin()


if __name__ == "__main__":
    main()
