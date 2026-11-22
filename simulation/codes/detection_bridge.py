#!/usr/bin/env python3
"""Bridge /objects (Detection2DArray) to /detections_world JSON with world poses.

Two modes:

  ~mode:=dlt       (default, real robot)  4x3 transform_3d.dat + arm mount + base TF
  ~mode:=pinhole   (simulation)           camera_info + camera TF + horizontal plane

On the real robot the D435 is top-down while the URDF camera TF is horizontal, so the
pinhole mode is wrong there. The DLT mode uses the same calibrated matrix as the
production grasp pipeline and maps arm millimetres to base_footprint with the URDF
mount (base_link -> Base at 0.044, 0, 0.138, no rotation).

Run on the robot:
    python3 ~/codes/detection_bridge.py _mode:=dlt _target_frame:=map \
        _matrix:=$HOME/codes/transform_3d.dat _world:=$HOME/codes/config/arena_map.yaml
"""

from __future__ import annotations

import json
import math
import os

import rospy
import tf2_ros
import yaml
from sensor_msgs.msg import CameraInfo as CameraInfoMsg
from std_msgs.msg import String
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
    inside_arena,
    load_dlt_matrix,
    pixel_to_arm_at_z,
    pixel_to_plane,
    rotation_matrix,
)


class DetectionBridge:
    def __init__(self) -> None:
        self.mode = rospy.get_param("~mode", "dlt")
        self.objects_topic = rospy.get_param("~objects_topic", "/objects")
        self.info_topic = rospy.get_param("~camera_info_topic",
                                          "/camera/color/camera_info")
        self.output_topic = rospy.get_param("~output_topic", "/detections_world")
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.camera_frame = rospy.get_param("~camera_frame",
                                            "camera_color_optical_frame")
        self.conf_min = float(rospy.get_param("~conf_min", 0.2))
        self.debug = bool(rospy.get_param("~debug", False))
        self.arena_filter = bool(rospy.get_param("~arena_filter", True))

        # pinhole mode
        self.plane_z_default = float(rospy.get_param("~plane_z_default", 0.05))
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
                rospy.loginfo("detection bridge: DLT %s shape %s"
                              % (self.matrix_path, self.matrix.shape))
            except Exception as exc:
                rospy.logerr("cannot load DLT matrix %s: %s" % (self.matrix_path, exc))

        self.world_path = rospy.get_param(
            "~world", os.path.expanduser("~/codes/config/arena_map.yaml"))
        self.arena = {}
        if os.path.exists(self.world_path):
            with open(self.world_path) as f:
                self.arena = yaml.safe_load(f).get("arena") or {}
        else:
            rospy.logwarn("world file not found (%s); arena filter disabled"
                          % self.world_path)

        self.camera = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher(self.output_topic, String, queue_size=5)
        rospy.Subscriber(self.info_topic, CameraInfoMsg, self._on_info, queue_size=1)
        rospy.Subscriber(self.objects_topic, Detection2DArray, self._on_objects,
                         queue_size=5)
        rospy.loginfo("detection bridge: %s -> %s mode=%s target=%s"
                      % (self.objects_topic, self.output_topic, self.mode,
                         self.target_frame))

    def _on_info(self, msg: CameraInfoMsg) -> None:
        if not msg.K or len(msg.K) < 6 or msg.K[0] <= 0.0:
            rospy.logwarn_throttle(5.0, "camera_info has no valid K")
            return
        self.camera = CameraInfo.from_k(list(msg.K), msg.width, msg.height)

    def _intrinsics(self):
        if self.camera is not None:
            return (self.camera.fx, self.camera.fy, self.camera.cx, self.camera.cy)
        return (DLT_FX, DLT_FY, DLT_CX, DLT_CY)

    def _publish(self, stamp, objects) -> None:
        payload = {"stamp": stamp.to_sec(), "objects": objects}
        self.pub.publish(String(data=json.dumps(payload)))

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

    def _on_objects(self, msg: Detection2DArray) -> None:
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()
        if self.mode == "dlt":
            objects = self._convert_dlt(msg, stamp)
        else:
            objects = self._convert_pinhole(msg, stamp)
        if objects is None:
            objects = []
        self._publish(stamp, objects)

    # -- dlt ----------------------------------------------------------------

    def _convert_dlt(self, msg, stamp):
        if self.matrix is None:
            rospy.logwarn_throttle(5.0, "no DLT matrix loaded")
            return []
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
                rospy.logwarn_throttle(5.0, "TF %s <- base_footprint unavailable"
                                       % self.target_frame)
                return []

        fx, fy, cx, cy = self._intrinsics()
        objects = []
        for cls, u, v, score in self._detections(msg):
            z = float(self.z_by_class.get(cls, self.z_default))
            arm = pixel_to_arm_at_z(self.matrix, u, v, z, fx, fy, cx, cy)
            if arm is None:
                continue
            bx, by = arm_to_base(arm[0] + self.fix_dx, arm[1] + self.fix_dy,
                                 self.mount_x, self.mount_y)
            if transform is not None:
                t = transform.transform.translation
                q = transform.transform.rotation
                r = rotation_matrix((q.x, q.y, q.z, q.w))
                nx = r[0][0] * bx + r[0][1] * by + t.x
                ny = r[1][0] * bx + r[1][1] * by + t.y
                bx, by = nx, ny
            filter_arena = (self.arena_filter and self.arena
                            and self.target_frame not in ("base_footprint", "base_link"))
            if filter_arena and not inside_arena(bx, by, self.arena):
                if self.debug:
                    rospy.loginfo("[%s] px=(%.0f,%.0f) -> (%.3f,%.3f) OUTSIDE arena"
                                  % (cls, u, v, bx, by))
                continue
            if self.debug:
                rospy.loginfo("[%s] px=(%.0f,%.0f) conf=%.2f z=%+.0f -> (%.3f,%.3f)"
                              % (cls, u, v, score, z, bx, by))
            objects.append({"class": cls, "confidence": round(score, 3),
                            "x": round(bx, 3), "y": round(by, 3)})
        return objects

    # -- pinhole ------------------------------------------------------------

    def _convert_pinhole(self, msg, stamp):
        if self.camera is None:
            rospy.logwarn_throttle(5.0, "no camera_info yet")
            return []
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
            rospy.logwarn_throttle(5.0, "TF %s <- %s unavailable; publishing empty"
                                   % (self.target_frame, self.camera_frame))
            return []

        t = transform.transform.translation
        q = transform.transform.rotation
        camera_xyz = (t.x, t.y, t.z)
        camera_quat = (q.x, q.y, q.z, q.w)

        objects = []
        for cls, u, v, score in self._detections(msg):
            plane_z = float(self.plane_by_class.get(cls, self.plane_z_default))
            pose = pixel_to_plane(self.camera, u, v, camera_xyz, camera_quat, plane_z)
            if pose is None:
                continue
            if self.arena_filter and self.arena and not inside_arena(pose[0], pose[1],
                                                                     self.arena):
                continue
            if self.debug:
                rospy.loginfo("[%s] px=(%.0f,%.0f) conf=%.2f -> (%.3f,%.3f) z=%.2f"
                              % (cls, u, v, score, pose[0], pose[1], plane_z))
            objects.append({"class": cls, "confidence": round(score, 3),
                            "x": round(pose[0], 3), "y": round(pose[1], 3)})
        return objects


def main() -> None:
    rospy.init_node("detection_bridge")
    DetectionBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
