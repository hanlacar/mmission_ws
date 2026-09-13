#!/usr/bin/env python3
"""
Same-domain bridge:
  /camera/end_branch  (std_msgs/String: AA, AB, END_AA, END_AB)
        ->
  /dr/end_branch      (std_msgs/String: END_AA, END_AB)

A valid selection is latched while drive_mode == 11 and republished so the
DR follower cannot miss a one-shot camera decision. It is cleared when leaving
mode 11.
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CameraEndBranchBridge(Node):
    def __init__(self):
        super().__init__("camera_end_branch_bridge")

        self.declare_parameter("camera_topic", "/camera/end_branch")
        self.declare_parameter("dr_topic", "/dr/end_branch")
        self.declare_parameter("mode_topic", "/drive_mode")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("require_mode_11", True)

        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.dr_topic = str(self.get_parameter("dr_topic").value)
        self.mode_topic = str(self.get_parameter("mode_topic").value)
        self.rate_hz = max(
            1.0, float(self.get_parameter("publish_rate_hz").value))
        self.require_mode_11 = bool(
            self.get_parameter("require_mode_11").value)

        self.mode = None
        self.selected = "NONE"
        self.last_camera_raw = "NONE"
        self.last_valid_rx = None

        self.pub = self.create_publisher(String, self.dr_topic, 10)
        self.status_pub = self.create_publisher(
            String, "/camera_end_branch_bridge/status", 10)

        self.create_subscription(
            String, self.camera_topic, self._camera_cb, 10)
        self.create_subscription(
            String, self.mode_topic, self._mode_cb, 10)

        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.create_timer(1.0, self._status)

        self.get_logger().info(
            f"ready: {self.camera_topic} [AA/AB] -> "
            f"{self.dr_topic} [END_AA/END_AB], "
            f"require_mode_11={self.require_mode_11}"
        )

    @staticmethod
    def normalize(value: str):
        v = str(value).strip().upper()
        if v in ("AA", "END_AA"):
            return "END_AA"
        if v in ("AB", "END_AB"):
            return "END_AB"
        if v in ("NONE", "UNKNOWN", ""):
            return "NONE"
        return None

    def _mode_cb(self, msg: String):
        new_mode = str(msg.data).strip()

        # Clear stale selection whenever route leaves mode 11.
        if self.mode == "11" and new_mode != "11":
            self.selected = "NONE"
            self.pub.publish(String(data="NONE"))

        # Also start mode 11 with a clean decision.
        if self.mode != "11" and new_mode == "11":
            self.selected = "NONE"
            self.pub.publish(String(data="NONE"))

        self.mode = new_mode

    def _camera_cb(self, msg: String):
        raw = str(msg.data).strip()
        self.last_camera_raw = raw
        normalized = self.normalize(raw)

        if normalized is None:
            self.get_logger().warn(
                f"ignored invalid camera branch value: {raw!r}")
            return

        if self.require_mode_11 and self.mode != "11":
            self.get_logger().warn(
                f"ignored camera branch outside mode 11: "
                f"value={raw!r}, mode={self.mode!r}")
            return

        # NONE/UNKNOWN does not erase an already valid decision in mode 11.
        # This prevents transient camera uncertainty from undoing a branch.
        if normalized == "NONE":
            return

        self.selected = normalized
        self.last_valid_rx = time.monotonic()
        self.pub.publish(String(data=self.selected))
        self.get_logger().info(
            f"camera branch {raw!r} -> {self.selected}")

    def _tick(self):
        if self.require_mode_11 and self.mode != "11":
            return
        if self.selected in ("END_AA", "END_AB"):
            self.pub.publish(String(data=self.selected))

    def _status(self):
        now = time.monotonic()
        age = None if self.last_valid_rx is None else now - self.last_valid_rx
        payload = {
            "mode": self.mode,
            "camera_topic": self.camera_topic,
            "camera_raw": self.last_camera_raw,
            "selected": self.selected,
            "dr_topic": self.dr_topic,
            "last_valid_age_sec": None if age is None else round(age, 3),
            "require_mode_11": self.require_mode_11,
        }
        self.status_pub.publish(
            String(data=json.dumps(payload, separators=(",", ":"))))


def main(args=None):
    rclpy.init(args=args)
    node = CameraEndBranchBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
