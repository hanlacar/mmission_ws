#!/usr/bin/env python3
"""Convert real camera traffic-light fusion into an optional END route branch.

This node does not drive the vehicle.

Inputs:
  /drive_mode                               std_msgs/String
  /camera/traffic_light_fused/aspect        std_msgs/String
  /camera/traffic_light_fused/confidence    std_msgs/Float32
  /camera/traffic_light_fused/diagnostics   std_msgs/String(JSON)

Output:
  /dr/end_branch                            std_msgs/String

Important:
- END_AA / END_AB mapping is NOT assumed here.
- green_down_branch and green_left_branch default to NONE.
- Until the physical course mapping is confirmed, this node publishes NONE.
- The DR follower retains its existing 5 s END_AA fallback.
"""

import json
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String


VALID_BRANCHES = ("NONE", "END_AA", "END_AB")
VALID_ASPECTS = (
    "RED", "RED_X", "YELLOW",
    "GREEN_CIRCLE", "GREEN_LEFT", "GREEN_DOWN",
    "GREEN_OTHER", "UNKNOWN",
)


class EndBranchAdapter(Node):
    def __init__(self):
        super().__init__("end_branch_adapter")

        self.declare_parameter("mode_topic", "/drive_mode")
        self.declare_parameter(
            "aspect_topic", "/camera/traffic_light_fused/aspect")
        self.declare_parameter(
            "confidence_topic", "/camera/traffic_light_fused/confidence")
        self.declare_parameter(
            "diagnostics_topic", "/camera/traffic_light_fused/diagnostics")

        self.declare_parameter("output_topic", "/dr/end_branch")
        self.declare_parameter("status_topic", "/dr/end_branch_status")

        self.declare_parameter("active_mode", "11")
        self.declare_parameter("mode_timeout_s", 1.0)
        self.declare_parameter("input_timeout_s", 0.50)
        self.declare_parameter("minimum_confidence", 0.60)
        self.declare_parameter("confirm_frames", 3)

        # Intentionally unconfigured until the real course mapping is known.
        self.declare_parameter("green_down_branch", "NONE")
        self.declare_parameter("green_left_branch", "NONE")

        # Match camera_ws EXIT_SIGNAL's stronger GREEN_DOWN contract.
        self.declare_parameter(
            "require_rgb_green_down_verified", True)

        gp = lambda n: self.get_parameter(n).value

        self.mode_topic = str(gp("mode_topic"))
        self.aspect_topic = str(gp("aspect_topic"))
        self.confidence_topic = str(gp("confidence_topic"))
        self.diagnostics_topic = str(gp("diagnostics_topic"))
        self.output_topic = str(gp("output_topic"))
        self.status_topic = str(gp("status_topic"))

        self.active_mode = str(gp("active_mode")).strip()
        self.mode_timeout = max(0.1, float(gp("mode_timeout_s")))
        self.input_timeout = max(0.1, float(gp("input_timeout_s")))
        self.minimum_confidence = float(gp("minimum_confidence"))
        self.confirm_frames = max(1, int(gp("confirm_frames")))
        self.green_down_branch = str(gp("green_down_branch")).strip().upper()
        self.green_left_branch = str(gp("green_left_branch")).strip().upper()
        self.require_rgb_green_down_verified = bool(
            gp("require_rgb_green_down_verified"))

        if self.green_down_branch not in VALID_BRANCHES:
            raise ValueError(
                "green_down_branch must be NONE, END_AA, or END_AB")
        if self.green_left_branch not in VALID_BRANCHES:
            raise ValueError(
                "green_left_branch must be NONE, END_AA, or END_AB")
        if not math.isfinite(self.minimum_confidence):
            raise ValueError("minimum_confidence must be finite")
        self.minimum_confidence = min(
            1.0, max(0.0, self.minimum_confidence))

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.branch_pub = self.create_publisher(
            String, self.output_topic, latched)
        self.status_pub = self.create_publisher(
            String, self.status_topic, latched)

        self.create_subscription(
            String, self.mode_topic, self._on_mode, 10)
        self.create_subscription(
            String, self.aspect_topic, self._on_aspect, 10)
        self.create_subscription(
            Float32, self.confidence_topic, self._on_confidence, 10)
        self.create_subscription(
            String, self.diagnostics_topic, self._on_diagnostics, 10)

        self.mode: Optional[str] = None
        self.mode_at: Optional[float] = None

        self.aspect = "UNKNOWN"
        self.aspect_at: Optional[float] = None
        self.confidence = 0.0
        self.confidence_at: Optional[float] = None

        self.rgb_green_down_verified = False
        self.diag_at: Optional[float] = None
        self.diag_stamp = None
        self.last_counted_stamp = None

        self.candidate_aspect: Optional[str] = None
        self.candidate_count = 0
        self.latched_branch: Optional[str] = None
        self.last_status = ""

        self.create_timer(0.05, self._tick)

        self._publish_branch("NONE")
        self._publish_status("IDLE")

        if (
            self.green_down_branch == "NONE"
            and self.green_left_branch == "NONE"
        ):
            self.get_logger().warning(
                "END branch mapping is unconfigured; "
                "publishing NONE until real course mapping is confirmed.")

    def _on_mode(self, msg):
        now = time.monotonic()
        new_mode = str(msg.data).strip()
        if new_mode != self.mode:
            old_mode = self.mode
            self.mode = new_mode
            self._reset_detection()
            if old_mode == self.active_mode:
                self.latched_branch = None
                self._publish_branch("NONE")
        self.mode_at = now

    def _on_aspect(self, msg):
        value = str(msg.data).strip().upper()
        if value not in VALID_ASPECTS:
            value = "UNKNOWN"
        self.aspect = value
        self.aspect_at = time.monotonic()

    def _on_confidence(self, msg):
        value = float(msg.data)
        if math.isfinite(value):
            self.confidence = min(1.0, max(0.0, value))
            self.confidence_at = time.monotonic()

    def _on_diagnostics(self, msg):
        now = time.monotonic()
        try:
            data = json.loads(msg.data)
            aspect = str(
                data.get("fused_aspect", self.aspect)).strip().upper()
            confidence = float(
                data.get("fused_confidence", self.confidence))
            stamp = data.get("stamp", data.get("stamp_ns"))
            verified = bool(
                data.get("rgb_green_down_verified", False))
            if aspect not in VALID_ASPECTS or not math.isfinite(confidence):
                raise ValueError("invalid fusion diagnostics")
            self.aspect = aspect
            self.aspect_at = now
            self.confidence = min(1.0, max(0.0, confidence))
            self.confidence_at = now
            self.rgb_green_down_verified = verified
            self.diag_stamp = stamp
            self.diag_at = now
        except (TypeError, ValueError, json.JSONDecodeError):
            self.rgb_green_down_verified = False
            self.diag_stamp = None
            self.diag_at = now

    def _fresh(self, received_at, timeout):
        return (
            received_at is not None
            and 0.0 <= time.monotonic() - received_at <= timeout
        )

    def _reset_detection(self):
        self.candidate_aspect = None
        self.candidate_count = 0
        self.last_counted_stamp = None

    def _branch_for_aspect(self, aspect):
        if aspect == "GREEN_DOWN":
            return self.green_down_branch
        if aspect == "GREEN_LEFT":
            return self.green_left_branch
        return "NONE"

    def _tick(self):
        if (
            self.mode != self.active_mode
            or not self._fresh(self.mode_at, self.mode_timeout)
        ):
            self._reset_detection()
            if self.latched_branch is None:
                self._publish_branch("NONE")
            self._publish_status("INACTIVE_OR_MODE_STALE")
            return

        if self.latched_branch is not None:
            self._publish_branch(self.latched_branch)
            self._publish_status(
                f"LATCHED:{self.latched_branch}")
            return

        if (
            not self._fresh(self.aspect_at, self.input_timeout)
            or not self._fresh(self.confidence_at, self.input_timeout)
            or not self._fresh(self.diag_at, self.input_timeout)
        ):
            self._reset_detection()
            self._publish_branch("NONE")
            self._publish_status("CAMERA_INPUT_STALE_FAIL_CLOSED")
            return

        mapped = self._branch_for_aspect(self.aspect)
        if mapped == "NONE":
            self._reset_detection()
            self._publish_branch("NONE")
            self._publish_status(
                f"UNMAPPED_OR_NON_BRANCH_ASPECT:{self.aspect}")
            return

        if self.confidence < self.minimum_confidence:
            self._reset_detection()
            self._publish_branch("NONE")
            self._publish_status(
                f"LOW_CONFIDENCE:{self.confidence:.3f}")
            return

        if (
            self.aspect == "GREEN_DOWN"
            and self.require_rgb_green_down_verified
            and not self.rgb_green_down_verified
        ):
            self._reset_detection()
            self._publish_branch("NONE")
            self._publish_status(
                "GREEN_DOWN_NOT_RGB_VERIFIED_FAIL_CLOSED")
            return

        # Count distinct fused samples only.
        sample_key = self.diag_stamp
        if sample_key is None:
            self._reset_detection()
            self._publish_branch("NONE")
            self._publish_status("FUSION_STAMP_MISSING_FAIL_CLOSED")
            return

        if sample_key != self.last_counted_stamp:
            if self.candidate_aspect == self.aspect:
                self.candidate_count += 1
            else:
                self.candidate_aspect = self.aspect
                self.candidate_count = 1
            self.last_counted_stamp = sample_key

        if self.candidate_count < self.confirm_frames:
            self._publish_branch("NONE")
            self._publish_status(
                f"CONFIRMING:{self.aspect}:"
                f"{self.candidate_count}/{self.confirm_frames}")
            return

        self.latched_branch = mapped
        self._publish_branch(mapped)
        self._publish_status(
            f"SELECTED:{mapped}:FROM_{self.aspect}")

    def _publish_branch(self, value):
        self.branch_pub.publish(String(data=str(value)))

    def _publish_status(self, value):
        value = str(value)
        if value != self.last_status:
            self.last_status = value
            self.get_logger().info(value)
        self.status_pub.publish(String(data=value))


def main(args=None):
    rclpy.init(args=args)
    node = EndBranchAdapter()
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
