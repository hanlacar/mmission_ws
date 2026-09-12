#!/usr/bin/env python3
"""Real-vehicle parking slot selector using only live front/rear LaserScan.

This node does NOT drive the vehicle. It only selects a stored DR branch:
  mode 7  -> /t_parking/selected_slot        : T_A or T_B
  mode 10 -> /parallel_parking/selected_slot : V_A or V_B

Safety policy:
- No map/Nav2/Gazebo/synthetic inputs.
- ROI geometry is disabled by default and must be calibrated on the real car.
- Missing/stale scans, incomplete ROI evidence, both occupied, or ambiguous
  evidence publish NONE and never guess.
- A valid selection is latched until the vehicle leaves that parking mode.
"""

from collections import deque
import json
import math
import time
from typing import Deque, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


UNKNOWN = "UNKNOWN"
FREE = "FREE"
OCCUPIED = "OCCUPIED"
DISABLED = "DISABLED"


class RealParkingSlotSelector(Node):
    BRANCHES = ("T_A", "T_B", "V_A", "V_B")
    SENSOR_NAMES = ("front", "rear")

    def __init__(self):
        super().__init__("real_parking_slot_selector")

        # Topics / timing
        self.declare_parameter("front_scan_topic", "/scan_front")
        self.declare_parameter("rear_scan_topic", "/scan_rear")
        self.declare_parameter("mode_topic", "/drive_mode")
        self.declare_parameter("t_selected_topic", "/t_parking/selected_slot")
        self.declare_parameter(
            "v_selected_topic", "/parallel_parking/selected_slot")
        self.declare_parameter(
            "status_topic", "/parking_slot_selector/status")
        self.declare_parameter(
            "debug_topic", "/parking_slot_selector/debug")
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("mode_timeout_s", 1.0)
        self.declare_parameter("scan_timeout_s", 0.50)

        # Multi-frame classification.
        self.declare_parameter("buffer_frames", 20)
        self.declare_parameter("minimum_frames", 8)
        self.declare_parameter("minimum_sector_rays", 5)
        self.declare_parameter("occupied_hits_per_frame", 3)
        self.declare_parameter("free_hits_per_frame", 0)
        self.declare_parameter("occupied_vote_ratio", 0.60)
        self.declare_parameter("free_vote_ratio", 0.70)

        # Fail-closed ambiguity policy.
        self.declare_parameter("allow_both_free_preference", False)
        self.declare_parameter("preferred_t_branch", "T_A")
        self.declare_parameter("preferred_v_branch", "V_A")

        # Each branch can use front, rear, or both sensors.
        # All ROI blocks are disabled by default. Real-car calibration must
        # explicitly enable the sectors that actually see each parking bay.
        for branch in self.BRANCHES:
            key = branch.lower()
            for sensor in self.SENSOR_NAMES:
                prefix = f"{key}_{sensor}"
                self.declare_parameter(f"{prefix}_enabled", False)
                self.declare_parameter(f"{prefix}_angle_min_deg", 0.0)
                self.declare_parameter(f"{prefix}_angle_max_deg", 0.0)
                self.declare_parameter(f"{prefix}_range_min_m", 0.0)
                self.declare_parameter(f"{prefix}_range_max_m", 0.0)

        gp = lambda name: self.get_parameter(name).value

        self.front_scan_topic = str(gp("front_scan_topic"))
        self.rear_scan_topic = str(gp("rear_scan_topic"))
        self.mode_topic = str(gp("mode_topic"))
        self.t_selected_topic = str(gp("t_selected_topic"))
        self.v_selected_topic = str(gp("v_selected_topic"))
        self.status_topic = str(gp("status_topic"))
        self.debug_topic = str(gp("debug_topic"))

        self.publish_hz = max(1.0, float(gp("publish_hz")))
        self.mode_timeout_s = max(0.1, float(gp("mode_timeout_s")))
        self.scan_timeout_s = max(0.1, float(gp("scan_timeout_s")))

        self.buffer_frames = max(3, int(gp("buffer_frames")))
        self.minimum_frames = max(
            1, min(self.buffer_frames, int(gp("minimum_frames"))))
        self.minimum_sector_rays = max(
            1, int(gp("minimum_sector_rays")))
        self.occupied_hits_per_frame = max(
            1, int(gp("occupied_hits_per_frame")))
        self.free_hits_per_frame = max(
            0, int(gp("free_hits_per_frame")))
        self.occupied_vote_ratio = min(
            1.0, max(0.0, float(gp("occupied_vote_ratio"))))
        self.free_vote_ratio = min(
            1.0, max(0.0, float(gp("free_vote_ratio"))))

        self.allow_both_free_preference = bool(
            gp("allow_both_free_preference"))
        self.preferred_t_branch = str(gp("preferred_t_branch")).strip()
        self.preferred_v_branch = str(gp("preferred_v_branch")).strip()

        self.rois = {}
        for branch in self.BRANCHES:
            key = branch.lower()
            self.rois[branch] = {}
            for sensor in self.SENSOR_NAMES:
                prefix = f"{key}_{sensor}"
                self.rois[branch][sensor] = {
                    "enabled": bool(gp(f"{prefix}_enabled")),
                    "angle_min_deg": float(
                        gp(f"{prefix}_angle_min_deg")),
                    "angle_max_deg": float(
                        gp(f"{prefix}_angle_max_deg")),
                    "range_min_m": float(
                        gp(f"{prefix}_range_min_m")),
                    "range_max_m": float(
                        gp(f"{prefix}_range_max_m")),
                }

        self._validate_parameters()

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.t_pub = self.create_publisher(
            String, self.t_selected_topic, latched)
        self.v_pub = self.create_publisher(
            String, self.v_selected_topic, latched)
        self.status_pub = self.create_publisher(
            String, self.status_topic, latched)
        self.debug_pub = self.create_publisher(
            String, self.debug_topic, 10)

        self.create_subscription(
            LaserScan, self.front_scan_topic,
            lambda msg: self._on_scan("front", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, self.rear_scan_topic,
            lambda msg: self._on_scan("rear", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String, self.mode_topic, self._on_mode, 10)

        self.mode: Optional[str] = None
        self.mode_received_at: Optional[float] = None

        self.last_scan_at = {"front": None, "rear": None}
        self.scan_sequence = {"front": 0, "rear": 0}

        # Per branch and sensor, retain only frame-level classification.
        self.frames: Dict[str, Dict[str, Deque[str]]] = {
            branch: {
                sensor: deque(maxlen=self.buffer_frames)
                for sensor in self.SENSOR_NAMES
            }
            for branch in self.BRANCHES
        }

        self.t_latched: Optional[str] = None
        self.v_latched: Optional[str] = None
        self.last_status = ""

        self.create_timer(1.0 / self.publish_hz, self._tick)

        # Publish a safe initial state.
        self._publish_t("NONE")
        self._publish_v("NONE")
        self._publish_status("IDLE")

        disabled = []
        for branch in self.BRANCHES:
            if not any(
                    self.rois[branch][sensor]["enabled"]
                    for sensor in self.SENSOR_NAMES):
                disabled.append(branch)
        if disabled:
            self.get_logger().warning(
                "Real-car ROI calibration required before selection. "
                "Disabled branches: " + ", ".join(disabled))

    # ------------------------------------------------------------------
    # Parameter validation
    # ------------------------------------------------------------------
    def _validate_parameters(self):
        if self.free_hits_per_frame >= self.occupied_hits_per_frame:
            raise ValueError(
                "free_hits_per_frame must be < occupied_hits_per_frame")
        if self.preferred_t_branch not in ("T_A", "T_B"):
            raise ValueError("preferred_t_branch must be T_A or T_B")
        if self.preferred_v_branch not in ("V_A", "V_B"):
            raise ValueError("preferred_v_branch must be V_A or V_B")

        for branch in self.BRANCHES:
            for sensor in self.SENSOR_NAMES:
                roi = self.rois[branch][sensor]
                if not roi["enabled"]:
                    continue
                if not (
                    math.isfinite(roi["angle_min_deg"])
                    and math.isfinite(roi["angle_max_deg"])
                    and math.isfinite(roi["range_min_m"])
                    and math.isfinite(roi["range_max_m"])
                ):
                    raise ValueError(
                        f"{branch}/{sensor}: non-finite ROI parameter")
                if roi["range_min_m"] < 0.0:
                    raise ValueError(
                        f"{branch}/{sensor}: range_min_m must be >= 0")
                if roi["range_max_m"] <= roi["range_min_m"]:
                    raise ValueError(
                        f"{branch}/{sensor}: range_max_m must be > range_min_m")

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _on_mode(self, msg: String):
        now = time.monotonic()
        new_mode = str(msg.data).strip()

        if new_mode != self.mode:
            old_mode = self.mode
            self.mode = new_mode
            self.mode_received_at = now
            self._on_mode_change(old_mode, new_mode)
        else:
            self.mode_received_at = now

    def _on_mode_change(
            self, old_mode: Optional[str], new_mode: Optional[str]):
        self._clear_frames()

        if old_mode == "7":
            self.t_latched = None
            self._publish_t("NONE")
        if old_mode == "10":
            self.v_latched = None
            self._publish_v("NONE")

        if new_mode == "7":
            self.t_latched = None
            self._publish_t("NONE")
            self._publish_status("MODE7_WAIT_LIDAR")
        elif new_mode == "10":
            self.v_latched = None
            self._publish_v("NONE")
            self._publish_status("MODE10_WAIT_LIDAR")
        else:
            self._publish_status(
                f"IDLE_MODE_{new_mode if new_mode is not None else 'NONE'}")

    def _on_scan(self, sensor: str, msg: LaserScan):
        now = time.monotonic()
        self.last_scan_at[sensor] = now
        self.scan_sequence[sensor] += 1

        if self.mode not in ("7", "10"):
            return

        branches = ("T_A", "T_B") if self.mode == "7" else ("V_A", "V_B")

        for branch in branches:
            state = self._classify_scan(msg, self.rois[branch][sensor])
            if state != DISABLED:
                self.frames[branch][sensor].append(state)

    # ------------------------------------------------------------------
    # Frame and branch classification
    # ------------------------------------------------------------------
    @staticmethod
    def _angle_in_sector(
            angle_deg: float, minimum_deg: float, maximum_deg: float) -> bool:
        angle = ((angle_deg + 180.0) % 360.0) - 180.0
        minimum = ((minimum_deg + 180.0) % 360.0) - 180.0
        maximum = ((maximum_deg + 180.0) % 360.0) - 180.0

        if minimum <= maximum:
            return minimum <= angle <= maximum
        # Wrapped sector, e.g. [170, -170].
        return angle >= minimum or angle <= maximum

    def _classify_scan(self, msg: LaserScan, roi: Dict[str, float]) -> str:
        if not roi["enabled"]:
            return DISABLED

        usable = 0
        obstacle_hits = 0

        angle = float(msg.angle_min)
        increment = float(msg.angle_increment)

        for value in msg.ranges:
            angle_deg = math.degrees(angle)
            angle += increment

            if not self._angle_in_sector(
                    angle_deg,
                    roi["angle_min_deg"],
                    roi["angle_max_deg"]):
                continue

            r = float(value)

            # NaN is unusable. +inf is a usable no-return/open ray.
            if math.isnan(r):
                continue
            if math.isfinite(r) and r < max(0.0, float(msg.range_min)):
                continue

            usable += 1

            if (
                math.isfinite(r)
                and roi["range_min_m"] <= r <= roi["range_max_m"]
            ):
                obstacle_hits += 1

        if usable < self.minimum_sector_rays:
            return UNKNOWN

        if obstacle_hits >= self.occupied_hits_per_frame:
            return OCCUPIED
        if obstacle_hits <= self.free_hits_per_frame:
            return FREE
        return UNKNOWN

    def _sensor_state(self, branch: str, sensor: str) -> str:
        roi = self.rois[branch][sensor]
        if not roi["enabled"]:
            return DISABLED

        samples = self.frames[branch][sensor]
        if len(samples) < self.minimum_frames:
            return UNKNOWN

        recent = list(samples)[-self.minimum_frames:]
        occupied = sum(1 for state in recent if state == OCCUPIED)
        free = sum(1 for state in recent if state == FREE)

        if occupied / len(recent) >= self.occupied_vote_ratio:
            return OCCUPIED
        if free / len(recent) >= self.free_vote_ratio:
            return FREE
        return UNKNOWN

    def _branch_state(self, branch: str) -> Tuple[str, Dict[str, str]]:
        sensor_states = {
            sensor: self._sensor_state(branch, sensor)
            for sensor in self.SENSOR_NAMES
        }

        enabled_states = [
            state for state in sensor_states.values()
            if state != DISABLED
        ]

        if not enabled_states:
            return UNKNOWN, sensor_states

        # Conservative fusion:
        # - any enabled sensor proving occupancy => occupied
        # - every enabled sensor proving free => free
        # - otherwise unknown
        if OCCUPIED in enabled_states:
            return OCCUPIED, sensor_states
        if all(state == FREE for state in enabled_states):
            return FREE, sensor_states
        return UNKNOWN, sensor_states

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def _tick(self):
        now = time.monotonic()

        if (
            self.mode_received_at is None
            or now - self.mode_received_at > self.mode_timeout_s
        ):
            self._publish_status("MODE_STALE_FAIL_CLOSED")
            self._clear_unlatched_outputs()
            return

        if self.mode == "7":
            self._evaluate_mode(
                ("T_A", "T_B"),
                parking_kind="T",
            )
        elif self.mode == "10":
            self._evaluate_mode(
                ("V_A", "V_B"),
                parking_kind="V",
            )

    def _evaluate_mode(
            self, branches: Tuple[str, str], parking_kind: str):
        if parking_kind == "T" and self.t_latched is not None:
            self._publish_t(self.t_latched)
            return
        if parking_kind == "V" and self.v_latched is not None:
            self._publish_v(self.v_latched)
            return

        now = time.monotonic()

        required_sensors = set()
        for branch in branches:
            for sensor in self.SENSOR_NAMES:
                if self.rois[branch][sensor]["enabled"]:
                    required_sensors.add(sensor)

        if not required_sensors:
            self._publish_status(
                f"{parking_kind}_ROI_NOT_CONFIGURED_FAIL_CLOSED")
            self._clear_unlatched_output(parking_kind)
            return

        stale = [
            sensor
            for sensor in required_sensors
            if (
                self.last_scan_at[sensor] is None
                or now - self.last_scan_at[sensor] > self.scan_timeout_s
            )
        ]
        if stale:
            self._publish_status(
                f"{parking_kind}_SCAN_STALE_FAIL_CLOSED:"
                + ",".join(sorted(stale)))
            self._clear_unlatched_output(parking_kind)
            return

        states = {}
        sensor_debug = {}
        for branch in branches:
            states[branch], sensor_debug[branch] = self._branch_state(branch)

        a, b = branches
        selected = None

        # Strongest safe decision: exactly one free and the other occupied.
        if states[a] == FREE and states[b] == OCCUPIED:
            selected = a
        elif states[b] == FREE and states[a] == OCCUPIED:
            selected = b
        elif states[a] == FREE and states[b] == FREE:
            if self.allow_both_free_preference:
                selected = (
                    self.preferred_t_branch
                    if parking_kind == "T"
                    else self.preferred_v_branch
                )

        debug = {
            "mode": self.mode,
            "kind": parking_kind,
            "states": states,
            "sensor_states": sensor_debug,
            "selected": selected or "NONE",
            "front_age_s": self._age("front"),
            "rear_age_s": self._age("rear"),
            "front_seq": self.scan_sequence["front"],
            "rear_seq": self.scan_sequence["rear"],
        }
        self.debug_pub.publish(
            String(data=json.dumps(debug, separators=(",", ":"))))

        if selected is None:
            self._publish_status(
                f"{parking_kind}_AMBIGUOUS_FAIL_CLOSED:"
                f"{a}={states[a]},{b}={states[b]}")
            self._clear_unlatched_output(parking_kind)
            return

        if parking_kind == "T":
            self.t_latched = selected
            self._publish_t(selected)
        else:
            self.v_latched = selected
            self._publish_v(selected)

        self._publish_status(
            f"{parking_kind}_SELECTED:{selected}")

    # ------------------------------------------------------------------
    # Output helpers
    # ------------------------------------------------------------------
    def _age(self, sensor: str) -> Optional[float]:
        if self.last_scan_at[sensor] is None:
            return None
        return max(0.0, time.monotonic() - self.last_scan_at[sensor])

    def _clear_frames(self):
        for branch in self.BRANCHES:
            for sensor in self.SENSOR_NAMES:
                self.frames[branch][sensor].clear()

    def _clear_unlatched_output(self, kind: str):
        if kind == "T" and self.t_latched is None:
            self._publish_t("NONE")
        elif kind == "V" and self.v_latched is None:
            self._publish_v("NONE")

    def _clear_unlatched_outputs(self):
        if self.t_latched is None:
            self._publish_t("NONE")
        if self.v_latched is None:
            self._publish_v("NONE")

    def _publish_t(self, value: str):
        self.t_pub.publish(String(data=str(value)))

    def _publish_v(self, value: str):
        self.v_pub.publish(String(data=str(value)))

    def _publish_status(self, value: str):
        value = str(value)
        if value != self.last_status:
            self.last_status = value
            self.get_logger().info(value)
        self.status_pub.publish(String(data=value))


def main(args=None):
    rclpy.init(args=args)
    node = RealParkingSlotSelector()
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
