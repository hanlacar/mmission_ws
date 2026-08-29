"""Read-only production monitor for GPS reference-route tracking accuracy."""

import csv
import json
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Int32, String
from std_srvs.srv import Trigger

from .geo_utils import latlon_to_xy
from .route_accuracy import AccuracyAccumulator, encoder_calibrated, valid_gps_fix
from .route_geometry import RouteGeometry
from .route_loader import RouteValidationError, load_route


CSV_COLUMNS = (
    "timestamp", "route_index", "reference_x_m", "reference_y_m",
    "actual_x_m", "actual_y_m", "cross_track_error_m", "mean_error_m",
    "rmse_m", "max_error_m", "accuracy_pct", "route_progress_pct",
    "gps_stability", "encoder_count", "encoder_distance_m",
)


class RouteAccuracyMonitor(Node):
    """Measures route adherence and never publishes vehicle commands."""

    def __init__(self) -> None:
        super().__init__("route_accuracy_monitor")
        defaults = {
            "route_csv": "",
            "fix_topic": "/fix",
            "position_source": "gps",
            "accuracy_tolerance_m": 0.30,
            "max_route_match_distance_m": 3.0,
            "gps_timeout_sec": 1.0,
            "gps_stability_topic": "/gps_stability",
            "encoder_topic": "/mcu/encoder",
            "encoder_distance_topic": "/mcu/distance_m",
            "odom_topic": "/odom",
            "encoder_counts_per_meter": 0.0,
            "save_csv": True,
            "output_csv": "/home/ww/mmission_ws/logs/route_accuracy.csv",
            "csv_flush_every": 20,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        try:
            self.route = load_route(self._text("route_csv"))
        except RouteValidationError as exc:
            self.get_logger().fatal(str(exc))
            raise RuntimeError(str(exc)) from exc
        self.geometry = RouteGeometry(self.route)
        tolerance = float(self.get_parameter("accuracy_tolerance_m").value)
        self.max_match_distance = float(
            self.get_parameter("max_route_match_distance_m").value)
        self.gps_timeout = float(self.get_parameter("gps_timeout_sec").value)
        if not math.isfinite(self.max_match_distance) or self.max_match_distance <= 0.0:
            raise ValueError("max_route_match_distance_m must be finite and > 0")
        if not math.isfinite(self.gps_timeout) or self.gps_timeout <= 0.0:
            raise ValueError("gps_timeout_sec must be finite and > 0")
        self.statistics = AccuracyAccumulator(tolerance)

        self.last_fix_time = None
        self.last_gps_xy = None
        self.gps_travel_distance_m = 0.0
        self.gps_stability = None
        self.encoder_count = None
        self.encoder_distance_raw_m = None
        self.encoder_distance_baseline_m = None
        self.encoder_distance_m = None
        self.last_odom = None
        self.encoder_counts_per_meter = float(
            self.get_parameter("encoder_counts_per_meter").value)
        self.encoder_available = encoder_calibrated(self.encoder_counts_per_meter)
        self.state = "WAITING_FOR_GPS"
        self.current_route_index = 0
        self.route_progress_pct = 0.0

        self.position_source = (
            self._text("position_source")
            .strip()
            .lower()
        )

        if self.position_source not in (
            "gps",
            "odom",
        ):
            raise ValueError(
                "position_source must be gps or odom"
            )

        self.heading_errors_deg = []
        self.current_heading_error_deg = 0.0
        self.mean_heading_error_deg = 0.0
        self.max_heading_error_deg = 0.0

        self.start_error_m = None
        self.goal_error_m = None

        self.csv_stream = None
        self.csv_writer = None
        self.csv_rows_since_flush = 0
        self.csv_flush_every = max(1, int(self.get_parameter("csv_flush_every").value))
        if bool(self.get_parameter("save_csv").value):
            output = Path(self._text("output_csv"))
            output.parent.mkdir(parents=True, exist_ok=True)
            self.csv_stream = output.open("w", newline="", encoding="utf-8", buffering=8192)
            self.csv_writer = csv.DictWriter(self.csv_stream, fieldnames=CSV_COLUMNS)
            self.csv_writer.writeheader()

        self.metric_publishers = {
            "current_error_m": self.create_publisher(
                Float32, "/route_accuracy/current_error_m", 10),
            "mean_error_m": self.create_publisher(
                Float32, "/route_accuracy/mean_error_m", 10),
            "rmse_m": self.create_publisher(Float32, "/route_accuracy/rmse_m", 10),
            "max_error_m": self.create_publisher(
                Float32, "/route_accuracy/max_error_m", 10),
            "accuracy_pct": self.create_publisher(
                Float32, "/route_accuracy/accuracy_pct", 10),
            "route_progress_pct": self.create_publisher(
                Float32, "/route_accuracy/route_progress_pct", 10),
        }
        self.route_index_pub = self.create_publisher(
            Int32, "/route_accuracy/current_route_index", 10)
        self.gps_distance_pub = self.create_publisher(
            Float32, "/route_accuracy/gps_travel_distance_m", 10)
        self.encoder_distance_pub = self.create_publisher(
            Float32, "/route_accuracy/encoder_travel_distance_m", 10)
        self.distance_difference_pub = self.create_publisher(
            Float32, "/route_accuracy/distance_difference_m", 10)
        self.total_distance_pub = self.create_publisher(
            Float32, "/route_accuracy/total_driven_distance_m", 10)
        self.status_pub = self.create_publisher(String, "/route_accuracy/status", 10)

        self.create_subscription(NavSatFix, self._text("fix_topic"), self._on_fix, 10)
        self.create_subscription(Float32, self._text("gps_stability_topic"),
                                 self._on_gps_stability, 10)
        self.create_subscription(Int32, self._text("encoder_topic"),
                                 self._on_encoder, 10)
        self.create_subscription(Float32, self._text("encoder_distance_topic"),
                                 self._on_encoder_distance, 10)
        self.create_subscription(Odometry, self._text("odom_topic"), self._on_odom, 10)
        self.create_service(Trigger, "/route_accuracy/reset", self._on_reset)
        self.create_timer(0.5, self._on_status_timer)

        if not self.encoder_available:
            self.get_logger().warning(
                "encoder_counts_per_meter<=0: encoder distance is UNAVAILABLE; "
                "GPS route accuracy remains active")
        self.get_logger().info(
            f"Monitoring route={self._text('route_csv')} tolerance={tolerance:.3f}m "
            f"max_match={self.max_match_distance:.3f}m")

    def _text(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_gps_stability(self, msg: Float32) -> None:
        self.gps_stability = float(msg.data) if math.isfinite(msg.data) else None

    def _on_encoder(self, msg: Int32) -> None:
        self.encoder_count = int(msg.data)

    def _on_encoder_distance(self, msg: Float32) -> None:
        if self.encoder_available and math.isfinite(msg.data):
            raw_distance = float(msg.data)
            if (self.encoder_distance_baseline_m is None or
                    (self.encoder_distance_raw_m is not None and
                     raw_distance < self.encoder_distance_raw_m)):
                # Establish a new-session baseline, and recover if the MCU restarts.
                self.encoder_distance_baseline_m = raw_distance
            self.encoder_distance_raw_m = raw_distance
            self.encoder_distance_m = max(
                0.0, raw_distance-self.encoder_distance_baseline_m)

    def _on_odom(
        self,
        msg: Odometry,
    ) -> None:

        self.last_odom = msg

        if self.position_source != "odom":
            return

        x = float(
            msg.pose.pose.position.x
        )

        y = float(
            msg.pose.pose.position.y
        )

        q = msg.pose.pose.orientation

        siny_cosp = (
            2.0
            * (
                q.w * q.z
                + q.x * q.y
            )
        )

        cosy_cosp = (
            1.0
            - 2.0
            * (
                q.y * q.y
                + q.z * q.z
            )
        )

        yaw = math.atan2(
            siny_cosp,
            cosy_cosp,
        )

        self._process_position(
            x,
            y,
            self._now(),
            heading_rad=yaw,
        )

    def _on_fix(self, msg: NavSatFix) -> None:

        if self.position_source != "gps":
            return

        now = self._now()
        if not valid_gps_fix(msg.status.status, msg.latitude, msg.longitude):
            self.state = "INVALID_GPS"
            self._publish_status(now)
            return
        self.last_fix_time = now
        metadata = self.route.metadata
        x, y = latlon_to_xy(msg.latitude, msg.longitude,
                            metadata.origin_lat, metadata.origin_lon)
        if self.last_gps_xy is not None:
            self.gps_travel_distance_m += math.hypot(
                x-self.last_gps_xy[0], y-self.last_gps_xy[1])
        self.last_gps_xy = (x, y)
        self._process_position(
            x,
            y,
            now,
            heading_rad=None,
        )

    def _process_position(
        self,
        x,
        y,
        now,
        heading_rad=None,
    ) -> None:

        projection = self.geometry.nearest(
            x,
            y,
        )

        if (
            projection.distance
            > self.max_match_distance
        ):
            self.state = "OFF_ROUTE"
            self._publish_status(now)
            return

        self.state = "TRACKING"

        snapshot = self.statistics.add(
            projection.distance
        )

        self.current_route_index = (
            projection.segment
        )

        self.route_progress_pct = (
            self.geometry.progress_pct(
                projection
            )
        )

        # 시작점 오차
        if self.start_error_m is None:

            first = self.route.waypoints[0]

            self.start_error_m = math.hypot(
                x - first.x_m,
                y - first.y_m,
            )

        # 종점 오차는 매 sample 갱신
        last = self.route.waypoints[-1]

        self.goal_error_m = math.hypot(
            x - last.x_m,
            y - last.y_m,
        )

        # DR odom일 때 heading error 계산
        if heading_rad is not None:

            route_heading = (
                self.geometry.tangent(
                    projection.segment
                )
            )

            diff = math.atan2(
                math.sin(
                    heading_rad
                    - route_heading
                ),
                math.cos(
                    heading_rad
                    - route_heading
                ),
            )

            heading_error_deg = abs(
                math.degrees(diff)
            )

            self.current_heading_error_deg = (
                heading_error_deg
            )

            self.heading_errors_deg.append(
                heading_error_deg
            )

            self.mean_heading_error_deg = (
                sum(
                    self.heading_errors_deg
                )
                / len(
                    self.heading_errors_deg
                )
            )

            self.max_heading_error_deg = max(
                self.max_heading_error_deg,
                heading_error_deg,
            )

        values = {
            "current_error_m":
                snapshot.current_error_m,
            "mean_error_m":
                snapshot.mean_error_m,
            "rmse_m":
                snapshot.rmse_m,
            "max_error_m":
                snapshot.max_error_m,
            "accuracy_pct":
                snapshot.accuracy_pct,
            "route_progress_pct":
                self.route_progress_pct,
        }

        for name, value in values.items():

            self.metric_publishers[name].publish(
                Float32(
                    data=float(value)
                )
            )

        self.route_index_pub.publish(
            Int32(
                data=self.current_route_index
            )
        )

        self._write_csv(
            now,
            projection,
            x,
            y,
            snapshot,
        )

        self._publish_status(
            now
        )

    def _write_csv(self, now, projection, x, y, snapshot) -> None:
        if self.csv_writer is None:
            return
        self.csv_writer.writerow({
            "timestamp": f"{now:.9f}",
            "route_index": projection.segment,
            "reference_x_m": f"{projection.x:.6f}",
            "reference_y_m": f"{projection.y:.6f}",
            "actual_x_m": f"{x:.6f}",
            "actual_y_m": f"{y:.6f}",
            "cross_track_error_m": f"{snapshot.current_error_m:.6f}",
            "mean_error_m": f"{snapshot.mean_error_m:.6f}",
            "rmse_m": f"{snapshot.rmse_m:.6f}",
            "max_error_m": f"{snapshot.max_error_m:.6f}",
            "accuracy_pct": f"{snapshot.accuracy_pct:.3f}",
            "route_progress_pct": f"{self.route_progress_pct:.3f}",
            "gps_stability": "" if self.gps_stability is None else f"{self.gps_stability:.3f}",
            "encoder_count": "" if self.encoder_count is None else self.encoder_count,
            "encoder_distance_m": ("" if self.encoder_distance_m is None
                                   else f"{self.encoder_distance_m:.6f}"),
        })
        self.csv_rows_since_flush += 1
        if self.csv_rows_since_flush >= self.csv_flush_every:
            self.csv_stream.flush()
            self.csv_rows_since_flush = 0

    def _on_status_timer(self) -> None:
        now = self._now()

        if self.position_source == "gps":

            if (
                self.last_fix_time is None
                or now - self.last_fix_time
                > self.gps_timeout
            ):
                self.state = "WAITING_FOR_GPS"

        elif self.position_source == "odom":

            if self.last_odom is None:
                self.state = "WAITING_FOR_ODOM"

        self._publish_status(now)

    def _publish_status(self, now: float) -> None:
        snapshot = self.statistics.snapshot()
        encoder_distance = self.encoder_distance_m
        status = {
            "state": self.state,
            "route_csv": self._text("route_csv"),
            "gps_age_sec": (None if self.last_fix_time is None
                            else round(max(0.0, now-self.last_fix_time), 3)),
            "current_error_m": snapshot.current_error_m,
            "mean_error_m": snapshot.mean_error_m,
            "rmse_m": snapshot.rmse_m,
            "max_error_m": snapshot.max_error_m,
            "p95_error_m": snapshot.p95_error_m,
            "accuracy_pct": snapshot.accuracy_pct,

            "position_source":
                self.position_source,

            "current_heading_error_deg":
                self.current_heading_error_deg,

            "mean_heading_error_deg":
                self.mean_heading_error_deg,

            "max_heading_error_deg":
                self.max_heading_error_deg,

            "start_error_m":
                self.start_error_m,

            "goal_error_m":
                self.goal_error_m,
            "valid_samples": snapshot.valid_samples,
            "success_samples": snapshot.success_samples,
            "current_route_index": self.current_route_index,
            "route_progress_pct": self.route_progress_pct,
            "gps_stability": self.gps_stability,
            "gps_travel_distance_m": self.gps_travel_distance_m,
            "total_driven_distance_m": (encoder_distance if encoder_distance is not None
                                        else self.gps_travel_distance_m),
            "encoder_state": "AVAILABLE" if encoder_distance is not None else "UNAVAILABLE",
            "encoder_count": self.encoder_count,
            "encoder_travel_distance_m": encoder_distance,
            "distance_difference_m": (None if encoder_distance is None else
                                      encoder_distance-self.gps_travel_distance_m),
            "odom_received": self.last_odom is not None,
        }
        self.status_pub.publish(String(data=json.dumps(status, separators=(",", ":"))))

    def _on_reset(self, _request, response):
        self.statistics.reset()
        self.gps_travel_distance_m = 0.0
        self.last_gps_xy = None
        self.encoder_distance_m = None
        self.encoder_distance_raw_m = None
        self.encoder_distance_baseline_m = None
        self.current_route_index = 0
        self.route_progress_pct = 0.0
        self.state = "WAITING_FOR_GPS"
        response.success = True
        response.message = "route accuracy statistics reset"
        return response

    def destroy_node(self) -> None:
        if self.csv_stream is not None:
            self.csv_stream.flush()
            self.csv_stream.close()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = RouteAccuracyMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
