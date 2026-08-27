#!/usr/bin/env python3

"""
GPS / DR localization supervisor.

입력:
  /fix                     실제 GPS
  /odom                    encoder + IMU 기반 DR
  저장 CSV                 GPS/DR 공통 좌표계 기준

출력:
  /localization/fix        gps_route_follower가 실제 사용할 위치
  /localization/source     GPS / DR / WAITING
  /localization/gps_dr_error_m

상태:
  GPS_PRIMARY
  DR_PRIMARY
  GPS_RECOVERING
  WAITING_FOR_ANCHOR

DR 좌표 정합:
  최초 정상 GPS 위치를 route local XY로 변환하고,
  그 시점의 /odom 위치를 anchor로 잡는다.

  odom 좌표계 x축은 차량 시작방향 기준이므로,
  저장 경로의 가장 가까운 segment tangent를 초기 heading으로 사용해
  odom delta를 route 좌표계로 회전한다.
"""

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32, String

from .geo_utils import latlon_to_xy, xy_to_latlon
from .route_loader import load_route
from .route_geometry import RouteGeometry


class GpsDrSupervisor(Node):

    def __init__(self):
        super().__init__("gps_dr_supervisor")

        # ------------------------------------------------------
        # Parameters
        # ------------------------------------------------------

        self.declare_parameter("route_csv", "")

        self.declare_parameter("gps_topic", "/fix")
        self.declare_parameter("odom_topic", "/odom")

        self.declare_parameter(
            "output_fix_topic",
            "/localization/fix",
        )

        self.declare_parameter(
            "source_topic",
            "/localization/source",
        )

        self.declare_parameter(
            "error_topic",
            "/localization/gps_dr_error_m",
        )

        self.declare_parameter(
            "gps_timeout_sec",
            1.0,
        )

        self.declare_parameter(
            "max_gps_covariance_m2",
            9.0,
        )

        # GPS → DR 전환
        self.declare_parameter(
            "switch_to_dr_error_m",
            1.0,
        )

        self.declare_parameter(
            "switch_to_dr_count",
            3,
        )

        # DR → GPS 복귀
        self.declare_parameter(
            "recover_to_gps_error_m",
            0.4,
        )

        self.declare_parameter(
            "recover_to_gps_count",
            10,
        )

        self.declare_parameter(
            "control_rate_hz",
            20.0,
        )

        # ------------------------------------------------------
        # Parameters read
        # ------------------------------------------------------

        self.route_csv = str(
            self.get_parameter(
                "route_csv"
            ).value
        )

        if not self.route_csv:
            raise RuntimeError(
                "route_csv is required"
            )

        self.gps_timeout = float(
            self.get_parameter(
                "gps_timeout_sec"
            ).value
        )

        self.max_cov = float(
            self.get_parameter(
                "max_gps_covariance_m2"
            ).value
        )

        self.dr_error_threshold = float(
            self.get_parameter(
                "switch_to_dr_error_m"
            ).value
        )

        self.dr_bad_required = int(
            self.get_parameter(
                "switch_to_dr_count"
            ).value
        )

        self.gps_recover_threshold = float(
            self.get_parameter(
                "recover_to_gps_error_m"
            ).value
        )

        self.gps_good_required = int(
            self.get_parameter(
                "recover_to_gps_count"
            ).value
        )

        # ------------------------------------------------------
        # Route
        # ------------------------------------------------------

        self.route = load_route(
            self.route_csv
        )

        self.geometry = RouteGeometry(
            self.route
        )

        self.origin_lat = (
            self.route.metadata.origin_lat
        )

        self.origin_lon = (
            self.route.metadata.origin_lon
        )

        # ------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------

        self.state = "WAITING_FOR_ANCHOR"

        self.last_gps = None
        self.last_gps_time = None
        self.last_gps_good = False

        self.last_odom = None

        # 최초 GPS/DR 정합 anchor
        self.anchor_ready = False

        self.anchor_gps_x = 0.0
        self.anchor_gps_y = 0.0

        self.anchor_odom_x = 0.0
        self.anchor_odom_y = 0.0

        # odom x축 → route 좌표계 회전각
        self.anchor_heading = 0.0

        self.dr_bad_count = 0
        self.gps_recover_count = 0

        self.current_error = None

        # ------------------------------------------------------
        # ROS
        # ------------------------------------------------------

        self.fix_pub = self.create_publisher(
            NavSatFix,
            str(
                self.get_parameter(
                    "output_fix_topic"
                ).value
            ),
            10,
        )

        self.source_pub = self.create_publisher(
            String,
            str(
                self.get_parameter(
                    "source_topic"
                ).value
            ),
            10,
        )

        self.error_pub = self.create_publisher(
            Float32,
            str(
                self.get_parameter(
                    "error_topic"
                ).value
            ),
            10,
        )

        self.create_subscription(
            NavSatFix,
            str(
                self.get_parameter(
                    "gps_topic"
                ).value
            ),
            self._on_gps,
            10,
        )

        self.create_subscription(
            Odometry,
            str(
                self.get_parameter(
                    "odom_topic"
                ).value
            ),
            self._on_odom,
            10,
        )

        hz = float(
            self.get_parameter(
                "control_rate_hz"
            ).value
        )

        self.create_timer(
            1.0 / hz,
            self._tick,
        )

        self.get_logger().info(
            "GPS/DR supervisor 시작 "
            f"(GPS→DR error>{self.dr_error_threshold:.2f}m, "
            f"DR→GPS error<{self.gps_recover_threshold:.2f}m)"
        )

    # ==========================================================
    # Helpers
    # ==========================================================

    def _now(self):
        return (
            self.get_clock()
            .now()
            .nanoseconds
            * 1e-9
        )

    def _gps_valid(self, msg):
        if msg is None:
            return False

        if (
            msg.status.status
            == NavSatStatus.STATUS_NO_FIX
        ):
            return False

        if not (
            math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
        ):
            return False

        covariance = max(
            msg.position_covariance[0],
            msg.position_covariance[4],
        )

        if not math.isfinite(
            covariance
        ):
            return False

        return (
            covariance
            <= self.max_cov
        )

    # ==========================================================
    # Inputs
    # ==========================================================

    def _on_gps(
        self,
        msg: NavSatFix,
    ):

        self.last_gps = msg
        self.last_gps_time = (
            self._now()
        )

        self.last_gps_good = (
            self._gps_valid(msg)
        )

        self._try_make_anchor()

    def _on_odom(
        self,
        msg: Odometry,
    ):

        self.last_odom = msg

        self._try_make_anchor()

    # ==========================================================
    # Initial GPS ↔ DR alignment
    # ==========================================================

    def _try_make_anchor(self):

        if self.anchor_ready:
            return

        if (
            not self.last_gps_good
            or self.last_gps is None
            or self.last_odom is None
        ):
            return

        gx, gy = latlon_to_xy(
            self.last_gps.latitude,
            self.last_gps.longitude,
            self.origin_lat,
            self.origin_lon,
        )

        ox = (
            self.last_odom
            .pose.pose.position.x
        )

        oy = (
            self.last_odom
            .pose.pose.position.y
        )

        # GPS 위치에서 가장 가까운 저장경로 segment
        projection = (
            self.geometry.nearest(
                gx,
                gy,
            )
        )

        segment = (
            projection.segment
        )

        p = (
            self.route
            .waypoints[segment]
        )

        q = (
            self.route
            .waypoints[segment + 1]
        )

        route_heading = math.atan2(
            q.y_m - p.y_m,
            q.x_m - p.x_m,
        )

        self.anchor_gps_x = gx
        self.anchor_gps_y = gy

        self.anchor_odom_x = ox
        self.anchor_odom_y = oy

        self.anchor_heading = (
            route_heading
        )

        self.anchor_ready = True
        self.state = "GPS_PRIMARY"

        self.get_logger().info(
            "GPS/DR anchor 완료: "
            f"GPS=({gx:.2f},{gy:.2f}) "
            f"ODOM=({ox:.2f},{oy:.2f}) "
            f"heading="
            f"{math.degrees(route_heading):.1f}deg"
        )

    # ==========================================================
    # DR → route local XY
    # ==========================================================

    def _dr_xy(self):

        if (
            not self.anchor_ready
            or self.last_odom is None
        ):
            return None

        ox = (
            self.last_odom
            .pose.pose.position.x
        )

        oy = (
            self.last_odom
            .pose.pose.position.y
        )

        dx = (
            ox
            - self.anchor_odom_x
        )

        dy = (
            oy
            - self.anchor_odom_y
        )

        c = math.cos(
            self.anchor_heading
        )

        s = math.sin(
            self.anchor_heading
        )

        # odom local → route local
        rx = (
            self.anchor_gps_x
            + c * dx
            - s * dy
        )

        ry = (
            self.anchor_gps_y
            + s * dx
            + c * dy
        )

        return rx, ry

    # ==========================================================
    # Synthetic DR NavSatFix
    # ==========================================================

    def _make_dr_fix(
        self,
        x,
        y,
    ):

        lat, lon = xy_to_latlon(
            x,
            y,
            self.origin_lat,
            self.origin_lon,
        )

        msg = NavSatFix()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = (
            "dr"
        )

        msg.status.status = (
            NavSatStatus.STATUS_FIX
        )

        msg.latitude = float(
            lat
        )

        msg.longitude = float(
            lon
        )

        msg.altitude = 0.0

        # DR은 GPS보다 신뢰도 낮게 표시
        msg.position_covariance[0] = 2.0
        msg.position_covariance[4] = 2.0
        msg.position_covariance[8] = 4.0

        msg.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        )

        return msg

    # ==========================================================
    # Main supervisor
    # ==========================================================

    def _tick(self):

        now = self._now()

        if not self.anchor_ready:
            self.source_pub.publish(
                String(
                    data="WAITING_FOR_ANCHOR"
                )
            )
            return

        dr_xy = self._dr_xy()

        if dr_xy is None:
            return

        gps_fresh = bool(
            self.last_gps_good
            and self.last_gps_time is not None
            and (
                now
                - self.last_gps_time
                <= self.gps_timeout
            )
        )

        error = None

        if (
            gps_fresh
            and self.last_gps is not None
        ):

            gx, gy = latlon_to_xy(
                self.last_gps.latitude,
                self.last_gps.longitude,
                self.origin_lat,
                self.origin_lon,
            )

            error = math.hypot(
                gx - dr_xy[0],
                gy - dr_xy[1],
            )

            self.current_error = error

            self.error_pub.publish(
                Float32(
                    data=float(error)
                )
            )

        # ======================================================
        # GPS PRIMARY
        # ======================================================

        if self.state == "GPS_PRIMARY":

            bad = (
                not gps_fresh
                or (
                    error is not None
                    and error
                    > self.dr_error_threshold
                )
            )

            if bad:
                self.dr_bad_count += 1
            else:
                self.dr_bad_count = 0

            if (
                self.dr_bad_count
                >= self.dr_bad_required
            ):

                self.state = "DR_PRIMARY"

                self.gps_recover_count = 0

                self.get_logger().warning(
                    "GPS_PRIMARY → DR_PRIMARY "
                    f"(gps_fresh={gps_fresh}, "
                    f"error={error})"
                )

        # ======================================================
        # DR PRIMARY
        # ======================================================

        elif self.state == "DR_PRIMARY":

            recover_candidate = bool(
                gps_fresh
                and error is not None
                and error
                < self.gps_recover_threshold
            )

            if recover_candidate:
                self.gps_recover_count += 1
                self.state = "GPS_RECOVERING"
            else:
                self.gps_recover_count = 0

        # ======================================================
        # GPS RECOVERING
        # ======================================================

        elif self.state == "GPS_RECOVERING":

            recover_ok = bool(
                gps_fresh
                and error is not None
                and error
                < self.gps_recover_threshold
            )

            if recover_ok:
                self.gps_recover_count += 1

                if (
                    self.gps_recover_count
                    >= self.gps_good_required
                ):

                    self.state = "GPS_PRIMARY"

                    self.dr_bad_count = 0

                    self.get_logger().info(
                        "GPS_RECOVERING → GPS_PRIMARY"
                    )

            else:
                self.state = "DR_PRIMARY"
                self.gps_recover_count = 0

        # ======================================================
        # Publish selected localization
        # ======================================================

        if (
            self.state == "GPS_PRIMARY"
            and gps_fresh
        ):

            self.fix_pub.publish(
                self.last_gps
            )

        else:

            dr_fix = self._make_dr_fix(
                dr_xy[0],
                dr_xy[1],
            )

            self.fix_pub.publish(
                dr_fix
            )

        self.source_pub.publish(
            String(
                data=self.state
            )
        )


def main():

    rclpy.init()

    node = GpsDrSupervisor()

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