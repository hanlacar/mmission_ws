"""
fake_gps_sim — 실차/시뮬 없이 mission_manager를 폐루프로 돌리는 가짜 GPS·IMU 시뮬레이터.

동작:
  1) /gps_drive, /gps_wheel 구독
  2) bicycle model로 가상 차량 x, y, heading 적분
  3) /fix, /vel 발행
  4) 가상 heading을 /imu/relative_yaw_deg 로 발행
  5) /imu/valid=True 발행

따라서 별도의 가짜 IMU publisher 없이
fake_gps_sim 하나가 GPS + IMU 역할을 모두 수행한다.
"""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Bool, Float32, Int32

from .geo_utils import xy_to_latlon
from .route_loader import load_route
from .process_singleton import acquire_process_lock


class FakeGpsSim(Node):
    def __init__(self):
        super().__init__("fake_gps_sim")

        # 같은 도메인에서 fake GPS 중복 실행 방지
        self._authority_lock = acquire_process_lock("fake_gps_sim")

        # ----------------------------------------------------------
        # 기본 파라미터
        # ----------------------------------------------------------

        self.declare_parameter("route_csv", "")

        self.declare_parameter("origin_lat", 37.5)
        self.declare_parameter("origin_lon", 127.0)

        self.declare_parameter("start_x", float("nan"))
        self.declare_parameter("start_y", float("nan"))
        self.declare_parameter("start_heading_deg", float("nan"))

        self.declare_parameter("wheelbase_m", 0.30)
        self.declare_parameter("rate_hz", 20.0)

        self.declare_parameter("jam_after_s", 1.0e9)
        self.declare_parameter("jam_duration_s", 6.0)

        self.declare_parameter("fix_status", 0)
        self.declare_parameter("good_cov_m2", 1.0)

        # drive level → m/s
        self.declare_parameter("drive_level_1_mps", 0.25)
        self.declare_parameter("drive_level_2_mps", 0.50)
        self.declare_parameter("drive_level_3_mps", 0.75)

        # ----------------------------------------------------------
        # 파라미터 읽기
        # ----------------------------------------------------------

        self.lat0 = float(
            self.get_parameter("origin_lat").value
        )

        self.lon0 = float(
            self.get_parameter("origin_lon").value
        )

        self.L = float(
            self.get_parameter("wheelbase_m").value
        )

        self.rate = float(
            self.get_parameter("rate_hz").value
        )

        self.jam_after = float(
            self.get_parameter("jam_after_s").value
        )

        self.jam_dur = float(
            self.get_parameter("jam_duration_s").value
        )

        self.fix_status = int(
            self.get_parameter("fix_status").value
        )

        self.good_cov = float(
            self.get_parameter("good_cov_m2").value
        )

        self.drive_speeds = {
            1: float(
                self.get_parameter(
                    "drive_level_1_mps"
                ).value
            ),
            2: float(
                self.get_parameter(
                    "drive_level_2_mps"
                ).value
            ),
            3: float(
                self.get_parameter(
                    "drive_level_3_mps"
                ).value
            ),
        }

        # ----------------------------------------------------------
        # 초기 위치 / heading
        # ----------------------------------------------------------

        sx = float(
            self.get_parameter("start_x").value
        )

        sy = float(
            self.get_parameter("start_y").value
        )

        sh = float(
            self.get_parameter(
                "start_heading_deg"
            ).value
        )

        rx, ry, rh = self._seed_from_route()

        self.x = (
            sx
            if not math.isnan(sx)
            else (
                rx
                if rx is not None
                else 0.0
            )
        )

        self.y = (
            sy
            if not math.isnan(sy)
            else (
                ry
                if ry is not None
                else 0.0
            )
        )

        if not math.isnan(sh):
            self.heading = math.radians(sh)

        elif rh is not None:
            self.heading = rh

        else:
            self.heading = 0.0

        # 상대 yaw 기준점
        self.heading0 = self.heading

        # 명령 초기값
        self.speed_cmd = 0.0
        self.steer_cmd_rad = 0.0

        # 시뮬레이션 조향 물리 한계
        self.max_steer_rad = math.radians(35.0)

        # ----------------------------------------------------------
        # 명령 구독
        # ----------------------------------------------------------

        self.create_subscription(
            Float32,
            "/gps_drive",
            self.on_drive,
            10,
        )

        self.create_subscription(
            Int32,
            "/gps_wheel",
            self.on_wheel,
            10,
        )

        # ----------------------------------------------------------
        # 센서 publisher
        # ----------------------------------------------------------

        self.fix_pub = self.create_publisher(
            NavSatFix,
            "/fix",
            10,
        )

        self.vel_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            "/vel",
            10,
        )

        # 가짜 IMU heading
        self.imu_yaw_pub = self.create_publisher(
            Float32,
            "/imu/relative_yaw_deg",
            10,
        )

        self.imu_valid_pub = self.create_publisher(
            Bool,
            "/imu/valid",
            10,
        )

        # ----------------------------------------------------------
        # Timer
        # ----------------------------------------------------------

        self.t0 = self.now_s()

        self.dt = 1.0 / self.rate

        self.timer = self.create_timer(
            self.dt,
            self.step,
        )

        self._jam_logged = False
        self._unjam_logged = False

        self.get_logger().info(
            f"fake_gps_sim 시작: "
            f"시작 x={self.x:.2f} "
            f"y={self.y:.2f} "
            f"heading={math.degrees(self.heading):.1f}° "
            f"origin=({self.lat0},{self.lon0}) "
            f"재밍 {self.jam_after:.0f}s"
            f"~{self.jam_after + self.jam_dur:.0f}s"
        )

    # --------------------------------------------------------------
    # 현재 ROS 시간
    # --------------------------------------------------------------

    def now_s(self):
        return (
            self.get_clock()
            .now()
            .nanoseconds
            * 1e-9
        )

    # --------------------------------------------------------------
    # CSV 첫 위치 / heading 읽기
    # --------------------------------------------------------------

    def _seed_from_route(self):
        path = self.get_parameter(
            "route_csv"
        ).value

        if not path:
            return None, None, None

        try:
            route = load_route(path)

            pts = route.waypoints

            if not pts:
                return None, None, None

            x0 = pts[0].x_m
            y0 = pts[0].y_m

            heading = None

            if len(pts) >= 2:
                x1 = pts[1].x_m
                y1 = pts[1].y_m

                if (
                    math.hypot(
                        x1 - x0,
                        y1 - y0,
                    )
                    > 1e-6
                ):
                    heading = math.atan2(
                        y1 - y0,
                        x1 - x0,
                    )

            return (
                x0,
                y0,
                heading,
            )

        except Exception as e:
            self.get_logger().warn(
                f"route_csv 읽기 실패: {e}"
            )

            return (
                None,
                None,
                None,
            )

    # --------------------------------------------------------------
    # /gps_drive
    # --------------------------------------------------------------

    def on_drive(self, msg: Float32):
        level = float(msg.data)

        # 정지
        if abs(level) < 0.01:
            self.speed_cmd = 0.0
            return

        level_abs = int(
            round(abs(level))
        )

        speed = self.drive_speeds.get(
            level_abs,
            0.0,
        )

        # 음수면 후진
        self.speed_cmd = math.copysign(
            speed,
            level,
        )

    # --------------------------------------------------------------
    # /gps_wheel
    # --------------------------------------------------------------

    def on_wheel(self, msg: Int32):
        wheel_deg = float(msg.data)

        wheel_rad = math.radians(
            wheel_deg
        )

        self.steer_cmd_rad = max(
            -self.max_steer_rad,
            min(
                self.max_steer_rad,
                wheel_rad,
            ),
        )

    # --------------------------------------------------------------
    # 주기 실행
    # --------------------------------------------------------------

    def step(self):
        # ----------------------------------------------------------
        # Bicycle model
        # ----------------------------------------------------------

        v = self.speed_cmd
        steering = self.steer_cmd_rad

        self.x += (
            v
            * math.cos(self.heading)
            * self.dt
        )

        self.y += (
            v
            * math.sin(self.heading)
            * self.dt
        )

        if abs(self.L) > 1e-6:
            self.heading += (
                (v / self.L)
                * math.tan(steering)
                * self.dt
            )

        # heading -pi ~ +pi 정규화
        self.heading = math.atan2(
            math.sin(self.heading),
            math.cos(self.heading),
        )

        # ----------------------------------------------------------
        # 가짜 IMU
        # ----------------------------------------------------------

        relative_yaw = math.atan2(
            math.sin(
                self.heading
                - self.heading0
            ),
            math.cos(
                self.heading
                - self.heading0
            ),
        )

        relative_yaw_deg = (
            math.degrees(
                relative_yaw
            )
        )

        self.imu_yaw_pub.publish(
            Float32(
                data=float(
                    relative_yaw_deg
                )
            )
        )

        self.imu_valid_pub.publish(
            Bool(
                data=True
            )
        )

        # ----------------------------------------------------------
        # 현재 시뮬 시간 / 재밍 상태
        # ----------------------------------------------------------

        t = (
            self.now_s()
            - self.t0
        )

        jamming = (
            t >= self.jam_after
            and
            t
            < (
                self.jam_after
                + self.jam_dur
            )
        )

        # ----------------------------------------------------------
        # /vel
        # ----------------------------------------------------------

        vel = TwistWithCovarianceStamped()

        vel.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        vel.header.frame_id = (
            "base_link"
        )

        vel.twist.twist.linear.x = float(
            v
            * math.cos(
                self.heading
            )
        )

        vel.twist.twist.linear.y = float(
            v
            * math.sin(
                self.heading
            )
        )

        self.vel_pub.publish(
            vel
        )

        # ----------------------------------------------------------
        # GPS 재밍
        # ----------------------------------------------------------

        if jamming:
            if not self._jam_logged:
                self.get_logger().warn(
                    f"[t={t:.1f}s] "
                    "GPS 재밍 시작 "
                    "(/fix 발행 중단)"
                )

                self._jam_logged = True

            return

        # GPS 복구
        if (
            self._jam_logged
            and
            not self._unjam_logged
            and
            t
            >= (
                self.jam_after
                + self.jam_dur
            )
        ):
            self.get_logger().info(
                f"[t={t:.1f}s] "
                "GPS 복구 "
                "(/fix 재개)"
            )

            self._unjam_logged = True

        # ----------------------------------------------------------
        # xy → GPS
        # ----------------------------------------------------------

        lat, lon = xy_to_latlon(
            self.x,
            self.y,
            self.lat0,
            self.lon0,
        )

        fix = NavSatFix()

        fix.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        fix.header.frame_id = "gps"

        fix.status.status = (
            self.fix_status
        )

        fix.latitude = float(lat)
        fix.longitude = float(lon)
        fix.altitude = 0.0

        # ----------------------------------------------------------
        # covariance
        # ----------------------------------------------------------

        covariance = [
            0.0
        ] * 9

        covariance[0] = (
            self.good_cov
        )

        covariance[4] = (
            self.good_cov
        )

        covariance[8] = (
            self.good_cov
        )

        fix.position_covariance = (
            covariance
        )

        # DIAGONAL_KNOWN
        fix.position_covariance_type = 2

        self.fix_pub.publish(
            fix
        )


def main(args=None):
    rclpy.init(args=args)

    node = FakeGpsSim()

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