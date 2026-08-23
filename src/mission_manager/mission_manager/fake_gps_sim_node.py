"""
fake_gps_sim — 실차/시뮬 없이 mission_manager를 폐루프로 돌리는 가짜 GPS·IMU 시뮬레이터.

목적:
  실물 GPS/IMU/엔코더 없이도 mission_manager의 전체 판단 체인
  (GPS 추종 주행 → 교차로 방위 판정 → 좌/우/직진 → 경로 복귀,
   그리고 재밍 → nav2 폴백 전환)을 노트북에서 검증한다.

동작(폐루프):
  1) mission_manager가 내는 /target_speed_mps, /target_steering_deg 를 구독.
  2) 자전거(bicycle) 모델로 가상 차량의 (x,y,heading)을 적분.
  3) 그 위치를 xy_to_latlon 으로 되돌려 /fix (NavSatFix) 발행.
     진행방향 속도를 /vel (TwistWithCovarianceStamped) 로 발행.
     heading 변화를 상대yaw(deg)로 /imu/relative_yaw_deg 발행.
  → mission_manager는 이걸 진짜 GPS/IMU로 알고 주행 판단을 내린다.

재밍 시험(레벨 2):
  jam_after_s 초 뒤부터 jam_duration_s 초 동안 /fix 발행을 멈춘다(재밍 흉내).
  이때 mission_manager는 gps_health 재밍 판정 → nav2 폴백으로 전환해야 한다.
  (nav2 폴백까지 실제로 보려면 slam/nav2/twist_mux 시뮬도 함께 띄워야 함.
   교차로 방위 로직만 볼 거면 jam_after_s 를 아주 크게(예: 99999) 두면 됨.)

파라미터:
  route_csv         : 초기 위치·방향을 잡을 경로(첫 두 점). mission.yaml과 같은 파일 권장.
  origin_lat/lon    : 로컬(0,0)에 대응할 위경도. mission_manager auto_origin이 첫 fix로
                      origin을 잡으므로, 여기 값과 mission_manager origin이 일치하면 좌표 정합.
  start_x/start_y   : 가상 차 시작 위치(m). 기본은 route_csv 첫 점.
  start_heading_deg : 시작 heading(수학기준: 동=0, 반시계+). 기본은 route_csv 첫→둘째 점 방향.
  wheelbase_m       : 자전거모델 축거.
  rate_hz           : 시뮬 적분·발행 주기.
  jam_after_s       : 이 시각 이후 /fix 끊음(재밍). 기본 1e9(=사실상 안 끊음).
  jam_duration_s    : 재밍 지속. 이후 /fix 복구.
  fix_status        : NavSatFix status.status (0=Fixed 계열). gps_health가 정상으로 볼 값.

실행 예:
  # 교차로 방위 로직만 검증(재밍 없음)
  ros2 run mission_manager fake_gps_sim --ros-args \
    -p route_csv:=/home/ww/mission_ws/routes/course.csv \
    -p jam_after_s:=1000000.0

  # 레벨 2: 8초 주행 후 6초 재밍 → 복구
  ros2 run mission_manager fake_gps_sim --ros-args \
    -p route_csv:=/home/ww/mission_ws/routes/course.csv \
    -p jam_after_s:=8.0 -p jam_duration_s:=6.0
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Float32, Int32
from .geo_utils import xy_to_latlon
from .route_loader import load_route
from .process_singleton import acquire_process_lock


class FakeGpsSim(Node):
    def __init__(self):
        super().__init__("fake_gps_sim")
        # 두 fake 위치가 번갈아 /fix에 섞이지 않도록 도메인당 하나만 허용합니다.
        self._authority_lock = acquire_process_lock('fake_gps_sim')

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
        self.declare_parameter("fix_status", 0)      # 0=STATUS_FIX(정상)
        self.declare_parameter("good_cov_m2", 1.0)   # 양호 공분산

        self.lat0 = float(self.get_parameter("origin_lat").value)
        self.lon0 = float(self.get_parameter("origin_lon").value)
        self.L = float(self.get_parameter("wheelbase_m").value)
        self.rate = float(self.get_parameter("rate_hz").value)
        self.jam_after = float(self.get_parameter("jam_after_s").value)
        self.jam_dur = float(self.get_parameter("jam_duration_s").value)
        self.fix_status = int(self.get_parameter("fix_status").value)
        self.good_cov = float(self.get_parameter("good_cov_m2").value)

        # ---- 초기 위치/방향: 파라미터 우선, 없으면 route_csv 첫 두 점 ----
        sx = float(self.get_parameter("start_x").value)
        sy = float(self.get_parameter("start_y").value)
        sh = float(self.get_parameter("start_heading_deg").value)
        rx, ry, rh = self._seed_from_route()
        self.x = sx if not math.isnan(sx) else (rx if rx is not None else 0.0)
        self.y = sy if not math.isnan(sy) else (ry if ry is not None else 0.0)
        if not math.isnan(sh):
            self.heading = math.radians(sh)
        elif rh is not None:
            self.heading = rh
        else:
            self.heading = 0.0

        self.heading0 = self.heading      # 상대yaw 기준
        self.speed_cmd = 0.0
        self.steer_cmd_rad = 0.0
        self.max_steer_rad = math.radians(35.0)  # 물리 한계(발산 방지)

        # ---- 구독: mission_manager 명령 ----
        self.declare_parameter("drive_level_1_mps", 0.25)
        self.declare_parameter("drive_level_2_mps", 0.50)
        self.declare_parameter("drive_level_3_mps", 0.75)
        self.drive_speeds = {
            1: float(self.get_parameter("drive_level_1_mps").value),
            2: float(self.get_parameter("drive_level_2_mps").value),
            3: float(self.get_parameter("drive_level_3_mps").value),
        }
        self.create_subscription(Float32, "/gps_drive", self.on_drive, 10)
        self.create_subscription(Int32, "/gps_wheel", self.on_wheel, 10)

        # ---- 발행: 가짜 센서 ----
        self.fix_pub = self.create_publisher(NavSatFix, "/fix", 10)
        self.vel_pub = self.create_publisher(
            TwistWithCovarianceStamped, "/vel", 10)

        self.t0 = self.now_s()
        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.step)
        self._jam_logged = False
        self._unjam_logged = False

        self.get_logger().info(
            f"fake_gps_sim 시작: 시작 x={self.x:.2f} y={self.y:.2f} "
            f"heading={math.degrees(self.heading):.0f}° "
            f"origin=({self.lat0},{self.lon0}) "
            f"재밍 {self.jam_after:.0f}s~{self.jam_after + self.jam_dur:.0f}s")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _seed_from_route(self):
        path = self.get_parameter("route_csv").value
        if not path:
            return None, None, None
        try:
            route = load_route(path)
            pts = route.waypoints
            x0, y0 = pts[0].x_m, pts[0].y_m
            h = None
            if len(pts) >= 2:
                x1, y1 = pts[1].x_m, pts[1].y_m
                if math.hypot(x1 - x0, y1 - y0) > 1e-6:
                    h = math.atan2(y1 - y0, x1 - x0)
            return x0, y0, h
        except Exception as e:
            self.get_logger().warn(f"route_csv 읽기 실패: {e}")
            return None, None, None

    def on_speed(self, msg: Float32):
        self.speed_cmd = float(msg.data)

    def on_steer(self, msg: Float32):
        s = math.radians(float(msg.data))
        self.steer_cmd_rad = max(-self.max_steer_rad,
                                 min(self.max_steer_rad, s))

    def on_drive(self, msg: Float32):
        level = float(msg.data)
        if abs(level) < 0.01:
            self.speed_cmd = 0.0
            return
        speed = self.drive_speeds.get(int(round(abs(level))), 0.0)
        self.speed_cmd = math.copysign(speed, level)

    def on_wheel(self, msg: Int32):
        self.steer_cmd_rad = max(-self.max_steer_rad, min(
            self.max_steer_rad, math.radians(float(msg.data))))

    def step(self):
        # ---- 자전거 모델 적분 ----
        v = self.speed_cmd
        d = self.steer_cmd_rad
        self.x += v * math.cos(self.heading) * self.dt
        self.y += v * math.sin(self.heading) * self.dt
        if abs(self.L) > 1e-6:
            self.heading += (v / self.L) * math.tan(d) * self.dt
        self.heading = math.atan2(math.sin(self.heading),
                                  math.cos(self.heading))

        t = self.now_s() - self.t0
        jamming = (t >= self.jam_after) and (t < self.jam_after + self.jam_dur)

        # ---- /vel (진행방향 속도) ----
        vel = TwistWithCovarianceStamped()
        vel.header.stamp = self.get_clock().now().to_msg()
        vel.header.frame_id = "base_link"
        vel.twist.twist.linear.x = float(v * math.cos(self.heading))
        vel.twist.twist.linear.y = float(v * math.sin(self.heading))
        self.vel_pub.publish(vel)

        # ---- /fix (재밍 중이면 발행 중단) ----
        if jamming:
            if not self._jam_logged:
                self.get_logger().warn(
                    f"[t={t:.1f}s] === GPS 재밍 시작 (/fix 중단) → nav2 폴백 기대 ===")
                self._jam_logged = True
            return

        if self._jam_logged and not self._unjam_logged and t >= self.jam_after + self.jam_dur:
            self.get_logger().info(
                f"[t={t:.1f}s] === GPS 복구 (/fix 재개) → GPS 복귀 기대 ===")
            self._unjam_logged = True

        lat, lon = xy_to_latlon(self.x, self.y, self.lat0, self.lon0)
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = "gps"
        fix.status.status = self.fix_status
        fix.latitude = float(lat)
        fix.longitude = float(lon)
        fix.altitude = 0.0
        # position_covariance: 대각선에 양호 공분산
        cov = [0.0] * 9
        cov[0] = self.good_cov
        cov[4] = self.good_cov
        cov[8] = self.good_cov
        fix.position_covariance = cov
        fix.position_covariance_type = 2  # DIAGONAL_KNOWN
        self.fix_pub.publish(fix)


def main():
    rclpy.init()
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
