"""
sim_node — 가상 센서 + 가상 차량 (실차 대체용, 폐루프).

동작:
  1) mission_manager의 /target_speed_mps, /target_steering_deg를 구독
  2) 그 명령으로 가상 차량(bicycle_model)을 굴림
  3) 차량의 현재 위치/자세를 기반으로 가짜 센서를 발행:
       /fix (NavSatFix)           <- rtk_node 대체
       /vel (TwistWithCovarianceStamped)
       /imu/relative_yaw_deg (Float32) <- imu_manager 대체
       /encoder/ticks (Int32)     <- 엔코더 노드 대체
  4) 재밍 존(jam_zones) 안에서는 GPS를 죽이거나(NO_FIX) 노이즈를 실음
  5) 경사 존(slope_zones)에서는 pitch를 주어 경사로를 흉내

이렇게 하면 하드웨어 없이도 'GPS 저장→재밍→추측항법→복귀' 전 과정을
실차와 동일한 토픽 구조로 검증할 수 있다.

시작 위치/heading, 재밍/경사 존은 config/sim.yaml에서 설정.
"""
import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Float32, Int32

from .bicycle_model import BicycleModel
from mission_manager.geo_utils import xy_to_latlon


class SimNode(Node):
    def __init__(self):
        super().__init__("mission_sim")
        self._declare_params()

        self.origin_lat = self.pf("origin_lat")
        self.origin_lon = self.pf("origin_lon")
        self.enc_m_per_tick = self.pf("encoder_m_per_tick")
        self.rate = self.pf("rate_hz")
        self.dt = 1.0 / self.rate

        self.car = BicycleModel(
            wheelbase=self.pf("wheelbase_m"),
            accel_gain=self.pf("accel_gain"))
        self.car.set_pose(self.pf("start_x"), self.pf("start_y"),
                          math.radians(self.pf("start_heading_deg")))

        self.cmd_v = 0.0
        self.cmd_steer = 0.0
        self.enc_ticks = 0
        self.yaw0 = self.car.heading   # relative yaw 기준
        self.t = 0.0

        self.jam_zones = self._load_zone_list("jam_zones")     # [(cx,cy,r)]
        self.slope_zones = self._load_zone_list("slope_zones") # [(cx,cy,r)]

        # 구독: mission_manager 명령
        self.create_subscription(Float32, "/target_speed_mps", self.on_speed, 10)
        self.create_subscription(Float32, "/target_steering_deg", self.on_steer, 10)

        # 발행: 가짜 센서
        self.fix_pub = self.create_publisher(NavSatFix, "/fix", 10)
        self.vel_pub = self.create_publisher(
            TwistWithCovarianceStamped, "/vel", 10)
        self.yaw_pub = self.create_publisher(Float32, "/imu/relative_yaw_deg", 10)
        self.enc_pub = self.create_publisher(Int32, "/encoder/ticks", 10)
        # 디버그: 실제 위치(정답값) 방송 — 추측항법 오차 확인용
        self.truth_pub = self.create_publisher(Float32, "/sim/truth_xy", 10)

        self.create_timer(self.dt, self.loop)
        self.get_logger().info(
            f"시뮬레이터 시작 (rate={self.rate}Hz, jam={len(self.jam_zones)}, "
            f"slope={len(self.slope_zones)})")

    def _declare_params(self):
        d = self.declare_parameter
        d("rate_hz", 20.0)
        d("origin_lat", 35.5384); d("origin_lon", 129.3114)
        d("wheelbase_m", 0.30); d("accel_gain", 2.0)
        d("start_x", 0.0); d("start_y", 0.0); d("start_heading_deg", 0.0)
        d("encoder_m_per_tick", 0.01)   # 시뮬레이터는 임의값 OK (1tick=1cm)
        d("gps_noise_std_m", 0.02)      # RTK급 저잡음
        d("jam_zones", [""])            # "cx,cy,r" 문자열 리스트
        d("slope_zones", [""])

    def pf(self, n): return float(self.get_parameter(n).value)

    def _load_zone_list(self, name):
        out = []
        for s in self.get_parameter(name).value:
            if not s:
                continue
            try:
                cx, cy, r = [float(v) for v in s.split(",")]
                out.append((cx, cy, r))
            except ValueError:
                self.get_logger().warn(f"{name} 파싱 실패: {s}")
        return out

    def on_speed(self, msg): self.cmd_v = msg.data
    def on_steer(self, msg): self.cmd_steer = math.radians(msg.data)

    def _in_any(self, zones):
        for cx, cy, r in zones:
            if math.hypot(self.car.x - cx, self.car.y - cy) <= r:
                return True
        return False

    def loop(self):
        self.t += self.dt

        # --- 경사 반영 ---
        self.car.pitch = math.radians(12.0) if self._in_any(self.slope_zones) else 0.0

        # --- 차량 물리 갱신 ---
        x, y, heading, v = self.car.step(self.cmd_v, self.cmd_steer, self.dt)

        # --- 엔코더: 이동거리 → tick 누적 ---
        ds = v * self.dt
        self.enc_ticks += int(round(ds / self.enc_m_per_tick))
        self.enc_pub.publish(Int32(data=self.enc_ticks))

        # --- IMU relative yaw ---
        rel_yaw = math.degrees(math.atan2(
            math.sin(heading - self.yaw0), math.cos(heading - self.yaw0)))
        self.yaw_pub.publish(Float32(data=rel_yaw))

        # --- GPS: 재밍 존이면 죽이거나 노이즈 ---
        jammed = self._in_any(self.jam_zones)
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = "gps"
        if jammed:
            # 재밍: NO_FIX + 공분산 폭증 (실제 재밍 증상 흉내)
            fix.status.status = NavSatStatus.STATUS_NO_FIX
            fix.latitude = float('nan')
            fix.longitude = float('nan')
            fix.position_covariance = [999.0, 0, 0, 0, 999.0, 0, 0, 0, 999.0]
        else:
            std = self.pf("gps_noise_std_m")
            nx = x + random.gauss(0, std)
            ny = y + random.gauss(0, std)
            lat, lon = xy_to_latlon(nx, ny, self.origin_lat, self.origin_lon)
            fix.status.status = NavSatStatus.STATUS_GBAS_FIX  # RTK fixed 급
            fix.status.service = NavSatStatus.SERVICE_GPS
            fix.latitude = lat
            fix.longitude = lon
            c = std * std
            fix.position_covariance = [c, 0, 0, 0, c, 0, 0, 0, c * 4]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix)

        # --- /vel (heading 정보 전달용) ---
        vel = TwistWithCovarianceStamped()
        vel.header.stamp = fix.header.stamp
        if not jammed:
            vel.twist.twist.linear.x = v * math.cos(heading)
            vel.twist.twist.linear.y = v * math.sin(heading)
        self.vel_pub.publish(vel)

        # 주기적 로그
        if int(self.t * self.rate) % int(self.rate) == 0:
            state = "JAM" if jammed else "OK "
            self.get_logger().info(
                f"[{state}] t={self.t:5.1f}s x={x:6.2f} y={y:6.2f} "
                f"hdg={math.degrees(heading):6.1f} v={v:4.2f} "
                f"cmd_v={self.cmd_v:4.2f} steer={math.degrees(self.cmd_steer):5.1f}")


def main():
    rclpy.init()

    node = SimNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
