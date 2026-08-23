"""
intersection_calib — 교차로 중심 좌표 캘리브레이션 노드.

목적:
  학교 사거리(교차로) 정중앙에 차를 세운 상태에서 /fix(GPS)를 일정 시간(sample_seconds)
  평균내어 교차로 중심의 로컬 좌표 (cx, cy)를 산출한다. 결과는 intersection.py가
  기대하는 yaml 조각(ix_center.center)으로 복붙 가능하게 출력한다.

  좌표 변환은 geo_utils.latlon_to_xy를 그대로 재사용한다(새 변환 로직 없음).

GPS 없이 테스트하는 법:
  이 노드는 /fix(sensor_msgs/NavSatFix)를 구독만 한다. 실제 GPS 하드웨어 대신
  fake_gps_sim 노드나 rosbag replay가 /fix를 발행하면 그대로 동작한다.

    # 터미널 1: 가짜 GPS로 /fix 공급 (예: 임의 경로로 폐루프 시뮬)
    ros2 run mission_manager fake_gps_sim --ros-args \
        -p route_csv:=$(ros2 pkg prefix mission_manager)/share/mission_manager/routes/test/06_self_intersection.csv \
        -p jam_after_s:=1000000.0

    # 터미널 2: 캘리브레이션 노드 실행 (5초간 평균)
    ros2 run mission_manager intersection_calib --ros-args -p sample_seconds:=5.0

  또는 rosbag replay: `ros2 bag play <bag>` 로 /fix가 재생되는 동안 위와 같이 실행하면 된다.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from .geo_utils import latlon_to_xy


class IntersectionCalibNode(Node):
    def __init__(self):
        super().__init__("intersection_calib")

        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("sample_seconds", 5.0)
        self.declare_parameter("origin_lat", 0.0)
        self.declare_parameter("origin_lon", 0.0)

        self.fix_topic = self.get_parameter("fix_topic").value
        self.sample_seconds = float(self.get_parameter("sample_seconds").value)
        self.origin_lat = float(self.get_parameter("origin_lat").value)
        self.origin_lon = float(self.get_parameter("origin_lon").value)
        self._origin_auto = (self.origin_lat == 0.0 and self.origin_lon == 0.0)

        self._lats = []
        self._lons = []
        self._start_time = None

        self.create_subscription(NavSatFix, self.fix_topic, self._on_fix, 10)

        self.get_logger().info(
            f"intersection_calib: {self.fix_topic} 구독 시작, "
            f"{self.sample_seconds:.1f}초간 fix 수집 (교차로 정중앙에서 정지 상태 유지)")
        if self._origin_auto:
            self.get_logger().info("origin_lat/lon 미지정 → 첫 유효 fix를 origin으로 자동 채택")

        # sample_seconds가 지나도 fix가 하나도 안 들어오면 종료되도록 워치독 타이머.
        self._timer = self.create_timer(0.5, self._check_done)

    def _valid(self, msg: NavSatFix) -> bool:
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return False
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return False
        return True

    def _on_fix(self, msg: NavSatFix):
        if not self._valid(msg):
            return

        if self._start_time is None:
            self._start_time = self.get_clock().now()

        if self._origin_auto and not self._lats:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(
                f"origin 자동 채택: lat={self.origin_lat:.8f}, lon={self.origin_lon:.8f}")

        self._lats.append(msg.latitude)
        self._lons.append(msg.longitude)

    def _check_done(self):
        if self._start_time is None:
            return  # 아직 첫 fix 안 들어옴
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        if elapsed < self.sample_seconds:
            return
        self._finish()

    def _finish(self):
        n = len(self._lats)
        if n == 0:
            self.get_logger().error("유효 fix를 하나도 못 받았습니다. /fix 소스를 확인하세요.")
            rclpy.shutdown()
            return

        lat_avg = sum(self._lats) / n
        lon_avg = sum(self._lons) / n
        cx, cy = latlon_to_xy(lat_avg, lon_avg, self.origin_lat, self.origin_lon)

        self.get_logger().info(
            f"샘플 {n}개 평균: lat={lat_avg:.8f}, lon={lon_avg:.8f} "
            f"→ local (cx={cx:.3f}, cy={cy:.3f})  [origin lat={self.origin_lat:.8f}, "
            f"lon={self.origin_lon:.8f}]")

        yaml_snippet = (
            "\n"
            "ix_center:\n"
            f"  center: [{cx:.3f}, {cy:.3f}]\n"
            "  radius_m: 2.5\n"
        )
        self.get_logger().info("결과 yaml 조각(복붙용):" + yaml_snippet)
        print(yaml_snippet)

        rclpy.shutdown()


def main():
    rclpy.init()
    node = IntersectionCalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()


if __name__ == "__main__":
    main()
