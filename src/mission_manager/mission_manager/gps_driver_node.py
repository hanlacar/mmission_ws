"""
gps_driver_node — GPS 하드웨어 드라이버 (골격).

역할:
  u-blox ZED-F9P(SparkFun RTK2)에서 위치·속도를 읽어
  mission_manager가 구독하는 토픽으로 발행한다.
    발행: /fix (sensor_msgs/NavSatFix)
          /vel (geometry_msgs/TwistWithCovarianceStamped)

★ 이 노드는 '골격'이다. 실제 하드웨어 통신부는 TODO로 비어 있다.
  실물 연결 시 _read_gps() 안을 채우면 나머지(발행 구조·타이머)는 그대로 동작.

연결 방식(실물 확정 후 결정):
  - ZED-F9P는 보통 USB(가상 시리얼 /dev/ttyACM0) 또는 UART로 붙는다.
  - 선택지 A: 기성 드라이버 `ublox_gps`(ublox_driver) 패키지를 그대로 쓰고
    이 노드는 삭제. → 가장 권장(검증된 드라이버).
  - 선택지 B: 직접 시리얼로 UBX/NMEA 파싱. 아래 골격이 B 경로용.
    NMEA면 pynmea2, UBX면 pyubx2 라이브러리 사용.

발행 규약(mission_manager가 기대하는 것):
  /fix.status.status : >=0 이면 fix 유효(STATUS_NO_FIX=-1). gps_health가 이걸 봄.
  /fix.position_covariance : 공분산(m^2). gps_health가 max_cov로 재밍 판정.
  /vel.twist.twist.linear.x,y : ENU 속도(m/s). mission_manager가 heading 계산.
    (vx,vy로 atan2 → 진행방향. GPS heading의 출처.)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistWithCovarianceStamped


class GpsDriver(Node):
    def __init__(self):
        super().__init__("gps_driver")

        # ---- 파라미터 (실물 확정 후 값 채우기) ----
        self.declare_parameter("port", "/dev/ttyACM0")   # TODO: 실제 포트 확인
        self.declare_parameter("baud", 115200)           # TODO: F9P 설정과 일치
        self.declare_parameter("frame_id", "gps")
        self.declare_parameter("publish_rate_hz", 10.0)  # F9P 출력 주기와 맞추기

        self.frame_id = self.get_parameter("frame_id").value

        # ---- 발행 ----
        self.fix_pub = self.create_publisher(NavSatFix, "/fix", 10)
        self.vel_pub = self.create_publisher(
            TwistWithCovarianceStamped, "/vel", 10)

        # ---- 하드웨어 연결 ----
        self._serial = None
        self._open_port()

        # ---- 주기 타이머 ----
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info("gps_driver 시작 (골격 — 실제 통신 TODO)")

    def _open_port(self):
        """시리얼 포트 열기. 실물 오면 구현."""
        # TODO(실물): 아래처럼 pyserial로 포트 오픈.
        #   import serial
        #   port = self.get_parameter("port").value
        #   baud = int(self.get_parameter("baud").value)
        #   self._serial = serial.Serial(port, baud, timeout=0.1)
        #   (권장) 기성 ublox_gps 드라이버를 쓰면 이 노드 자체가 불필요.
        self.get_logger().warn(
            "GPS 포트 미연결(골격). _open_port/_read_gps 채우세요.")

    def _read_gps(self):
        """
        시리얼에서 한 프레임 읽어 파싱 → dict 반환.
        실물 연결 전까지는 None(발행 안 함).

        반환 형식(실물 구현 시 이 dict를 채워 반환):
          {
            "lat": float, "lon": float, "alt": float,
            "fix_ok": bool,          # fix 유효 여부
            "cov": float,            # 위치 공분산(m^2), 없으면 추정값
            "vx": float, "vy": float # ENU 속도(m/s). UBX NAV-VELNED 등에서.
          }
        """
        # TODO(실물): pynmea2(NMEA) 또는 pyubx2(UBX)로 self._serial 파싱.
        #   NMEA: GGA(위치·fix), RMC/VTG(속도), GST(공분산)
        #   UBX : NAV-PVT 한 방에 위치·속도·정확도 다 나옴(권장).
        #   속도는 heading(cog)+speed면 vx=speed*cos, vy=speed*sin 로 변환.
        return None

    def on_timer(self):
        data = self._read_gps()
        if data is None:
            # 골격 상태: 아직 하드웨어 없음 → 아무것도 발행 안 함.
            # (mission_manager는 /fix 미수신 → GPS 없음으로 보고 추측항법/카메라로 감)
            return

        now = self.get_clock().now().to_msg()

        # ---- /fix ----
        fix = NavSatFix()
        fix.header.stamp = now
        fix.header.frame_id = self.frame_id
        fix.status.status = (NavSatStatus.STATUS_FIX if data["fix_ok"]
                             else NavSatStatus.STATUS_NO_FIX)
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = float(data["lat"])
        fix.longitude = float(data["lon"])
        fix.altitude = float(data.get("alt", 0.0))
        # 대각 공분산만 채움(수평 동일 가정). gps_health가 [0]을 max_cov와 비교.
        cov = float(data.get("cov", 0.0))
        fix.position_covariance = [cov, 0.0, 0.0,
                                   0.0, cov, 0.0,
                                   0.0, 0.0, cov]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix)

        # ---- /vel ----
        vel = TwistWithCovarianceStamped()
        vel.header.stamp = now
        vel.header.frame_id = self.frame_id
        vel.twist.twist.linear.x = float(data.get("vx", 0.0))
        vel.twist.twist.linear.y = float(data.get("vy", 0.0))
        self.vel_pub.publish(vel)


def main():
    rclpy.init()
    node = GpsDriver()
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
