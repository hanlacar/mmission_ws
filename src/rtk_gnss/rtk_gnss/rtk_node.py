
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistWithCovarianceStamped

import serial
import socket
import base64
import threading
import time
import math
import pynmea2


# ===== 설정 =====
PORT = "/dev/ttyACM0"
BAUD = 38400

NTRIP_HOST = "www.gnssdata.or.kr"
NTRIP_PORT = 2101
MOUNTPOINT = "WSJG-RTCM32"
NTRIP_USER = "db01040902325@gmail.com"
NTRIP_PASS = "gnss"

DEFAULT_GGA = "$GPGGA,000000.00,3532.70,N,12915.29,E,1,08,1.0,50.0,M,20.0,M,,*XX"

# GST 메시지 활성화 명령 (CFG-VALSET, RAM layer, CFG-MSGOUT-NMEA_ID_GST_USB=1)
UBX_ENABLE_GST = bytes([
    0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00,
    0x00, 0x01, 0x00, 0x00,
    0xD6, 0x00, 0x91, 0x20, 0x01,
    0x22, 0x53
])

# 측정 주기 10Hz 설정 (CFG-VALSET, RAM, CFG-RATE-MEAS=100ms)
# 1Hz는 자율주행에 너무 느림 → 100ms(10Hz)로 상향
UBX_RATE_10HZ = bytes([
    0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00,
    0x00, 0x01, 0x00, 0x00,
    0x01, 0x00, 0x21, 0x30, 0x64, 0x00,
    0x51, 0xB9
])


class RtkGnssNode(Node):
    def __init__(self):
        super().__init__('rtk_gnss_node')

        # 퍼블리셔
        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        # EKF가 twist를 소비하려면 covariance가 있어야 함 → TwistWithCovarianceStamped
        self.vel_pub = self.create_publisher(
            TwistWithCovarianceStamped, 'vel', 10)

        # 시리얼 연결
        self.ser = serial.Serial(PORT, BAUD, timeout=1)
        self.get_logger().info(f"[Serial] Connected to {PORT} @ {BAUD}")

        # GST 메시지 출력 활성화 (위치 오차 표준편차)
        self.ser.write(UBX_ENABLE_GST)
        self.get_logger().info("[Config] GST output enabled on USB")

        # 측정 주기 10Hz 상향
        self.ser.write(UBX_RATE_10HZ)
        self.get_logger().info("[Config] Nav rate set to 10Hz")

        # GST에서 받은 최신 오차 표준편차 [m] (아직 못 받았으면 None)
        self.std_lat = None
        self.std_lon = None
        self.std_alt = None

        self.latest_gga = DEFAULT_GGA
        self.gga_lock = threading.Lock()

        # 마지막 fix 품질 (로그용)
        self.last_fix = None

        # GPS 읽기 스레드
        self.gps_thread = threading.Thread(target=self.read_gps, daemon=True)
        self.gps_thread.start()

        # NTRIP 스레드
        self.ntrip_thread = threading.Thread(
            target=self.ntrip_client, daemon=True)
        self.ntrip_thread.start()

    def read_gps(self):
        """GPS NMEA 읽기 → 파싱 → ROS2 토픽 퍼블리시"""
        while rclpy.ok():
            try:
                line = self.ser.readline().decode(
                    "ascii", errors="replace").strip()

                # --- GGA: 위치 (NavSatFix) ---
                if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                    with self.gga_lock:
                        self.latest_gga = line
                    self.publish_fix(line)

                # --- RMC: 속도/헤딩 (TwistWithCovarianceStamped) ---
                elif line.startswith("$GNRMC") or line.startswith("$GPRMC"):
                    self.publish_vel(line)

                # --- GST: 위치 오차 표준편차 (공분산용) ---
                elif line.startswith("$GNGST") or line.startswith("$GPGST"):
                    self.parse_gst(line)

            except Exception:
                continue

    def publish_fix(self, line):
        try:
            msg = pynmea2.parse(line)
            if msg.gps_qual is None:
                return

            fix = NavSatFix()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.header.frame_id = "gps"

            # fix 품질 → NavSatStatus
            qual = int(msg.gps_qual)
            if qual == 0:
                fix.status.status = NavSatStatus.STATUS_NO_FIX
            elif qual in (4, 5):  # RTK Fixed / Float
                fix.status.status = NavSatStatus.STATUS_GBAS_FIX
            elif qual == 2:       # DGPS
                fix.status.status = NavSatStatus.STATUS_SBAS_FIX
            else:                 # Standalone
                fix.status.status = NavSatStatus.STATUS_FIX
            fix.status.service = NavSatStatus.SERVICE_GPS

            if msg.latitude and msg.longitude:
                fix.latitude = msg.latitude
                fix.longitude = msg.longitude
                fix.altitude = float(msg.altitude) if msg.altitude else 0.0

            # 공분산: GST 오차 표준편차 → 분산(제곱)
            # NavSatFix는 ENU 순서 → [0]=East(경도), [4]=North(위도), [8]=Up(고도)
            #
            # fix 품질 게이팅: RTK Float/Standalone일 때 공분산에 배율을 곱해
            # EKF가 신뢰도 낮은 값을 덜 반영하게 함 (실내/멀티패스 튐 완화).
            # 배율은 표준편차가 아니라 분산(제곱)에 곱하므로,
            # 위치 불확실성이 sqrt(배율)배 커지는 효과.
            if qual == 4:        # RTK Fixed: cm급, GST 그대로 신뢰
                cov_scale = 1.0
            elif qual == 5:      # RTK Float: dm급, 다소 불신
                cov_scale = 25.0     # 표준편차 5배
            elif qual == 2:      # DGPS: 수십 cm~m급
                cov_scale = 100.0    # 표준편차 10배
            else:                # Standalone(1) 등: m급, 강하게 불신
                cov_scale = 2500.0   # 표준편차 50배

            if self.std_lat is not None:
                # GST가 있어도 게이팅 배율 적용
                var_e = (self.std_lon ** 2) * cov_scale
                var_n = (self.std_lat ** 2) * cov_scale
                var_u = (self.std_alt ** 2) * cov_scale
                fix.position_covariance = [
                    var_e, 0.0, 0.0,
                    0.0, var_n, 0.0,
                    0.0, 0.0, var_u,
                ]
                fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            else:
                # GST 미수신: fix 품질만으로 대략적 공분산 부여
                # RTK Fixed 기준 표준편차 ~2cm 가정 후 게이팅
                base_var = 0.02 ** 2 * cov_scale
                fix.position_covariance = [
                    base_var, 0.0, 0.0,
                    0.0, base_var, 0.0,
                    0.0, 0.0, base_var * 4,
                ]
                fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            self.fix_pub.publish(fix)

            # fix 품질 바뀔 때만 로그
            if qual != self.last_fix:
                names = {0: "No Fix", 1: "Standalone", 2: "DGPS",
                         4: "RTK FIXED", 5: "RTK Float"}
                self.get_logger().info(
                    f"[GPS] Fix:{qual} ({names.get(qual, qual)})  "
                    f"Sats:{msg.num_sats}")
                self.last_fix = qual

        except pynmea2.ParseError:
            pass

    def parse_gst(self, line):
        """GST 문장에서 위도/경도/고도 오차 표준편차 추출"""
        try:
            msg = pynmea2.parse(line)
            self.std_lat = float(msg.std_dev_latitude)
            self.std_lon = float(msg.std_dev_longitude)
            self.std_alt = float(msg.std_dev_altitude)
        except (pynmea2.ParseError, ValueError, TypeError):
            pass  # 빈 필드/파싱 실패 시 이전 값 유지

    def publish_vel(self, line):
        try:
            msg = pynmea2.parse(line)
            if msg.spd_over_grnd is None:
                return

            vel = TwistWithCovarianceStamped()
            vel.header.stamp = self.get_clock().now().to_msg()
            # base_link 프레임: EKF가 body-frame twist로 소비
            vel.header.frame_id = "base_link"

            # 속도: knots → m/s
            speed_ms = float(msg.spd_over_grnd) * 0.514444
            # 헤딩(true course, deg) → 라디안, 없으면 0
            heading_deg = float(msg.true_course) if msg.true_course else 0.0
            heading_rad = math.radians(heading_deg)

            # ENU 기준 x=동, y=북 성분으로 분해
            vx = speed_ms * math.sin(heading_rad)
            vy = speed_ms * math.cos(heading_rad)
            vel.twist.twist.linear.x = vx
            vel.twist.twist.linear.y = vy

            # 6x6 공분산 (row-major, 순서: vx,vy,vz,v_roll,v_pitch,v_yaw)
            # GNSS 속도 정확도 대략치. 정지 시 방향 신뢰 낮으므로
            # 속도가 매우 작으면 공분산을 크게 → EKF가 덜 신뢰
            if speed_ms < 0.3:
                var_v = 5.0     # 저속: 방향 불확실 → 큰 분산
            else:
                var_v = 0.05    # 주행 중: 신뢰
            cov = [0.0] * 36
            cov[0] = var_v      # vx
            cov[7] = var_v      # vy
            cov[14] = 1e6       # vz (미사용)
            cov[21] = 1e6       # v_roll (미사용)
            cov[28] = 1e6       # v_pitch (미사용)
            cov[35] = 1e6       # v_yaw (미사용)
            vel.twist.covariance = cov

            self.vel_pub.publish(vel)

        except (pynmea2.ParseError, ValueError):
            pass

    def ntrip_client(self):
        """NTRIP 캐스터 접속 → RTCM 수신 → ZED-F9P로 포워딩"""
        while rclpy.ok():
            try:
                self.get_logger().info(
                    f"[NTRIP] Connecting to "
                    f"{NTRIP_HOST}:{NTRIP_PORT}/{MOUNTPOINT}")
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(10)
                sock.connect((NTRIP_HOST, NTRIP_PORT))

                userpass = f"{NTRIP_USER}:{NTRIP_PASS}"
                auth = base64.b64encode(userpass.encode()).decode()
                request = (
                    f"GET /{MOUNTPOINT} HTTP/1.1\r\n"
                    f"Host: {NTRIP_HOST}\r\n"
                    f"Ntrip-Version: Ntrip/2.0\r\n"
                    f"User-Agent: NTRIP PythonClient/1.0\r\n"
                    f"Authorization: Basic {auth}\r\n"
                    f"\r\n"
                )
                sock.send(request.encode())

                response = sock.recv(4096)
                if b"200" not in response and b"ICY 200 OK" not in response:
                    self.get_logger().warn(
                        f"[NTRIP] 접속 실패: {response[:120]}")
                    sock.close()
                    time.sleep(5)
                    continue
                self.get_logger().info("[NTRIP] Connected! RTCM 수신 시작")

                with self.gga_lock:
                    sock.send((self.latest_gga + "\r\n").encode())

                last_gga_send = time.time()
                while rclpy.ok():
                    sock.settimeout(5)
                    data = sock.recv(4096)
                    if not data:
                        self.get_logger().warn("[NTRIP] 연결 끊김, 재접속")
                        break
                    self.ser.write(data)  # RTCM → ZED-F9P

                    now = time.time()
                    if now - last_gga_send >= 10:
                        with self.gga_lock:
                            sock.send((self.latest_gga + "\r\n").encode())
                        last_gga_send = now

                sock.close()
            except Exception as e:
                self.get_logger().error(
                    f"[NTRIP] 오류: {e}, 5초 후 재접속")
                time.sleep(5)


def main(args=None):
    rclpy.init(args=args)
    node = RtkGnssNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()