#!/usr/bin/env python3
"""
encoder_driver_node.py — 쿼드러처 엔코더 드라이버 골격

RS-775 구동모터에 달린 쿼드러처 엔코더 → MCU(아두이노)가 tick 카운트를
시리얼로 넘겨준다는 전제. 이 노드는 그 tick을 읽어 /encoder/ticks 로 발행.

발행:
  /encoder/ticks (std_msgs/Int32)  — 누적 tick (부호 있음: 전진+, 후진-)

핵심:
  - m/tick 보정은 이 노드가 '직접' 하지 않는다. tick 원본만 내보내고,
    거리 환산(m/tick)은 추측항법(dead_reckoning) 쪽에서 곱한다.
    → 여기서는 tick의 '부호 방향'만 하드웨어와 일치시키면 됨.
  - 다만 실측 보정을 쉽게 하라고, 이 노드에 tick→m 환산값을 로그로
    찍어주는 보조 모드(CALIBRATE)를 넣어둠. 1m 굴리고 tick 세는 용도.

★ 실물 오면 채울 곳: _read_encoder() 의 시리얼 파싱 TODO 만.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


# ────────────────────────── 튜닝 상수 ──────────────────────────
# ── 시리얼(MCU) ──
SERIAL_PORT   = "/dev/ttyACM0"   # 엔코더 tick 주는 MCU 포트 (모터 MCU와 별개면 여기)
SERIAL_BAUD   = 115200
PUBLISH_HZ    = 50.0             # tick 발행 주기

# ── 부호 방향 (실차에서 반드시 확인) ──
# 차를 손으로 '전진' 방향으로 밀었을 때 tick이 증가해야 정상.
# 감소하면 True 로 바꿔서 뒤집는다. (추측항법 거리 부호가 여기 걸림)
TICK_INVERT   = False

# ── m/tick 실측 보정용 (이 노드는 참고 로그만; 실제 곱은 dead_reckoning) ──
# 아래 값은 '기록용 메모'이자 CALIBRATE 모드 로그 계산에만 쓰인다.
# 실측 절차: 아래 [보정 절차] 참고. 측정 후 이 값을 갱신하고,
#            같은 값을 dead_reckoning 파라미터에도 넣어야 실제로 반영됨.
METERS_PER_TICK = 0.0            # 0이면 미보정 상태 (거리 환산 잠김 경고)

# ── 보조 모드 ──
# True 로 두면 1초마다 "지난 1초 tick / 총 tick / (m/tick 있으면)추정거리" 로그.
# 실측 보정할 때만 켜고, 평소엔 False.
CALIBRATE_MODE = False
# ───────────────────────────────────────────────────────────────


class EncoderDriverNode(Node):
    def __init__(self):
        super().__init__("encoder_driver_node")

        self.pub = self.create_publisher(Int32, "/encoder/ticks", 10)

        # 내부 상태
        self._raw_ticks = 0        # MCU가 준 원본 누적 tick
        self._last_log_ticks = 0   # CALIBRATE 로그용

        # ── 시리얼 연결 (실물 없으면 골격 모드로 계속 돎) ──
        self._serial = None
        self._open_serial()

        # 발행 타이머
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self._tick)

        # 보정 로그 타이머 (1Hz)
        if CALIBRATE_MODE:
            self.create_timer(1.0, self._calibrate_log)
            self.get_logger().warn(
                "CALIBRATE_MODE ON — 차를 정확히 1m 직진시키고, "
                "누적 tick 증가량을 읽어 METERS_PER_TICK = 1.0/그tick 으로 계산하세요."
            )

        if METERS_PER_TICK == 0.0:
            self.get_logger().warn(
                "METERS_PER_TICK=0 (미보정). 추측항법·미션거리 환산이 잠깁니다. "
                "실측 보정 후 이 값과 dead_reckoning 파라미터를 함께 갱신하세요."
            )

        self.get_logger().info("encoder_driver_node 시작 (골격 모드)")

    def _open_serial(self):
        try:
            import serial  # pyserial
            self._serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.02)
            self.get_logger().info(f"엔코더 시리얼 열림: {SERIAL_PORT}@{SERIAL_BAUD}")
        except Exception as e:
            self._serial = None
            self.get_logger().warn(
                f"시리얼 못 엶({e}). 실물 없이 골격 모드로 계속 (tick=0 발행)."
            )

    def _read_encoder(self):
        """
        MCU에서 누적 tick 한 줄 읽어 self._raw_ticks 갱신.

        ★★★ TODO (실물 오면 여기만 채움) ★★★
        전제 프로토콜(모터 드라이버와 대칭): MCU가 한 줄에 누적 tick을
        텍스트로 흘려준다고 가정.  예)  "T12345\n"
        (누적값을 주는 방식 권장 — 패킷 유실 시 델타 누적보다 안전)

        예시 구현:
            line = self._serial.readline().decode(errors="ignore").strip()
            if line.startswith("T"):
                self._raw_ticks = int(line[1:])

        만약 MCU가 '델타(증분)'만 준다면 self._raw_ticks += delta 로.
        지금은 골격이라 아무것도 안 하고 넘어감(=tick 0 유지).
        """
        if self._serial is None:
            return
        # --- TODO: 위 예시대로 파싱해서 self._raw_ticks 갱신 ---
        pass

    def _tick(self):
        # 1) 하드웨어에서 tick 읽기 (골격이면 no-op)
        self._read_encoder()

        # 2) 부호 방향 보정
        ticks = -self._raw_ticks if TICK_INVERT else self._raw_ticks

        # 3) 발행
        msg = Int32()
        msg.data = int(ticks)
        self.pub.publish(msg)

    def _calibrate_log(self):
        cur = -self._raw_ticks if TICK_INVERT else self._raw_ticks
        delta = cur - self._last_log_ticks
        self._last_log_ticks = cur
        if METERS_PER_TICK > 0.0:
            self.get_logger().info(
                f"[CAL] Δtick(1s)={delta}  누적={cur}  "
                f"추정거리={cur * METERS_PER_TICK:.3f} m"
            )
        else:
            self.get_logger().info(
                f"[CAL] Δtick(1s)={delta}  누적={cur}  "
                f"(METERS_PER_TICK 미설정 → 거리환산 없음)"
            )


def main():
    rclpy.init()
    node = EncoderDriverNode()
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
