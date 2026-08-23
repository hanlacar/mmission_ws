"""
mission_sequencer — 교차로 통과를 세어 '코스 구간(segment)'을 진행·발행한다.

■ 목적
  코스가 교차로로 나뉜 여러 구간으로 이루어질 때, 지금이 몇 번째 구간인지를
  바깥(로깅/카메라/MCU/상위 로직)에 알려준다. 그리고 각 교차로에서 어떤
  방위(active_dir)로 어떤 기동(직진/좌/우)을 할지 자동으로 발행한다.

  예시 코스:
    구간1 출발+경사로 ─교차로─ 구간2 S코스 ─교차로─ 구간3 T주차
        ─교차로─ 구간4 가속+평행주차 ─교차로─ 구간5 출차

■ 동작
  - /intersection/complete (Bool, latched) 를 구독한다.
    이 값이 false→true 로 바뀔 때마다 '교차로 1회 통과'로 보고 segment += 1.
  - 현재 구간을 두 토픽으로 발행한다(둘 다 latched, 늦게 켜도 마지막 값 수신):
      /mission_segment     (String) : 사람이 읽는 구간명 ("SEG3_TPARK")
      /mission_segment_id  (UInt8)  : 구간 번호 (1,2,3,...)
  - 각 구간에 진입하면, 그 구간이 끝날 때 통과할 교차로의
    방위와 기동을 미리 발행한다(선택 기능, publish_commands=True일 때):
      /intersection/active_dir_request (String) : "N"/"E"/"S"/"W"
      /intersection/command            (UInt8)  : 1=STRAIGHT,2=RIGHT,3=LEFT

■ 설정 (config yaml)
  segment_names 리스트로 구간을 정의한다. 각 구간마다:
      <name>.exit_dir       : 이 구간 끝 교차로에서의 진입 방위 N/E/S/W
      <name>.exit_maneuver  : 그 교차로에서의 기동 straight/left/right
  마지막 구간(출차)은 뒤에 교차로가 없으면 exit_dir/maneuver 를 비워둔다.

  예)
    mission_sequencer:
      ros__parameters:
        publish_commands: true
        auto_advance: true
        segment_names:    ["SEG1_START_SLOPE","SEG2_SCOURSE","SEG3_TPARK",
                           "SEG4_ACCEL_PARALLEL","SEG5_EXIT"]
        SEG1_START_SLOPE.exit_dir: "N"
        SEG1_START_SLOPE.exit_maneuver: "straight"
        SEG2_SCOURSE.exit_dir: "E"
        SEG2_SCOURSE.exit_maneuver: "left"
        SEG3_TPARK.exit_dir: "S"
        SEG3_TPARK.exit_maneuver: "right"
        SEG4_ACCEL_PARALLEL.exit_dir: "W"
        SEG4_ACCEL_PARALLEL.exit_maneuver: "straight"
        SEG5_EXIT.exit_dir: ""
        SEG5_EXIT.exit_maneuver: ""

■ 수동 제어 (검증용)
  ros2 topic pub --once /mission_sequencer/advance std_msgs/msg/Bool "{data: true}"
    → 교차로 없이 강제로 다음 구간으로 넘긴다(테스트용).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String, UInt8


_MANEUVER_CODE = {"straight": 1, "right": 2, "left": 3, "": 0}


class MissionSequencer(Node):
    def __init__(self) -> None:
        super().__init__("mission_sequencer")
        self.declare_parameter("publish_commands", True)
        self.declare_parameter("auto_advance", True)
        self.declare_parameter("segment_names", [""])

        names = [n for n in self.get_parameter("segment_names").value if n]
        if not names:
            names = ["SEG1"]
            self.get_logger().warn("segment_names 미설정 → 기본 [SEG1] 사용")

        self.segments = []
        for name in names:
            self.declare_parameter(f"{name}.exit_dir", "")
            self.declare_parameter(f"{name}.exit_maneuver", "")
            self.segments.append({
                "name": name,
                "exit_dir": str(self.get_parameter(f"{name}.exit_dir").value).strip().upper(),
                "exit_maneuver": str(self.get_parameter(f"{name}.exit_maneuver").value).strip().lower(),
            })

        self.publish_commands = bool(self.get_parameter("publish_commands").value)
        self.auto_advance = bool(self.get_parameter("auto_advance").value)

        self.index = 0                 # 현재 구간 인덱스 (0-base)
        self.prev_complete = False     # 교차로 완료 엣지 검출용

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.seg_pub = self.create_publisher(String, "/mission_segment", latched)
        self.seg_id_pub = self.create_publisher(UInt8, "/mission_segment_id", latched)
        self.dir_pub = self.create_publisher(String, "/intersection/active_dir_request", latched)
        self.cmd_pub = self.create_publisher(UInt8, "/intersection/command", 10)

        self.create_subscription(Bool, "/intersection/complete", self._on_complete, latched)
        self.create_subscription(Bool, "/mission_sequencer/advance", self._on_manual_advance, 10)
        self.add_on_set_parameters_callback(self._on_param)

        self._publish_current()
        self.get_logger().info(
            f"mission_sequencer 시작: {len(self.segments)}개 구간, "
            f"auto_advance={self.auto_advance}, publish_commands={self.publish_commands}")

    def _publish_current(self) -> None:
        seg = self.segments[self.index]
        self.seg_pub.publish(String(data=seg["name"]))
        self.seg_id_pub.publish(UInt8(data=self.index + 1))
        self.get_logger().info(
            f"구간 {self.index + 1}/{len(self.segments)}: {seg['name']}"
            + (f"  (끝 교차로: {seg['exit_dir']} {seg['exit_maneuver']})"
               if seg["exit_dir"] else "  (마지막 구간)"))
        if self.publish_commands and seg["exit_dir"]:
            self.dir_pub.publish(String(data=seg["exit_dir"]))
            code = _MANEUVER_CODE.get(seg["exit_maneuver"], 0)
            if code:
                self.cmd_pub.publish(UInt8(data=code))

    def _advance(self) -> None:
        if self.index + 1 >= len(self.segments):
            self.get_logger().info("마지막 구간 — 더 진행할 구간 없음")
            return
        self.index += 1
        self._publish_current()

    def _on_complete(self, msg: Bool) -> None:
        now = bool(msg.data)
        if now and not self.prev_complete:   # false→true 상승엣지
            if self.auto_advance:
                self.get_logger().info("교차로 통과 감지 → 다음 구간")
                self._advance()
        self.prev_complete = now

    def _on_manual_advance(self, msg: Bool) -> None:
        if bool(msg.data):
            self.get_logger().info("수동 advance 요청")
            self._advance()

    def _on_param(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == "reset" and bool(p.value):
                self.index = 0
                self.prev_complete = False
                self._publish_current()
                self.get_logger().info("구간 1로 리셋")
        return SetParametersResult(successful=True)


def main() -> None:
    rclpy.init()
    node = MissionSequencer()
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
