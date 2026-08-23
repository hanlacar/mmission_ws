"""
intersection_command — /intersection/command(std_msgs/msg/UInt8) 값 정의.

★ 설계 변경 배경 ★
기존에는 교차로 진입 heading(GPS)을 나침반 방위(N/E/S/W)로 양자화해 그 방위에
대응하는 plan(직진/좌/우)을 자동으로 골랐다(intersection.py 옛 Road/plan 구조).
하지만 동일 방위로 진입해도 상황에 따라 직진/좌/우 어느 쪽이든 필요할 수 있으므로
"이쪽에서 들어왔으니 자동으로 이 기동" 방식은 폐기한다.

이제 IMU는 회전량·회전완료 판정에만 쓰고, 기동 선택(직진/좌/우) 자체는
이 토픽으로 외부에서 명시적으로 지정한다:
  - 현재 단계: 사람이 ros2 topic pub으로 직접 발행(교차로 Controller 독립 검증).
  - 향후: Mission Sequencer가 코스 진행 상태(INTERSECTION_1/2/3 등)에 따라 자동 발행.

값 정의:
  NONE     = 0   기동 없음. 진행 중인 ERROR 상태를 해제(초기화)하는 용도로도 쓰인다.
  STRAIGHT = 1
  RIGHT    = 2
  LEFT     = 3

발행 예 (수동 검증):
  ros2 topic pub --once /intersection/command std_msgs/msg/UInt8 "{data: 1}"  # STRAIGHT
  ros2 topic pub --once /intersection/command std_msgs/msg/UInt8 "{data: 2}"  # RIGHT
  ros2 topic pub --once /intersection/command std_msgs/msg/UInt8 "{data: 3}"  # LEFT
  ros2 topic pub --once /intersection/command std_msgs/msg/UInt8 "{data: 0}"  # NONE(초기화)
"""

NONE = 0
STRAIGHT = 1
RIGHT = 2
LEFT = 3

VALID = (NONE, STRAIGHT, RIGHT, LEFT)

_NAMES = {NONE: "NONE", STRAIGHT: "STRAIGHT", RIGHT: "RIGHT", LEFT: "LEFT"}


def name(value):
    """정수 명령값을 사람이 읽을 이름으로. 정의되지 않은 값은 숫자를 그대로 노출."""
    return _NAMES.get(value, f"UNKNOWN({value})")
