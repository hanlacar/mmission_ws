#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class OdomRouteRecorder(Node):

    def __init__(self):
        super().__init__("odom_route_recorder")

        # =========================================================
        # 파라미터
        # =========================================================

        # DR 위치 입력
        self.declare_parameter(
            "odom_topic",
            "/odom",
        )

        # 키보드 실제 구동 입력
        self.declare_parameter(
            "manual_drive_topic",
            "/manual_drive",
        )

        # 저장 파일
        self.declare_parameter(
            "out_csv",
            str(
                Path.home()
                / "mmission_ws"
                / "routes"
                / "odom_route.csv"
            ),
        )

        # 기본 진행방향
        # 필요하면 ros2 param set으로 forward/reverse 변경
        self.declare_parameter(
            "record_direction",
            "forward",
        )

        # 코스 모드 1~11
        self.declare_parameter(
            "record_mode",
            1,
        )

        # 이벤트
        self.declare_parameter(
            "record_event",
            "NONE",
        )

        # 최소 저장 거리
        self.declare_parameter(
            "min_distance_m",
            0.05,
        )

        # =========================================================
        # 동적 파라미터 변경 처리
        # =========================================================

        self.add_on_set_parameters_callback(
            self._on_parameters
        )

        # =========================================================
        # 파일 준비
        # =========================================================

        self.out_csv = Path(
            self.get_parameter(
                "out_csv"
            ).value
        ).expanduser()

        self.out_csv.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.file = self.out_csv.open(
            "w",
            newline="",
        )

        self.writer = csv.writer(
            self.file
        )

        # CSV 헤더
        self.writer.writerow(
            [
                "index",
                "x_m",
                "y_m",
                "yaw_deg",
                "direction",
                "mode",
                "drive_level",
                "event",
            ]
        )

        self.file.flush()

        # =========================================================
        # 내부 상태
        # =========================================================

        self.index = 0

        self.last_x = None
        self.last_y = None

        # 현재 키보드 drive 값
        self.current_drive_level = 0.0

        # 마지막 drive 값을 로그용으로 저장
        self.last_logged_drive = None

        # =========================================================
        # 토픽 이름
        # =========================================================

        self.odom_topic = str(
            self.get_parameter(
                "odom_topic"
            ).value
        )

        self.manual_drive_topic = str(
            self.get_parameter(
                "manual_drive_topic"
            ).value
        )

        # =========================================================
        # Subscriber
        # =========================================================

        # DR odom
        self.odom_subscription = (
            self.create_subscription(
                Odometry,
                self.odom_topic,
                self._odom_callback,
                20,
            )
        )

        # 키보드 drive
        self.drive_subscription = (
            self.create_subscription(
                Float32,
                self.manual_drive_topic,
                self._drive_callback,
                20,
            )
        )

        # =========================================================
        # 시작 로그
        # =========================================================

        self.get_logger().info(
            f"Odom route recording started: "
            f"{self.out_csv}"
        )

        self.get_logger().info(
            f"Odom topic: "
            f"{self.odom_topic}"
        )

        self.get_logger().info(
            f"Manual drive topic: "
            f"{self.manual_drive_topic}"
        )

        self.get_logger().info(
            "drive_level은 /manual_drive의 "
            "실제 키보드 입력값을 저장합니다."
        )

    # =============================================================
    # 파라미터 변경
    # =============================================================

    def _on_parameters(
        self,
        params,
    ):

        for param in params:

            # mode 1~11만 허용
            if param.name == "record_mode":

                try:
                    value = int(
                        param.value
                    )

                except (
                    ValueError,
                    TypeError,
                ):
                    return (
                        SetParametersResult(
                            successful=False,
                            reason=(
                                "record_mode must "
                                "be integer 1~11"
                            ),
                        )
                    )

                if not 1 <= value <= 11:
                    return (
                        SetParametersResult(
                            successful=False,
                            reason=(
                                "record_mode must "
                                "be integer 1~11"
                            ),
                        )
                    )

            # 전진/후진
            if (
                param.name
                == "record_direction"
            ):

                if str(
                    param.value
                ) not in (
                    "forward",
                    "reverse",
                ):
                    return (
                        SetParametersResult(
                            successful=False,
                            reason=(
                                "record_direction "
                                "must be forward "
                                "or reverse"
                            ),
                        )
                    )

        return SetParametersResult(
            successful=True
        )

    # =============================================================
    # 키보드 drive 입력
    # =============================================================

    def _drive_callback(
        self,
        msg: Float32,
    ):

        self.current_drive_level = float(
            msg.data
        )

        # 값이 바뀔 때만 로그
        if (
            self.last_logged_drive
            is None
            or abs(
                self.current_drive_level
                - self.last_logged_drive
            )
            > 1e-6
        ):

            self.get_logger().info(
                "manual_drive 변경: "
                f"{self.current_drive_level:.2f}"
            )

            self.last_logged_drive = (
                self.current_drive_level
            )

    # =============================================================
    # quaternion → yaw
    # =============================================================

    @staticmethod
    def _yaw_from_quaternion(q):

        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = (
            1.0
            - 2.0
            * (
                q.y * q.y
                + q.z * q.z
            )
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    # =============================================================
    # ODOM → CSV 저장
    # =============================================================

    def _odom_callback(
        self,
        msg: Odometry,
    ):

        x = float(
            msg.pose.pose.position.x
        )

        y = float(
            msg.pose.pose.position.y
        )

        # ---------------------------------------------------------
        # 최소 거리 필터
        # ---------------------------------------------------------

        if self.last_x is not None:

            distance = math.hypot(
                x - self.last_x,
                y - self.last_y,
            )

            min_distance = float(
                self.get_parameter(
                    "min_distance_m"
                ).value
            )

            if (
                distance
                < min_distance
            ):
                return

        # ---------------------------------------------------------
        # yaw
        # ---------------------------------------------------------

        yaw_rad = (
            self._yaw_from_quaternion(
                msg.pose.pose.orientation
            )
        )

        yaw_deg = math.degrees(
            yaw_rad
        )

        # ---------------------------------------------------------
        # direction
        # ---------------------------------------------------------

        direction_text = str(
            self.get_parameter(
                "record_direction"
            ).value
        )

        if (
            direction_text
            == "reverse"
        ):
            direction = -1

        else:
            direction = 1

        # ---------------------------------------------------------
        # mode
        # ---------------------------------------------------------

        mode = int(
            self.get_parameter(
                "record_mode"
            ).value
        )

        # ---------------------------------------------------------
        # drive level
        #
        # ★ 기존처럼 파라미터 고정값을 읽지 않음
        # ★ 현재 /manual_drive 실제 입력값 저장
        # ---------------------------------------------------------

        drive_level = float(
            self.current_drive_level
        )

        # ---------------------------------------------------------
        # event
        # ---------------------------------------------------------

        event = str(
            self.get_parameter(
                "record_event"
            ).value
        )

        # ---------------------------------------------------------
        # CSV 저장
        # ---------------------------------------------------------

        self.writer.writerow(
            [
                self.index,
                f"{x:.6f}",
                f"{y:.6f}",
                f"{yaw_deg:.3f}",
                direction,
                mode,
                f"{drive_level:.2f}",
                event,
            ]
        )

        # 실시간 저장
        self.file.flush()

        self.last_x = x
        self.last_y = y

        self.index += 1

    # =============================================================
    # 종료
    # =============================================================

    def destroy_node(self):

        try:
            self.file.flush()
            self.file.close()

        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(
        args=args
    )

    node = (
        OdomRouteRecorder()
    )

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
