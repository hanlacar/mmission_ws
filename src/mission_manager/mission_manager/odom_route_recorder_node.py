#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from nav_msgs.msg import Odometry


class OdomRouteRecorder(Node):

    def __init__(self):
        super().__init__("odom_route_recorder")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter(
            "out_csv",
            str(Path.home() / "mmission_ws/routes/odom_route.csv"),
        )

        self.declare_parameter("record_direction", "forward")
        self.declare_parameter("record_mode", 1)
        self.declare_parameter("record_drive_level", 2.0)
        self.declare_parameter("record_event", "NONE")

        # 최소 저장 거리
        self.declare_parameter("min_distance_m", 0.05)

        self.add_on_set_parameters_callback(
            self._on_parameters
        )

        self.out_csv = Path(
            self.get_parameter("out_csv")
            .get_parameter_value()
            .string_value
        ).expanduser()

        self.out_csv.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.file = self.out_csv.open(
            "w",
            newline="",
        )

        self.writer = csv.writer(self.file)

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

        self.index = 0
        self.last_x = None
        self.last_y = None

        odom_topic = (
            self.get_parameter("odom_topic")
            .get_parameter_value()
            .string_value
        )

        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            20,
        )

        self.get_logger().info(
            f"Odom route recording started: {self.out_csv}"
        )
        self.get_logger().info(
            f"Odom topic: {odom_topic}"
        )

    def _on_parameters(self, params):
        for param in params:

            if param.name == "record_mode":
                try:
                    value = int(param.value)
                except (ValueError, TypeError):
                    return SetParametersResult(
                        successful=False,
                        reason="record_mode must be integer 1~11",
                    )

                if not 1 <= value <= 11:
                    return SetParametersResult(
                        successful=False,
                        reason="record_mode must be integer 1~11",
                    )

            if param.name == "record_direction":
                if str(param.value) not in (
                    "forward",
                    "reverse",
                ):
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            "record_direction must be "
                            "forward or reverse"
                        ),
                    )

        return SetParametersResult(successful=True)

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp,
        )

    def _odom_callback(self, msg):

        x = float(
            msg.pose.pose.position.x
        )

        y = float(
            msg.pose.pose.position.y
        )

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

            if distance < min_distance:
                return

        yaw_rad = self._yaw_from_quaternion(
            msg.pose.pose.orientation
        )

        yaw_deg = math.degrees(
            yaw_rad
        )

        direction_text = str(
            self.get_parameter(
                "record_direction"
            ).value
        )

        if direction_text == "reverse":
            direction = -1
        else:
            direction = 1

        mode = int(
            self.get_parameter(
                "record_mode"
            ).value
        )

        drive_level = float(
            self.get_parameter(
                "record_drive_level"
            ).value
        )

        event = str(
            self.get_parameter(
                "record_event"
            ).value
        )

        self.writer.writerow(
            [
                self.index,
                f"{x:.6f}",
                f"{y:.6f}",
                f"{yaw_deg:.3f}",
                direction,
                mode,
                drive_level,
                event,
            ]
        )

        self.file.flush()

        self.last_x = x
        self.last_y = y
        self.index += 1

    def destroy_node(self):
        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = OdomRouteRecorder()

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
