"""Record versioned GPS routes with explicit direction, mode and drive level."""
import csv
import math
from datetime import datetime, timezone
from pathlib import Path
import rclpy
import yaml
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from .geo_utils import latlon_to_xy


class RouteRecorder(Node):
    def __init__(self) -> None:
        super().__init__("gps_route_recorder")
        for name, value in (("out_csv", "reference_course.csv"), ("fix_topic", "/fix"),
                            ("min_spacing_m", 0.15),
                            ("record_direction", "forward"), ("record_mode", "NORMAL"),
                            ("record_drive_level", 2.0)):
            self.declare_parameter(name, value)
        self.out = Path(str(self.get_parameter("out_csv").value))
        self.spacing = float(self.get_parameter("min_spacing_m").value)
        self.origin = None
        self.last = None
        self.count = 0
        self.force_record = True
        self.out.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.out.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.stream)
        self.writer.writerow(("index", "latitude", "longitude", "x_m", "y_m",
                              "direction", "mode", "drive_level"))
        self.add_on_set_parameters_callback(self._parameters_changed)
        self.create_subscription(NavSatFix, str(self.get_parameter("fix_topic").value), self._on_fix, 10)
        self.get_logger().info(
            'Recording GPS reference route\n'
            f'CSV: {self.out.resolve()}\nMetadata: {self.out.with_suffix(".yaml").resolve()}\n'
            f'Direction: {str(self.get_parameter("record_direction").value).upper()}\n'
            f'Mode: {self.get_parameter("record_mode").value}\n'
            f'Drive level: {float(self.get_parameter("record_drive_level").value):.2f}')

    def _parameters_changed(self, params):
        for param in params:
            if param.name == "record_direction" and param.value not in ("forward", "reverse"):
                return SetParametersResult(successful=False, reason="direction must be forward/reverse")
            if param.name == "record_drive_level" and float(param.value) not in (1.0, 2.0, 3.0):
                return SetParametersResult(successful=False, reason="drive level must be 1/2/3")
            if param.name in ("record_direction", "record_mode", "record_drive_level"):
                self.force_record = True
                self.get_logger().info(f'Recording boundary requested: {param.name}={param.value}')
        return SetParametersResult(successful=True)

    def _on_fix(self, msg: NavSatFix) -> None:
        if (msg.status.status == NavSatStatus.STATUS_NO_FIX or
                not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude)):
            return
        if self.origin is None:
            self.origin = (msg.latitude, msg.longitude)
            self._write_metadata()
        x, y = latlon_to_xy(msg.latitude, msg.longitude, *self.origin)
        if self.force_record or self.last is None or math.hypot(x-self.last[0], y-self.last[1]) >= self.spacing:
            direction = 1 if self.get_parameter("record_direction").value == "forward" else -1
            mode = str(self.get_parameter("record_mode").value)
            level = float(self.get_parameter("record_drive_level").value)
            self.writer.writerow((self.count, f"{msg.latitude:.10f}", f"{msg.longitude:.10f}",
                                  f"{x:.3f}", f"{y:.3f}", direction, mode, f"{level:.2f}"))
            self.stream.flush()
            self.last, self.count, self.force_record = (x, y), self.count + 1, False

    def _write_metadata(self) -> None:
        metadata = {"format_version": 1, "origin_lat": self.origin[0], "origin_lon": self.origin[1],
                    "loop": False, "created_at": datetime.now(timezone.utc).isoformat()}
        with self.out.with_suffix(".yaml").open("w", encoding="utf-8") as stream:
            yaml.safe_dump(metadata, stream, sort_keys=False)

    def destroy_node(self) -> None:
        self.stream.close(); super().destroy_node()


def main() -> None:
    rclpy.init(); node = RouteRecorder()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
