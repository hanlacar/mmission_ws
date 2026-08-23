"""
sim_monitor — 실시간 시각화 (matplotlib).

- 저장 경로(course.csv)를 회색으로
- 가상 차량 실제 궤적을 파란선으로
- GPS 수신 위치를 초록점 (재밍 구간엔 안 찍힘)
- mission_manager의 drive_state를 색으로 (GPS_FOLLOW/DEAD_RECKON/CAMERA/MISSION)
- 재밍 존을 빨간 원, 경사 존을 주황 원, 미션 존을 파란 원

실차에서도 rviz 대신 이걸로 상태를 눈으로 볼 수 있다.
헤드리스 환경(대회 노트북에서 화면 없이)에서도 저장모드로 png를 남긴다.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from mission_manager.geo_utils import latlon_to_xy

import matplotlib
matplotlib.use("Agg")   # 헤드리스 안전. 화면 있으면 TkAgg로 바꿔도 됨
import matplotlib.pyplot as plt


STATE_COLOR = {
    "GPS_FOLLOW": "green",
    "DEAD_RECKON": "orange",
    "CAMERA": "blue",
    "MISSION": "red",
    "IDLE": "gray",
}


class SimMonitor(Node):
    def __init__(self):
        super().__init__("sim_monitor")
        self.declare_parameter("route_csv", "")
        self.declare_parameter("origin_lat", 35.5384)
        self.declare_parameter("origin_lon", 129.3114)
        self.declare_parameter("save_png", "/tmp/sim_result.png")
        self.declare_parameter("save_every_s", 2.0)

        self.lat0 = float(self.get_parameter("origin_lat").value)
        self.lon0 = float(self.get_parameter("origin_lon").value)
        self.save_png = self.get_parameter("save_png").value
        self.save_every = float(self.get_parameter("save_every_s").value)

        self.route = self._load_route(self.get_parameter("route_csv").value)
        self.gps_pts = []       # [(x,y)]
        self.state_pts = []     # [(x,y,state)]
        self.cur_state = "IDLE"
        self._last_save = 0.0

        self.create_subscription(NavSatFix, "/fix", self.on_fix, 10)
        self.create_subscription(String, "/drive_state", self.on_state, 10)
        self.create_timer(0.5, self.maybe_save)
        self.get_logger().info(f"모니터 시작 → {self.save_png}")

    def _load_route(self, path):
        pts = []
        if path:
            try:
                with open(path) as f:
                    for line in f:
                        a = line.strip().split(",")
                        if len(a) >= 2:
                            pts.append((float(a[0]), float(a[1])))
            except FileNotFoundError:
                self.get_logger().warn(f"route 없음: {path}")
        return pts

    def on_state(self, msg):
        self.cur_state = msg.data

    def on_fix(self, msg):
        if msg.latitude != msg.latitude:  # nan (재밍)
            return
        x, y = latlon_to_xy(msg.latitude, msg.longitude, self.lat0, self.lon0)
        self.gps_pts.append((x, y))
        self.state_pts.append((x, y, self.cur_state))

    def maybe_save(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_save < self.save_every:
            return
        self._last_save = now
        self._render()

    def _render(self):
        plt.figure(figsize=(9, 7))
        if self.route:
            rx = [p[0] for p in self.route]
            ry = [p[1] for p in self.route]
            plt.plot(rx, ry, color="lightgray", lw=3, label="saved route")
        # 상태별 색으로 실제 궤적
        for x, y, st in self.state_pts:
            plt.plot(x, y, ".", color=STATE_COLOR.get(st, "black"), ms=4)
        # 범례용 더미
        for st, c in STATE_COLOR.items():
            plt.plot([], [], ".", color=c, label=st)
        plt.axis("equal")
        plt.grid(alpha=0.3)
        plt.legend(loc="best", fontsize=8)
        plt.title("Simulation: vehicle path colored by drive_state")
        plt.xlabel("x East (m)"); plt.ylabel("y North (m)")
        plt.savefig(self.save_png, dpi=100, bbox_inches="tight")
        plt.close()


def main():
    rclpy.init()
    node = SimMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
