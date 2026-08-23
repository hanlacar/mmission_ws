"""sim_live — 실시간 주행 시각화 (matplotlib 창)."""
import csv
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

STATE_COLOR = {
    "GPS_FOLLOW": "green", "DEAD_RECKON": "orange", "CAMERA": "blue",
    "MISSION": "red", "INTERSECTION": "black", "IDLE": "gray",
}


class SimLive(Node):
    def __init__(self):
        super().__init__("sim_live")
        self.declare_parameter("route_csv", "")
        self.declare_parameter("intersections", [""])
        self.route = self._load_route(self.get_parameter("route_csv").value)
        self.ix_circles = self._load_ix(self.get_parameter("intersections").value)
        self.traj = []
        self.cur = None
        self.cur_state = "IDLE"
        self.create_subscription(String, "/drive_state", self.on_state, 10)
        self.create_subscription(Point, "/estimated_pose", self.on_pose, 10)
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 7))
        try:
            self.fig.canvas.manager.set_window_title("자율주행 실시간 모니터")
        except Exception:
            pass
        self.get_logger().info("실시간 모니터 창 시작")

    def _load_route(self, path):
        pts = []
        if path:
            try:
                with open(path, newline="", encoding="utf-8-sig") as f:
                    rows = csv.DictReader(f)
                    if rows.fieldnames and {"x_m", "y_m"}.issubset(rows.fieldnames):
                        for row in rows:
                            try:
                                pts.append((float(row["x_m"]), float(row["y_m"])))
                            except (TypeError, ValueError):
                                continue
                    else:
                        f.seek(0)
                        for row in csv.reader(f):
                            if len(row) < 2:
                                continue
                            try:
                                pts.append((float(row[0]), float(row[1])))
                            except (TypeError, ValueError):
                                continue
            except FileNotFoundError:
                self.get_logger().warn(f"route 없음: {path}")
        return pts

    def _load_ix(self, items):
        out = []
        for s in items:
            if not s:
                continue
            try:
                cx, cy, r = [float(v) for v in s.split(",")]
                out.append((cx, cy, r))
            except ValueError:
                pass
        return out

    def on_state(self, msg):
        self.cur_state = msg.data

    def on_pose(self, msg):
        self.cur = (msg.x, msg.y, msg.z)
        self.traj.append((msg.x, msg.y, self.cur_state))

    def redraw(self):
        self.ax.clear()
        if self.route:
            rx = [p[0] for p in self.route]
            ry = [p[1] for p in self.route]
            self.ax.plot(rx, ry, color="lightgray", lw=3, label="route", zorder=1)
        for cx, cy, r in self.ix_circles:
            self.ax.add_patch(plt.Circle((cx, cy), r, color="steelblue",
                                         fill=False, lw=2, zorder=2))
            self.ax.plot(cx, cy, "+", color="steelblue", ms=10)
        for x, y, st in self.traj[-2000:]:
            self.ax.plot(x, y, ".", color=STATE_COLOR.get(st, "black"), ms=3, zorder=3)
        if self.cur is not None:
            x, y, h = self.cur
            c = STATE_COLOR.get(self.cur_state, "black")
            self.ax.plot(x, y, "o", color=c, ms=14, zorder=5, markeredgecolor="black")
            self.ax.arrow(x, y, 1.5 * math.cos(h), 1.5 * math.sin(h),
                          head_width=0.6, color="black", zorder=6)
            self.ax.set_title(f"상태: {self.cur_state}  위치:({x:.1f},{y:.1f})", fontsize=13)
        for st, c in STATE_COLOR.items():
            self.ax.plot([], [], "o", color=c, label=st)
        self.ax.legend(loc="upper right", fontsize=8)
        self.ax.set_aspect("equal")
        self.ax.grid(alpha=0.3)
        self.ax.set_xlabel("x East (m)")
        self.ax.set_ylabel("y North (m)")
        self.fig.canvas.draw_idle()


def main():
    rclpy.init()
    node = SimLive()
    try:
        while rclpy.ok():
            # ROS 콜백을 잠깐 처리(논블로킹)
            rclpy.spin_once(node, timeout_sec=0.0)
            # 그다음 화면 갱신 (메인 스레드에서)
            node.redraw()
            plt.pause(0.05)
            if not plt.fignum_exists(node.fig.number):
                break  # 창을 닫으면 종료
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
