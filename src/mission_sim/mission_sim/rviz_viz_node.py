"""rviz_viz — RViz2용 시각화 발행 노드."""
import csv
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Vector3, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

STATE_RGBA = {
    "GPS_FOLLOW": (0.1, 0.8, 0.1, 1.0),
    "DEAD_RECKON": (1.0, 0.6, 0.0, 1.0),
    "CAMERA": (0.1, 0.3, 0.9, 1.0),
    "MISSION": (0.9, 0.1, 0.1, 1.0),
    "INTERSECTION": (0.1, 0.1, 0.1, 1.0),
    "IDLE": (0.5, 0.5, 0.5, 1.0),
}


def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


class RvizViz(Node):
    def __init__(self):
        super().__init__("rviz_viz")
        self.declare_parameter("route_csv", "")
        self.declare_parameter("intersections", [""])
        self.frame = "map"
        self.cur = None
        self.cur_state = "IDLE"
        self.trail = []
        self.create_subscription(Point, "/estimated_pose", self.on_pose, 10)
        self.create_subscription(String, "/drive_state", self.on_state, 10)
        self.veh_pub = self.create_publisher(Marker, "/viz/vehicle", 10)
        self.trail_pub = self.create_publisher(Marker, "/viz/trail", 10)
        self.route_pub = self.create_publisher(Path, "/viz/route", 1)
        self.ix_pub = self.create_publisher(MarkerArray, "/viz/intersections", 1)
        self.tf = TransformBroadcaster(self)
        self.route_msg = self._build_route(self.get_parameter("route_csv").value)
        self.ix_msg = self._build_ix(self.get_parameter("intersections").value)
        self.create_timer(1.0, self.publish_static)
        self.create_timer(0.05, self.publish_dynamic)
        self.get_logger().info("RViz 시각화 노드 시작 (Fixed Frame: map)")

    def _build_route(self, path):
        msg = Path()
        msg.header.frame_id = self.frame
        if path:
            try:
                with open(path, newline="", encoding="utf-8-sig") as f:
                    rows = csv.DictReader(f)
                    if rows.fieldnames and {"x_m", "y_m"}.issubset(rows.fieldnames):
                        coordinates = ((row.get("x_m"), row.get("y_m")) for row in rows)
                    else:
                        f.seek(0)
                        coordinates = ((row[0], row[1]) for row in csv.reader(f)
                                       if len(row) >= 2)
                    for x_value, y_value in coordinates:
                        try:
                            ps = PoseStamped()
                            ps.header.frame_id = self.frame
                            ps.pose.position.x = float(x_value)
                            ps.pose.position.y = float(y_value)
                            ps.pose.orientation.w = 1.0
                            msg.poses.append(ps)
                        except (TypeError, ValueError):
                            continue
            except FileNotFoundError:
                self.get_logger().warn(f"route 없음: {path}")
        return msg

    def _build_ix(self, items):
        arr = MarkerArray()
        i = 0
        for s in items:
            if not s:
                continue
            try:
                cx, cy, r = [float(v) for v in s.split(",")]
            except ValueError:
                continue
            m = Marker()
            m.header.frame_id = self.frame
            m.ns = "intersections"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.orientation.w = 1.0
            m.scale = Vector3(x=2 * r, y=2 * r, z=0.05)
            m.color = ColorRGBA(r=0.2, g=0.5, b=0.9, a=0.35)
            arr.markers.append(m)
            i += 1
        return arr

    def on_pose(self, msg):
        self.cur = (msg.x, msg.y, msg.z)
        self.trail.append((msg.x, msg.y))
        if len(self.trail) > 5000:
            self.trail = self.trail[-5000:]

    def on_state(self, msg):
        self.cur_state = msg.data

    def publish_static(self):
        now = self.get_clock().now().to_msg()
        self.route_msg.header.stamp = now
        for ps in self.route_msg.poses:
            ps.header.stamp = now
        self.route_pub.publish(self.route_msg)
        for m in self.ix_msg.markers:
            m.header.stamp = now
        self.ix_pub.publish(self.ix_msg)

    def publish_dynamic(self):
        if self.cur is None:
            return
        x, y, yaw = self.cur
        now = self.get_clock().now().to_msg()
        rgba = STATE_RGBA.get(self.cur_state, (0, 0, 0, 1))
        tfs = TransformStamped()
        tfs.header.stamp = now
        tfs.header.frame_id = self.frame
        tfs.child_frame_id = "base_link"
        tfs.transform.translation.x = x
        tfs.transform.translation.y = y
        tfs.transform.rotation = yaw_to_quat(yaw)
        self.tf.sendTransform(tfs)
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = now
        m.ns = "vehicle"
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.1
        m.pose.orientation = yaw_to_quat(yaw)
        m.scale = Vector3(x=1.5, y=0.4, z=0.4)
        m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
        self.veh_pub.publish(m)
        t = Marker()
        t.header.frame_id = self.frame
        t.header.stamp = now
        t.ns = "trail"
        t.id = 0
        t.type = Marker.LINE_STRIP
        t.action = Marker.ADD
        t.scale.x = 0.15
        t.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=0.8)
        t.pose.orientation.w = 1.0
        for tx, ty in self.trail[-2000:]:
            t.points.append(Point(x=tx, y=ty, z=0.05))
        self.trail_pub.publish(t)


def main():
    rclpy.init()
    node = RvizViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
