"""RViz-only visualization adapter for the production route follower."""
import json
import math
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from .geo_utils import latlon_to_xy
from .route_loader import load_route


class GpsRouteVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('gps_route_visualizer')
        self.declare_parameter('route_path', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('max_traveled_points', 5000)
        self.route = load_route(str(self.get_parameter('route_path').value))
        self.frame = str(self.get_parameter('frame_id').value)
        self.max_points = int(self.get_parameter('max_traveled_points').value)
        self.status = {}
        self.position = None
        self.reference_pub = self.create_publisher(Path, '/gps_nav/reference_path', 1)
        self.traveled_pub = self.create_publisher(Path, '/gps_nav/traveled_path', 1)
        self.direction_pub = self.create_publisher(Marker, '/gps_nav/direction_segments', 1)
        self.vehicle_pub = self.create_publisher(Marker, '/gps_nav/vehicle_marker', 10)
        self.lookahead_pub = self.create_publisher(Marker, '/gps_nav/lookahead_marker', 10)
        self.status_pub = self.create_publisher(Marker, '/gps_nav/status_marker', 10)
        self.traveled = Path(); self.traveled.header.frame_id = self.frame
        self.create_subscription(NavSatFix, '/fix', self._fix, 10)
        self.create_subscription(String, '/gps_navigation/status', self._status, 10)
        self.create_timer(0.2, self._publish)

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _fix(self, msg: NavSatFix) -> None:
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        meta = self.route.metadata
        self.position = latlon_to_xy(msg.latitude, msg.longitude, meta.origin_lat, meta.origin_lon)
        pose = PoseStamped(); pose.header.frame_id = self.frame; pose.header.stamp = self._stamp()
        pose.pose.position.x, pose.pose.position.y = self.position
        pose.pose.orientation.w = 1.0
        if not self.traveled.poses or math.hypot(
                pose.pose.position.x-self.traveled.poses[-1].pose.position.x,
                pose.pose.position.y-self.traveled.poses[-1].pose.position.y) >= 0.03:
            self.traveled.poses.append(pose)
            self.traveled.poses = self.traveled.poses[-self.max_points:]

    def _status(self, msg: String) -> None:
        try: self.status = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError): return

    def _reference_path(self) -> Path:
        path = Path(); path.header.frame_id = self.frame; path.header.stamp = self._stamp()
        for wp in self.route.waypoints:
            pose = PoseStamped(); pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.w = wp.x_m, wp.y_m, 1.0
            path.poses.append(pose)
        if self.route.metadata.loop:
            closing = PoseStamped(); closing.header = path.header
            closing.pose = path.poses[0].pose
            path.poses.append(closing)
        return path

    def _direction_marker(self) -> Marker:
        marker = Marker(); marker.header.frame_id = self.frame; marker.header.stamp = self._stamp()
        marker.ns = 'route_direction'; marker.id = 0; marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD; marker.scale.x = 0.09; marker.pose.orientation.w = 1.0
        from std_msgs.msg import ColorRGBA
        points = list(self.route.waypoints)
        pairs = list(zip(points, points[1:]))
        if self.route.metadata.loop:
            pairs.append((points[-1], points[0]))
        for a, b in pairs:
            marker.points.extend((Point(x=a.x_m, y=a.y_m), Point(x=b.x_m, y=b.y_m)))
            color = ColorRGBA(r=0.1, g=0.9, b=0.2, a=1.0) if a.direction.value > 0 else ColorRGBA(r=1.0, g=0.25, b=0.05, a=1.0)
            marker.colors.extend((color, color))
        return marker

    def _publish(self) -> None:
        self.reference_pub.publish(self._reference_path())
        self.direction_pub.publish(self._direction_marker())
        self.traveled.header.stamp = self._stamp(); self.traveled_pub.publish(self.traveled)
        if self.position is None: return
        heading = math.radians(float(self.status.get('world_heading_deg') or 0.0))
        vehicle = Marker(); vehicle.header.frame_id = self.frame; vehicle.header.stamp = self._stamp()
        vehicle.ns = 'vehicle'; vehicle.id = 0; vehicle.type = Marker.ARROW; vehicle.action = Marker.ADD
        vehicle.pose.position.x, vehicle.pose.position.y = self.position
        vehicle.pose.orientation.z, vehicle.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        vehicle.scale.x, vehicle.scale.y, vehicle.scale.z = 0.75, 0.18, 0.18
        vehicle.color.r, vehicle.color.g, vehicle.color.b, vehicle.color.a = 0.15, 0.45, 1.0, 1.0
        self.vehicle_pub.publish(vehicle)
        target = Marker(); target.header = vehicle.header; target.ns = 'lookahead'; target.id = 0
        target.type = Marker.SPHERE; target.action = Marker.ADD; target.pose.orientation.w = 1.0
        target.pose.position.x = float(self.status.get('target_x_m') or self.position[0])
        target.pose.position.y = float(self.status.get('target_y_m') or self.position[1])
        target.scale.x = target.scale.y = target.scale.z = 0.28
        target.color.r, target.color.g, target.color.b, target.color.a = 1.0, 0.9, 0.0, 1.0
        self.lookahead_pub.publish(target)
        text = Marker(); text.header = vehicle.header; text.ns = 'status'; text.id = 0
        text.type = Marker.TEXT_VIEW_FACING; text.action = Marker.ADD; text.pose.orientation.w = 1.0
        text.pose.position.x, text.pose.position.y, text.pose.position.z = self.position[0], self.position[1], 1.2
        text.scale.z = 0.25; text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        fields = ('state', 'route_index', 'direction', 'mode', 'drive_level', 'gps_drive',
                  'gps_wheel', 'cross_track_error_m', 'imu_yaw_deg', 'world_heading_deg')
        text.text = '\n'.join(f'{name}: {self.status.get(name, "-")}' for name in fields)
        self.status_pub.publish(text)


def main() -> None:
    rclpy.init(); node = GpsRouteVisualizer()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
