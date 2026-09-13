#!/usr/bin/env python3
import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def norm(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_q(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_yaw(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion(); q.z = math.sin(yaw / 2.0); q.w = math.cos(yaw / 2.0); return q


class SegmentedVisualizer(Node):
    def __init__(self):
        super().__init__('dr_segmented_visualizer')
        self.declare_parameter('network_path', '')
        self.declare_parameter('start_segment', 'START_A')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('segment_topic', '/dr_navigation/current_segment')
        self.declare_parameter('status_topic', '/dr_navigation/status')
        gp = lambda n: self.get_parameter(n).value
        self.path = Path(str(gp('network_path')).replace('~', str(Path.home())))
        self.start_segment = str(gp('start_segment'))
        self.frame = str(gp('frame_id'))
        if not self.path.is_file():
            raise RuntimeError(f'network CSV not found: {self.path}')
        self.raw = self.load(self.path)
        if self.start_segment not in self.raw:
            raise RuntimeError(f'start segment not found: {self.start_segment}')
        self.segs = {k: [dict(p) for p in v] for k, v in self.raw.items()}
        self.aligned = False
        self.active = self.start_segment
        self.status = 'WAITING'
        self.actual = []
        self.last_odom = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.network_pub = self.create_publisher(MarkerArray, '/dr_viz/network', latched)
        self.active_pub = self.create_publisher(NavPath, '/dr_viz/active_path', latched)
        self.actual_pub = self.create_publisher(NavPath, '/dr_viz/actual_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dr_viz/markers', 10)
        self.create_subscription(Odometry, str(gp('odom_topic')), self.on_odom, 30)
        self.create_subscription(String, str(gp('segment_topic')), self.on_segment, 10)
        self.create_subscription(String, str(gp('status_topic')), self.on_status, 20)
        self.create_timer(0.10, self.publish)

    def load(self, path):
        segs = {}
        with path.open(newline='', encoding='utf-8-sig') as f:
            r = csv.DictReader(f)
            required = {'segment_id', 'point_index', 'x_m', 'y_m'}
            if not required.issubset(set(r.fieldnames or [])):
                raise RuntimeError('segmented CSV columns missing')
            for row in r:
                sid = str(row['segment_id']).strip()
                segs.setdefault(sid, []).append({'i': int(float(row['point_index'])), 'x': float(row['x_m']), 'y': float(row['y_m']), 'yaw': 0.0})
        for sid, pts in segs.items():
            pts.sort(key=lambda p: p['i'])
            for i in range(max(0, len(pts) - 1)):
                dx = pts[i+1]['x'] - pts[i]['x']; dy = pts[i+1]['y'] - pts[i]['y']
                pts[i]['yaw'] = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-9 else (pts[i-1]['yaw'] if i else 0.0)
            if len(pts) > 1: pts[-1]['yaw'] = pts[-2]['yaw']
        return segs

    def align(self, odom):
        self.segs = {k: [dict(p) for p in v] for k, v in self.raw.items()}
        first = self.segs[self.start_segment][0]
        first_yaw = first['yaw']
        oyaw = yaw_from_q(odom.pose.pose.orientation)
        off = norm(oyaw - first_yaw)
        c, s = math.cos(off), math.sin(off)
        x0, y0 = first['x'], first['y']
        ox, oy = float(odom.pose.pose.position.x), float(odom.pose.pose.position.y)
        for pts in self.segs.values():
            for p in pts:
                dx, dy = p['x'] - x0, p['y'] - y0
                p['x'] = ox + c * dx - s * dy
                p['y'] = oy + s * dx + c * dy
                p['yaw'] = norm(p['yaw'] + off)
        self.aligned = True

    def on_odom(self, msg):
        self.last_odom = msg
        if not self.aligned:
            self.align(msg)
        self.actual.append((float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)))
        if len(self.actual) > 20000:
            del self.actual[:-20000]

    def on_segment(self, msg):
        sid = str(msg.data).strip()
        if sid in self.segs:
            self.active = sid

    def on_status(self, msg):
        self.status = str(msg.data)

    def path_msg(self, pts):
        m = NavPath(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = self.frame
        for p in pts:
            ps = PoseStamped(); ps.header = m.header
            if isinstance(p, tuple):
                ps.pose.position.x, ps.pose.position.y = p; ps.pose.orientation.w = 1.0
            else:
                ps.pose.position.x = p['x']; ps.pose.position.y = p['y']; ps.pose.orientation = quat_yaw(p['yaw'])
            m.poses.append(ps)
        return m

    def publish_network(self):
        arr = MarkerArray(); stamp = self.get_clock().now().to_msg()
        for mid, (sid, pts) in enumerate(sorted(self.segs.items())):
            m = Marker(); m.header.frame_id = self.frame; m.header.stamp = stamp
            m.ns = 'network'; m.id = mid; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
            m.pose.orientation.w = 1.0; m.scale.x = 0.05
            if sid == self.active:
                m.color.g = 1.0; m.color.a = 1.0; m.scale.x = 0.10
            else:
                m.color.r = 0.55; m.color.g = 0.55; m.color.b = 0.55; m.color.a = 0.65
            for p in pts:
                q = Point(); q.x = p['x']; q.y = p['y']; q.z = 0.02; m.points.append(q)
            arr.markers.append(m)
        self.network_pub.publish(arr)

    def publish_vehicle(self):
        if self.last_odom is None: return
        arr = MarkerArray(); stamp = self.get_clock().now().to_msg()
        o = self.last_odom
        x = float(o.pose.pose.position.x); y = float(o.pose.pose.position.y)
        a = Marker(); a.header.frame_id = self.frame; a.header.stamp = stamp; a.ns = 'vehicle'; a.id = 0
        a.type = Marker.ARROW; a.action = Marker.ADD; a.pose = o.pose.pose
        a.scale.x = 0.8; a.scale.y = 0.25; a.scale.z = 0.25; a.color.r = a.color.g = a.color.b = a.color.a = 1.0
        arr.markers.append(a)
        t = Marker(); t.header.frame_id = self.frame; t.header.stamp = stamp; t.ns = 'status'; t.id = 1
        t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD; t.pose.position.x = x; t.pose.position.y = y; t.pose.position.z = 0.9
        t.scale.z = 0.3; t.color.r = t.color.g = t.color.b = t.color.a = 1.0; t.text = self.status
        arr.markers.append(t)
        self.marker_pub.publish(arr)

    def publish(self):
        if not self.aligned: return
        self.publish_network()
        if self.active in self.segs:
            self.active_pub.publish(self.path_msg(self.segs[self.active]))
        self.actual_pub.publish(self.path_msg(self.actual))
        self.publish_vehicle()


def main(args=None):
    rclpy.init(args=args)
    node = SegmentedVisualizer()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
