#!/usr/bin/env python3

import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32, String


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class DrRouteFollower(Node):

    def __init__(self):
        super().__init__('dr_route_follower')

        # =========================================================
        # 파라미터
        # =========================================================
        self.declare_parameter('route_path', '')
        self.declare_parameter('odom_topic', '/odom')

        self.declare_parameter('drive_topic', '/gps_drive')
        self.declare_parameter('wheel_topic', '/gps_wheel')
        self.declare_parameter('status_topic', '/dr_navigation/status')

        # T870 실차
        self.declare_parameter('wheelbase_m', 0.73)
        self.declare_parameter('max_steer_deg', 27.0)

        # 조향 부호가 실차와 반대면 -1
        self.declare_parameter('steering_sign', 1)

        # Pure Pursuit
        self.declare_parameter('lookahead_forward_m', 0.80)
        self.declare_parameter('lookahead_reverse_m', 0.60)

        # waypoint 진행 판정
        self.declare_parameter('advance_distance_m', 0.35)

        # 최종 도달
        self.declare_parameter('goal_tolerance_m', 0.30)

        # 경로 이탈
        self.declare_parameter('off_route_warn_m', 1.0)
        self.declare_parameter('off_route_stop_m', 2.0)

        # odom 두절
        self.declare_parameter('odom_timeout_s', 0.5)

        # 시작 시 저장 경로 첫 점을 현재 odom 위치에 맞춤
        self.declare_parameter('align_route_to_start', True)

        # follower 시작 즉시 움직이지 않게 함
        self.declare_parameter('auto_start', False)

        # =========================================================
        # 파라미터 읽기
        # =========================================================
        self.route_path = os.path.expanduser(
            str(self.get_parameter('route_path').value)
        )

        if not self.route_path:
            raise RuntimeError('route_path가 비어 있습니다.')

        self.odom_topic = str(
            self.get_parameter('odom_topic').value
        )

        self.drive_topic = str(
            self.get_parameter('drive_topic').value
        )

        self.wheel_topic = str(
            self.get_parameter('wheel_topic').value
        )

        self.status_topic = str(
            self.get_parameter('status_topic').value
        )

        self.wheelbase = float(
            self.get_parameter('wheelbase_m').value
        )

        self.max_steer_deg = float(
            self.get_parameter('max_steer_deg').value
        )

        self.steering_sign = int(
            self.get_parameter('steering_sign').value
        )

        self.lookahead_forward = float(
            self.get_parameter('lookahead_forward_m').value
        )

        self.lookahead_reverse = float(
            self.get_parameter('lookahead_reverse_m').value
        )

        self.advance_distance = float(
            self.get_parameter('advance_distance_m').value
        )

        self.goal_tolerance = float(
            self.get_parameter('goal_tolerance_m').value
        )

        self.off_route_warn = float(
            self.get_parameter('off_route_warn_m').value
        )

        self.off_route_stop = float(
            self.get_parameter('off_route_stop_m').value
        )

        self.odom_timeout = float(
            self.get_parameter('odom_timeout_s').value
        )

        self.align_route_to_start = bool(
            self.get_parameter('align_route_to_start').value
        )

        self.started = bool(
            self.get_parameter('auto_start').value
        )

        # =========================================================
        # 경로 읽기
        # =========================================================
        self.path = self.load_route(self.route_path)

        if len(self.path) < 2:
            raise RuntimeError(
                f'경로 점이 부족합니다: {len(self.path)}'
            )

        self.get_logger().info(
            f'DR 경로 로드 완료: {self.route_path}'
        )

        self.get_logger().info(
            f'waypoints={len(self.path)}'
        )

        # =========================================================
        # 상태
        # =========================================================
        self.cursor = 0
        self.finished = False

        self.last_odom_time = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.route_aligned = False

        # =========================================================
        # ROS
        # =========================================================
        self.drive_pub = self.create_publisher(
            Float32,
            self.drive_topic,
            10,
        )

        self.wheel_pub = self.create_publisher(
            Int32,
            self.wheel_topic,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            10,
        )

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            20,
        )

        # 서비스
        from std_srvs.srv import Trigger

        self.start_srv = self.create_service(
            Trigger,
            '/dr_route/start',
            self.start_callback,
        )

        self.stop_srv = self.create_service(
            Trigger,
            '/dr_route/stop',
            self.stop_callback,
        )

        self.reset_srv = self.create_service(
            Trigger,
            '/dr_route/reset',
            self.reset_callback,
        )

        # 안전 감시
        self.create_timer(
            0.1,
            self.watchdog,
        )

        self.publish_stop()

        self.get_logger().info(
            'DR follower READY'
        )

        self.get_logger().info(
            '시작 명령: '
            'ros2 service call /dr_route/start '
            'std_srvs/srv/Trigger "{}"'
        )

    # =============================================================
    # CSV
    # =============================================================
    def load_route(self, path):

        points = []

        with open(
            path,
            'r',
            newline='',
            encoding='utf-8',
        ) as f:

            reader = csv.DictReader(f)

            required = {
                'dr_x_m',
                'dr_y_m',
                'direction',
            }

            if not required.issubset(
                set(reader.fieldnames or [])
            ):
                raise RuntimeError(
                    'CSV 형식 오류. 필요한 컬럼: '
                    'x_m,y_m,direction'
                )

            for row in reader:

                x = float(row['dr_x_m'])
                y = float(row['dr_y_m'])

                yaw_deg = float(
                    row.get('dr_yaw_deg', 0.0)
                    or 0.0
                )

                direction = int(
                    float(
                        row.get(
                            'direction',
                            1,
                        )
                    )
                )

                mode = int(
                    float(
                        row.get(
                            'mode',
                            1,
                        )
                    )
                )

                drive_level = float(
                    row.get(
                        'drive_level',
                        2.0,
                    )
                    or 2.0
                )

                points.append({
                    'x': x,
                    'y': y,
                    'yaw': math.radians(yaw_deg),
                    'direction': 1 if direction >= 0 else -1,
                    'mode': mode,
                    'drive_level': drive_level,
                })

        return points

    # =============================================================
    # 시작 위치에 경로 정렬
    # =============================================================
    def align_route(self):

        if self.route_aligned:
            return

        if not self.align_route_to_start:
            self.route_aligned = True
            return

        first = self.path[0]

        route_yaw0 = first['yaw']

        yaw_offset = normalize_angle(
            self.current_yaw - route_yaw0
        )

        cos_a = math.cos(yaw_offset)
        sin_a = math.sin(yaw_offset)

        x0 = first['x']
        y0 = first['y']

        for p in self.path:

            dx = p['x'] - x0
            dy = p['y'] - y0

            rotated_x = (
                cos_a * dx
                - sin_a * dy
            )

            rotated_y = (
                sin_a * dx
                + cos_a * dy
            )

            p['x'] = (
                self.current_x
                + rotated_x
            )

            p['y'] = (
                self.current_y
                + rotated_y
            )

            p['yaw'] = normalize_angle(
                p['yaw']
                + yaw_offset
            )

        self.route_aligned = True

        self.get_logger().info(
            '저장 경로 첫 점을 현재 DR 위치/방향에 정렬했습니다.'
        )

    # =============================================================
    # 서비스
    # =============================================================
    def start_callback(
        self,
        request,
        response,
    ):

        if self.finished:
            response.success = False
            response.message = (
                '이미 경로가 완료되었습니다. '
                '/dr_route/reset 후 다시 시작하십시오.'
            )
            return response

        if self.last_odom_time is None:
            response.success = False
            response.message = '/odom 수신 없음'
            return response

        self.align_route()

        self.started = True

        response.success = True
        response.message = 'DR 경로 추종 시작'

        self.get_logger().info(
            'DR 경로 추종 START'
        )

        return response

    def stop_callback(
        self,
        request,
        response,
    ):

        self.started = False
        self.publish_stop()

        response.success = True
        response.message = 'DR 경로 추종 정지'

        self.get_logger().warn(
            'DR 경로 추종 STOP'
        )

        return response

    def reset_callback(
        self,
        request,
        response,
    ):

        self.started = False
        self.finished = False
        self.cursor = 0
        self.route_aligned = False

        # 파일 다시 읽어서 정렬 전 원본 복원
        self.path = self.load_route(
            self.route_path
        )

        self.publish_stop()

        response.success = True
        response.message = (
            'DR follower reset 완료'
        )

        self.get_logger().info(
            'DR follower RESET'
        )

        return response

    # =============================================================
    # 출력
    # =============================================================
    def publish_drive(
        self,
        drive,
        wheel,
    ):

        wheel = int(
            max(
                -self.max_steer_deg,
                min(
                    self.max_steer_deg,
                    wheel,
                ),
            )
        )

        wheel *= self.steering_sign

        self.drive_pub.publish(
            Float32(
                data=float(drive)
            )
        )

        self.wheel_pub.publish(
            Int32(
                data=int(wheel)
            )
        )

    def publish_stop(self):

        self.drive_pub.publish(
            Float32(data=0.0)
        )

        self.wheel_pub.publish(
            Int32(data=0)
        )

    def status(self, text):

        self.status_pub.publish(
            String(data=str(text))
        )

    # =============================================================
    # ODOM
    # =============================================================
    def odom_callback(self, msg):

        self.last_odom_time = time.monotonic()

        self.current_x = float(
            msg.pose.pose.position.x
        )

        self.current_y = float(
            msg.pose.pose.position.y
        )

        self.current_yaw = yaw_from_quaternion(
            msg.pose.pose.orientation
        )

        if (
            not self.started
            or self.finished
        ):
            return

        if not self.route_aligned:
            self.align_route()

        self.follow()

    # =============================================================
    # Pure Pursuit
    # =============================================================
    def follow(self):

        x = self.current_x
        y = self.current_y
        yaw = self.current_yaw

        # ---------------------------------------------------------
        # 가장 가까운 앞쪽 waypoint 찾기
        # ---------------------------------------------------------
        search_end = min(
            len(self.path),
            self.cursor + 80,
        )

        nearest_idx = self.cursor
        nearest_dist = float('inf')

        for i in range(
            self.cursor,
            search_end,
        ):

            p = self.path[i]

            d = math.hypot(
                p['x'] - x,
                p['y'] - y,
            )

            if d < nearest_dist:
                nearest_dist = d
                nearest_idx = i

        if nearest_idx > self.cursor:
            self.cursor = nearest_idx

        # ---------------------------------------------------------
        # 경로 이탈
        # ---------------------------------------------------------
        if nearest_dist > self.off_route_stop:

            self.publish_stop()

            self.status(
                f'OFF_ROUTE_STOP distance={nearest_dist:.2f}'
            )

            self.get_logger().error(
                f'경로 이탈 {nearest_dist:.2f}m '
                '→ 정지'
            )

            return

        if nearest_dist > self.off_route_warn:
            self.status(
                f'OFF_ROUTE_WARN distance={nearest_dist:.2f}'
            )

        # ---------------------------------------------------------
        # Goal
        # ---------------------------------------------------------
        goal = self.path[-1]

        goal_distance = math.hypot(
            goal['x'] - x,
            goal['y'] - y,
        )

        if (
            self.cursor >= len(self.path) - 2
            and goal_distance
            <= self.goal_tolerance
        ):

            self.finished = True
            self.started = False

            self.publish_stop()

            self.status('GOAL_REACHED')

            self.get_logger().info(
                'DR 경로 최종 목표 도달 → 정지'
            )

            return

        # ---------------------------------------------------------
        # 현재 진행 방향
        # ---------------------------------------------------------
        current = self.path[
            self.cursor
        ]

        direction = current[
            'direction'
        ]

        if direction >= 0:
            lookahead = (
                self.lookahead_forward
            )
        else:
            lookahead = (
                self.lookahead_reverse
            )

        # ---------------------------------------------------------
        # lookahead target
        # ---------------------------------------------------------
        target_idx = self.cursor

        for i in range(
            self.cursor,
            len(self.path),
        ):

            # direction 바뀌는 지점을 넘어가지 않음
            if (
                self.path[i]['direction']
                != direction
            ):
                break

            d = math.hypot(
                self.path[i]['x'] - x,
                self.path[i]['y'] - y,
            )

            target_idx = i

            if d >= lookahead:
                break

        target = self.path[
            target_idx
        ]

        dx = target['x'] - x
        dy = target['y'] - y

        # 차량 좌표계
        local_x = (
            math.cos(yaw) * dx
            + math.sin(yaw) * dy
        )

        local_y = (
            -math.sin(yaw) * dx
            + math.cos(yaw) * dy
        )

        # ---------------------------------------------------------
        # 후진이면 차량 진행축을 반대로 본다
        # ---------------------------------------------------------
        if direction < 0:

            local_x = -local_x
            local_y = -local_y

        ld = math.hypot(
            local_x,
            local_y,
        )

        if ld < 0.05:
            self.publish_stop()
            return

        curvature = (
            2.0
            * local_y
            / (ld * ld)
        )

        # 후진에서는 차량 yaw 변화와 진행방향 관계가 반대
        if direction < 0:
            curvature = -curvature

        steer_rad = math.atan(
            self.wheelbase
            * curvature
        )

        steer_deg = math.degrees(
            steer_rad
        )

        steer_deg = max(
            -self.max_steer_deg,
            min(
                self.max_steer_deg,
                steer_deg,
            ),
        )

        # ---------------------------------------------------------
        # drive
        # ---------------------------------------------------------
        if direction >= 0:

            drive_level = float(
                current['drive_level']
            )

            # 허용값만 사용
            if drive_level < 1.0:
                drive_level = 1.0

            if drive_level > 3.0:
                drive_level = 3.0

            drive_cmd = drive_level

        else:
            # 팀 GPS 규약
            drive_cmd = -1.0

        # 큰 조향이면 감속
        if (
            direction >= 0
            and abs(steer_deg) >= 18.0
        ):
            drive_cmd = 1.0

        self.publish_drive(
            drive_cmd,
            round(steer_deg),
        )

        self.status(
            'TRACKING '
            f'idx={self.cursor}/{len(self.path)-1} '
            f'dist={nearest_dist:.2f} '
            f'drive={drive_cmd:.1f} '
            f'wheel={steer_deg:.1f}'
        )

    # =============================================================
    # watchdog
    # =============================================================
    def watchdog(self):

        if not self.started:
            return

        if self.last_odom_time is None:
            self.publish_stop()
            return

        age = (
            time.monotonic()
            - self.last_odom_time
        )

        if age > self.odom_timeout:

            self.publish_stop()

            self.status(
                f'ODOM_TIMEOUT age={age:.2f}'
            )

            self.get_logger().error(
                '/odom 수신 두절 → 정지'
            )


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = DrRouteFollower()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        print(
            f'dr_route_follower 오류: {exc}'
        )

    finally:

        if node is not None:
            node.publish_stop()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
