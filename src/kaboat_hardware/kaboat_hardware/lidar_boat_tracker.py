"""고정 TG-50으로 선체 횡단 판을 검출해 수조 기준 선체 pose를 발행한다.

두 봉을 잇는 0.60 m 판의 스캔 점에 직선을 맞춘다. 판의 중점은 선체 원점이고
판 기울기에서 90°를 빼면 선체 yaw가 된다. 판만으로는 앞뒤 180°를 구분할 수
없으므로 Jetson의 indoor_lidar_odom EKF가 초기 방향 보정과 IMU로 분기를 정한다.
출력 `/boat_pose`에는 x, y, yaw와 공분산이 포함된다.
`/boat_position`과 `/detections`는 기존 도구 호환을 위해 함께 발행한다.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import (
    Point, PointStamped, PoseStamped, PoseWithCovarianceStamped, Quaternion,
    TransformStamped,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from kaboat_hardware.lidar_marker_pose import laser_points_to_pool
from kaboat_hardware.lidar_panel_pose import select_panel_line


def yaw_to_quaternion(yaw_rad):
    q = Quaternion()
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


class LidarBoatTracker(Node):
    def __init__(self):
        super().__init__('lidar_boat_tracker')

        # 오른쪽 벽 중앙에서 수조 안쪽(-X)을 바라보는 기본 설치 위치.
        self.declare_parameter('lidar_pos_x', 10.0)
        self.declare_parameter('lidar_pos_y', 2.5)
        self.declare_parameter('lidar_yaw_deg', 180.0)
        self.declare_parameter('pool_size_x', 10.0)
        self.declare_parameter('pool_size_y', 5.0)
        self.declare_parameter('wall_margin', 0.18)

        self.declare_parameter('cluster_dist_tol', 0.15)
        self.declare_parameter('panel_span', 0.60)
        self.declare_parameter('panel_min_visible_span', 0.35)
        self.declare_parameter('panel_max_visible_span', 0.80)
        self.declare_parameter('panel_min_points', 6)
        self.declare_parameter('panel_max_line_rms', 0.03)
        self.declare_parameter('max_position_jump', 0.75)
        self.declare_parameter('max_yaw_jump_deg', 60.0)

        self.declare_parameter('pos_ema_alpha', 0.45)
        self.declare_parameter('vel_ema_alpha', 0.30)
        self.declare_parameter('position_stddev', 0.03)
        self.declare_parameter('panel_point_stddev', 0.02)
        self.declare_parameter('min_yaw_stddev_deg', 1.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'shore_laser_frame')
        self.declare_parameter('publish_odom', False)
        self.declare_parameter('publish_tf', False)

        self.lidar_x = float(self.get_parameter('lidar_pos_x').value)
        self.lidar_y = float(self.get_parameter('lidar_pos_y').value)
        self.lidar_yaw = math.radians(float(
            self.get_parameter('lidar_yaw_deg').value))
        self.pool_size_x = float(self.get_parameter('pool_size_x').value)
        self.pool_size_y = float(self.get_parameter('pool_size_y').value)
        self.wall_margin = float(self.get_parameter('wall_margin').value)
        self.cluster_tol = float(self.get_parameter('cluster_dist_tol').value)
        self.panel_span = float(self.get_parameter('panel_span').value)
        self.panel_min_span = float(
            self.get_parameter('panel_min_visible_span').value)
        self.panel_max_span = float(
            self.get_parameter('panel_max_visible_span').value)
        self.panel_min_points = int(
            self.get_parameter('panel_min_points').value)
        self.panel_max_line_rms = float(
            self.get_parameter('panel_max_line_rms').value)
        if not (0.0 < self.panel_min_span <= self.panel_span
                <= self.panel_max_span):
            raise ValueError('판의 길이와 관측 허용 길이 설정을 확인하세요')
        if self.panel_min_points < 2 or self.panel_max_line_rms <= 0.0:
            raise ValueError('판의 최소 스캔 점 수와 직선 오차 설정을 확인하세요')
        self.max_position_jump = float(
            self.get_parameter('max_position_jump').value)
        self.max_yaw_jump = math.radians(float(
            self.get_parameter('max_yaw_jump_deg').value))

        self.pos_alpha = float(self.get_parameter('pos_ema_alpha').value)
        self.vel_alpha = float(self.get_parameter('vel_ema_alpha').value)
        self.position_stddev = float(
            self.get_parameter('position_stddev').value)
        self.panel_point_stddev = float(
            self.get_parameter('panel_point_stddev').value)
        self.min_yaw_stddev = math.radians(float(
            self.get_parameter('min_yaw_stddev_deg').value))
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.laser_frame = str(self.get_parameter('laser_frame').value)
        self.publish_odom = bool(self.get_parameter('publish_odom').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.filtered_pos = None
        self.last_meas_pos = None
        self.last_time = None
        self.last_raw_pose = None
        self.vx = 0.0
        self.vy = 0.0
        self.target_lost_count = 0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._broadcast_static_tf()

        self.pose_pub = self.create_publisher(PoseStamped, '/detections', 10)
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/boat_pose', 10)
        self.point_pub = self.create_publisher(PointStamped, '/boat_position', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.filtered_scan_pub = self.create_publisher(
            LaserScan, '/lidar_tracker/filtered_scan', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/lidar_tracker/markers', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            '🚀 [lidar_boat_tracker] 횡단 판 2D pose 추적 시작\n'
            f'   - 판 길이: {self.panel_span:.2f}m '
            f'(관측 {self.panel_min_span:.2f}~{self.panel_max_span:.2f}m)\n'
            f'   - 라이다 수조 좌표: ({self.lidar_x:.2f}, {self.lidar_y:.2f})m, '
            f'전방 yaw={math.degrees(self.lidar_yaw):.1f}°\n'
            '   - 발행: /boat_pose (x, y, 180° 모호한 yaw, covariance)')

    def _on_scan(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if ranges.size == 0:
            return
        angles = msg.angle_min + np.arange(
            ranges.size, dtype=np.float32) * msg.angle_increment

        range_min = max(0.08, float(msg.range_min))
        range_max = min(15.0, float(msg.range_max))
        valid_mask = (
            (ranges > range_min) & (ranges < range_max) & np.isfinite(ranges))
        r_valid = ranges[valid_mask]
        a_valid = angles[valid_mask]
        if r_valid.size == 0:
            self._target_lost(msg.header.stamp)
            return

        points_pool = laser_points_to_pool(
            r_valid, a_valid, self.lidar_x, self.lidar_y, self.lidar_yaw)

        roi_mask = (
            (points_pool[:, 0] >= self.wall_margin)
            & (points_pool[:, 0] <= self.pool_size_x - self.wall_margin)
            & (points_pool[:, 1] >= self.wall_margin)
            & (points_pool[:, 1] <= self.pool_size_y - self.wall_margin))
        pts_roi = points_pool[roi_mask]

        filtered_ranges = np.full_like(ranges, np.inf)
        valid_indices = np.where(valid_mask)[0][roi_mask]
        filtered_ranges[valid_indices] = ranges[valid_indices]
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.ranges = filtered_ranges.tolist()
        filtered_msg.intensities = list(msg.intensities)
        self.filtered_scan_pub.publish(filtered_msg)

        previous_pose = self.last_raw_pose if self.target_lost_count <= 3 else None
        panel = select_panel_line(
            clusters=self._euclidean_clustering(pts_roi, self.cluster_tol),
            expected_span=self.panel_span,
            min_points=self.panel_min_points,
            min_span=self.panel_min_span,
            max_span=self.panel_max_span,
            max_line_rms=self.panel_max_line_rms,
            previous_pose=previous_pose,
            max_position_jump=self.max_position_jump,
            max_yaw_jump=self.max_yaw_jump)
        if panel is None:
            self._target_lost(msg.header.stamp)
            return

        raw_position = np.array([panel.x, panel.y])
        now = self.get_clock().now()
        if self.filtered_pos is None or self.target_lost_count > 3:
            self.filtered_pos = raw_position.copy()
            self.last_meas_pos = raw_position.copy()
            self.last_time = now
            self.vx = self.vy = 0.0
        else:
            self.filtered_pos = (
                self.pos_alpha * raw_position
                + (1.0 - self.pos_alpha) * self.filtered_pos)
            dt = (now - self.last_time).nanoseconds * 1e-9
            if dt > 0.001:
                instant_velocity = (self.filtered_pos - self.last_meas_pos) / dt
                self.vx = (
                    self.vel_alpha * float(instant_velocity[0])
                    + (1.0 - self.vel_alpha) * self.vx)
                self.vy = (
                    self.vel_alpha * float(instant_velocity[1])
                    + (1.0 - self.vel_alpha) * self.vy)
            self.last_meas_pos = self.filtered_pos.copy()
            self.last_time = now

        self.target_lost_count = 0
        self.last_raw_pose = (panel.x, panel.y, panel.yaw)
        x = float(self.filtered_pos[0])
        y = float(self.filtered_pos[1])
        yaw_stddev = max(
            self.min_yaw_stddev,
            math.sqrt(2.0) * max(
                self.panel_point_stddev, panel.line_rms) / panel.span)
        position_stddev = max(
            self.position_stddev, 0.5 * (self.panel_span - panel.span))
        self._publish_outputs(
            msg.header.stamp, x, y, panel.yaw, self.vx, self.vy,
            position_stddev, yaw_stddev)
        self._publish_markers(msg.header.stamp, panel, x, y)

    @staticmethod
    def _euclidean_clustering(points, tolerance):
        if len(points) == 0:
            return []
        clusters = []
        current = [points[0]]
        for point in points[1:]:
            if np.linalg.norm(point - current[-1]) <= tolerance:
                current.append(point)
            else:
                clusters.append(np.asarray(current))
                current = [point]
        clusters.append(np.asarray(current))
        return clusters

    def _target_lost(self, stamp):
        self.target_lost_count += 1
        self._publish_markers(stamp, None, None, None)
        if self.target_lost_count > 15:
            self.vx = self.vy = 0.0
            if self.target_lost_count == 16:
                self.get_logger().warn(
                    '⚠️ 횡단 판 선분 미검출 — /boat_pose 발행 중단')

    def _publish_outputs(self, stamp, x, y, yaw, vx, vy,
                         position_stddev, yaw_stddev):
        orientation = yaw_to_quaternion(yaw)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.odom_frame
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.orientation = orientation
        self.pose_pub.publish(pose_msg)

        pose_cov = PoseWithCovarianceStamped()
        pose_cov.header = pose_msg.header
        pose_cov.pose.pose = pose_msg.pose
        pose_cov.pose.covariance[0] = position_stddev ** 2
        pose_cov.pose.covariance[7] = position_stddev ** 2
        pose_cov.pose.covariance[35] = yaw_stddev ** 2
        self.pose_cov_pub.publish(pose_cov)

        point_msg = PointStamped()
        point_msg.header = pose_msg.header
        point_msg.point.x = x
        point_msg.point.y = y
        self.point_pub.publish(point_msg)

        if self.publish_odom:
            odom = Odometry()
            odom.header = pose_msg.header
            odom.child_frame_id = self.base_frame
            odom.pose = pose_cov.pose
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_boat = TransformStamped()
            tf_boat.header = pose_msg.header
            tf_boat.child_frame_id = self.base_frame
            tf_boat.transform.translation.x = x
            tf_boat.transform.translation.y = y
            tf_boat.transform.rotation = orientation
            self.tf_broadcaster.sendTransform(tf_boat)

        self.get_logger().info(
            f'📍 판 중심 pose: ({x:.3f}, {y:.3f})m, '
            f'yaw={math.degrees(yaw):.1f}° (180° 모호), '
            f'σyaw={math.degrees(yaw_stddev):.1f}°',
            throttle_duration_sec=1.0)

    def _broadcast_static_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.laser_frame
        transform.transform.translation.x = self.lidar_x
        transform.transform.translation.y = self.lidar_y
        transform.transform.rotation = yaw_to_quaternion(self.lidar_yaw)
        self.static_tf_broadcaster.sendTransform(transform)

    def _publish_markers(self, stamp, panel, x, y):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        pool = Marker()
        pool.header.stamp = stamp
        pool.header.frame_id = self.odom_frame
        pool.ns = 'pool_bounds'
        pool.id = 0
        pool.type = Marker.LINE_STRIP
        pool.action = Marker.ADD
        pool.scale.x = 0.05
        pool.color.r, pool.color.g, pool.color.b, pool.color.a = 0.2, 0.6, 1.0, 1.0
        pool.points = [
            Point(x=0.0, y=0.0), Point(x=self.pool_size_x, y=0.0),
            Point(x=self.pool_size_x, y=self.pool_size_y),
            Point(x=0.0, y=self.pool_size_y), Point(x=0.0, y=0.0)]
        markers.markers.append(pool)

        if panel is not None:
            line = Marker()
            line.header.stamp = stamp
            line.header.frame_id = self.odom_frame
            line.ns = 'detected_panel'
            line.id = 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.06
            line.color.r, line.color.g, line.color.b, line.color.a = 0.0, 1.0, 0.2, 1.0
            line.points = [
                Point(x=float(panel.start[0]), y=float(panel.start[1])),
                Point(x=float(panel.end[0]), y=float(panel.end[1]))]
            markers.markers.append(line)

            center = Marker()
            center.header = line.header
            center.ns = 'boat_pose_lidar'
            center.id = 2
            center.type = Marker.SPHERE
            center.action = Marker.ADD
            center.pose.position.x = x
            center.pose.position.y = y
            center.pose.position.z = 0.12
            center.scale.x = center.scale.y = center.scale.z = 0.18
            center.color.r, center.color.g, center.color.b, center.color.a = 1.0, 0.6, 0.0, 1.0
            markers.markers.append(center)

            label = Marker()
            label.header = line.header
            label.ns = 'boat_pose_label'
            label.id = 3
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.45
            label.text = (
                f'Panel ({x:.2f}, {y:.2f}), '
                f'axis yaw {math.degrees(panel.yaw):.1f}° ±180°')
            label.scale.z = 0.22
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            markers.markers.append(label)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = LidarBoatTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
