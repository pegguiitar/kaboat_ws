"""고정 TG-50으로 선수/선미 표식을 검출해 수조 기준 선체 pose를 발행한다.

선수의 얇은 봉과 선미의 두꺼운 봉을 클러스터 직경으로 구분하고, 알려진 장착
좌표와 1 m 간격 제약으로 잘못된 쌍을 제거한다. 출력 `/boat_pose`에는 x, y,
yaw와 공분산이 포함되며 젯슨의 indoor_lidar_odom EKF가 이를 IMU와 융합한다.
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

from kaboat_hardware.lidar_marker_pose import MarkerCandidate, select_marker_pair


def yaw_to_quaternion(yaw_rad):
    q = Quaternion()
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


class LidarBoatTracker(Node):
    def __init__(self):
        super().__init__('lidar_boat_tracker')

        self.declare_parameter('lidar_pos_x', 5.0)
        self.declare_parameter('lidar_pos_y', 0.0)
        self.declare_parameter('lidar_yaw_deg', 90.0)
        self.declare_parameter('pool_size_x', 10.0)
        self.declare_parameter('pool_size_y', 5.0)
        self.declare_parameter('wall_margin', 0.18)

        self.declare_parameter('cluster_dist_tol', 0.25)
        self.declare_parameter('min_cluster_pts', 2)
        self.declare_parameter('max_cluster_pts', 80)
        self.declare_parameter('min_target_diameter', 0.02)
        self.declare_parameter('max_target_diameter', 0.45)

        # 실제 봉 제작 후 실측 직경에 맞춰 YAML에서 조정한다.
        self.declare_parameter('bow_min_diameter', 0.02)
        self.declare_parameter('bow_max_diameter', 0.10)
        self.declare_parameter('stern_min_diameter', 0.12)
        self.declare_parameter('stern_max_diameter', 0.30)
        self.declare_parameter('marker_separation_tolerance', 0.20)
        self.declare_parameter('max_position_jump', 0.75)
        self.declare_parameter('max_yaw_jump_deg', 60.0)

        # 기본 표식 간격 1.0 m. 중점이 base_link 원점이다.
        self.declare_parameter('bow_marker_body_x', 0.50)
        self.declare_parameter('bow_marker_body_y', 0.0)
        self.declare_parameter('stern_marker_body_x', -0.50)
        self.declare_parameter('stern_marker_body_y', 0.0)

        self.declare_parameter('pos_ema_alpha', 0.45)
        self.declare_parameter('vel_ema_alpha', 0.30)
        self.declare_parameter('position_stddev', 0.03)
        self.declare_parameter('marker_point_stddev', 0.02)
        self.declare_parameter('min_yaw_stddev_deg', 1.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser_frame')
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
        self.min_pts = int(self.get_parameter('min_cluster_pts').value)
        self.max_pts = int(self.get_parameter('max_cluster_pts').value)
        self.min_diam = float(self.get_parameter('min_target_diameter').value)
        self.max_diam = float(self.get_parameter('max_target_diameter').value)
        self.bow_diameter_range = (
            float(self.get_parameter('bow_min_diameter').value),
            float(self.get_parameter('bow_max_diameter').value))
        self.stern_diameter_range = (
            float(self.get_parameter('stern_min_diameter').value),
            float(self.get_parameter('stern_max_diameter').value))
        self.separation_tolerance = float(
            self.get_parameter('marker_separation_tolerance').value)
        self.max_position_jump = float(
            self.get_parameter('max_position_jump').value)
        self.max_yaw_jump = math.radians(float(
            self.get_parameter('max_yaw_jump_deg').value))
        self.bow_body = np.array([
            float(self.get_parameter('bow_marker_body_x').value),
            float(self.get_parameter('bow_marker_body_y').value)])
        self.stern_body = np.array([
            float(self.get_parameter('stern_marker_body_x').value),
            float(self.get_parameter('stern_marker_body_y').value)])
        self.marker_separation = float(np.linalg.norm(
            self.bow_body - self.stern_body))
        if self.marker_separation <= 0.0:
            raise ValueError('선수/선미 표식 장착 좌표가 같을 수 없습니다')

        self.pos_alpha = float(self.get_parameter('pos_ema_alpha').value)
        self.vel_alpha = float(self.get_parameter('vel_ema_alpha').value)
        self.position_stddev = float(
            self.get_parameter('position_stddev').value)
        self.marker_point_stddev = float(
            self.get_parameter('marker_point_stddev').value)
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
            '🚀 [lidar_boat_tracker] 두 표식 2D pose 추적 시작\n'
            f'   - 얇은 선수 봉 직경: {self.bow_diameter_range[0]:.2f}~'
            f'{self.bow_diameter_range[1]:.2f}m\n'
            f'   - 두꺼운 선미 봉 직경: {self.stern_diameter_range[0]:.2f}~'
            f'{self.stern_diameter_range[1]:.2f}m\n'
            f'   - 설정 표식 간격: {self.marker_separation:.2f}m '
            f'(허용 ±{self.separation_tolerance:.2f}m)\n'
            '   - 발행: /boat_pose (x, y, yaw, covariance)')

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

        x_l = r_valid * np.cos(a_valid)
        y_l = r_valid * np.sin(a_valid)
        c = math.cos(self.lidar_yaw)
        s = math.sin(self.lidar_yaw)
        x_pool = self.lidar_x + c * x_l - s * y_l
        y_pool = self.lidar_y + s * x_l + c * y_l

        roi_mask = (
            (x_pool >= self.wall_margin)
            & (x_pool <= self.pool_size_x - self.wall_margin)
            & (y_pool >= self.wall_margin)
            & (y_pool <= self.pool_size_y - self.wall_margin))
        pts_roi = np.column_stack([x_pool[roi_mask], y_pool[roi_mask]])

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

        candidates = []
        for cluster in self._euclidean_clustering(pts_roi, self.cluster_tol):
            count = len(cluster)
            if count < self.min_pts or count > self.max_pts:
                continue
            diameter = float(np.linalg.norm(
                np.max(cluster, axis=0) - np.min(cluster, axis=0)))
            if not self.min_diam <= diameter <= self.max_diam:
                continue
            candidates.append(MarkerCandidate(
                center=np.median(cluster, axis=0),
                diameter=diameter,
                point_count=count))

        previous_pose = self.last_raw_pose if self.target_lost_count <= 3 else None
        pair = select_marker_pair(
            candidates=candidates,
            bow_diameter_range=self.bow_diameter_range,
            stern_diameter_range=self.stern_diameter_range,
            bow_body=self.bow_body,
            stern_body=self.stern_body,
            separation_tolerance=self.separation_tolerance,
            previous_pose=previous_pose,
            max_position_jump=self.max_position_jump,
            max_yaw_jump=self.max_yaw_jump)
        if pair is None:
            self._target_lost(msg.header.stamp)
            return

        raw_position = np.array([pair.x, pair.y])
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
        self.last_raw_pose = (pair.x, pair.y, pair.yaw)
        x = float(self.filtered_pos[0])
        y = float(self.filtered_pos[1])
        yaw_stddev = max(
            self.min_yaw_stddev,
            math.sqrt(2.0) * self.marker_point_stddev / pair.separation)
        self._publish_outputs(
            msg.header.stamp, x, y, pair.yaw, self.vx, self.vy, yaw_stddev)
        self._publish_markers(msg.header.stamp, pair, x, y)

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
                    '⚠️ 선수/선미 표식 쌍 미검출 — /boat_pose 발행 중단')

    def _publish_outputs(self, stamp, x, y, yaw, vx, vy, yaw_stddev):
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
        pose_cov.pose.covariance[0] = self.position_stddev ** 2
        pose_cov.pose.covariance[7] = self.position_stddev ** 2
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
            f'📍 LiDAR 선체 pose: ({x:.3f}, {y:.3f})m, '
            f'yaw={math.degrees(yaw):.1f}°, σyaw={math.degrees(yaw_stddev):.1f}°',
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

    def _publish_markers(self, stamp, pair, x, y):
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

        if pair is not None:
            for marker_id, name, candidate, color in (
                (1, 'bow_thin', pair.bow, (0.0, 1.0, 1.0)),
                (2, 'stern_thick', pair.stern, (1.0, 0.0, 1.0)),
            ):
                marker = Marker()
                marker.header.stamp = stamp
                marker.header.frame_id = self.odom_frame
                marker.ns = name
                marker.id = marker_id
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose.position.x = float(candidate.center[0])
                marker.pose.position.y = float(candidate.center[1])
                marker.pose.position.z = 0.15
                marker.scale.x = max(candidate.diameter, 0.04)
                marker.scale.y = max(candidate.diameter, 0.04)
                marker.scale.z = 0.30
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 1.0
                markers.markers.append(marker)

            baseline = Marker()
            baseline.header.stamp = stamp
            baseline.header.frame_id = self.odom_frame
            baseline.ns = 'marker_baseline'
            baseline.id = 3
            baseline.type = Marker.ARROW
            baseline.action = Marker.ADD
            baseline.scale.x, baseline.scale.y, baseline.scale.z = 0.04, 0.10, 0.12
            baseline.color.r, baseline.color.g, baseline.color.b, baseline.color.a = 0.0, 1.0, 0.2, 1.0
            baseline.points = [
                Point(x=float(pair.stern.center[0]), y=float(pair.stern.center[1])),
                Point(x=float(pair.bow.center[0]), y=float(pair.bow.center[1]))]
            markers.markers.append(baseline)

            center = Marker()
            center.header = baseline.header
            center.ns = 'boat_pose_lidar'
            center.id = 4
            center.type = Marker.SPHERE
            center.action = Marker.ADD
            center.pose.position.x = x
            center.pose.position.y = y
            center.pose.position.z = 0.12
            center.scale.x = center.scale.y = center.scale.z = 0.18
            center.color.r, center.color.g, center.color.b, center.color.a = 1.0, 0.6, 0.0, 1.0
            markers.markers.append(center)

            label = Marker()
            label.header = baseline.header
            label.ns = 'boat_pose_label'
            label.id = 5
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.45
            label.text = (
                f'Boat ({x:.2f}, {y:.2f}) '
                f'yaw {math.degrees(pair.yaw):.1f}°')
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
