"""고정 라이다로 선체의 ┛ 모양 판을 검출해 수조 기준 선체 pose를 발행한다.

횡단 판(base_link 기준 좌현 0.45 m, 우현 0.30 m)과 우현 끝에서 선미로 뻗은
0.45 m 세로판을 함께 본다. 스캔에서 뽑은 직선으로 pose 후보를 만들고 판 모델
raycast가 실제 빔과 가장 잘 맞는 후보를 고른다(`lidar_sail_pose`). 두 판이
보이거나 반대 해석이 빔으로 기각되면 yaw가 360° 범위에서 정해지고, 모호하면
이전 pose에 가까운 쪽을 택한다. Jetson의 indoor_lidar_odom은 기존처럼 IMU
예측값에 가까운 분기를 고르므로 그대로 동작한다.
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
from kaboat_hardware.lidar_sail_pose import SailGeometry, estimate_sail_pose
from kaboat_hardware.pose_velocity import normalize_angle


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
        # 먼 곳일수록 빔 간격이 벌어지므로 라이다 거리 1 m 당 허용 간격을 늘린다.
        self.declare_parameter('cluster_dist_per_m', 0.04)
        self.declare_parameter('panel_left_length', 0.45)
        self.declare_parameter('panel_right_length', 0.30)
        self.declare_parameter('sail_length', 0.45)
        self.declare_parameter('panel_min_points', 3)
        self.declare_parameter('panel_max_line_rms', 0.03)
        self.declare_parameter('max_mean_cost', 3.0)
        # 앞뒤가 모호하고 이전 pose 도 없을 때 가정할 선수 방향 (라이다 전방 -X)
        self.declare_parameter('default_yaw_deg', 180.0)
        # 놓쳤다가 다시 잡을 때 이 시간 안이면 마지막 yaw 쪽으로 앞뒤를 고른다.
        self.declare_parameter('yaw_memory_sec', 10.0)
        self.declare_parameter('max_position_jump', 0.75)
        # 확실한 L자(두 판 모두 빔 l_min_arm_hits 개 이상)가 아니면 기존 yaw 에서
        # 이 범위 안의 해석만 허용한다 → 90°/180° 튐 방지.
        self.declare_parameter('yaw_follow_window_deg', 50.0)
        self.declare_parameter('l_min_arm_hits', 2)

        self.declare_parameter('pos_ema_alpha', 0.45)
        # 출력 yaw 도 직전 값과 섞는다. 확실한 L자로 크게 재정렬될 때는 바로 반영.
        self.declare_parameter('yaw_ema_alpha', 0.5)
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
        self.cluster_tol_per_m = float(
            self.get_parameter('cluster_dist_per_m').value)
        self.geom = SailGeometry(
            left=float(self.get_parameter('panel_left_length').value),
            right=float(self.get_parameter('panel_right_length').value),
            sail=float(self.get_parameter('sail_length').value))
        if min(self.geom.left, self.geom.right, self.geom.sail) <= 0.0:
            raise ValueError('판 치수 설정을 확인하세요')
        self.panel_min_points = int(
            self.get_parameter('panel_min_points').value)
        self.panel_max_line_rms = float(
            self.get_parameter('panel_max_line_rms').value)
        self.max_mean_cost = float(self.get_parameter('max_mean_cost').value)
        self.default_yaw = math.radians(float(
            self.get_parameter('default_yaw_deg').value))
        self.yaw_memory = float(self.get_parameter('yaw_memory_sec').value)
        if self.panel_min_points < 2 or self.panel_max_line_rms <= 0.0:
            raise ValueError('판의 최소 스캔 점 수와 직선 오차 설정을 확인하세요')
        self.max_position_jump = float(
            self.get_parameter('max_position_jump').value)
        self.yaw_follow_window = math.radians(float(
            self.get_parameter('yaw_follow_window_deg').value))
        self.l_min_arm_hits = int(self.get_parameter('l_min_arm_hits').value)

        self.pos_alpha = float(self.get_parameter('pos_ema_alpha').value)
        self.yaw_alpha = float(self.get_parameter('yaw_ema_alpha').value)
        self.filtered_yaw = None
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
        self.last_seen_time = None
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
            '🚀 [lidar_boat_tracker] ┛ 판 2D pose 추적 시작\n'
            f'   - 횡단 판: 좌현 {self.geom.left:.2f}m / 우현 {self.geom.right:.2f}m, '
            f'세로판(우현→선미) {self.geom.sail:.2f}m\n'
            f'   - 판 점: {self.panel_min_points}개 이상, '
            f'직선 RMS 오차: {self.panel_max_line_rms:.3f}m 이하\n'
            f'   - 모호할 때 기본 선수 방향: {math.degrees(self.default_yaw):.0f}°\n'
            f'   - 라이다 수조 좌표: ({self.lidar_x:.2f}, {self.lidar_y:.2f})m, '
            f'전방 yaw={math.degrees(self.lidar_yaw):.1f}°\n'
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
        reference_yaw = None
        if self.last_raw_pose is not None and self.last_seen_time is not None:
            lost_sec = (self.get_clock().now()
                        - self.last_seen_time).nanoseconds * 1e-9
            if lost_sec <= self.yaw_memory:
                reference_yaw = self.last_raw_pose[2]
        # raycast 비교에는 ROI 밖 빔도 쓴다: 판 뒤 벽이 보이면 그 자리에 판이 없다.
        panel = estimate_sail_pose(
            clusters=self._euclidean_clustering(
                pts_roi, self.cluster_tol, self.cluster_tol_per_m,
                (self.lidar_x, self.lidar_y)),
            origin=(self.lidar_x, self.lidar_y),
            angles=angles.astype(float) + self.lidar_yaw,
            ranges=np.where(valid_mask, ranges, 0.0).astype(float),
            valid=valid_mask,
            geom=self.geom,
            min_points=self.panel_min_points,
            line_rms_tol=self.panel_max_line_rms,
            max_mean_cost=self.max_mean_cost,
            previous_pose=previous_pose,
            max_position_jump=self.max_position_jump,
            reference_yaw=reference_yaw,
            yaw_follow_window=self.yaw_follow_window,
            min_arm_hits=self.l_min_arm_hits,
            default_yaw=self.default_yaw)
        if panel is None:
            self._target_lost(msg.header.stamp)
            return

        raw_position = np.array([panel.x, panel.y])
        now = self.get_clock().now()
        jump = (self.filtered_yaw is not None and abs(normalize_angle(
            panel.yaw - self.filtered_yaw)) > math.radians(45))
        if (self.filtered_yaw is None or self.target_lost_count > 3
                or (jump and panel.yaw_unique)):
            self.filtered_yaw = panel.yaw
        else:
            self.filtered_yaw = normalize_angle(
                self.filtered_yaw + self.yaw_alpha
                * normalize_angle(panel.yaw - self.filtered_yaw))
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
        self.last_seen_time = now
        x = float(self.filtered_pos[0])
        y = float(self.filtered_pos[1])
        # 확실한 L자는 모서리로 위치까지 정확하고, 한 직선 추종은 조금 키운다.
        # yaw 자체는 직선 기울기로 정밀하므로 앞뒤 모호성으로 부풀리지 않는다.
        scale = 1.0 if panel.yaw_unique else 1.5
        yaw_stddev = scale * max(
            self.min_yaw_stddev,
            math.sqrt(2.0) * self.panel_point_stddev
            / (self.geom.left + self.geom.right))
        position_stddev = scale * self.position_stddev
        self._publish_outputs(
            msg.header.stamp, x, y, self.filtered_yaw, self.vx, self.vy,
            position_stddev, yaw_stddev, panel.yaw_unique)
        self._publish_markers(msg.header.stamp, panel, x, y, self.filtered_yaw)

    @staticmethod
    def _euclidean_clustering(points, tolerance, tolerance_per_m=0.0,
                              origin=(0.0, 0.0)):
        if len(points) == 0:
            return []
        clusters = []
        current = [points[0]]
        for point in points[1:]:
            reach = math.hypot(point[0] - origin[0], point[1] - origin[1])
            if (np.linalg.norm(point - current[-1])
                    <= tolerance + tolerance_per_m * reach):
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
                    '⚠️ ┛ 판 미검출 — /boat_pose 발행 중단')

    def _publish_outputs(self, stamp, x, y, yaw, vx, vy,
                         position_stddev, yaw_stddev, yaw_unique=True):
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
            f'📍 선체 pose: ({x:.3f}, {y:.3f})m, '
            f'yaw={math.degrees(yaw):.1f}°'
            f'{" [L자 확인]" if yaw_unique else " (기존 yaw 추종)"}, '
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

    def _publish_markers(self, stamp, panel, x, y, yaw=None):
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
            # 추정 pose 에 놓은 판 모델 (실제 스캔 점과 겹쳐 보이면 정상)
            line = Marker()
            line.header.stamp = stamp
            line.header.frame_id = self.odom_frame
            line.ns = 'detected_panel'
            line.id = 1
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.scale.x = 0.05
            line.color.r, line.color.g, line.color.b, line.color.a = 0.0, 1.0, 0.2, 1.0
            for a, b in panel.segments:
                line.points.append(Point(x=float(a[0]), y=float(a[1])))
                line.points.append(Point(x=float(b[0]), y=float(b[1])))
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
            center.scale.x = center.scale.y = center.scale.z = 0.14
            center.color.r, center.color.g, center.color.b, center.color.a = 1.0, 0.6, 0.0, 1.0
            markers.markers.append(center)

            heading = Marker()
            heading.header = line.header
            heading.ns = 'boat_heading'
            heading.id = 4
            heading.type = Marker.ARROW
            heading.action = Marker.ADD
            heading.pose.position.x = x
            heading.pose.position.y = y
            heading.pose.position.z = 0.12
            heading.pose.orientation = yaw_to_quaternion(yaw)
            heading.scale.x, heading.scale.y, heading.scale.z = 0.6, 0.06, 0.06
            if panel.yaw_unique:
                heading.color.r, heading.color.g, heading.color.b = 1.0, 0.6, 0.0
            else:
                heading.color.r, heading.color.g, heading.color.b = 0.7, 0.7, 0.7
            heading.color.a = 1.0
            markers.markers.append(heading)

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
                f'({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.0f}°'
                f'{"" if panel.yaw_unique else " ?"}')
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
