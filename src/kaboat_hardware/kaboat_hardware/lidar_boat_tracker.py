"""lidar_boat_tracker — 실내 수조 고정 2D 라이다 기반 순수 배 (X, Y) 좌표 추적기.

배치 조건:
  - 수조 좌하단 원점 (X=0m, Y=0m) 기준, (X=5.0m, Y=0.0m) 지점에 2D 라이다 설치
  - 수조 크기: 가로 10.0m x 세로 5.0m
  - 라이다는 전방(+Y 수조 내부)을 향해 스캔

출력 정보:
  - 배의 수조 절대 위치 (X, Y) [m] (중앙값 클러스터링 + EMA 필터 적용)
  - /detections (geometry_msgs/msg/PoseStamped on 'odom') -> X, Y 위치 순수 전달
  - /boat_position (geometry_msgs/msg/PointStamped on 'odom') -> 순수 X, Y 좌표 포인트
  - /odom (nav_msgs/msg/Odometry on 'odom') -> X, Y 위치 및 이동속도
  - /tf ('odom' -> 'base_link', 'odom' -> 'laser_frame')
  - /lidar_tracker/markers (visualization_msgs/msg/MarkerArray) -> RViz 시각화
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster


class LidarBoatTracker(Node):
    def __init__(self):
        super().__init__('lidar_boat_tracker')

        # ── 파라미터 선언 ──────────────────────────────────────────
        self.declare_parameter('lidar_pos_x', 5.0)          # 수조 원점 기준 라이다 X 위치 [m] (우하단 5,0 모서리)
        self.declare_parameter('lidar_pos_y', 0.0)          # 수조 원점 기준 라이다 Y 위치 [m]
        self.declare_parameter('lidar_yaw_deg', 90.0)       # 라이다 방향 (+Y 수조 위쪽 = 90도)
        self.declare_parameter('pool_size_x', 5.0)          # 수조 가로 폭 [m] (오른쪽: +X)
        self.declare_parameter('pool_size_y', 10.0)         # 수조 세로 길이 [m] (위쪽 주황색 화살표 방향: +Y)
        self.declare_parameter('wall_margin', 0.18)         # 수조 벽면 제거 마진 [m]
        self.declare_parameter('cluster_dist_tol', 0.25)    # 클러스터링 거리 허용오차 [m]
        self.declare_parameter('min_cluster_pts', 2)        # 기둥 인식 최소 포인트 수
        self.declare_parameter('max_cluster_pts', 80)       # 기둥 인식 최대 포인트 수
        self.declare_parameter('min_target_diameter', 0.02) # 기둥 최소 직경 [m]
        self.declare_parameter('max_target_diameter', 0.45) # 기둥 최대 직경 [m]
        self.declare_parameter('pos_ema_alpha', 0.45)       # 위치 EMA 필터 계수 (0~1)
        self.declare_parameter('vel_ema_alpha', 0.30)       # 속도 EMA 필터 계수 (0~1)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser_frame')

        # 파라미터 로드
        self.lidar_x = float(self.get_parameter('lidar_pos_x').value)
        self.lidar_y = float(self.get_parameter('lidar_pos_y').value)
        self.lidar_yaw = math.radians(float(self.get_parameter('lidar_yaw_deg').value))
        self.pool_size_x = float(self.get_parameter('pool_size_x').value)
        self.pool_size_y = float(self.get_parameter('pool_size_y').value)
        self.wall_margin = float(self.get_parameter('wall_margin').value)
        self.cluster_tol = float(self.get_parameter('cluster_dist_tol').value)
        self.min_pts = int(self.get_parameter('min_cluster_pts').value)
        self.max_pts = int(self.get_parameter('max_cluster_pts').value)
        self.min_diam = float(self.get_parameter('min_target_diameter').value)
        self.max_diam = float(self.get_parameter('max_target_diameter').value)
        self.pos_alpha = float(self.get_parameter('pos_ema_alpha').value)
        self.vel_alpha = float(self.get_parameter('vel_ema_alpha').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.laser_frame = str(self.get_parameter('laser_frame').value)

        # ── EMA 필터 상태 변수 ─────────────────────────────────────
        self.filtered_pos = None      # np.array([X_filt, Y_filt])
        self.last_meas_pos = None     # np.array([X_meas, Y_meas])
        self.last_time = None         # Time
        self.vx = 0.0                 # m/s
        self.vy = 0.0                 # m/s
        self.target_lost_count = 0

        # ── ROS 2 통신 ────────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)

        # 젯슨 수신 표준 토픽 (배의 순수 X, Y 좌표 전달)
        self.pose_pub = self.create_publisher(PoseStamped, '/detections', 10)
        self.point_pub = self.create_publisher(PointStamped, '/boat_position', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.filtered_scan_pub = self.create_publisher(LaserScan, '/lidar_tracker/filtered_scan', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/lidar_tracker/markers', 10)

        # 라이다 스캔 구독
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            f"🚀 [lidar_boat_tracker] 배 (X, Y) 순수 좌표 추적기 시작 완료!\n"
            f"   - 수조 크기: {self.pool_size_x}m x {self.pool_size_y}m (원점: 좌하단 0,0)\n"
            f"   - 라이다 위치: (X={self.lidar_x}m, Y={self.lidar_y}m)\n"
            f"   - 발행 토픽: /detections (PoseStamped), /boat_position (PointStamped), /odom"
        )

    def _on_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)
        if n == 0:
            return

        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment

        # 1. 유효 거리 필터 (0.08m ~ 15m)
        valid_mask = (ranges > 0.08) & (ranges < 15.0) & np.isfinite(ranges)
        r_valid = ranges[valid_mask]
        a_valid = angles[valid_mask]

        if len(r_valid) == 0:
            self._handle_target_lost(msg.header.stamp)
            return

        # 2. 라이다 로컬 2D 좌표 (x_l, y_l)
        x_l = r_valid * np.cos(a_valid)
        y_l = r_valid * np.sin(a_valid)

        # 3. 수조 절대 좌표계 (X_pool, Y_pool) 변환
        cos_y = math.cos(self.lidar_yaw)
        sin_y = math.sin(self.lidar_yaw)
        x_pool = self.lidar_x + (x_l * cos_y - y_l * sin_y)
        y_pool = self.lidar_y + (x_l * sin_y + y_l * cos_y)

        # 4. [ROI 필터]: 수조 내부 영역(벽면 및 외부 hit 제외) 판정
        min_x = self.wall_margin
        max_x = self.pool_size_x - self.wall_margin
        min_y = self.wall_margin
        max_y = self.pool_size_y - self.wall_margin

        roi_mask = (x_pool >= min_x) & (x_pool <= max_x) & (y_pool >= min_y) & (y_pool <= max_y)
        pts_roi = np.column_stack([x_pool[roi_mask], y_pool[roi_mask]])

        # 디버그용 필터링 스캔 발행
        filtered_ranges = np.full_like(ranges, np.inf)
        valid_indices = np.where(valid_mask)[0][roi_mask]
        filtered_ranges[valid_indices] = ranges[valid_indices]
        filt_msg = msg
        filt_msg.ranges = filtered_ranges.tolist()
        self.filtered_scan_pub.publish(filt_msg)

        if len(pts_roi) < self.min_pts:
            self._handle_target_lost(msg.header.stamp)
            self._publish_markers(msg.header.stamp, None)
            return

        # 5. 유클리디안 클러스터링 (수조 내부 장애물/기둥 분리)
        clusters = self._euclidean_clustering(pts_roi, self.cluster_tol)

        # 6. 배 돌출 구조물(기둥) 규격에 맞는 최적 클러스터 선정 + [중앙값(Median) 추출]
        best_target = None
        min_dist_to_last = float('inf')

        for c_pts in clusters:
            count = len(c_pts)
            if count < self.min_pts or count > self.max_pts:
                continue

            # 클러스터 직경 계산
            diam = np.linalg.norm(np.max(c_pts, axis=0) - np.min(c_pts, axis=0))
            if diam < self.min_diam or diam > self.max_diam:
                continue

            # ⭐ 다중 점 중앙값(Median) 계산 ⭐
            median_center = np.median(c_pts, axis=0)

            # 이전 추적 위치가 있으면 가장 가까운 클러스터 우선 선택
            if self.filtered_pos is not None:
                d = np.linalg.norm(median_center - self.filtered_pos)
                if d < min_dist_to_last:
                    min_dist_to_last = d
                    best_target = (median_center, diam, count)
            else:
                best_target = (median_center, diam, count)
                break

        if best_target is None:
            self._handle_target_lost(msg.header.stamp)
            self._publish_markers(msg.header.stamp, None)
            return

        # 7. ⭐ [위치 EMA 필터 적용] ⭐
        raw_meas_pos = best_target[0]
        curr_time = self.get_clock().now()

        if self.filtered_pos is None:
            self.filtered_pos = raw_meas_pos.copy()
            self.last_meas_pos = raw_meas_pos.copy()
            self.last_time = curr_time
            self.vx = 0.0
            self.vy = 0.0
        else:
            # 1) 위치 EMA 필터: X_filt = alpha * X_meas + (1 - alpha) * X_prev
            self.filtered_pos = (self.pos_alpha * raw_meas_pos +
                                 (1.0 - self.pos_alpha) * self.filtered_pos)

            # 2) 속도 계산
            dt = (curr_time - self.last_time).nanoseconds * 1e-9
            if dt > 0.001:
                inst_vx = (self.filtered_pos[0] - self.last_meas_pos[0]) / dt
                inst_vy = (self.filtered_pos[1] - self.last_meas_pos[1]) / dt

                self.vx = self.vel_alpha * inst_vx + (1.0 - self.vel_alpha) * self.vx
                self.vy = self.vel_alpha * inst_vy + (1.0 - self.vel_alpha) * self.vy

            self.last_meas_pos = self.filtered_pos.copy()
            self.last_time = curr_time

        self.target_lost_count = 0
        tx = float(self.filtered_pos[0])
        ty = float(self.filtered_pos[1])

        # 8. ROS 2 토픽 발행 (순수 X, Y 좌표 발행)
        self._publish_outputs(msg.header.stamp, tx, ty, self.vx, self.vy)
        self._publish_markers(msg.header.stamp, (tx, ty))

    def _euclidean_clustering(self, points, tol):
        clusters = []
        if len(points) == 0:
            return clusters

        curr_cluster = [points[0]]
        for i in range(1, len(points)):
            dist = np.linalg.norm(points[i] - points[i - 1])
            if dist <= tol:
                curr_cluster.append(points[i])
            else:
                clusters.append(np.array(curr_cluster))
                curr_cluster = [points[i]]
        if len(curr_cluster) > 0:
            clusters.append(np.array(curr_cluster))
        return clusters

    def _handle_target_lost(self, stamp):
        self.target_lost_count += 1
        if self.target_lost_count > 15:
            self.vx = 0.0
            self.vy = 0.0
            if self.target_lost_count == 16:
                self.get_logger().warn("⚠️ [lidar_boat_tracker] 수조 내부에서 배 구조물 미검출 (Searching...)")

    def _publish_outputs(self, stamp, x, y, vx, vy):
        # 1. PoseStamped (/detections) 발행 (Orientation은 항등 쿼터니언)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.odom_frame
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        self.pose_pub.publish(pose_msg)

        # 2. PointStamped (/boat_position) 순수 X,Y 좌표 발행
        point_msg = PointStamped()
        point_msg.header.stamp = stamp
        point_msg.header.frame_id = self.odom_frame
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = 0.0
        self.point_pub.publish(point_msg)

        # 3. Odometry (/odom) 발행
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = 0.0
        self.odom_pub.publish(odom_msg)

        # 4. TF 발행 (odom -> base_link)
        tf_boat = TransformStamped()
        tf_boat.header.stamp = stamp
        tf_boat.header.frame_id = self.odom_frame
        tf_boat.child_frame_id = self.base_frame
        tf_boat.transform.translation.x = x
        tf_boat.transform.translation.y = y
        tf_boat.transform.translation.z = 0.0
        tf_boat.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_boat)

        # 5. TF 발행 (odom -> laser_frame: 고정 라이다 위치 5,0)
        tf_lidar = TransformStamped()
        tf_lidar.header.stamp = stamp
        tf_lidar.header.frame_id = self.odom_frame
        tf_lidar.child_frame_id = self.laser_frame
        tf_lidar.transform.translation.x = self.lidar_x
        tf_lidar.transform.translation.y = self.lidar_y
        tf_lidar.transform.translation.z = 0.0
        tf_lidar.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_lidar)

        # 터미널 로깅 (1초 주기)
        speed = math.hypot(vx, vy)
        self.get_logger().info(
            f"📍 [LiDAR Tracker] 배 위치 -> X: {x:.3f} m,  Y: {y:.3f} m  (속도: {speed:.2f} m/s)",
            throttle_duration_sec=1.0)

    def _publish_markers(self, stamp, target_pos):
        """RViz 시각화 마커 발행."""
        ma = MarkerArray()

        # 1. 수조 4면 테두리 (10m x 5m)
        m_pool = Marker()
        m_pool.header.stamp = stamp
        m_pool.header.frame_id = self.odom_frame
        m_pool.ns = "pool_bounds"
        m_pool.id = 0
        m_pool.type = Marker.LINE_STRIP
        m_pool.action = Marker.ADD
        m_pool.scale.x = 0.05
        m_pool.color.r = 0.2
        m_pool.color.g = 0.6
        m_pool.color.b = 1.0
        m_pool.color.a = 1.0
        from geometry_msgs.msg import Point
        pts = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=self.pool_size_x, y=0.0, z=0.0),
            Point(x=self.pool_size_x, y=self.pool_size_y, z=0.0),
            Point(x=0.0, y=self.pool_size_y, z=0.0),
            Point(x=0.0, y=0.0, z=0.0)
        ]
        m_pool.points = pts
        ma.markers.append(m_pool)

        # 2. 라이다 고정 설치 위치 마커 (5, 0)
        m_lidar = Marker()
        m_lidar.header.stamp = stamp
        m_lidar.header.frame_id = self.odom_frame
        m_lidar.ns = "lidar_sensor"
        m_lidar.id = 1
        m_lidar.type = Marker.CYLINDER
        m_lidar.action = Marker.ADD
        m_lidar.pose.position.x = self.lidar_x
        m_lidar.pose.position.y = self.lidar_y
        m_lidar.pose.position.z = 0.1
        m_lidar.scale.x = 0.2
        m_lidar.scale.y = 0.2
        m_lidar.scale.z = 0.15
        m_lidar.color.r = 1.0
        m_lidar.color.g = 0.0
        m_lidar.color.b = 0.0
        m_lidar.color.a = 1.0
        ma.markers.append(m_lidar)

        # 3. 배 위치 실시간 마커 (선명한 원형 실린더/구)
        if target_pos is not None:
            tx, ty = target_pos
            m_boat = Marker()
            m_boat.header.stamp = stamp
            m_boat.header.frame_id = self.odom_frame
            m_boat.ns = "boat_target"
            m_boat.id = 2
            m_boat.type = Marker.CYLINDER
            m_boat.action = Marker.ADD
            m_boat.pose.position.x = tx
            m_boat.pose.position.y = ty
            m_boat.pose.position.z = 0.15
            m_boat.scale.x = 0.35  # 기둥 지름
            m_boat.scale.y = 0.35
            m_boat.scale.z = 0.30  # 기둥 높이
            m_boat.color.r = 1.0
            m_boat.color.g = 0.55
            m_boat.color.b = 0.0
            m_boat.color.a = 1.0
            ma.markers.append(m_boat)

            # 배 위치 텍스트 표시
            m_text = Marker()
            m_text.header.stamp = stamp
            m_text.header.frame_id = self.odom_frame
            m_text.ns = "boat_label"
            m_text.id = 3
            m_text.type = Marker.TEXT_VIEW_FACING
            m_text.action = Marker.ADD
            m_text.pose.position.x = tx
            m_text.pose.position.y = ty + 0.35
            m_text.pose.position.z = 0.4
            m_text.text = f"Boat [X:{tx:.2f}m, Y:{ty:.2f}m]"
            m_text.scale.z = 0.25
            m_text.color.r = 1.0
            m_text.color.g = 1.0
            m_text.color.b = 0.0
            m_text.color.a = 1.0
            ma.markers.append(m_text)

        self.marker_pub.publish(ma)


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
