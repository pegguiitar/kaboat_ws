"""lidar_boat_tracker — 실내 수조 고정 2D 라이다 기반 배 위치/방향 추적기 (중앙값 + EMA 노이즈 필터링).

배치 조건:
  - 수조 좌하단 원점 (X=0m, Y=0m) 기준, (X=5.0m, Y=0.0m) 지점에 2D 라이다 설치
  - 수조 크기: 가로 10.0m x 세로 5.0m
  - 라이다는 전방(+Y 수조 내부)을 향해 스캔

핵심 노이즈 필터링:
  1. [ROI 필터]: 수조 내부(0.18m 마진) 외의 벽면/외부 hit 100% 제거
  2. [클러스터 중앙값(Median)]: 기둥 표면에 여러 점이 찍혔을 때 평균 대신 중앙값(Median)을 취해 외곽 이상치(Outlier) 제거
  3. [위치 EMA 필터]: X_filt = alpha * X_meas + (1 - alpha) * X_filt 로 라이다 센서 지터(Jitter) 완벽 억제
  4. [속도/선수각 EMA 필터]: 순간 미분 노이즈를 완화하여 매끄러운 진행 방향(Yaw) 출력
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw_rad):
    """Yaw(rad) 회전각을 쿼터니언 메시지로 변환."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


class LidarBoatTracker(Node):
    def __init__(self):
        super().__init__('lidar_boat_tracker')

        # ── 파라미터 선언 ──────────────────────────────────────────
        self.declare_parameter('lidar_pos_x', 5.0)          # 수조 원점 기준 라이다 X 위치 [m]
        self.declare_parameter('lidar_pos_y', 0.0)          # 수조 원점 기준 라이다 Y 위치 [m]
        self.declare_parameter('lidar_yaw_deg', 90.0)       # 라이다 방향 (+Y 수조 안쪽 = 90도)
        self.declare_parameter('pool_size_x', 10.0)         # 수조 가로 길이 [m]
        self.declare_parameter('pool_size_y', 5.0)          # 수조 세로 길이 [m]
        self.declare_parameter('wall_margin', 0.18)         # 수조 벽면 제거 마진 [m]
        self.declare_parameter('cluster_dist_tol', 0.25)    # 클러스터링 거리 허용오차 [m]
        self.declare_parameter('min_cluster_pts', 2)        # 기둥 인식 최소 포인트 수
        self.declare_parameter('max_cluster_pts', 80)       # 기둥 인식 최대 포인트 수
        self.declare_parameter('min_target_diameter', 0.02) # 기둥 최소 직경 [m]
        self.declare_parameter('max_target_diameter', 0.45) # 기둥 최대 직경 [m]
        self.declare_parameter('pos_ema_alpha', 0.45)       # 위치 EMA 필터 계수 (0~1, 작을수록 부드러움)
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
        self.filtered_yaw = 0.0       # rad
        self.vx = 0.0                 # m/s
        self.vy = 0.0                 # m/s
        self.target_lost_count = 0

        # ── ROS 2 통신 ────────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)

        # 젯슨 수신 표준 토픽
        self.pose_pub = self.create_publisher(PoseStamped, '/detections', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.filtered_scan_pub = self.create_publisher(LaserScan, '/lidar_tracker/filtered_scan', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/lidar_tracker/markers', 10)

        # 라이다 스캔 구독
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            f"🚀 [lidar_boat_tracker] 노이즈 필터링(중앙값 + EMA) 활성화!\n"
            f"   - 수조 크기: {self.pool_size_x}m x {self.pool_size_y}m (원점: 좌하단)\n"
            f"   - 라이다 위치: (X={self.lidar_x}m, Y={self.lidar_y}m, Yaw={math.degrees(self.lidar_yaw):.1f}°)\n"
            f"   - 위치 EMA Alpha: {self.pos_alpha} | 속도 EMA Alpha: {self.vel_alpha}"
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

            # ⭐ 여러 점이 찍혔을 때 이상치(Outlier)를 잡기 위해 중앙값(Median) 사용 ⭐
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

        # 7. ⭐ [위치 및 속도/자세 EMA 필터 적용] ⭐
        raw_meas_pos = best_target[0]
        curr_time = self.get_clock().now()

        if self.filtered_pos is None:
            # 첫 측정값 초기화
            self.filtered_pos = raw_meas_pos.copy()
            self.last_meas_pos = raw_meas_pos.copy()
            self.last_time = curr_time
            self.vx = 0.0
            self.vy = 0.0
        else:
            # 1) 위치 EMA 필터: X_filt = alpha * X_meas + (1 - alpha) * X_prev
            self.filtered_pos = (self.pos_alpha * raw_meas_pos +
                                 (1.0 - self.pos_alpha) * self.filtered_pos)

            # 2) 속도 및 선수각 계산
            dt = (curr_time - self.last_time).nanoseconds * 1e-9
            if dt > 0.001:
                inst_vx = (self.filtered_pos[0] - self.last_meas_pos[0]) / dt
                inst_vy = (self.filtered_pos[1] - self.last_meas_pos[1]) / dt

                # 속도 EMA 필터
                self.vx = self.vel_alpha * inst_vx + (1.0 - self.vel_alpha) * self.vx
                self.vy = self.vel_alpha * inst_vy + (1.0 - self.vel_alpha) * self.vy

                speed = math.hypot(self.vx, self.vy)
                if speed > 0.04:  # 4cm/s 이상 이동 시 선수각 매끄럽게 갱신
                    target_yaw = math.atan2(self.vy, self.vx)
                    # 각도 차이 보정 (-pi ~ +pi)
                    dyaw = math.atan2(math.sin(target_yaw - self.filtered_yaw),
                                      math.cos(target_yaw - self.filtered_yaw))
                    self.filtered_yaw += 0.35 * dyaw

            self.last_meas_pos = self.filtered_pos.copy()
            self.last_time = curr_time

        self.target_lost_count = 0
        tx = float(self.filtered_pos[0])
        ty = float(self.filtered_pos[1])

        # 8. ROS 2 토픽 발행
        self._publish_outputs(msg.header.stamp, tx, ty, self.filtered_yaw, self.vx, self.vy)
        self._publish_markers(msg.header.stamp, (tx, ty, self.filtered_yaw))

    def _euclidean_clustering(self, points, tol):
        """연속된 인접 점들을 유클리디안 거리로 군집화."""
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
        if self.target_lost_count > 15:  # 0.5초 이상 미검출 시
            self.vx = 0.0
            self.vy = 0.0
            if self.target_lost_count == 16:
                self.get_logger().warn("⚠️ [lidar_boat_tracker] 수조 내부에서 배 구조물 미검출 (Searching...)")

    def _publish_outputs(self, stamp, x, y, yaw, vx, vy):
        q = yaw_to_quaternion(yaw)

        # 1. PoseStamped (/detections) 발행
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.odom_frame
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = q
        self.pose_pub.publish(pose_msg)

        # 2. Odometry (/odom) 발행
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = q
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = 0.0
        self.odom_pub.publish(odom_msg)

        # 3. TF 발행 (odom -> base_link)
        tf_boat = TransformStamped()
        tf_boat.header.stamp = stamp
        tf_boat.header.frame_id = self.odom_frame
        tf_boat.child_frame_id = self.base_frame
        tf_boat.transform.translation.x = x
        tf_boat.transform.translation.y = y
        tf_boat.transform.translation.z = 0.0
        tf_boat.transform.rotation = q
        self.tf_broadcaster.sendTransform(tf_boat)

        # TF 발행 (odom -> laser_frame: 고정 라이다 위치)
        q_lidar = yaw_to_quaternion(self.lidar_yaw)
        tf_lidar = TransformStamped()
        tf_lidar.header.stamp = stamp
        tf_lidar.header.frame_id = self.odom_frame
        tf_lidar.child_frame_id = self.laser_frame
        tf_lidar.transform.translation.x = self.lidar_x
        tf_lidar.transform.translation.y = self.lidar_y
        tf_lidar.transform.translation.z = 0.0
        tf_lidar.transform.rotation = q_lidar
        self.tf_broadcaster.sendTransform(tf_lidar)

        # 터미널 로깅 (1초 주기)
        speed = math.hypot(vx, vy)
        self.get_logger().info(
            f"🎯 [LiDAR Tracker] 배 위치:[X:{x:.2f}m, Y:{y:.2f}m] | 선수각:{math.degrees(yaw):+.1f}° | 속도:{speed:.2f}m/s",
            throttle_duration_sec=1.0)

    def _publish_markers(self, stamp, target_info):
        """RViz에서 수조 테두리 및 배 위치를 선명하게 보여주는 마커 발행."""
        ma = MarkerArray()

        # 1. 수조 4면 테두리 사각형 (10m x 5m)
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

        # 2. 라이다 설치 위치 마커 (5m, 0m)
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

        # 3. 배 위치 실시간 마커
        if target_info is not None:
            tx, ty, tyaw = target_info
            m_boat = Marker()
            m_boat.header.stamp = stamp
            m_boat.header.frame_id = self.odom_frame
            m_boat.ns = "boat_target"
            m_boat.id = 2
            m_boat.type = Marker.ARROW
            m_boat.action = Marker.ADD
            m_boat.pose.position.x = tx
            m_boat.pose.position.y = ty
            m_boat.pose.position.z = 0.05
            m_boat.pose.orientation = yaw_to_quaternion(tyaw)
            m_boat.scale.x = 0.7   # 화살표 길이
            m_boat.scale.y = 0.15  # 화살표 폭
            m_boat.scale.z = 0.15  # 화살표 높이
            m_boat.color.r = 1.0
            m_boat.color.g = 0.55
            m_boat.color.b = 0.0
            m_boat.color.a = 1.0
            ma.markers.append(m_boat)

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
