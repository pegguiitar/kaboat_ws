"""obstacle_detector — LiDAR 클러스터링으로 장애물 위치 확정 (상시 가동, ① 레이어)

입력   /scan (sensor_msgs/LaserScan)  — sensor_drivers(sim은 gz bridge)의 raw
출력   /obstacles (kaboat_msgs/ObstacleArray)

원리 (skeleton — 인접 거리 기반 클러스터링):
  연속한 빔의 range 차이가 cluster_gap 이하면 같은 물체로 묶고,
  클러스터의 중심각/평균거리로 장애물 중심과 근사 반지름을 계산한다.
  odom→base_link→lidar TF 가 있으면 odom 좌표로 변환해 발행하고 (behavior 들이
  전역 좌표로 쓰기 좋게), TF 를 아직 못 받았으면 lidar 프레임 그대로 발행한다
  — frame_id 를 반드시 확인하고 쓸 것.

TODO(팀): DBSCAN 등 제대로 된 클러스터링, /camera/depth/points 융합, 시간 필터링
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose

from kaboat_msgs.msg import Obstacle, ObstacleArray

import tf2_ros


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.declare_parameter('cluster_gap', 0.5)     # 같은 물체로 묶는 인접 빔 거리차 [m]
        self.declare_parameter('min_points', 3)        # 클러스터 최소 빔 수 (노이즈 제거)
        self.declare_parameter('max_range', 30.0)      # 이보다 먼 빔은 무시 [m]

        self.cluster_gap = float(self.get_parameter('cluster_gap').value)
        self.min_points = int(self.get_parameter('min_points').value)
        self.max_range = float(self.get_parameter('max_range').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.pub = self.create_publisher(ObstacleArray, '/obstacles', 10)
        # RViz 확인용 (PoseArray 는 RViz 기본 표시 가능)
        self.pub_debug = self.create_publisher(PoseArray, '/obstacles/debug_poses', 10)

        self.get_logger().info('obstacle_detector 시작 — /scan → /obstacles')

    def on_scan(self, scan: LaserScan):
        # 1) 유효 빔만 (거리, 각도) 목록으로
        points = []  # (angle, range)
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < min(scan.range_max, self.max_range) and math.isfinite(r):
                points.append((angle, r))
            angle += scan.angle_increment

        # 2) 인접 빔 거리차 기반 클러스터링
        clusters = []
        current = []
        for i, (a, r) in enumerate(points):
            if current and abs(r - current[-1][1]) > self.cluster_gap:
                clusters.append(current)
                current = []
            current.append((a, r))
        if current:
            clusters.append(current)

        # 3) 클러스터 → 장애물 (lidar 프레임 기준 중심/반지름)
        out = ObstacleArray()
        out.header.stamp = scan.header.stamp
        out.header.frame_id = scan.header.frame_id

        obstacles_local = []
        for c in clusters:
            if len(c) < self.min_points:
                continue
            mid_a = (c[0][0] + c[-1][0]) / 2.0
            mean_r = sum(r for _, r in c) / len(c)
            # 클러스터 양 끝 사이 호 길이의 절반을 근사 반지름으로
            arc = mean_r * abs(c[-1][0] - c[0][0])
            obstacles_local.append((mean_r * math.cos(mid_a),
                                    mean_r * math.sin(mid_a),
                                    max(arc / 2.0, 0.1)))

        # 4) TF 가 있으면 odom 좌표로 변환 (없으면 lidar 프레임 그대로)
        try:
            t = self.tf_buffer.lookup_transform('odom', scan.header.frame_id,
                                                rclpy.time.Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            obstacles_local = [(tx + x * cos_y - y * sin_y,
                                ty + x * sin_y + y * cos_y, rad)
                               for x, y, rad in obstacles_local]
            out.header.frame_id = 'odom'
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass  # TF 미가용 — lidar 프레임 그대로

        debug = PoseArray()
        debug.header = out.header
        for x, y, rad in obstacles_local:
            ob = Obstacle()
            ob.position.x, ob.position.y = x, y
            ob.radius = rad
            out.obstacles.append(ob)
            p = Pose()
            p.position.x, p.position.y = x, y
            p.orientation.w = 1.0
            debug.poses.append(p)

        self.pub.publish(out)
        self.pub_debug.publish(debug)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
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
