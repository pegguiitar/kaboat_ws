#!/usr/bin/env python3
"""노트북 RViz 표시용: 스캔·배 위치를 odom 바닥면(z ≈ 0)에 납작하게 (view_lidar 가 같이 켬).

/odom 의 z(원점 대비 GNSS 고도, 1 m 넘게 나옴)와 롤·피치 때문에 RViz 에서 /scan 이 지도
위 허공에 비스듬히 뜬다. 이 노드는 /odom 의 x, y, 선수각만 써서

  /scan/flat  (PointCloud2, frame odom)  스캔 점을 바닥면 z = flat_z 에
  /odom/flat  (Odometry)                 z = 0, 선수각만 남긴 배 위치

를 낸다. 표시 전용 — 젯슨의 /odom·TF·자율주행은 그대로다.
라이다 장착 위치는 선체 기준 lidar_offset (기본 x 0.4 m, onboard_lidar.launch.py 의 TF 와 같음).
"""

import bisect
import math

import numpy as np

STAMP_TOLERANCE_SEC = 0.3     # 스캔과 이보다 시각 차이가 큰 /odom 만 있으면 그리지 않음


def yaw_of(q) -> float:
    """쿼터니언 (x, y, z, w) → 선수각 [rad]."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def flatten_scan(ranges, angle_min, angle_increment, range_min, range_max,
                 pose, lidar_offset=(0.4, 0.0, 0.0), z=0.05):
    """LaserScan → odom 바닥면 점 (N×3 float32).

    pose = (x, y, yaw) 배(base_link) 위치, lidar_offset = (x, y, yaw) 선체 기준 라이다.
    """
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + angle_increment * np.arange(r.size)
    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    r, a = r[ok], a[ok]
    ox, oy, oyaw = lidar_offset
    bx = ox + r * np.cos(a + oyaw)
    by = oy + r * np.sin(a + oyaw)
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    pts = np.empty((r.size, 3), dtype=np.float32)
    pts[:, 0] = x + c * bx - s * by
    pts[:, 1] = y + s * bx + c * by
    pts[:, 2] = z
    return pts


class PoseHistory:
    """/odom (시각, x, y, yaw) 최근 기록 — 스캔 시각에 가장 가까운 것을 찾는다."""

    def __init__(self, keep_sec=10.0):
        self.keep = keep_sec
        self.stamps, self.poses = [], []

    def add(self, stamp, pose):
        if self.stamps and stamp <= self.stamps[-1]:
            if stamp < self.stamps[-1] - self.keep:   # 시각이 크게 되돌아감(재시작) → 비움
                self.stamps, self.poses = [], []
            else:
                return                                # 중복·순서 뒤바뀜은 무시
        self.stamps.append(stamp)
        self.poses.append(pose)
        cut = bisect.bisect_left(self.stamps, stamp - self.keep)
        if cut:
            del self.stamps[:cut], self.poses[:cut]

    def nearest(self, stamp, tolerance=STAMP_TOLERANCE_SEC):
        if not self.stamps:
            return None
        i = bisect.bisect_left(self.stamps, stamp)
        best = min((j for j in (i - 1, i) if 0 <= j < len(self.stamps)),
                   key=lambda j: abs(self.stamps[j] - stamp))
        return self.poses[best] if abs(self.stamps[best] - stamp) <= tolerance else None


def main(args=None):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header

    def stamp_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    class FlatScanView(Node):
        def __init__(self):
            super().__init__('flat_scan_view')
            self.declare_parameter('lidar_offset_x', 0.4)
            self.declare_parameter('lidar_offset_y', 0.0)
            self.declare_parameter('lidar_offset_yaw_deg', 0.0)
            self.declare_parameter('flat_z', 0.05)    # 지도(z 0) 면에 묻히지 않게 살짝 위
            g = lambda n: float(self.get_parameter(n).value)
            self.offset = (g('lidar_offset_x'), g('lidar_offset_y'),
                           math.radians(g('lidar_offset_yaw_deg')))
            self.z = g('flat_z')
            self.history = PoseHistory()
            self.cloud_pub = self.create_publisher(
                point_cloud2.PointCloud2, '/scan/flat', 5)
            self.odom_pub = self.create_publisher(Odometry, '/odom/flat', 10)
            self.create_subscription(Odometry, '/odom', self._on_odom, 20)
            self.create_subscription(LaserScan, '/scan', self._on_scan,
                                     qos_profile_sensor_data)
            self.get_logger().info('/scan → /scan/flat, /odom → /odom/flat (바닥면 표시용)')

        def _on_odom(self, msg):
            o = msg.pose.pose.orientation
            yaw = yaw_of((o.x, o.y, o.z, o.w))
            p = msg.pose.pose.position
            self.history.add(stamp_sec(msg.header.stamp), (p.x, p.y, yaw))
            flat = Odometry()
            flat.header = msg.header
            flat.child_frame_id = msg.child_frame_id
            flat.pose.pose.position.x, flat.pose.pose.position.y = p.x, p.y
            flat.pose.pose.orientation.z = math.sin(yaw / 2.0)
            flat.pose.pose.orientation.w = math.cos(yaw / 2.0)
            flat.twist = msg.twist
            self.odom_pub.publish(flat)

        def _on_scan(self, msg):
            pose = self.history.nearest(stamp_sec(msg.header.stamp))
            if pose is None:
                self.get_logger().warn('스캔 시각에 맞는 /odom 없음 — 그 스캔은 건너뜀',
                                       throttle_duration_sec=5.0)
                return
            pts = flatten_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                               msg.range_min, msg.range_max, pose, self.offset, self.z)
            header = Header(stamp=msg.header.stamp, frame_id='odom')
            self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, pts))

    rclpy.init(args=args)
    node = FlatScanView()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
