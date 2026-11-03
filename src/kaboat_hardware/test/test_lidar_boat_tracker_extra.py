"""외벽 라이다 선체 추적 — 실패 시 안전(C) 보강: 판이 없는 스캔에서는 pose 를 내지 않는다."""
import math

import pytest
import rclpy
from sensor_msgs.msg import LaserScan

from kaboat_hardware.lidar_boat_tracker import LidarBoatTracker


@pytest.mark.check('C')
def test_empty_pool_scan_publishes_no_pose():
    rclpy.init()
    node = LidarBoatTracker()
    try:
        poses = []
        node.pose_cov_pub.publish = poses.append
        scan = LaserScan()
        scan.header.frame_id = 'shore_laser_frame'
        scan.header.stamp.sec = 1
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = 2.0 * math.pi / 719
        scan.range_min, scan.range_max = 0.08, 15.0
        scan.ranges = [12.0] * 720              # 먼 벽만 보이는 빈 수조
        node._on_scan(scan)
        scan.ranges = [math.inf] * 720          # 반사 없음
        node._on_scan(scan)
        assert poses == []
    finally:
        node.destroy_node()
        rclpy.shutdown()
