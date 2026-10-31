"""Mask boat-structure returns and filter tilted scans using IMU in onboard TG-50.

Angles are measured in the published LaserScan frame, counterclockwise from
its +X axis. The default 220..320 degree sector equals -140..-40 degrees in
a scan whose angle range is -180..180 degrees.

IMU tilt filter rejects scans when boat roll or pitch exceeds tilt_threshold_deg
(default: 5.0 degrees), preventing surface reflections or sky points from corrupting
downstream occupancy and DRI maps.
"""

import copy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


def quaternion_to_roll_pitch(x: float, y: float, z: float, w: float):
    """Convert orientation quaternion to roll and pitch angles in radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return roll, pitch


def is_tilted(roll: float, pitch: float, threshold_rad: float) -> bool:
    """Return True if roll or pitch magnitude exceeds threshold_rad."""
    return abs(roll) > threshold_rad or abs(pitch) > threshold_rad


def mask_sector(scan: LaserScan, start_deg: float, end_deg: float) -> LaserScan:
    """Return a copy with inclusive sector ranges invalidated as +inf."""
    if not (0.0 <= start_deg < end_deg <= 360.0):
        raise ValueError('mask sector must satisfy 0 <= start < end <= 360 degrees')

    filtered = copy.deepcopy(scan)
    has_intensities = len(filtered.intensities) == len(filtered.ranges)
    for i in range(len(filtered.ranges)):
        angle = math.degrees(scan.angle_min + i * scan.angle_increment) % 360.0
        if start_deg - 1e-6 <= angle <= end_deg + 1e-6:
            filtered.ranges[i] = math.inf
            if has_intensities:
                filtered.intensities[i] = 0.0
    return filtered


def invalidate_all_ranges(scan: LaserScan) -> LaserScan:
    """Return a copy where all ranges are math.inf and intensities are 0.0."""
    filtered = copy.deepcopy(scan)
    filtered.ranges = [math.inf] * len(filtered.ranges)
    if len(filtered.intensities) > 0:
        filtered.intensities = [0.0] * len(filtered.intensities)
    return filtered


class OnboardScanFilter(Node):
    def __init__(self):
        super().__init__('onboard_scan_filter')

        # Sector mask parameters
        self.declare_parameter('enable_sector_mask', True)
        self.declare_parameter('mask_start_deg', 220.0)
        self.declare_parameter('mask_end_deg', 320.0)

        # IMU tilt filter parameters
        self.declare_parameter('enable_imu_filter', True)
        self.declare_parameter('tilt_threshold_deg', 5.0)
        self.declare_parameter('imu_timeout_sec', 1.0)
        self.declare_parameter('tilt_action', 'drop')  # 'drop' or 'invalidate'

        self.enable_sector_mask = bool(self.get_parameter('enable_sector_mask').value)
        self.start_deg = float(self.get_parameter('mask_start_deg').value)
        self.end_deg = float(self.get_parameter('mask_end_deg').value)

        self.enable_imu_filter = bool(self.get_parameter('enable_imu_filter').value)
        self.tilt_threshold_deg = float(self.get_parameter('tilt_threshold_deg').value)
        self.threshold_rad = math.radians(self.tilt_threshold_deg)
        self.imu_timeout_sec = float(self.get_parameter('imu_timeout_sec').value)
        self.tilt_action = str(self.get_parameter('tilt_action').value).lower()

        if self.enable_sector_mask:
            if not (math.isfinite(self.start_deg) and math.isfinite(self.end_deg)
                    and 0.0 <= self.start_deg < self.end_deg <= 360.0):
                raise ValueError('invalid onboard scan mask angles')

        if not (math.isfinite(self.tilt_threshold_deg) and self.tilt_threshold_deg > 0.0):
            raise ValueError('tilt_threshold_deg must be positive and finite')

        if self.tilt_action not in ('drop', 'invalidate'):
            raise ValueError("tilt_action must be either 'drop' or 'invalidate'")

        # State tracking
        self.latest_imu = None
        self.latest_imu_time = None
        self.roll = 0.0
        self.pitch = 0.0
        self.tilted = False

        self.publisher = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan/raw', self._on_scan, qos_profile_sensor_data)

        if self.enable_imu_filter:
            self.create_subscription(
                Imu, '/imu/data', self._on_imu, qos_profile_sensor_data)

        self.get_logger().info(
            f'OnboardScanFilter ready | Sector mask: {self.enable_sector_mask} '
            f'({self.start_deg:.1f}°..{self.end_deg:.1f}°) | '
            f'IMU tilt filter: {self.enable_imu_filter} (threshold={self.tilt_threshold_deg:.1f}°, '
            f'action={self.tilt_action})')

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        self.roll, self.pitch = quaternion_to_roll_pitch(q.x, q.y, q.z, q.w)
        self.tilted = is_tilted(self.roll, self.pitch, self.threshold_rad)
        self.latest_imu = msg
        self.latest_imu_time = time.monotonic()

    def _on_scan(self, scan: LaserScan):
        # 1. IMU tilt filter
        if self.enable_imu_filter:
            now = time.monotonic()
            if self.latest_imu_time is None:
                self.get_logger().warn(
                    'IMU 데이터 수신 대기 중 — 스캔 필터 통과 유지',
                    throttle_duration_sec=3.0)
            elif (now - self.latest_imu_time) > self.imu_timeout_sec:
                self.get_logger().warn(
                    f'IMU 데이터 지연 ({now - self.latest_imu_time:.2f}s > {self.imu_timeout_sec}s)',
                    throttle_duration_sec=3.0)
            elif self.tilted:
                r_deg = math.degrees(self.roll)
                p_deg = math.degrees(self.pitch)
                self.get_logger().warn(
                    f'배 기울기 임계 초과로 스캔 차단: roll={r_deg:.1f}°, pitch={p_deg:.1f}° '
                    f'(임계={self.tilt_threshold_deg:.1f}°)',
                    throttle_duration_sec=1.0)
                if self.tilt_action == 'drop':
                    return
                elif self.tilt_action == 'invalidate':
                    self.publisher.publish(invalidate_all_ranges(scan))
                    return

        # 2. Sector mask
        out_scan = scan
        if self.enable_sector_mask:
            out_scan = mask_sector(scan, self.start_deg, self.end_deg)

        self.publisher.publish(out_scan)


def main(args=None):
    rclpy.init(args=args)
    node = OnboardScanFilter()
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
