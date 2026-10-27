"""Mask boat-structure returns in the onboard TG-50 scan before mapping.

Angles are measured in the published LaserScan frame, counterclockwise from
its +X axis. The default 220..320 degree sector equals -140..-40 degrees in
a scan whose angle range is -180..180 degrees.
"""

import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


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


class OnboardScanFilter(Node):
    def __init__(self):
        super().__init__('onboard_scan_filter')
        self.declare_parameter('mask_start_deg', 220.0)
        self.declare_parameter('mask_end_deg', 320.0)
        self.start_deg = float(self.get_parameter('mask_start_deg').value)
        self.end_deg = float(self.get_parameter('mask_end_deg').value)
        if not (math.isfinite(self.start_deg) and math.isfinite(self.end_deg)
                and 0.0 <= self.start_deg < self.end_deg <= 360.0):
            raise ValueError('invalid onboard scan mask angles')

        self.publisher = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, '/scan/raw', self._on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            f'Onboard TG-50 mask: {self.start_deg:.1f}°..{self.end_deg:.1f}° '
            '(published /scan frame)')

    def _on_scan(self, scan: LaserScan):
        self.publisher.publish(mask_sector(scan, self.start_deg, self.end_deg))


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
