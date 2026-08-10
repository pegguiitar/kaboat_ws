"""실물 센서 토픽의 수신 여부와 품질을 /diagnostics 및 콘솔로 보고한다."""

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, NavSatFix, PointCloud2

from kaboat_hardware.health import ERROR, OK, WARN, TopicTracker


SENSOR_TYPES = {
    'color': Image,
    'depth': Image,
    'camera_info': CameraInfo,
    'scan': LaserScan,
    'imu': Imu,
    'gps': NavSatFix,
    'odom': Odometry,
    'pointcloud': PointCloud2,
}


class SensorHealthMonitor(Node):
    def __init__(self):
        super().__init__('sensor_health_monitor')
        self.declare_parameter('report_period_sec', 2.0)
        self.declare_parameter('startup_grace_sec', 10.0)

        defaults = {
            'color': ('/camera/color/image_raw', True, 10.0, 1.0),
            'depth': ('/camera/depth/image_raw', True, 10.0, 1.0),
            'camera_info': ('/camera/camera_info', True, 5.0, 1.0),
            'scan': ('/scan', True, 5.0, 1.0),
            'imu': ('/imu/data', True, 20.0, 0.5),
            'gps': ('/gps/fix', True, 1.0, 3.0),
            'odom': ('/odom', True, 10.0, 1.0),
            'pointcloud': ('/camera/depth/points', False, 5.0, 1.0),
        }

        self.trackers = {}
        self.latest_fields = {}
        self.data_levels = {}
        # rclpy.Node가 내부적으로 사용하는 ``_subscriptions`` 이름을 덮어쓰면
        # destroy_node()에서 구독 목록이 깨지므로 별도 이름으로 참조를 보관한다.
        self.sensor_subscriptions = []
        for name, (topic, enabled, min_rate, timeout) in defaults.items():
            self.declare_parameter(f'{name}.enabled', enabled)
            self.declare_parameter(f'{name}.topic', topic)
            self.declare_parameter(f'{name}.min_rate_hz', min_rate)
            self.declare_parameter(f'{name}.timeout_sec', timeout)

            tracker = TopicTracker(
                enabled=bool(self.get_parameter(f'{name}.enabled').value),
                topic=str(self.get_parameter(f'{name}.topic').value),
                min_rate_hz=float(self.get_parameter(f'{name}.min_rate_hz').value),
                timeout_sec=float(self.get_parameter(f'{name}.timeout_sec').value),
            )
            self.trackers[name] = tracker
            self.latest_fields[name] = {}
            self.data_levels[name] = OK
            if tracker.enabled:
                sub = self.create_subscription(
                    SENSOR_TYPES[name], tracker.topic,
                    self._callback_for(name), qos_profile_sensor_data)
                self.sensor_subscriptions.append(sub)

        self.start_sec = time.monotonic()
        self.startup_grace_sec = float(
            self.get_parameter('startup_grace_sec').value)
        report_period = float(self.get_parameter('report_period_sec').value)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self.create_timer(report_period, self._report)
        enabled = ', '.join(
            f'{name}={tracker.topic}' for name, tracker in self.trackers.items()
            if tracker.enabled)
        self.get_logger().info(f'실물 센서 진단 시작: {enabled}')

    def _callback_for(self, name):
        def callback(msg):
            header = getattr(msg, 'header', None)
            frame_id = getattr(header, 'frame_id', '') if header else ''
            self.trackers[name].observe(time.monotonic(), frame_id)
            self.data_levels[name], self.latest_fields[name] = self._inspect(name, msg)
        return callback

    @staticmethod
    def _inspect(name, msg):
        level = OK
        fields = {}
        if name in ('color', 'depth'):
            fields = {
                'resolution': f'{msg.width}x{msg.height}',
                'encoding': msg.encoding,
            }
            if msg.width == 0 or msg.height == 0 or not msg.data:
                level = ERROR
                fields['data_check'] = 'empty image'
        elif name == 'camera_info':
            fields = {'resolution': f'{msg.width}x{msg.height}', 'fx': f'{msg.k[0]:.3f}'}
            if msg.width == 0 or msg.height == 0 or msg.k[0] <= 0.0:
                level = ERROR
                fields['calibration_check'] = 'invalid intrinsics'
        elif name == 'scan':
            valid = sum(
                math.isfinite(value) and msg.range_min <= value <= msg.range_max
                for value in msg.ranges)
            fields = {'beams': str(len(msg.ranges)), 'valid_ranges': str(valid)}
            if not msg.ranges:
                level = ERROR
                fields['data_check'] = 'empty scan'
            elif valid == 0:
                level = WARN
                fields['data_check'] = 'no valid returns'
        elif name == 'imu':
            q = msg.orientation
            q_norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            fields = {'orientation_norm': f'{q_norm:.4f}'}
            if msg.orientation_covariance[0] == -1.0:
                level = WARN
                fields['orientation_check'] = 'orientation unavailable'
            elif not 0.8 <= q_norm <= 1.2:
                level = ERROR
                fields['orientation_check'] = 'invalid quaternion norm'
        elif name == 'gps':
            fields = {
                'status': str(msg.status.status),
                'latitude': f'{msg.latitude:.8f}',
                'longitude': f'{msg.longitude:.8f}',
            }
            if msg.status.status < 0:
                level = WARN
                fields['fix_check'] = 'no GNSS fix'
            elif not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
                level = ERROR
                fields['fix_check'] = 'non-finite coordinates'
        elif name == 'odom':
            position = msg.pose.pose.position
            velocity = msg.twist.twist.linear
            values = (
                position.x, position.y, position.z,
                velocity.x, velocity.y, velocity.z,
            )
            fields = {
                'position_xyz': (
                    f'{position.x:.3f}, {position.y:.3f}, {position.z:.3f}'),
                'speed_mps': f'{math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2):.3f}',
                'child_frame_id': msg.child_frame_id or '<empty>',
            }
            if not all(math.isfinite(value) for value in values):
                level = ERROR
                fields['odometry_check'] = 'non-finite pose or velocity'
            elif not msg.child_frame_id:
                level = WARN
                fields['odometry_check'] = 'empty child_frame_id'
        elif name == 'pointcloud':
            fields = {'width': str(msg.width), 'height': str(msg.height)}
            if msg.width == 0:
                level = ERROR
                fields['data_check'] = 'empty point cloud'
        return level, fields

    @staticmethod
    def _key_value(key, value):
        item = KeyValue()
        item.key = str(key)
        item.value = str(value)
        return item

    def _report(self):
        now = time.monotonic()
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        summaries = []
        worst = OK

        for name, tracker in self.trackers.items():
            level, message, age, rate = tracker.evaluate(
                now, self.start_sec, self.startup_grace_sec)
            level = max(level, self.data_levels[name]) if tracker.count else level
            if level > WARN and message == 'receiving':
                message = 'invalid data'
            elif level == WARN and message == 'receiving':
                message = 'receiving with warning'

            status = DiagnosticStatus()
            status.name = f'kaboat/sensors/{name}'
            status.hardware_id = 'kaboat-real'
            # Humble의 diagnostic_msgs/DiagnosticStatus.level은 IDL ``byte``라
            # Python에서 길이 1의 bytes로 대입해야 한다.
            status.level = bytes([level])
            status.message = message
            status.values = [
                self._key_value('topic', tracker.topic),
                self._key_value('messages', tracker.count),
                self._key_value('rate_hz', f'{rate:.2f}'),
                self._key_value('age_sec', f'{age:.3f}'),
                self._key_value('frame_id', tracker.frame_id or '<empty>'),
            ]
            status.values.extend(
                self._key_value(key, value)
                for key, value in self.latest_fields[name].items())
            array.status.append(status)
            worst = max(worst, level)
            summaries.append(f'{name}:{message}({rate:.1f}Hz)')

        self.diagnostics_pub.publish(array)
        summary = ' | '.join(summaries)
        if worst >= ERROR:
            self.get_logger().error(summary)
        elif worst == WARN:
            self.get_logger().warning(summary)
        else:
            self.get_logger().info(summary)


def main(args=None):
    rclpy.init(args=args)
    node = SensorHealthMonitor()
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
