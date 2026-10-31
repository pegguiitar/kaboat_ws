#!/usr/bin/env python3
"""Print real-time boat roll and pitch from /imu/data with tilt threshold warning."""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

THRESHOLD_DEG = 5.0


def quaternion_to_euler(x: float, y: float, z: float, w: float):
    # Roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)

    # Yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class RollPitchViewer(Node):
    def __init__(self):
        super().__init__('roll_pitch_viewer')
        self.sub = self.create_subscription(
            Imu, '/imu/data', self._on_imu, qos_profile_sensor_data)
        self.last_msg_time = None
        self.msg_count = 0
        self.get_logger().info('대기 중: /imu/data 토픽 수신을 기다립니다...')

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.msg_count += 1
        self.last_msg_time = time.monotonic()

        tilted = abs(roll) > THRESHOLD_DEG or abs(pitch) > THRESHOLD_DEG
        status = "\033[91m[ ⚠️ TILTED > 5° (스캔 차단됨) ]\033[0m" if tilted else "\033[92m[ OK (정상 주행) ]\033[0m"

        output = (f"\r\033[K"
                  f"Roll: {roll:+6.2f}° | "
                  f"Pitch: {pitch:+6.2f}° | "
                  f"Yaw: {yaw:+6.2f}° | "
                  f"상태: {status}")
        sys.stdout.write(output)
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = RollPitchViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n종료합니다.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

