"""실내 수조 시험 시작 전 정지 gyro bias를 계산하고 LiDAR yaw로 정렬한다.

먼저 LiDAR 두 봉 추적과 indoor_tank.launch.py를 실행하고 선체를 어느 방향이든
움직이지 않게 고정한 뒤 호출한다. 실제 보정 계산과 적용은
indoor_lidar_odom 노드가 담당하며, 이 클라이언트는 충분한 정지 표본이 모일
때까지 서비스를 반복 호출한다.
"""

import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class IndoorImuCalibrationClient(Node):
    def __init__(self):
        super().__init__('calibrate_indoor_imu')
        self.declare_parameter('service_name', '/calibrate_imu_yaw')
        self.declare_parameter('timeout_sec', 30.0)
        self.declare_parameter('retry_interval_sec', 0.5)

        self.service_name = str(self.get_parameter('service_name').value)
        self.timeout_sec = max(1.0, float(self.get_parameter('timeout_sec').value))
        self.retry_interval_sec = max(
            0.1, float(self.get_parameter('retry_interval_sec').value))
        self.client = self.create_client(Trigger, self.service_name)

    def calibrate(self) -> bool:
        deadline = time.monotonic() + self.timeout_sec
        self.get_logger().info(
            '선체를 움직이지 마세요. LiDAR 절대 yaw와 안정된 gyro 표본을 기다립니다.')

        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            wait_time = max(0.05, min(1.0, remaining))
            if not self.client.wait_for_service(timeout_sec=wait_time):
                self.get_logger().info(
                    f"'{self.service_name}' 서비스를 기다리는 중입니다. "
                    'indoor_tank.launch.py가 실행 중인지 확인하세요.')
                continue

            future = self.client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=min(2.0, max(0.1, remaining)))
            if not future.done():
                future.cancel()
                self.get_logger().warn('yaw 보정 서비스 응답을 기다리는 중입니다.')
            elif future.exception() is not None:
                self.get_logger().error(
                    f'yaw 보정 서비스 호출 실패: {future.exception()}')
            else:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'✅ {response.message}')
                    return True
                self.get_logger().info(response.message)

            time.sleep(min(self.retry_interval_sec, max(0.0, deadline - time.monotonic())))

        self.get_logger().error(
            f'{self.timeout_sec:.1f}초 안에 yaw 보정을 완료하지 못했습니다. '
            'IMU 토픽, 센서 상태 및 선체 고정 상태를 확인하세요.')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = IndoorImuCalibrationClient()
    try:
        return 0 if node.calibrate() else 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
