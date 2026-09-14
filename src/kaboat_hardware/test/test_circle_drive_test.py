import math
import time
import unittest

import rclpy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from kaboat_hardware.circle_drive_test import (
    CircleDriveTest, normalize_angle, yaw_to_quaternion
)


class TestCircleDriveTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = CircleDriveTest()
        self.published_cmds = []

        # cmd_pub 모킹
        self.node.cmd_pub.publish = lambda msg: self.published_cmds.append(msg)

    def tearDown(self):
        self.node.destroy_node()

    def _feed_odom(self, x: float, y: float, yaw: float, yaw_rate: float = 0.0):
        msg = Odometry()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = yaw_to_quaternion(yaw)
        msg.twist.twist.angular.z = yaw_rate
        self.node._on_odom(msg)

    def test_normalize_angle(self):
        self.assertAlmostEqual(normalize_angle(0.0), 0.0)
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi)
        self.assertAlmostEqual(normalize_angle(-3.0 * math.pi), -math.pi)

    def test_initial_state_and_wait(self):
        """시작 신호 대기 중에는 모터 출력이 0인지 확인."""
        self.assertTrue(self.node.wait_for_start)
        self.assertFalse(self.node.started)

        # odom 입력 후 루프 실행
        self._feed_odom(x=5.0, y=1.3, yaw=0.0)
        self.node._control_loop()

        self.assertGreater(len(self.published_cmds), 0)
        latest_cmd: Twist = self.published_cmds[-1]
        self.assertEqual(latest_cmd.linear.x, 0.0)
        self.assertEqual(latest_cmd.angular.z, 0.0)

    def test_start_mission_and_control(self):
        """출발 신호 수신 후 선회 제어 출력 발행 검증."""
        start_msg = Bool()
        start_msg.data = True
        self.node._on_start_mission(start_msg)
        self.assertTrue(self.node.started)

        self.published_cmds.clear()
        # 원 하단(5.0, 1.3)에서 +X 방향(yaw=0)으로 진행 중
        self._feed_odom(x=5.0, y=1.3, yaw=0.0)
        self.node._control_loop()

        self.assertGreater(len(self.published_cmds), 0)
        latest_cmd: Twist = self.published_cmds[-1]
        self.assertGreater(latest_cmd.linear.x, 0.0)

    def test_start_service(self):
        """/start_test 서비스 호출로 출발 활성화 확인."""
        class MockResponse:
            success = False
            message = ""

        resp = self.node._on_start_service(None, MockResponse())
        self.assertTrue(resp.success)
        self.assertTrue(self.node.started)

    def test_lap_completion_stop(self):
        """목표 바퀴 수 완주 시 안전 제동 및 종료 확인."""
        self.node.started = True
        self.node.target_laps = 1.0

        # 원 1바퀴를 8개 점으로 순회 피딩 (반시계)
        num_steps = 16
        for i in range(num_steps + 2):
            th = 2.0 * math.pi * i / num_steps
            px = 5.0 + 1.2 * math.cos(th)
            py = 2.5 + 1.2 * math.sin(th)
            yaw = th + math.pi / 2.0
            self._feed_odom(x=px, y=py, yaw=yaw)
            self.node._control_loop()

        self.assertTrue(self.node.mission_finished)
        latest_cmd: Twist = self.published_cmds[-1]
        self.assertEqual(latest_cmd.linear.x, 0.0)
        self.assertEqual(latest_cmd.angular.z, 0.0)

    def test_emergency_stop(self):
        """/emergency_stop 수신 시 즉각 제동 검증."""
        self.node.started = True
        self._feed_odom(x=5.0, y=1.3, yaw=0.0)

        estop_msg = Bool()
        estop_msg.data = True
        self.node._on_estop(estop_msg)

        self.published_cmds.clear()
        self.node._control_loop()

        self.assertGreater(len(self.published_cmds), 0)
        latest_cmd: Twist = self.published_cmds[-1]
        self.assertEqual(latest_cmd.linear.x, 0.0)
        self.assertEqual(latest_cmd.angular.z, 0.0)


if __name__ == '__main__':
    unittest.main()
