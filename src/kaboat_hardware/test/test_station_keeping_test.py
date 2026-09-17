import math
import unittest

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from kaboat_hardware.station_keeping_test import (
    StationKeepingTest, normalize_angle, yaw_to_quaternion
)


class TestStationKeepingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = StationKeepingTest()
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

    def test_wait_for_start(self):
        """시작 신호 전에는 모터 출력 0 확인."""
        self.assertTrue(self.node.wait_for_start)
        self.assertFalse(self.node.started)

        self._feed_odom(x=4.0, y=2.5, yaw=0.0)
        self.node._control_loop()

        self.assertGreater(len(self.published_cmds), 0)
        latest_cmd: Twist = self.published_cmds[-1]
        self.assertEqual(latest_cmd.linear.x, 0.0)
        self.assertEqual(latest_cmd.angular.z, 0.0)

    def test_start_and_drive_towards_target(self):
        """이격 위치(4.0, 2.5)에서 목표(5.0, 2.5)를 향해 전진 명령 검증."""
        start_msg = Bool()
        start_msg.data = True
        self.node._on_start_mission(start_msg)
        self.assertTrue(self.node.started)

        self.published_cmds.clear()
        # 목표점은 (5.0, 2.5), 현재 (4.0, 2.5)에서 동쪽(+X, yaw=0)을 바라보고 있음
        self._feed_odom(x=4.0, y=2.5, yaw=0.0)
        self.node._control_loop()

        self.assertGreater(len(self.published_cmds), 0)
        latest_cmd: Twist = self.published_cmds[-1]
        # 목표를 향해 전진해야 함
        self.assertGreater(latest_cmd.linear.x, 0.0)

    def test_deadband_idle(self):
        """목표점 불감대(50cm, 8°) 안착 시 모터 완전 정지(휴지) 검증."""
        self.node.started = True
        self.node.target_x = 5.0
        self.node.target_y = 2.5
        self.node.target_yaw = math.pi  # 180도

        self.published_cmds.clear()
        # 오차: 위치 2cm, 각도 2도 (불감대 내부)
        self._feed_odom(x=5.02, y=2.51, yaw=math.pi + math.radians(2.0))
        self.node._control_loop()

        self.assertGreater(len(self.published_cmds), 0)
        latest_cmd: Twist = self.published_cmds[-1]
        self.assertEqual(latest_cmd.linear.x, 0.0)
        self.assertEqual(latest_cmd.angular.z, 0.0)

    def test_deadband_idle_within_0_5m(self):
        """목표점 반경 0.5m 이내(예: 40cm 이격) 진입 시 정점 안착(출력 0) 성공 판정 검증."""
        self.node.started = True
        self.node.target_x = 5.0
        self.node.target_y = 2.5
        self.node.target_yaw = math.pi  # 180도

        self.published_cmds.clear()
        # 오차: 위치 40cm (과거 15cm 기준에서는 이탈이었으나 50cm 기준에서는 불감대 안착), 각도 일치
        self._feed_odom(x=5.40, y=2.5, yaw=math.pi)
        self.node._control_loop()

        self.assertGreater(len(self.published_cmds), 0)
        latest_cmd: Twist = self.published_cmds[-1]
        self.assertEqual(latest_cmd.linear.x, 0.0)
        self.assertEqual(latest_cmd.angular.z, 0.0)

    def test_reverse_on_slight_overshoot(self):
        """목표점 통과 후 0.65m 이격(후방 영역) 시 180도 선회 대신 후진 복귀 명령 검증."""
        self.node.started = True
        self.node.target_x = 5.0
        self.node.target_y = 2.5
        self.node.target_yaw = math.pi  # 180도

        self.published_cmds.clear()
        # 목표(5.0, 2.5)를 지나쳐 (4.35, 2.5)에 위치하고 선수각 180도(서쪽) 유지 중
        # 목표점은 동쪽(+X)에 있으므로 후방 영역(rel_bearing ~ 180도), 이격거리 0.65m < 0.80m
        self._feed_odom(x=4.35, y=2.5, yaw=math.pi)
        self.node._control_loop()

        self.assertGreater(len(self.published_cmds), 0)
        latest_cmd: Twist = self.published_cmds[-1]
        # 후진 명령(-linear.x)이 발행되어야 함
        self.assertLess(latest_cmd.linear.x, 0.0)

    def test_heading_correction_in_pos_deadband(self):
        """위치는 불감대 안이지만 각도가 틀어졌을 때 제자리 회전 제어 검증."""
        self.node.started = True
        self.node.target_x = 5.0
        self.node.target_y = 2.5
        self.node.target_yaw = math.pi  # 180도

        self.published_cmds.clear()
        # 위치는 정확히 (5.0, 2.5)이지만, 선수가 150도 (30도 오차 > 8도 불감대)
        self._feed_odom(x=5.0, y=2.5, yaw=math.radians(150.0))
        self.node._control_loop()

        self.assertGreater(len(self.published_cmds), 0)
        latest_cmd: Twist = self.published_cmds[-1]
        # 전진은 0이고 회전만 발생해야 함
        self.assertEqual(latest_cmd.linear.x, 0.0)
        self.assertNotEqual(latest_cmd.angular.z, 0.0)

    def test_emergency_stop(self):
        """/emergency_stop 수신 시 즉각 제동 검증."""
        self.node.started = True
        self._feed_odom(x=4.0, y=2.5, yaw=0.0)

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
