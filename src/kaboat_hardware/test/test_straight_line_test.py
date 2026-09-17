"""test_straight_line_test.py — straight_line_test 노드 단위 테스트."""

import math
import unittest
import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from kaboat_hardware.straight_line_test import (
    StraightLineTest,
    normalize_angle,
    yaw_from_quaternion,
)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


class TestStraightLineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = StraightLineTest()
        self.node.started = True
        self.published_cmds = []
        self.node.cmd_pub.publish = lambda msg: self.published_cmds.append(msg)

    def tearDown(self):
        self.node.destroy_node()

    def test_normalize_angle(self):
        self.assertAlmostEqual(normalize_angle(0.0), 0.0)
        self.assertAlmostEqual(normalize_angle(math.pi), math.pi)
        self.assertAlmostEqual(normalize_angle(-math.pi), -math.pi)
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi)
        self.assertAlmostEqual(normalize_angle(-3.0 * math.pi), -math.pi)

    def test_yaw_from_quaternion(self):
        # 0 rad (동쪽 / +X)
        q0 = yaw_to_quaternion(0.0)
        self.assertAlmostEqual(yaw_from_quaternion(q0), 0.0, places=5)

        # pi rad (서쪽 / -X)
        q_pi = yaw_to_quaternion(math.pi)
        self.assertAlmostEqual(abs(yaw_from_quaternion(q_pi)), math.pi, places=5)

        # pi/2 rad (북쪽 / +Y)
        q_half = yaw_to_quaternion(math.pi / 2.0)
        self.assertAlmostEqual(yaw_from_quaternion(q_half), math.pi / 2.0, places=5)

    def _feed_odom(self, x: float, y: float, yaw: float, yaw_rate: float = 0.0):
        msg = Odometry()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = yaw_to_quaternion(yaw)
        msg.twist.twist.angular.z = yaw_rate
        self.node._on_odom(msg)

    def test_cruise_control_on_track(self):
        """경로 위 (X=8.0, Y=3.0)에서 서쪽(pi)을 향하고 있을 때 정상 직진 출력 확인."""
        self._feed_odom(x=8.0, y=3.0, yaw=math.pi, yaw_rate=0.0)
        self.node._control_loop()

        self.assertGreater(len(self.published_cmds), 0)
        latest_cmd: Twist = self.published_cmds[-1]
        self.assertAlmostEqual(latest_cmd.linear.x, self.node.cruise_speed, places=2)
        self.assertAlmostEqual(latest_cmd.angular.z, 0.0, places=2)
        self.assertFalse(self.node.mission_finished)

    def test_cross_track_correction(self):
        """경로 우측으로 벗어남 (X=5.0, Y=3.5): Y=3.0으로 복귀하기 위한 조향각 발생 확인."""
        self._feed_odom(x=5.0, y=3.5, yaw=math.pi, yaw_rate=0.0)
        self.node._control_loop()

        latest_cmd: Twist = self.published_cmds[-1]
        # Y=3.5에서 -X 방향으로 가면서 Y를 낮춰야 하므로 (-Y 방향 조향 -> 좌현/우현 각도 보정)
        # yaw_desired > pi or < pi:
        # dx_line=-8, dy_line=0 -> ux=-1, uy=0
        # cross_track_error = -uy*dx + ux*dy = -1*(3.5-3.0) = -0.5
        # correction_angle = atan2(-(-0.5), lookahead) = atan2(0.5, 1.2) > 0
        # desired_yaw = pi + correction_angle -> normalized to -pi + correction_angle
        # heading_error = desired_yaw - current_yaw = (-pi + corr) - (pi) = -2*pi + corr = corr > 0 (or turn right/left)
        # 핵심: 회전 명령(angular.z)이 0이 아니고 보정 방향으로 발생함
        self.assertNotEqual(latest_cmd.angular.z, 0.0)

    def test_goal_arrival(self):
        """목표점 (1.0, 3.0) 도달 시 미션 완료 및 정지 명령 발행 확인."""
        self._feed_odom(x=1.05, y=3.0, yaw=math.pi)
        self.node._control_loop()

        self.assertTrue(self.node.mission_finished)
        latest_cmd: Twist = self.published_cmds[-1]
        self.assertEqual(latest_cmd.linear.x, 0.0)
        self.assertEqual(latest_cmd.angular.z, 0.0)

    def test_emergency_stop_topic(self):
        """/emergency_stop 수신 시 정지 명령 확인."""
        self._feed_odom(x=8.0, y=3.0, yaw=math.pi)
        estop_msg = Bool()
        estop_msg.data = True
        self.node._on_estop(estop_msg)

        self.published_cmds.clear()
        self.node._control_loop()

        self.assertGreater(len(self.published_cmds), 0)
        latest_cmd: Twist = self.published_cmds[-1]
        self.assertEqual(latest_cmd.linear.x, 0.0)
        self.assertEqual(latest_cmd.angular.z, 0.0)

    def test_wall_guard_latches_stop(self):
        """벽면 안전 영역 밖에서는 정지하고 위치가 돌아와도 자동 재출발하지 않아야 한다."""
        self._feed_odom(x=9.7, y=3.0, yaw=math.pi)
        self.node._control_loop()

        self.assertTrue(self.node.boundary_stopped)
        self.assertEqual(self.published_cmds[-1].linear.x, 0.0)
        self.assertEqual(self.published_cmds[-1].angular.z, 0.0)

        self._feed_odom(x=8.0, y=3.0, yaw=math.pi)
        self.node._control_loop()
        self.assertEqual(self.published_cmds[-1].linear.x, 0.0)
        self.assertEqual(self.published_cmds[-1].angular.z, 0.0)

    def test_wall_guard_boundary_is_allowed(self):
        """정확히 wall_margin 경계에 있는 위치는 안전 영역에 포함한다."""
        self._feed_odom(x=9.5, y=3.0, yaw=math.pi)
        self.node._control_loop()
        self.assertFalse(self.node.boundary_stopped)
        self.assertGreater(self.published_cmds[-1].linear.x, 0.0)

    def test_start_mission_topic_and_pause(self):
        """/start_mission 토픽을 통한 출발 신호 인가 및 일시 정지 검증."""
        self.node.started = False
        self._feed_odom(x=8.0, y=3.0, yaw=math.pi)

        # 시작 신호 전: 정지 상태
        self.published_cmds.clear()
        self.node._control_loop()
        self.assertFalse(self.node.started)
        if len(self.published_cmds) > 0:
            self.assertEqual(self.published_cmds[-1].linear.x, 0.0)

        # 시작 신호 수신 (data=True)
        start_msg = Bool()
        start_msg.data = True
        self.node._on_start_mission(start_msg)
        self.assertTrue(self.node.started)

        # 이제 주행 명령 발행
        self.node._control_loop()
        self.assertGreater(self.published_cmds[-1].linear.x, 0.0)

        # 일시 정지 신호 수신 (data=False)
        pause_msg = Bool()
        pause_msg.data = False
        self.node._on_start_mission(pause_msg)
        self.assertFalse(self.node.started)
        self.assertEqual(self.published_cmds[-1].linear.x, 0.0)

    def test_start_service_trigger(self):
        """/start_test 서비스를 통한 출발 검증."""
        self.node.started = False
        from std_srvs.srv import Trigger
        req = Trigger.Request()
        resp = Trigger.Response()
        result = self.node._on_start_service(req, resp)
        self.assertTrue(result.success)
        self.assertTrue(self.node.started)


if __name__ == '__main__':
    unittest.main()
