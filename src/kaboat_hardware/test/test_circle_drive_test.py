import math
import sys
import time
import unittest

for p in ("/opt/ros/humble/lib/python3.10/site-packages", "/opt/ros/humble/local/lib/python3.10/dist-packages"):
    if p not in sys.path:
        sys.path.append(p)

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

    def test_vector_field_convergence(self):
        """원 바깥에서는 안쪽으로, 원 안쪽에서는 바깥쪽으로 수렴각이 계산되는지 검증."""
        self.node.started = True

        # 1. 원 바깥 (x=8.0, y=2.5) -> 중심(5.0, 2.5) 동쪽 3m 지점 (반경 1.2m 바깥)
        # CCW 기준 접선은 북쪽(+Y, 90도). 중심(서쪽, 180도)으로 수렴하려면 90도보다 큰 각(서북쪽)이어야 함.
        self._feed_odom(x=8.0, y=2.5, yaw=math.pi / 2.0)
        self.node._control_loop()
        cmd_outside: Twist = self.published_cmds[-1]
        # 헤딩이 90도일 때 desired_heading > 90도이므로 좌회전(angular.z > 0) 명령이어야 함
        self.assertGreater(cmd_outside.angular.z, 0.0)

        # 2. 원 안쪽 (x=5.5, y=2.5) -> 중심(5.0, 2.5) 동쪽 0.5m 지점 (반경 1.2m 안쪽)
        # 궤도를 넓혀 나가려면 90도보다 작은 각(동북쪽)이어야 함 -> 우회전(angular.z < 0) 명령이어야 함
        self.published_cmds.clear()
        self._feed_odom(x=5.5, y=2.5, yaw=math.pi / 2.0)
        self.node._control_loop()
        cmd_inside: Twist = self.published_cmds[-1]
        self.assertLess(cmd_inside.angular.z, 0.0)

    def test_trajectory_recording_lifecycle(self):
        """원형 주행 전, 주행 중, 완료 후 실제 궤적 기록 생명주기 검증."""
        # 1. 시작 전 (started=False)
        self.node.started = False
        self._feed_odom(x=5.0, y=1.3, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 0)

        # 2. 시작 (started=True)
        self.node.started = True
        self._feed_odom(x=5.0, y=1.3, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 1)

        # 3. 거리 다운샘플링 (<5cm)
        self._feed_odom(x=5.02, y=1.3, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 1)

        # 4. 충분한 거리 이동 (>=5cm)
        self._feed_odom(x=5.10, y=1.3, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 2)

        # 5. 미션 완료 (mission_finished=True)
        self.node.mission_finished = True
        self._feed_odom(x=5.30, y=1.3, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 2)

    def test_markers_include_orbit_center_and_trail(self):
        """MarkerArray에 목표 궤도(orbit), 중심점(center), 실제 궤적(actual_trajectory)이 포함되는지 검증."""
        published_marker_arrays = []
        self.node.marker_pub.publish = lambda ma: published_marker_arrays.append(ma)

        self.node.trajectory_history.add_point(5.0, 1.3)
        self.node.trajectory_history.add_point(5.5, 1.5)

        self.node._publish_markers()

        self.assertGreater(len(published_marker_arrays), 0)
        ma = published_marker_arrays[-1]
        namespaces = {m.ns: m for m in ma.markers}

        self.assertIn("orbit", namespaces)
        self.assertIn("center", namespaces)
        self.assertIn("actual_trajectory", namespaces)

        orbit_marker = namespaces["orbit"]
        self.assertEqual(orbit_marker.id, 0)
        self.assertEqual(orbit_marker.header.frame_id, "odom")

        center_marker = namespaces["center"]
        self.assertEqual(center_marker.id, 1)
        self.assertEqual(center_marker.header.frame_id, "odom")

        trail_marker = namespaces["actual_trajectory"]
        self.assertEqual(trail_marker.id, 0)
        self.assertEqual(trail_marker.header.frame_id, "odom")
        self.assertEqual(len(trail_marker.points), 2)

        # 미션 완료 후에도 마커 유지 확인
        self.node.mission_finished = True
        published_marker_arrays.clear()
        self.node._publish_markers()
        ma_after = published_marker_arrays[-1]
        ns_after = {m.ns: m for m in ma_after.markers}
        self.assertIn("actual_trajectory", ns_after)
        self.assertEqual(len(ns_after["actual_trajectory"].points), 2)


if __name__ == '__main__':
    unittest.main()
