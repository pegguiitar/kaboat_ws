import math
import sys
import unittest

for p in ("/opt/ros/humble/lib/python3.10/site-packages", "/opt/ros/humble/local/lib/python3.10/dist-packages"):
    if p not in sys.path:
        sys.path.append(p)

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

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

    def test_trajectory_recording_lifecycle(self):
        """정점 유지 전, 유지 중, 완료 후 실제 궤적 기록 생명주기 검증."""
        # 1. 시작 전 (started=False)
        self.node.started = False
        self._feed_odom(x=4.0, y=2.5, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 0)

        # 2. 시작 (started=True)
        self.node.started = True
        self._feed_odom(x=4.0, y=2.5, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 1)

        # 3. 거리 다운샘플링 (<5cm)
        self._feed_odom(x=4.02, y=2.5, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 1)

        # 4. 충분한 거리 이동 (>=5cm)
        self._feed_odom(x=4.10, y=2.5, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 2)

        # 5. 미션 완료 (mission_finished=True)
        self.node.mission_finished = True
        self._feed_odom(x=4.30, y=2.5, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 2)

    def test_markers_include_waypoint_deadband_heading_and_trail(self):
        """MarkerArray에 목표점(waypoint), 불감대(deadband), 헤딩 화살표(heading), 실제 궤적(actual_trajectory)이 포함되는지 검증."""
        published_marker_arrays = []
        self.node.marker_pub.publish = lambda ma: published_marker_arrays.append(ma)

        self.node.trajectory_history.add_point(4.0, 2.5)
        self.node.trajectory_history.add_point(4.5, 2.5)

        self.node._publish_markers()

        self.assertGreater(len(published_marker_arrays), 0)
        ma = published_marker_arrays[-1]
        namespaces = {m.ns: m for m in ma.markers}

        self.assertIn("waypoint", namespaces)
        self.assertIn("deadband", namespaces)
        self.assertIn("heading", namespaces)
        self.assertIn("actual_trajectory", namespaces)

        wp_marker = namespaces["waypoint"]
        self.assertEqual(wp_marker.id, 0)
        self.assertEqual(wp_marker.header.frame_id, "odom")

        deadband_marker = namespaces["deadband"]
        self.assertEqual(deadband_marker.id, 1)
        self.assertEqual(deadband_marker.header.frame_id, "odom")
        self.assertGreater(len(deadband_marker.points), 10)

        heading_marker = namespaces["heading"]
        self.assertEqual(heading_marker.id, 2)
        self.assertEqual(heading_marker.header.frame_id, "odom")

        trail_marker = namespaces["actual_trajectory"]
        self.assertEqual(trail_marker.id, 0)
        self.assertEqual(trail_marker.header.frame_id, "odom")
        self.assertEqual(len(trail_marker.points), 2)
        self.assertEqual(trail_marker.lifetime.sec, 1)
        self.assertEqual(trail_marker.lifetime.nanosec, 0)

        # 목표 시각화 마커 수명 무한(0) 유지 검증
        self.assertEqual(wp_marker.lifetime.sec, 0)
        self.assertEqual(wp_marker.lifetime.nanosec, 0)
        self.assertEqual(deadband_marker.lifetime.sec, 0)
        self.assertEqual(deadband_marker.lifetime.nanosec, 0)
        self.assertEqual(heading_marker.lifetime.sec, 0)
        self.assertEqual(heading_marker.lifetime.nanosec, 0)

        # 미션 완료 후에도 마커 유지 확인
        self.node.mission_finished = True
        published_marker_arrays.clear()
        self.node._publish_markers()
        ma_after = published_marker_arrays[-1]
        ns_after = {m.ns: m for m in ma_after.markers}
        self.assertIn("actual_trajectory", ns_after)
        self.assertEqual(len(ns_after["actual_trajectory"].points), 2)
        self.assertEqual(ns_after["actual_trajectory"].lifetime.sec, 1)
        self.assertEqual(ns_after["actual_trajectory"].lifetime.nanosec, 0)

    def test_clear_trajectory_service_registered(self):
        """노드에 /clear_trajectory 서비스가 등록되어 있는지 검증."""
        service_names = [s.srv_name for s in self.node.services]
        self.assertIn('/clear_trajectory', service_names)

    def test_clear_trajectory_clears_populated_trail_and_response_content(self):
        """/clear_trajectory 호출 시 기록된 궤적이 비워지고 올바른 응답을 반환하는지 검증."""
        self.node.trajectory_history.add_point(4.0, 2.5)
        self.node.trajectory_history.add_point(4.5, 2.5)
        self.assertEqual(len(self.node.trajectory_history), 2)

        req = Trigger.Request()
        resp = Trigger.Response()
        result = self.node._on_clear_trajectory(req, resp)

        self.assertTrue(result.success)
        self.assertGreater(len(result.message), 0)
        self.assertEqual(len(self.node.trajectory_history), 0)
        self.assertTrue(self.node.trajectory_history.is_empty())

    def test_clear_trajectory_immediate_empty_marker_and_preserves_target_markers(self):
        """/clear_trajectory 호출 즉시 빈 actual_trajectory 마커와 보존된 목표 마커들이 발행되는지 검증."""
        published_marker_arrays = []
        self.node.marker_pub.publish = lambda ma: published_marker_arrays.append(ma)

        self.node.trajectory_history.add_point(4.0, 2.5)
        self.node.trajectory_history.add_point(4.5, 2.5)

        req = Trigger.Request()
        resp = Trigger.Response()
        self.node._on_clear_trajectory(req, resp)

        self.assertEqual(len(published_marker_arrays), 1)
        ma = published_marker_arrays[0]
        namespaces = {m.ns: m for m in ma.markers}

        self.assertIn("actual_trajectory", namespaces)
        self.assertEqual(len(namespaces["actual_trajectory"].points), 0)

        # 웨이포인트, 불감대, 헤딩 마커 보존 확인
        self.assertIn("waypoint", namespaces)
        self.assertIn("deadband", namespaces)
        self.assertIn("heading", namespaces)

    def test_clear_trajectory_preserves_mission_and_control_state_and_no_cmd_vel(self):
        """/clear_trajectory 호출 시 정점 유지 타이머 및 제어 상태가 보존되고 cmd_vel이 발행되지 않는지 검증."""
        self.node.started = True
        self.node.mission_finished = False
        self.node.emergency_stopped = False
        self.node.hold_start_time = 1234.5
        self.node.target_x = 5.0
        self.node.target_y = 2.5
        self.node.target_yaw = math.pi

        self.published_cmds.clear()
        req = Trigger.Request()
        resp = Trigger.Response()
        self.node._on_clear_trajectory(req, resp)

        self.assertTrue(self.node.started)
        self.assertFalse(self.node.mission_finished)
        self.assertFalse(self.node.emergency_stopped)
        self.assertEqual(self.node.hold_start_time, 1234.5)
        self.assertEqual(self.node.target_x, 5.0)
        self.assertEqual(self.node.target_y, 2.5)
        self.assertEqual(self.node.target_yaw, math.pi)
        self.assertEqual(len(self.published_cmds), 0)

    def test_clear_trajectory_records_new_trail_on_next_odom(self):
        """정점 유지 미션 활성 상태에서 궤적 초기화 후 다음 오도메트리가 새로운 궤적의 첫 점으로 기록되는지 검증."""
        self.node.started = True
        self.node.mission_finished = False

        self._feed_odom(x=4.0, y=2.5, yaw=0.0)
        self._feed_odom(x=4.1, y=2.5, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 2)

        # 궤적 초기화
        req = Trigger.Request()
        resp = Trigger.Response()
        self.node._on_clear_trajectory(req, resp)
        self.assertEqual(len(self.node.trajectory_history), 0)

        # 초기화 직후 동일 위치라도 첫 점은 즉시 새 궤적의 시작점으로 기록되어야 함
        self._feed_odom(x=4.1, y=2.5, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 1)
        self.assertAlmostEqual(self.node.trajectory_history[0][0], 4.1)

        # 이후 거리 판정 기준 충족 시 추가 기록
        self._feed_odom(x=4.2, y=2.5, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 2)


if __name__ == '__main__':
    unittest.main()
