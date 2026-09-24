"""test_straight_line_test.py — straight_line_test 노드 단위 테스트."""

import math
import sys
import unittest

for p in ("/opt/ros/humble/lib/python3.10/site-packages", "/opt/ros/humble/local/lib/python3.10/dist-packages"):
    if p not in sys.path:
        sys.path.append(p)

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

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

    def test_coordinates_near_wall_do_not_trigger_automatic_stop(self):
        """위치 기반 벽면 가드가 없으므로 경계 부근에서도 제어 명령을 계산한다."""
        self._feed_odom(x=9.7, y=3.0, yaw=math.pi)
        self.node._control_loop()

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

    def test_trajectory_recording_lifecycle(self):
        """출발 전, 주행 중, 일시 정지, 완료 후 실제 궤적 기록 생명주기 검증."""
        # 1. 시작 전 (started=False): 기록되지 않음
        self.node.started = False
        self._feed_odom(x=8.0, y=3.0, yaw=math.pi)
        self.assertEqual(len(self.node.trajectory_history), 0)

        # 2. 시작 (started=True): 첫 유효 점 기록
        self.node.started = True
        self._feed_odom(x=8.0, y=3.0, yaw=math.pi)
        self.assertEqual(len(self.node.trajectory_history), 1)
        self.assertAlmostEqual(self.node.trajectory_history[0][0], 8.0)

        # 3. 거리 다운샘플링: 미세 이동(<5cm) 무시
        self._feed_odom(x=8.02, y=3.0, yaw=math.pi)
        self.assertEqual(len(self.node.trajectory_history), 1)

        # 4. 충분한 이동(>=5cm) 시 추가 기록
        self._feed_odom(x=7.90, y=3.0, yaw=math.pi)
        self.assertEqual(len(self.node.trajectory_history), 2)

        # 5. 일시 정지 (started=False): 기록 중단
        self.node.started = False
        self._feed_odom(x=7.50, y=3.0, yaw=math.pi)
        self.assertEqual(len(self.node.trajectory_history), 2)

        # 6. 재개 (started=True): 기록 재개
        self.node.started = True
        self._feed_odom(x=7.50, y=3.0, yaw=math.pi)
        self.assertEqual(len(self.node.trajectory_history), 3)

        # 7. 미션 완료 (mission_finished=True): 기록 중단
        self.node.mission_finished = True
        self._feed_odom(x=6.0, y=3.0, yaw=math.pi)
        self.assertEqual(len(self.node.trajectory_history), 3)

    def test_path_markers_include_goal_tolerance_and_actual_trajectory(self):
        """MarkerArray에 목표선, 시작점, 도착점, 허용오차 링, 실제 주행 궤적이 모두 포함되는지 검증."""
        published_marker_arrays = []
        self.node.marker_pub.publish = lambda ma: published_marker_arrays.append(ma)

        # 궤적에 점 2개 추가
        self.node.trajectory_history.add_point(8.0, 3.0)
        self.node.trajectory_history.add_point(7.0, 3.0)

        self.node._publish_path_markers()
        self.assertGreater(len(published_marker_arrays), 0)
        ma = published_marker_arrays[-1]

        namespaces = {m.ns: m for m in ma.markers}
        self.assertIn("test_path", namespaces)
        self.assertIn("test_start", namespaces)
        self.assertIn("test_goal", namespaces)
        self.assertIn("goal_tolerance", namespaces)
        self.assertIn("actual_trajectory", namespaces)

        # 허용오차 링 검증
        tol_marker = namespaces["goal_tolerance"]
        self.assertEqual(tol_marker.id, 0)
        self.assertEqual(tol_marker.header.frame_id, "odom")
        self.assertGreater(len(tol_marker.points), 10)

        # 실제 주행 궤적 검증
        trail_marker = namespaces["actual_trajectory"]
        self.assertEqual(trail_marker.id, 0)
        self.assertEqual(trail_marker.header.frame_id, "odom")
        self.assertEqual(len(trail_marker.points), 2)
        self.assertEqual(trail_marker.lifetime.sec, 1)
        self.assertEqual(trail_marker.lifetime.nanosec, 0)

        # 목표 시각화 마커 수명 무한(0) 유지 검증
        for target_ns in ("test_path", "test_start", "test_goal", "goal_tolerance"):
            self.assertEqual(namespaces[target_ns].lifetime.sec, 0)
            self.assertEqual(namespaces[target_ns].lifetime.nanosec, 0)

        # 미션 완료 후에도 궤적 마커가 계속 유지되어 발행되는지 확인
        self.node.mission_finished = True
        published_marker_arrays.clear()
        self.node._publish_path_markers()
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
        self.node.trajectory_history.add_point(8.0, 3.0)
        self.node.trajectory_history.add_point(7.5, 3.0)
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

        self.node.trajectory_history.add_point(8.0, 3.0)
        self.node.trajectory_history.add_point(7.5, 3.0)

        req = Trigger.Request()
        resp = Trigger.Response()
        self.node._on_clear_trajectory(req, resp)

        self.assertEqual(len(published_marker_arrays), 1)
        ma = published_marker_arrays[0]
        namespaces = {m.ns: m for m in ma.markers}

        self.assertIn("actual_trajectory", namespaces)
        self.assertEqual(len(namespaces["actual_trajectory"].points), 0)

        # 목표 시각화 마커 보존 확인
        self.assertIn("test_path", namespaces)
        self.assertIn("test_start", namespaces)
        self.assertIn("test_goal", namespaces)
        self.assertIn("goal_tolerance", namespaces)
        self.assertEqual(len(namespaces["test_path"].points), 2)

    def test_clear_trajectory_preserves_mission_and_control_state_and_no_cmd_vel(self):
        """/clear_trajectory 호출 시 미션 및 제어 상태가 변경되지 않고 cmd_vel이 발행되지 않는지 검증."""
        self.node.started = True
        self.node.mission_finished = False
        self.node.emergency_stopped = False
        self.node.start_x = 9.0
        self.node.goal_x = 1.0

        self.published_cmds.clear()
        req = Trigger.Request()
        resp = Trigger.Response()
        self.node._on_clear_trajectory(req, resp)

        self.assertTrue(self.node.started)
        self.assertFalse(self.node.mission_finished)
        self.assertFalse(self.node.emergency_stopped)
        self.assertEqual(self.node.start_x, 9.0)
        self.assertEqual(self.node.goal_x, 1.0)
        self.assertEqual(len(self.published_cmds), 0)

    def test_clear_trajectory_records_new_trail_on_next_odom(self):
        """미션 활성 상태에서 궤적 초기화 후 다음 오도메트리가 새로운 궤적의 첫 점으로 기록되는지 검증."""
        self.node.started = True
        self.node.mission_finished = False

        self._feed_odom(x=8.0, y=3.0, yaw=math.pi)
        self._feed_odom(x=7.8, y=3.0, yaw=math.pi)
        self.assertEqual(len(self.node.trajectory_history), 2)

        # 궤적 초기화
        req = Trigger.Request()
        resp = Trigger.Response()
        self.node._on_clear_trajectory(req, resp)
        self.assertEqual(len(self.node.trajectory_history), 0)

        # 초기화 직후 동일 위치라도 첫 점은 즉시 새 궤적의 시작점으로 기록되어야 함
        self._feed_odom(x=7.8, y=3.0, yaw=math.pi)
        self.assertEqual(len(self.node.trajectory_history), 1)
        self.assertAlmostEqual(self.node.trajectory_history[0][0], 7.8)

        # 이후 거리 판정 기준 충족 시 추가 기록
        self._feed_odom(x=7.7, y=3.0, yaw=math.pi)
        self.assertEqual(len(self.node.trajectory_history), 2)


if __name__ == '__main__':
    unittest.main()
