"""test_bspline_track_test.py — B-Spline 곡선 경로 생성 및 추종 노드 단위 테스트."""

import math
import sys
import unittest
import numpy as np
from scipy.interpolate import BSpline

for p in ("/opt/ros/humble/lib/python3.10/site-packages", "/opt/ros/humble/local/lib/python3.10/dist-packages"):
    if p not in sys.path:
        sys.path.append(p)

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from kaboat_hardware.bspline_track_test import (
    BSplineTrackTest,
    generate_bspline_path,
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


class TestBSplineTrackTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = BSplineTrackTest()
        self.node.started = True
        self.published_cmds = []
        self.node.cmd_pub.publish = lambda msg: self.published_cmds.append(msg)

    def tearDown(self):
        self.node.destroy_node()

    def test_bspline_generation_and_tank_bounds(self):
        """기본 제어점으로 생성된 B-Spline 곡선이 10m x 5m 수조 내부(0.5m 안전 마진)에 완벽히 들어오는지 검증."""
        cps = [(8.5, 2.0), (7.0, 3.5), (5.0, 1.5), (3.0, 3.5), (1.5, 2.5)]
        xs, ys, headings, curvatures, total_len = generate_bspline_path(cps, spacing=0.05, degree=3)

        self.assertGreater(len(xs), 10)
        self.assertEqual(len(xs), len(ys))
        self.assertEqual(len(xs), len(headings))
        self.assertEqual(len(xs), len(curvatures))
        self.assertGreater(total_len, 7.0)

        # 수조 안전 경계 검사: [0.5, 9.5] x [0.5, 4.5]
        self.assertGreater(np.min(xs), 0.5)
        self.assertLess(np.max(xs), 9.5)
        self.assertGreater(np.min(ys), 0.5)
        self.assertLess(np.max(ys), 4.5)

    def test_clamped_bspline_passes_only_endpoints(self):
        """양 끝점은 통과하고 중간 제어점은 보간하지 않는지 검증."""
        cps = np.asarray([
            (0.0, 0.0),
            (1.0, 3.0),
            (2.0, -2.0),
            (3.0, 2.0),
            (4.0, -1.0),
            (5.0, 0.0),
        ])
        xs, ys, _, _, _ = generate_bspline_path(cps.tolist(), spacing=0.005, degree=3)
        samples = np.column_stack((xs, ys))

        np.testing.assert_allclose(samples[0], cps[0], atol=1e-12)
        np.testing.assert_allclose(samples[-1], cps[-1], atol=1e-12)

        # 이 형상에서 각 중간 제어점은 clamped B-spline 곡선으로부터 충분히
        # 떨어져 있다. 과거 splprep(s=0) 보간 구현이면 거리가 거의 0이 된다.
        min_distances = [
            float(np.min(np.linalg.norm(samples - control_point, axis=1)))
            for control_point in cps[1:-1]
        ]
        self.assertTrue(all(distance > 0.1 for distance in min_distances), min_distances)

    def test_six_point_cubic_matches_simulation_bspline(self):
        """6개 제어점의 knot와 곡선이 시뮬레이션의 clamped cubic과 같은지 검증."""
        cps = np.asarray([
            (0.0, 0.0),
            (1.0, 3.0),
            (2.0, -2.0),
            (3.0, 2.0),
            (4.0, -1.0),
            (5.0, 0.0),
        ])
        spacing = 0.037
        xs, ys, _, _, _ = generate_bspline_path(cps.tolist(), spacing=spacing, degree=3)

        sim_knots = np.asarray([0.0, 0.0, 0.0, 0.0, 1.0 / 3.0, 2.0 / 3.0,
                                1.0, 1.0, 1.0, 1.0])
        sim_spline = BSpline(sim_knots, cps, 3)
        dense_u = np.linspace(0.0, 1.0, 1000)
        dense_points = sim_spline(dense_u)
        cumulative_length = np.concatenate((
            [0.0],
            np.cumsum(np.linalg.norm(np.diff(dense_points, axis=0), axis=1)),
        ))
        target_length = np.arange(0.0, cumulative_length[-1], spacing)
        target_length = np.append(target_length, cumulative_length[-1])
        sampled_u = np.interp(target_length, cumulative_length, dense_u)
        expected_points = sim_spline(sampled_u)

        np.testing.assert_allclose(np.column_stack((xs, ys)), expected_points, atol=1e-12)

    def test_two_point_bspline_is_finite_straight_line(self):
        """제어점 2개의 1차 spline은 유한한 직선이고 곡률이 0인지 검증."""
        cps = [(1.0, 2.0), (4.0, 6.0)]
        xs, ys, headings, curvatures, total_len = generate_bspline_path(
            cps, spacing=0.2, degree=3)

        np.testing.assert_allclose([xs[0], ys[0]], cps[0], atol=1e-12)
        np.testing.assert_allclose([xs[-1], ys[-1]], cps[-1], atol=1e-12)
        self.assertTrue(np.all(np.isfinite(xs)))
        self.assertTrue(np.all(np.isfinite(ys)))
        self.assertTrue(np.all(np.isfinite(headings)))
        np.testing.assert_allclose(curvatures, 0.0, atol=1e-12)
        self.assertAlmostEqual(total_len, 5.0, places=9)

    def test_bspline_few_points_exception(self):
        """제어점이 2개 미만일 때 예외 발생 검증."""
        with self.assertRaises(ValueError):
            generate_bspline_path([(1.0, 1.0)])

    def _feed_odom(self, x: float, y: float, yaw: float, yaw_rate: float = 0.0):
        msg = Odometry()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = yaw_to_quaternion(yaw)
        msg.twist.twist.angular.z = yaw_rate
        self.node._on_odom(msg)

    def test_curve_tracking_initial_step(self):
        """시작점 부근에서 정상 주행 명령(전진 및 조향) 발행 검증."""
        start_x = self.node.path_x[0]
        start_y = self.node.path_y[0]
        start_yaw = self.node.path_headings[0]

        self._feed_odom(x=start_x, y=start_y, yaw=start_yaw, yaw_rate=0.0)
        self.node._control_loop()

        self.assertGreater(len(self.published_cmds), 0)
        cmd: Twist = self.published_cmds[-1]
        self.assertGreater(cmd.linear.x, 0.0)
        self.assertFalse(self.node.mission_finished)

    def test_goal_arrival_stop(self):
        """B-spline 곡선 종점 도달 시 미션 종료 및 완전 정지 검증."""
        end_x = float(self.node.path_x[-1])
        end_y = float(self.node.path_y[-1])
        end_yaw = float(self.node.path_headings[-1])

        # 종점 10cm 위치
        self._feed_odom(x=end_x + 0.1, y=end_y, yaw=end_yaw)
        self.node._control_loop()

        self.assertTrue(self.node.mission_finished)
        cmd: Twist = self.published_cmds[-1]
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(cmd.angular.z, 0.0)

    def test_emergency_stop_handling(self):
        """/emergency_stop 토픽 수신 시 모터 즉각 정지 검증."""
        start_x = self.node.path_x[0]
        start_y = self.node.path_y[0]
        self._feed_odom(x=start_x, y=start_y, yaw=math.pi)

        estop_msg = Bool()
        estop_msg.data = True
        self.node._on_estop(estop_msg)

        self.published_cmds.clear()
        self.node._control_loop()

        self.assertGreater(len(self.published_cmds), 0)
        cmd: Twist = self.published_cmds[-1]
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(cmd.angular.z, 0.0)

    def test_start_mission_topic_and_pause(self):
        """/start_mission 토픽을 통한 B-Spline 출발 신호 인가 및 일시 정지 검증."""
        self.node.started = False
        start_x = self.node.path_x[0]
        start_y = self.node.path_y[0]
        start_yaw = self.node.path_headings[0]
        self._feed_odom(x=start_x, y=start_y, yaw=start_yaw)

        # 시작 신호 전: 모터 정지 유지
        self.published_cmds.clear()
        self.node._control_loop()
        self.assertFalse(self.node.started)
        if len(self.published_cmds) > 0:
            self.assertEqual(self.published_cmds[-1].linear.x, 0.0)

        # 출발 신호 수신 (data=True)
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
        """/start_test 서비스를 통한 B-Spline 출발 검증."""
        self.node.started = False
        from std_srvs.srv import Trigger
        req = Trigger.Request()
        resp = Trigger.Response()
        result = self.node._on_start_service(req, resp)
        self.assertTrue(result.success)
        self.assertTrue(self.node.started)

    def test_trajectory_recording_lifecycle(self):
        """B-spline 주행 전, 주행 중, 완료 후 실제 궤적 기록 생명주기 검증."""
        start_x = self.node.path_x[0]
        start_y = self.node.path_y[0]

        # 1. 시작 전 (started=False)
        self.node.started = False
        self._feed_odom(x=start_x, y=start_y, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 0)

        # 2. 시작 (started=True)
        self.node.started = True
        self._feed_odom(x=start_x, y=start_y, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 1)

        # 3. 거리 다운샘플링 (<5cm)
        self._feed_odom(x=start_x + 0.02, y=start_y, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 1)

        # 4. 충분한 거리 이동 (>=5cm)
        self._feed_odom(x=start_x + 0.10, y=start_y, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 2)

        # 5. 미션 완료 (mission_finished=True)
        self.node.mission_finished = True
        self._feed_odom(x=start_x + 0.30, y=start_y, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 2)

    def test_rviz_vis_markers_include_goal_tolerance_and_trail(self):
        """RViz 시각화 발행 시 B-spline Path, 제어점, 허용오차 링, 실제 궤적이 발행되는지 검증."""
        published_marker_arrays = []
        published_paths = []
        self.node.marker_pub.publish = lambda ma: published_marker_arrays.append(ma)
        self.node.path_pub.publish = lambda p: published_paths.append(p)

        self.node.trajectory_history.add_point(8.5, 2.0)
        self.node.trajectory_history.add_point(7.0, 3.5)

        self.node._publish_rviz_vis()

        self.assertGreater(len(published_paths), 0)
        self.assertGreater(len(published_marker_arrays), 0)

        path_msg = published_paths[-1]
        self.assertEqual(path_msg.header.frame_id, "odom")
        self.assertGreater(len(path_msg.poses), 0)

        ma = published_marker_arrays[-1]
        namespaces = {m.ns: m for m in ma.markers}
        self.assertIn("bspline_control_points", namespaces)
        self.assertIn("goal_tolerance", namespaces)
        self.assertIn("actual_trajectory", namespaces)

        # 허용오차 링
        tol_marker = namespaces["goal_tolerance"]
        self.assertEqual(tol_marker.id, 0)
        self.assertEqual(tol_marker.header.frame_id, "odom")
        self.assertGreater(len(tol_marker.points), 10)

        # 실제 궤적
        trail_marker = namespaces["actual_trajectory"]
        self.assertEqual(trail_marker.id, 0)
        self.assertEqual(trail_marker.header.frame_id, "odom")
        self.assertEqual(len(trail_marker.points), 2)
        self.assertEqual(trail_marker.lifetime.sec, 1)
        self.assertEqual(trail_marker.lifetime.nanosec, 0)

        # 목표 시각화 마커 수명 무한(0) 유지 검증
        for m in ma.markers:
            if m.ns != "actual_trajectory":
                self.assertEqual(m.lifetime.sec, 0)
                self.assertEqual(m.lifetime.nanosec, 0)

        # 미션 완료 후에도 궤적이 유지되는지 확인
        self.node.mission_finished = True
        published_marker_arrays.clear()
        self.node._publish_rviz_vis()
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
        self.node.trajectory_history.add_point(8.5, 2.0)
        self.node.trajectory_history.add_point(7.0, 3.5)
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
        published_paths = []
        self.node.marker_pub.publish = lambda ma: published_marker_arrays.append(ma)
        self.node.path_pub.publish = lambda p: published_paths.append(p)

        self.node.trajectory_history.add_point(8.5, 2.0)
        self.node.trajectory_history.add_point(7.0, 3.5)

        req = Trigger.Request()
        resp = Trigger.Response()
        self.node._on_clear_trajectory(req, resp)

        self.assertEqual(len(published_marker_arrays), 1)
        self.assertEqual(len(published_paths), 1)

        ma = published_marker_arrays[0]
        namespaces = {m.ns: m for m in ma.markers}

        self.assertIn("actual_trajectory", namespaces)
        self.assertEqual(len(namespaces["actual_trajectory"].points), 0)

        # 제어점 마커 및 허용오차 링 보존 확인
        self.assertIn("bspline_control_points", namespaces)
        self.assertIn("goal_tolerance", namespaces)

    def test_clear_trajectory_preserves_mission_and_control_state_and_no_cmd_vel(self):
        """/clear_trajectory 호출 시 B-spline 진행 상태 및 제어 상태가 보존되고 cmd_vel이 발행되지 않는지 검증."""
        self.node.started = True
        self.node.mission_finished = False
        self.node.emergency_stopped = False
        self.node.progress_idx = 42

        self.published_cmds.clear()
        req = Trigger.Request()
        resp = Trigger.Response()
        self.node._on_clear_trajectory(req, resp)

        self.assertTrue(self.node.started)
        self.assertFalse(self.node.mission_finished)
        self.assertFalse(self.node.emergency_stopped)
        self.assertEqual(self.node.progress_idx, 42)
        self.assertEqual(len(self.published_cmds), 0)

    def test_clear_trajectory_records_new_trail_on_next_odom(self):
        """B-Spline 미션 활성 상태에서 궤적 초기화 후 다음 오도메트리가 새로운 궤적의 첫 점으로 기록되는지 검증."""
        self.node.started = True
        self.node.mission_finished = False
        start_x = self.node.path_x[0]
        start_y = self.node.path_y[0]

        self._feed_odom(x=start_x, y=start_y, yaw=0.0)
        self._feed_odom(x=start_x + 0.1, y=start_y, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 2)

        # 궤적 초기화
        req = Trigger.Request()
        resp = Trigger.Response()
        self.node._on_clear_trajectory(req, resp)
        self.assertEqual(len(self.node.trajectory_history), 0)

        # 초기화 직후 동일 위치라도 첫 점은 즉시 새 궤적의 시작점으로 기록되어야 함
        self._feed_odom(x=start_x + 0.1, y=start_y, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 1)
        self.assertAlmostEqual(self.node.trajectory_history[0][0], start_x + 0.1)

        # 이후 거리 판정 기준 충족 시 추가 기록
        self._feed_odom(x=start_x + 0.2, y=start_y, yaw=0.0)
        self.assertEqual(len(self.node.trajectory_history), 2)


if __name__ == '__main__':
    unittest.main()
