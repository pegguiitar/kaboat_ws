"""test_bspline_track_test.py — B-Spline 곡선 경로 생성 및 추종 노드 단위 테스트."""

import math
import sys
import unittest
import numpy as np

for p in ("/opt/ros/humble/lib/python3.10/site-packages", "/opt/ros/humble/local/lib/python3.10/dist-packages"):
    if p not in sys.path:
        sys.path.append(p)

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

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

        # 미션 완료 후에도 궤적이 유지되는지 확인
        self.node.mission_finished = True
        published_marker_arrays.clear()
        self.node._publish_rviz_vis()
        ma_after = published_marker_arrays[-1]
        ns_after = {m.ns: m for m in ma_after.markers}
        self.assertIn("actual_trajectory", ns_after)
        self.assertEqual(len(ns_after["actual_trajectory"].points), 2)


if __name__ == '__main__':
    unittest.main()

