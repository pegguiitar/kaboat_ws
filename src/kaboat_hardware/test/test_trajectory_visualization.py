"""test_trajectory_visualization.py — BoundedTrajectoryHistory 및 마커 유틸리티 단위 테스트."""

import math
import sys
import unittest

# ROS 환경이 존재하면 sys.path에 추가 (선택적 마커 테스트 지원)
for p in ("/opt/ros/humble/lib/python3.10/site-packages", "/opt/ros/humble/local/lib/python3.10/dist-packages"):
    if p not in sys.path:
        sys.path.append(p)

from kaboat_hardware.trajectory_visualization import (
    BoundedTrajectoryHistory,
    DEFAULT_MAX_POINTS,
    DEFAULT_MIN_DISTANCE,
    TRAIL_NAMESPACE,
    TRAIL_MARKER_ID,
    TRAIL_COLOR,
    create_trail_marker,
    create_goal_tolerance_marker,
)


class TestBoundedTrajectoryHistory(unittest.TestCase):
    def test_initialization_defaults(self):
        th = BoundedTrajectoryHistory()
        self.assertEqual(th.max_points, DEFAULT_MAX_POINTS)
        self.assertEqual(th.min_distance, DEFAULT_MIN_DISTANCE)
        self.assertEqual(len(th), 0)
        self.assertTrue(th.is_empty())
        self.assertIsNone(th.last_point)
        self.assertEqual(th.points, [])

    def test_invalid_parameters(self):
        with self.assertRaises(ValueError):
            BoundedTrajectoryHistory(max_points=0)
        with self.assertRaises(ValueError):
            BoundedTrajectoryHistory(min_distance=-0.01)

    def test_always_records_first_eligible_point(self):
        th = BoundedTrajectoryHistory(min_distance=1.0)
        # 원점 (0, 0) 첫 점은 무조건 기록되어야 함
        added = th.add_point(0.0, 0.0)
        self.assertTrue(added)
        self.assertEqual(len(th), 1)
        self.assertFalse(th.is_empty())
        self.assertEqual(th.last_point, (0.0, 0.0, 0.04))

    def test_distance_downsampling(self):
        th = BoundedTrajectoryHistory(min_distance=0.1)
        self.assertTrue(th.add_point(0.0, 0.0))

        # 0.05m 이동 (0.1m 미만): 무시되어야 함
        self.assertFalse(th.add_point(0.05, 0.0))
        self.assertEqual(len(th), 1)

        # 0.08m 이동 (0.1m 미만): 무시되어야 함
        self.assertFalse(th.add_point(0.0, 0.08))
        self.assertEqual(len(th), 1)

        # 0.12m 이동 (0.1m 이상): 추가되어야 함
        self.assertTrue(th.add_point(0.12, 0.0))
        self.assertEqual(len(th), 2)
        self.assertEqual(th.last_point, (0.12, 0.0, 0.04))

        # 다음 점은 마지막으로 기록된 점(0.12, 0.0) 기준으로 판정
        self.assertFalse(th.add_point(0.15, 0.0))  # 0.03m 차이 -> 무시
        self.assertTrue(th.add_point(0.25, 0.0))   # 0.13m 차이 -> 기록
        self.assertEqual(len(th), 3)

    def test_bounded_capacity(self):
        max_capacity = 5
        th = BoundedTrajectoryHistory(max_points=max_capacity, min_distance=0.1)

        # 10개 점 순차 추가 (각 0.2m 간격)
        for i in range(10):
            th.add_point(float(i) * 0.2, 0.0)

        # 최대 용량을 초과하지 않아야 함
        self.assertEqual(len(th), max_capacity)
        points = th.points
        self.assertEqual(len(points), max_capacity)

        # 가장 오래된 점(0.0~0.8)은 폐기되고 최근 5개(1.0, 1.2, 1.4, 1.6, 1.8)만 유지
        expected_xs = [float(i) * 0.2 for i in range(5, 10)]
        actual_xs = [pt[0] for pt in points]
        for act, exp in zip(actual_xs, expected_xs):
            self.assertAlmostEqual(act, exp)

    def test_reset_and_clear(self):
        th = BoundedTrajectoryHistory(min_distance=0.1)
        th.add_point(1.0, 1.0)
        th.add_point(2.0, 2.0)
        self.assertEqual(len(th), 2)

        th.reset()
        self.assertEqual(len(th), 0)
        self.assertTrue(th.is_empty())
        self.assertIsNone(th.last_point)

        # reset 후 첫 점도 정상 기록되는지 확인
        self.assertTrue(th.add_point(5.0, 5.0))
        self.assertEqual(len(th), 1)

    def test_iteration_and_indexing(self):
        th = BoundedTrajectoryHistory(min_distance=0.1)
        th.add_point(1.0, 2.0, 0.05)
        th.add_point(2.0, 3.0, 0.05)

        self.assertEqual(th[0], (1.0, 2.0, 0.05))
        self.assertEqual(th[1], (2.0, 3.0, 0.05))

        pts_list = [p for p in th]
        self.assertEqual(len(pts_list), 2)
        self.assertEqual(pts_list[0], (1.0, 2.0, 0.05))


class TestMarkerCreation(unittest.TestCase):
    def setUp(self):
        try:
            import visualization_msgs.msg
            self.has_ros = True
        except ImportError:
            self.has_ros = False

    def test_create_trail_marker(self):
        if not self.has_ros:
            self.skipTest("ROS visualization_msgs not available")

        points = [(1.0, 2.0, 0.04), (3.0, 4.0, 0.04)]
        marker = create_trail_marker(points, frame_id="odom", ns="actual_trajectory", marker_id=0)

        self.assertEqual(marker.header.frame_id, "odom")
        self.assertEqual(marker.ns, "actual_trajectory")
        self.assertEqual(marker.id, 0)
        self.assertEqual(marker.type, 4)  # Marker.LINE_STRIP == 4
        self.assertEqual(len(marker.points), 2)
        self.assertAlmostEqual(marker.points[0].x, 1.0)
        self.assertAlmostEqual(marker.points[1].x, 3.0)
        self.assertAlmostEqual(marker.color.r, TRAIL_COLOR[0])
        self.assertAlmostEqual(marker.color.g, TRAIL_COLOR[1])
        self.assertAlmostEqual(marker.color.b, TRAIL_COLOR[2])
        self.assertAlmostEqual(marker.color.a, TRAIL_COLOR[3])

    def test_create_trail_marker_single_point(self):
        if not self.has_ros:
            self.skipTest("ROS visualization_msgs not available")

        # 단일 점일 때 라인 스트립 표시를 위해 점이 복제되는지 확인
        points = [(1.0, 2.0, 0.04)]
        marker = create_trail_marker(points)
        self.assertEqual(len(marker.points), 2)
        self.assertAlmostEqual(marker.points[0].x, 1.0)
        self.assertAlmostEqual(marker.points[1].x, 1.0)

    def test_create_goal_tolerance_marker(self):
        if not self.has_ros:
            self.skipTest("ROS visualization_msgs not available")

        marker = create_goal_tolerance_marker(
            center_x=1.0, center_y=3.0, radius=0.35, frame_id="odom", num_segments=36
        )

        self.assertEqual(marker.header.frame_id, "odom")
        self.assertEqual(marker.ns, "goal_tolerance")
        self.assertEqual(marker.id, 0)
        self.assertEqual(marker.type, 4)  # LINE_STRIP
        self.assertEqual(len(marker.points), 37)  # 36 segments + closed point
        # 첫 점과 마지막 점이 동일하여 닫힌 원인지 확인
        self.assertAlmostEqual(marker.points[0].x, marker.points[-1].x)
        self.assertAlmostEqual(marker.points[0].y, marker.points[-1].y)


if __name__ == '__main__':
    unittest.main()
