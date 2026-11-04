"""Synthetic ROS-message checks for the real LiDAR avoidance test adapter."""

import math
import time
import unittest

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from kaboat_hardware.real_obstacle_avoid_test import RealObstacleAvoidTest


class TestRealObstacleAvoidTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RealObstacleAvoidTest()
        self.motor_commands = []
        self.preview_commands = []
        self.node.cmd_pub.publish = self.motor_commands.append
        self.node.preview_cmd_pub.publish = self.preview_commands.append

    def tearDown(self):
        self.node.destroy_node()

    def feed_scene(self, with_obstacle=True):
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.pose.pose.orientation.w = 0.0
        odom.pose.pose.orientation.z = 1.0  # heading = pi, toward -X
        self.node._on_odom(odom)

        grid = OccupancyGrid()
        grid.header.frame_id = 'odom'
        grid.info.resolution = 0.2
        grid.info.width = grid.info.height = 50
        grid.info.origin.position.x = -5.0
        grid.info.origin.position.y = -5.0
        cells = [-1] * 2500
        if with_obstacle:
            for iy in range(22, 28):
                for ix in range(10, 14):
                    cells[iy * 50 + ix] = 99
        grid.data = cells
        self.node._on_grid(grid)

        scan = LaserScan()
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [2.0]
        self.node._on_scan(scan)

    def test_preview_uses_obstacle_and_stale_scan_disarms(self):
        self.feed_scene()
        self.node._tick()
        plan = self.node.preview_plan
        self.assertIsNotNone(plan)
        self.assertLess(self.node.preview_obstacle_distance, 3.0)
        self.assertTrue(any(abs(y) > 0.2 for _, y in plan.samples))
        self.assertEqual(self.motor_commands, [])

        ok, _ = self.node._arm()
        self.assertTrue(ok)
        self.node._tick()
        self.assertTrue(self.node.started)
        self.assertIsNotNone(self.node.fsm.plan)
        self.assertEqual(self.motor_commands, [])

        self.node.scan_received -= 1.0
        self.node._tick()
        self.assertFalse(self.node.started)
        self.assertAlmostEqual(self.preview_commands[-1].linear.x, 0.0)
        self.assertAlmostEqual(self.preview_commands[-1].angular.z, 0.0)

    def test_start_requires_confirmed_front_obstacle(self):
        self.feed_scene(with_obstacle=False)
        self.node._tick()
        self.assertIsNotNone(self.node.preview_plan)
        ok, reason = self.node._arm()
        self.assertFalse(ok)
        self.assertIn('obstacle', reason)
        self.assertEqual(self.motor_commands, [])

    def test_side_wall_does_not_count_as_front_obstacle(self):
        self.feed_scene(with_obstacle=False)
        cells = list(self.node.grid.data)
        for iy in range(38, 42):
            for ix in range(10, 14):
                cells[iy * 50 + ix] = 99
        self.node.grid.data = cells
        self.node._tick()
        self.assertIsNone(self.node.preview_obstacle_distance)
        self.assertFalse(self.node._arm()[0])

    def test_invalid_yaw_blocks_start(self):
        self.feed_scene()
        self.node.odom.pose.pose.orientation.w = 1.0
        self.assertIn('quaternion', self.node._input_error(time.monotonic()))

    def test_drive_mode_caps_commands_and_stops_on_scan_loss(self):
        self.node.drive_enabled = True
        self.feed_scene()
        self.node._tick()
        self.assertTrue(self.node._arm()[0])
        self.node._tick()
        self.node._tick()  # first FSM tick acquires the path; next tick follows it
        command = self.motor_commands[-1]
        self.assertGreater(command.linear.x, 0.0)
        self.assertLessEqual(abs(command.linear.x), self.node.max_linear_cmd)
        self.assertLessEqual(abs(command.angular.z), self.node.max_angular_cmd)

        self.node.scan_received -= 1.0
        self.node._tick()
        self.assertEqual(self.motor_commands[-1].linear.x, 0.0)
        self.assertEqual(self.motor_commands[-1].angular.z, 0.0)

    def test_rviz_path_and_markers_clear_on_scan_loss(self):
        paths, marker_arrays = [], []
        self.node.path_pub.publish = paths.append
        self.node.marker_pub.publish = marker_arrays.append
        self.feed_scene()
        self.node._tick()
        self.assertGreater(len(paths[-1].poses), 2)
        self.assertEqual(paths[-1].header.frame_id, 'odom')
        self.assertTrue(any(m.ns == 'control_points'
                            for array in marker_arrays for m in array.markers))
        self.assertTrue(any(m.ns == 'test_status' and m.text == 'PREVIEW'
                            for array in marker_arrays for m in array.markers))

        self.node.scan_received -= 1.0
        self.node._tick()
        self.assertEqual(len(paths[-1].poses), 0)
        self.assertTrue(any(m.action == Marker.DELETEALL
                            for m in marker_arrays[-2].markers))

    def enable_tank(self, bounds, exclude_walls=False, margin=1.0, band=0.3,
                    wall_dri=0.5):
        self.node.tank_enabled = True
        self.node.tank_bounds = bounds
        self.node.exclude_tank_walls = exclude_walls
        self.node.tank_stop_margin = margin
        self.node.tank_wall_band = band
        self.node.tank_wall_dri = wall_dri

    def test_tank_boundary_stops_a_running_test(self):
        self.node.drive_enabled = True
        self.feed_scene()
        self.node._tick()
        self.assertTrue(self.node._arm()[0])
        self.node._tick()
        self.assertTrue(self.node.started)

        # 배(0, 0)가 x_min=-0.5 에서 0.5 m — 정지 여유 1.0 m 안쪽
        self.enable_tank((-0.5, -5.0, 10.0, 5.0))
        self.node._tick()
        self.assertFalse(self.node.started)
        self.assertAlmostEqual(self.preview_commands[-1].linear.x, 0.0)
        self.assertAlmostEqual(self.preview_commands[-1].angular.z, 0.0)

    def test_start_refused_near_tank_boundary_or_with_goal_outside(self):
        self.feed_scene()
        self.enable_tank((-0.5, -5.0, 10.0, 5.0))        # 배가 경계에서 0.5 m
        self.node._tick()
        ok, reason = self.node._arm()
        self.assertFalse(ok)
        self.assertIn('boat', reason)
        self.assertIn('tank boundary', reason)

        self.enable_tank((-2.5, -5.0, 10.0, 5.0))        # 배는 2.5 m, 목표(x=-3)는 밖
        self.node._tick()
        ok, reason = self.node._arm()
        self.assertFalse(ok)
        self.assertIn('goal', reason)
        self.assertEqual(self.motor_commands, [])

    def test_tank_walls_are_excluded_from_dri_only_when_enabled(self):
        self.feed_scene()
        # 장애물 셀(x -2.9 ~ -2.3)이 모두 경계 밖 → 벽으로 보고 DRI 에서 제외
        self.enable_tank((-2.0, -5.0, 10.0, 5.0), exclude_walls=True)
        self.node._tick()
        self.assertIsNone(self.node.preview_obstacle_distance)
        self.node.exclude_tank_walls = False
        self.node._tick()
        self.assertLess(self.node.preview_obstacle_distance, 3.0)

    def test_wall_dri_is_capped_below_the_planner_filter_not_deleted(self):
        self.feed_scene()
        self.enable_tank((-2.0, -5.0, 10.0, 5.0), exclude_walls=True, wall_dri=0.5)
        boat, yaw, _, _ = self.node._boat_state()
        capped = self.node._build_dri(boat, yaw)
        self.assertEqual(len(capped.occ_xy), 0)           # 벽은 기준 장애물이 아니다
        self.assertGreater(float(capped.data.max()), 0.1)  # 하지만 DRI 에는 남아 있다
        self.assertLessEqual(float(capped.data.max()), 0.5 + 1e-6)

        self.node.tank_wall_dri = 0.0                      # 0 이면 완전히 지움
        self.assertEqual(float(self.node._build_dri(boat, yaw).data.max()), 0.0)
        self.node.exclude_tank_walls = False               # 끄면 벽도 장애물
        self.assertGreater(float(self.node._build_dri(boat, yaw).data.max()), 0.85)

    def test_tank_settings_are_off_by_default(self):
        self.assertFalse(self.node.tank_enabled)
        self.assertFalse(self.node.exclude_tank_walls)
        self.assertIsNone(self.node._tank_edge(0.0, 0.0))

    def enable_fixed_goal(self, goal, start=(0.0, 0.0), start_tol=0.5,
                          heading_tol_deg=45.0):
        self.node.fixed_goal_enabled = True
        self.node.fixed_goal_xy = goal
        self.node.fixed_start_xy = start
        self.node.start_tolerance = start_tol
        self.node.start_heading_tolerance = math.radians(heading_tol_deg)

    def test_fixed_goal_replaces_the_relative_goal_and_can_start(self):
        self.feed_scene()                       # 배 (0, 0), 선수 180°, 장애물 x -2.9 ~ -2.3
        self.enable_fixed_goal((-4.0, 0.0))
        self.node._tick()
        self.assertEqual(self.node.preview_goal, (-4.0, 0.0))
        self.assertIsNotNone(self.node.preview_plan)
        self.assertLess(self.node.preview_obstacle_distance, 4.0)
        ok, reason = self.node._arm()
        self.assertTrue(ok, reason)
        self.assertEqual(self.node.goal, (-4.0, 0.0))

    def test_fixed_goal_start_must_be_near_the_start_point(self):
        self.feed_scene()
        self.enable_fixed_goal((-4.0, 0.0), start=(5.0, 0.0))
        self.node._tick()
        ok, reason = self.node._arm()
        self.assertFalse(ok)
        self.assertIn('start point', reason)
        self.node.start_tolerance = 0.0         # 0 이면 위치 검사를 끈다
        self.assertTrue(self.node._arm()[0])

    def test_fixed_goal_boat_must_face_the_goal(self):
        self.feed_scene()                       # 선수는 -X
        self.enable_fixed_goal((4.0, 0.0))      # 목표는 +X (정반대)
        self.node._tick()
        ok, reason = self.node._arm()
        self.assertFalse(ok)
        self.assertIn('heading', reason)
        self.assertEqual(self.motor_commands, [])

    def test_fixed_goal_front_lane_points_at_the_goal_not_the_bow(self):
        self.feed_scene()                       # 장애물은 선수(-X) 쪽
        self.node._tick()
        self.assertIsNotNone(self.node.preview_obstacle_distance)
        self.enable_fixed_goal((0.0, 4.0), heading_tol_deg=180.0)   # 목표는 +Y
        self.node._tick()
        self.assertIsNone(self.node.preview_obstacle_distance)

    def test_fixed_goal_is_off_by_default(self):
        self.assertFalse(self.node.fixed_goal_enabled)

    def test_auto_arm_when_wait_for_start_false(self):
        self.node.wait_for_start = False
        self.feed_scene()
        self.node._tick()
        self.assertTrue(self.node.started)


if __name__ == '__main__':
    unittest.main()
