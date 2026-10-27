"""Synthetic ROS-message checks for the real LiDAR avoidance test adapter."""

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


if __name__ == '__main__':
    unittest.main()
