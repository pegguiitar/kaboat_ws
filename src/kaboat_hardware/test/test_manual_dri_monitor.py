"""Manual RC monitoring computes DRI but never becomes a command source."""

import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

from kaboat_behaviors.dri import DriParams, build_dri
from kaboat_hardware.manual_dri_monitor import ManualDriMonitor, dri_visualization


def _grid():
    msg = OccupancyGrid()
    msg.header.frame_id = 'odom'
    msg.info.resolution = 0.2
    msg.info.width = msg.info.height = 20
    msg.info.origin.position.x = msg.info.origin.position.y = -2.0
    msg.info.origin.orientation.w = 1.0
    data = np.zeros((20, 20), dtype=np.int8)
    data[10, 14] = 100
    msg.data = data.ravel().tolist()
    return msg


def _odom():
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.pose.pose.orientation.w = 1.0
    return msg


def _scan():
    msg = LaserScan()
    msg.range_min = 0.1
    msg.range_max = 25.0
    msg.ranges = [1.0]
    return msg


def test_visualization_is_scaled_existing_dri_in_odom_frame():
    grid = _grid()
    dri = build_dri(grid, (0.0, 0.0), 0.0, DriParams())
    result = dri_visualization(dri, grid, 4.0, grid.header.stamp)
    expected = np.rint(np.clip(dri.data / 4.0 * 100.0, 0, 100))
    assert result.header.frame_id == 'odom'
    assert result.info.resolution == grid.info.resolution
    assert np.array_equal(np.asarray(result.data).reshape(20, 20), expected)
    assert max(result.data) > 0


def test_sensor_loss_clears_manual_dri_without_command_publisher():
    rclpy.init()
    node = None
    try:
        node = ManualDriMonitor()
        node._on_odom(_odom())
        node._on_grid(_grid())
        node._on_scan(_scan())
        node._tick()
        assert node._state == 'READY'
        assert node._last_valid_info is not None
        assert node.count_publishers('/cmd_vel') == 0
        assert node.count_publishers('/rc/cmd_vel') == 0

        node.scan_at = time.monotonic() - 2.0
        node._tick()
        assert node._state == 'WAIT_SCAN'
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
