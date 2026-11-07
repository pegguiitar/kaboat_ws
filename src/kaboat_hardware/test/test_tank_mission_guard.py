"""Output gating for the indoor gate/avoid mission tests."""

import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from kaboat_hardware.tank_mission_guard import TankMissionGuard


@pytest.fixture(autouse=True)
def isolated_ros_domain(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '219')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')


def _inputs(node):
    node._on_state(String(data='gate'))
    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.orientation.w = 1.0
    node._on_odom(odom)
    grid = OccupancyGrid()
    grid.header.frame_id = 'odom'
    grid.info.resolution = 0.2
    grid.info.width = grid.info.height = 1
    grid.data = [70]
    node._on_grid(grid)
    cmd = Twist()
    cmd.linear.x, cmd.angular.z = 0.09, 0.03
    node._on_cmd(cmd)


def test_preview_has_no_cmd_vel_publisher():
    rclpy.init()
    node = TankMissionGuard()
    try:
        _inputs(node)
        response = node._on_start(None, Trigger.Response())
        assert not response.success
        assert node.cmd_pub is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_drive_clamps_and_disarms_on_stale_odom():
    rclpy.init(args=['--ros-args', '-p', 'drive_enabled:=true'])
    node = TankMissionGuard()
    try:
        published = []

        class Capture:
            def publish(self, msg):
                published.append((msg.linear.x, msg.angular.z))

        node.cmd_pub = Capture()
        _inputs(node)
        response = node._on_start(None, Trigger.Response())
        assert response.success
        node._tick()
        assert published[-1] == (node.max_linear, node.max_angular)

        node._on_state(String(data='approach'))
        assert published[-1] == (0.0, 0.0)
        node._tick()
        assert node.armed and published[-1] == (0.0, 0.0)
        node._on_cmd(Twist())
        node._on_state(String(data='gate'))
        assert published[-1] == (0.0, 0.0)
        node._tick()
        assert node.armed and published[-1] == (0.0, 0.0)
        node._on_cmd(Twist())

        node.received['odom'] = time.monotonic() - 1.0
        node._tick()
        assert not node.armed
        assert published[-1] == (0.0, 0.0)

        _inputs(node)
        reverse = Twist()
        reverse.linear.x = -0.01
        node._on_cmd(reverse)
        assert not node._on_start(None, Trigger.Response()).success
        node._on_estop(Bool(data=True))
        assert not node._on_start(None, Trigger.Response()).success
        assert published[-1] == (0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
