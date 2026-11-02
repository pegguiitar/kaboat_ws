"""Five targets advance in order and the tank test stays preview-only by default."""

import math

import pytest
import rclpy
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from kaboat_behaviors.waypoint_controller import (
    WaypointController, WaypointParams)
from kaboat_hardware.waypoint_sequence_test import (
    WaypointSequenceTest, build_waypoints)


def test_build_requires_five_safe_targets_with_optional_headings():
    targets = build_waypoints(
        [8.5, 7.0, 5.0, 3.0, 1.5],
        [2.5, 3.5, 2.5, 1.5, 2.5],
        [180.0, 0.0, 90.0, 0.0, 0.0],
        [True, False, True, False, False],
        0.5, 10.0, 5.0, 0.25)
    assert len(targets) == 5
    assert targets[0].yaw == pytest.approx(math.pi)
    assert targets[1].yaw is None
    assert targets[2].yaw == pytest.approx(math.pi / 2)
    assert all(target.thrust_ratio == 0.5 for target in targets)

    with pytest.raises(ValueError, match='exactly five'):
        build_waypoints([1.0], [1.0], [0.0], [False], 0.5, 10.0, 5.0, 0.25)
    with pytest.raises(ValueError, match='outside'):
        build_waypoints([0.1] * 5, [2.5] * 5, [0.0] * 5,
                        [False] * 5, 0.5, 10.0, 5.0, 0.25)


def _odom(x, y, yaw=0.0):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    return msg


def test_ros_node_holds_each_goal_then_reacquires_final_position():
    rclpy.init()
    node = None
    try:
        node = WaypointSequenceTest()
        sent = []
        node._send = sent.append
        node.count_publishers = lambda topic: 1
        node.controller = WaypointController(WaypointParams(settle_time=0.0))
        node.controller.set_goal(node.targets[0])
        clock = [99.0]
        node._now = lambda: clock[0]
        assert node.cmd_pub is None
        assert node.last_status == 'WAIT_START'

        response = node._on_start(Trigger.Request(), Trigger.Response())
        assert response.success
        node._on_odom(_odom(9.0, 2.5))
        node._tick()
        assert node.index == 0
        assert 0 < sent[-1].linear.x <= 12.0 / 118.6 * 0.5

        first = node.targets[0]
        clock[0] = 100.0
        node._on_odom(_odom(first.x, first.y))
        node._tick()
        assert node.index == 0 and node.last_status == 'WAYPOINT_1/5_HOLD'
        # A drift outside tolerance resets the continuous hold timer.
        clock[0] = 101.0
        node._on_odom(_odom(first.x + 0.5, first.y))
        node._tick()
        assert node._hold_since is None and node.index == 0
        clock[0] = 102.0
        node._on_odom(_odom(first.x, first.y))
        node._tick()
        clock[0] = 104.9
        node._on_odom(_odom(first.x, first.y))
        node._tick()
        assert node.index == 0
        clock[0] = 105.1
        node._on_odom(_odom(first.x, first.y))
        node._tick()
        assert node.index == 1

        for i, target in enumerate(node.targets[1:], start=2):
            clock[0] += 0.1
            node._on_odom(_odom(target.x, target.y,
                                target.yaw if target.yaw is not None else 0.0))
            node._tick()
            assert node.index == i - 1  # starts holding at this target
            assert sent[-1].linear.x == sent[-1].angular.z == 0.0
            clock[0] += 3.1
            node._on_odom(_odom(target.x, target.y,
                                target.yaw if target.yaw is not None else 0.0))
            node._tick()
            assert node.index == min(i, 4)

        assert node.completed and node.last_status == 'COMPLETE_HOLD'
        final = node.targets[-1]
        clock[0] += 0.1
        node._on_odom(_odom(final.x + 0.6, final.y))
        node._tick()
        assert node.index == 4 and sent[-1].linear.x > 0.0
        assert node.last_status == 'PREVIEW_WAYPOINT_5/5_REACQUIRE'
        clock[0] += 0.1
        node._on_odom(_odom(final.x, final.y))
        node._tick()
        assert sent[-1].linear.x == 0.0
        assert node.last_status == 'COMPLETE_HOLD'
        pause = node._on_pause(Trigger.Request(), Trigger.Response())
        assert pause.success and not node.started and sent[-1].linear.x == 0.0
        resume = node._on_start(Trigger.Request(), Trigger.Response())
        assert resume.success and node.started and node.index == 4
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_stale_odom_does_not_advance_or_command_motion():
    rclpy.init()
    node = None
    try:
        node = WaypointSequenceTest()
        sent = []
        node._send = sent.append
        node.count_publishers = lambda topic: 1
        node.started = True
        node._on_odom(_odom(node.targets[0].x, node.targets[0].y))
        node.last_odom_time -= 1.0
        node._tick()
        assert node.index == 0
        assert node.last_status == 'ODOM_TIMEOUT'
        assert sent[-1].linear.x == sent[-1].angular.z == 0.0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
