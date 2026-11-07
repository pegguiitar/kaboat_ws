"""Shore-laptop tank visualization: fixed routes and Jetson mission plans."""

import math

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger

from kaboat_hardware.tank_test_visualization import (
    MISSION_PATH_TOPICS, TEST_KINDS, TankTestVisualization,
    mission_path_points, reference_points)


@pytest.fixture(autouse=True)
def isolated_ros_domain(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '218')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')


def _path(points, frame='odom'):
    msg = Path()
    msg.header.frame_id = frame
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        msg.poses.append(pose)
    return msg


def _boat(x, y, frame='odom'):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
    return msg


def _node(kind):
    rclpy.init(args=['--ros-args', '-p', f'test_kind:={kind}'])
    return TankTestVisualization()


def test_mission_kinds_are_registered_without_a_fixed_route():
    assert set(MISSION_PATH_TOPICS) == {'gate', 'avoid'}
    for kind in MISSION_PATH_TOPICS:
        assert kind in TEST_KINDS
        assert reference_points(kind) == []
    assert MISSION_PATH_TOPICS == {'gate': '/gate/path', 'avoid': '/avoid/plan'}


def test_mission_path_points_accepts_only_usable_plans():
    assert mission_path_points(_path([(1, 1), (2, 1)])) == [
        (1.0, 1.0, 0.05), (2.0, 1.0, 0.05)]
    assert mission_path_points(_path([])) is None
    assert mission_path_points(_path([(1, 1)])) is None
    assert mission_path_points(_path([(1, 1), (2, 1)], frame='map')) is None
    assert mission_path_points(_path([(1, 1), (math.nan, 1)])) is None


@pytest.mark.parametrize('kind', ['gate', 'avoid'])
def test_mission_plan_becomes_target_and_survives_empty_updates(kind):
    node = _node(kind)
    try:
        assert node.reference == []
        node._on_mission_path(_path([(8, 2), (5, 2.5), (2, 2)]))
        assert [(x, y) for x, y, _ in node.reference] == [
            (8.0, 2.0), (5.0, 2.5), (2.0, 2.0)]

        # The behavior publishes an empty path whenever it has no plan.
        node._on_mission_path(_path([]))
        node._on_mission_path(_path([(1, 1), (2, 2)], frame='map'))
        assert len(node.reference) == 3

        node._on_mission_path(_path([(7, 2), (3, 2)]))
        assert [(x, y) for x, y, _ in node.reference] == [(7.0, 2.0), (3.0, 2.0)]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_clear_drops_mission_plan_and_trajectory_but_not_fixed_route():
    node = _node('avoid')
    try:
        node._on_mission_path(_path([(8, 2), (2, 2)]))
        node._on_pose(_boat(5, 2.5))
        node._on_pose(_boat(5.5, 2.5))
        assert len(node.history) == 2
        response = node._clear_trajectory(None, Trigger.Response())
        assert response.success
        assert node.reference == [] and len(node.history) == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()

    node = _node('los')
    try:
        before = list(node.reference)
        assert before
        node._on_pose(_boat(8.5, 2.0))
        node._clear_trajectory(None, Trigger.Response())
        assert node.reference == before and len(node.history) == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_fixed_kinds_do_not_subscribe_to_mission_paths():
    node = _node('straight')
    try:
        assert node.mission_topic is None
        node._on_mission_path(_path([(1, 1), (2, 2)]))
        # Callback is unreachable for fixed kinds, but must not corrupt anyway.
        assert node.kind == 'straight'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_actual_trajectory_ignores_bad_poses():
    node = _node('gate')
    try:
        node._on_pose(_boat(5, 2.5))
        node._on_pose(_boat(3, 3, frame='map'))
        node._on_pose(_boat(50, 50))
        node._on_pose(_boat(math.nan, 1))
        assert len(node.history) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
