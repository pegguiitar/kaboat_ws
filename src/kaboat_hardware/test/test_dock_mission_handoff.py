"""Competition mission handoff enters the measured-normal search directly."""

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from std_msgs.msg import String

from kaboat_hardware.dock_ctrl_v2 import DockCtrlV2


def test_dock_state_starts_fresh_scan_without_staging():
    rclpy.init()
    node = DockCtrlV2(parameter_overrides=[Parameter('mission_mode', value=True)])
    try:
        node.complete_sent = True
        node._on_mission_state(String(data='dock'))
        assert node.phase == 'SCAN'
        assert node.route_enu is None
        assert not node.complete_sent
        assert node.cmd.topic_name == '/cmd/dock'
        node._on_mission_state(String(data='done'))
        assert node.phase == 'IDLE'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _odom(x, y):
    from nav_msgs.msg import Odometry
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
    msg.pose.pose.orientation.w = 1.0
    return msg


def test_default_position_is_odom_and_needs_no_gps():
    # 2026-11-12: 야외는 GQ7 RTK 필터 /odom(25 Hz, 선체 중심)을 쓴다 — /gps/fix 원시 2 Hz 는 늦다
    rclpy.init()
    node = DockCtrlV2(parameter_overrides=[Parameter('mission_mode', value=True)])
    try:
        assert node.position_source == 'odom' and node.marker_order is None
        assert node._inputs() is not None                      # odom 도 아직 없음
        node._odom(_odom(12.5, -3.0))
        assert node._inputs() is None                          # GPS 없어도 됨
        assert node._position().tolist() == [12.5, -3.0]
        assert node._stage() is None                           # 시작점 미설정
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_gps_source_still_requires_fix_and_marker_order_is_validated():
    rclpy.init()
    node = DockCtrlV2(parameter_overrides=[
        Parameter('mission_mode', value=True), Parameter('position_source', value='gps'),
        Parameter('marker_order', value='circle,triangle,square')])
    try:
        node._odom(_odom(0.0, 0.0))
        assert node._inputs() == 'GPS fix stale or missing'
        assert node.marker_order == ('circle', 'triangle', 'square')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    rclpy.init()
    try:
        with pytest.raises(ValueError):
            DockCtrlV2(parameter_overrides=[Parameter('position_source', value='filter')])
    finally:
        rclpy.shutdown()
