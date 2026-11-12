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
