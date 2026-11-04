"""Allowlist checks for the Jetson <-> laptop Zenoh bridge."""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import zenoh_monitor_link as link  # noqa: E402

JETSON_IP = '100.64.0.10'
LAPTOP_IP = '100.64.0.20'


def _allowed(role):
    ip = JETSON_IP if role == 'jetson' else LAPTOP_IP
    other = LAPTOP_IP if role == 'jetson' else JETSON_IP
    config = link.build_config(role, ip, None if role == 'jetson' else other)
    allow = config['plugins']['ros2dds']['allow']
    strip = lambda names: {n[1:-1] for n in names}
    return strip(allow['publishers']), strip(allow['subscribers'])


def test_gate_and_mission_monitoring_topics_reach_the_laptop():
    jetson_pub, _ = _allowed('jetson')
    _, laptop_sub = _allowed('laptop')
    for topic in ('/gate/path', '/gate/pairs', '/gate/status',
                  '/gate/cmd_preview', '/avoid/plan', '/avoid/cps',
                  '/avoid/dri', '/tank_mission/status', '/mission/state',
                  '/mission/goal', '/mission/entry_goal', '/occupancy_grid'):
        assert topic in jetson_pub and topic in laptop_sub


def test_motor_and_start_commands_are_never_bridged():
    for role in ('jetson', 'laptop'):
        publishers, subscribers = _allowed(role)
        for topic in ('/cmd_vel', '/tank_mission/cmd_raw', '/cmd/gate',
                      '/cmd/avoid', '/start_mission', '/emergency_stop'):
            assert topic not in publishers and topic not in subscribers


def test_laptop_only_sends_pose_and_teleop_and_no_services():
    jetson_pub, jetson_sub = _allowed('jetson')
    laptop_pub, _ = _allowed('laptop')
    expected = {'/boat_pose', '/teleop/enable', '/teleop/cmd_vel'}
    assert laptop_pub == expected and jetson_sub == expected
    config = link.build_config('jetson', JETSON_IP)
    allow = config['plugins']['ros2dds']['allow']
    assert allow['service_servers'] == [] and allow['action_servers'] == []
    json.dumps(config)
