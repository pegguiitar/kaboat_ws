"""출발 직진(mission_manager undock_linear)이 실물 설정에서 텔레옵 4단계와 같은 PWM 인지.

thrusters.yaml(neutral·deadband·max_forward·teleop_level_pwm_offsets)을 바꾸면 0.4667 이 더는
1700 µs 가 아니게 된다 — 그때 이 테스트가 깨져서 mission_manager UNDOCK_LINEAR 도 같이 고치게 한다.
"""

import os

import yaml

from kaboat_hardware.thruster_backend import ThrusterConfig, ThrusterMixer
from kaboat_hardware.thruster_driver import level_to_pwm
from kaboat_mission.mission_manager import UNDOCK_LINEAR

YAML = os.path.join(os.path.dirname(__file__), '..', 'config', 'thrusters.yaml')


def test_undock_linear_equals_teleop_level4_pwm():
    p = yaml.safe_load(open(YAML))['thruster_driver']['ros__parameters']
    cfg = ThrusterConfig(
        neutral_pwm=p['neutral_pwm'], max_forward_pwm=p['max_forward_pwm'],
        max_reverse_pwm=p['max_reverse_pwm'], deadband_us=p['deadband_us'],
        left_trim=p['left_trim'], right_trim=p['right_trim'],
        invert_left=p['invert_left'], invert_right=p['invert_right'],
        max_slew_rate=1e9, max_output_ratio=p['max_output_ratio'])
    level4 = level_to_pwm(4, p['teleop_level_pwm_offsets'], p['neutral_pwm'])
    assert level4 == 1700
    left, right, _, _ = ThrusterMixer(cfg).mix(UNDOCK_LINEAR, 0.0, now=0.0)
    assert left == right == level4
