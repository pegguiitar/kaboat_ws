"""thruster_driver — 자율 명령 → PWM, 비상정지, 명령 우선순위, 워치독, 파라미터.

노트북 텔레옵 전환은 test_thruster_driver_teleop.py 가 확인한다.
dummy 백엔드를 쓰고, time.monotonic 을 가짜 시계로 바꿔 시각을 직접 넘긴다.
"""
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from kaboat_hardware.thruster_driver import ThrusterDriver


def _twist(linear=0.0, angular=0.0):
    msg = Twist()
    msg.linear.x = float(linear)
    msg.angular.z = float(angular)
    return msg


@pytest.fixture
def make_driver(monkeypatch):
    clock = {'t': 100.0}
    monkeypatch.setattr(time, 'monotonic', lambda: clock['t'])
    nodes = []

    def factory(*params):
        args = ['--ros-args']
        for p in params:
            args += ['-p', p]
        rclpy.init(args=args)
        node = ThrusterDriver()
        node.clock = clock
        node._resend = None
        nodes.append(node)
        return node
    yield factory
    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def _settle(node, seconds=1.0, dt=0.02):
    """슬루 제한이 목표에 도달할 만큼 제어 루프를 돌린다. 명령은 매 주기 새로 받는다."""
    for _ in range(int(seconds / dt)):
        node.clock['t'] += dt
        if node.active_mode in ('auto', 'idle', 'timeout') and node._resend is not None:
            node._on_cmd_vel(node._resend)
        node._control_loop()
    return node.backend.last_left, node.backend.last_right


def _drive(node, cmd):
    node._resend = cmd
    node._on_cmd_vel(cmd)
    return _settle(node)


@pytest.mark.check('A')
def test_forward_command_raises_both_pwm_equally(make_driver):
    node = make_driver()
    left, right = _drive(node, _twist(0.5))
    assert node.active_mode == 'auto'
    assert left == right > node.config.neutral_pwm


@pytest.mark.check('A')
def test_positive_angular_turns_left(make_driver):
    node = make_driver()
    left, right = _drive(node, _twist(0.0, 0.3))
    assert right > node.config.neutral_pwm > left


@pytest.mark.check('A')
def test_emergency_stop_forces_neutral_until_released(make_driver):
    node = make_driver()
    _drive(node, _twist(0.5))
    node._on_estop(Bool(data=True))
    left, right = _settle(node, 0.1)
    assert node.active_mode == 'estop'
    assert left == right == node.config.neutral_pwm
    node._on_estop(Bool(data=False))
    left, _ = _settle(node)
    assert node.active_mode == 'auto' and left > node.config.neutral_pwm


@pytest.mark.check('C')
def test_nan_autonomy_command_is_neutral(make_driver):
    node = make_driver()
    left, right = _drive(node, _twist(float('nan'), 0.0))
    assert left == right == node.config.neutral_pwm


@pytest.mark.check('B')
def test_rc_override_beats_auto_then_hands_back(make_driver):
    node = make_driver()
    _drive(node, _twist(0.5))
    node._on_rc_cmd(_twist(-0.3))
    node.clock['t'] += 0.02
    node._control_loop()
    assert node.active_mode == 'rc_override'
    node.clock['t'] += 0.6                       # rc_timeout_sec 0.5 경과
    node._on_cmd_vel(_twist(0.5))
    node._control_loop()
    assert node.active_mode == 'auto'


@pytest.mark.check('B')
def test_auto_watchdog_boundary_drops_to_neutral_without_slew(make_driver):
    node = make_driver()
    node._resend = None
    node._on_cmd_vel(_twist(0.5))
    for _ in range(50):
        node.clock['t'] += 0.005                  # 0.25s — 명령 갱신 없이 300ms 이내
        node._control_loop()
    assert node.active_mode == 'auto'
    node.clock['t'] += 0.06                       # 마지막 명령 후 0.31s
    node._control_loop()
    assert node.active_mode == 'timeout'
    assert node.backend.last_left == node.backend.last_right == node.config.neutral_pwm


@pytest.mark.check('B')
def test_max_output_ratio_caps_pwm(make_driver):
    node = make_driver('max_output_ratio:=0.5')
    left, right = _drive(node, _twist(1.0))
    cfg = node.config
    half = cfg.neutral_pwm + cfg.deadband_us + 0.5 * (
        cfg.max_forward_pwm - cfg.neutral_pwm - cfg.deadband_us)
    assert left == right == pytest.approx(half, abs=1)   # 전출력 명령도 50% 에서 멈춘다


@pytest.mark.check('B')
@pytest.mark.parametrize('param', [
    'max_output_ratio:=1.5', 'teleop_timeout_sec:=3.0', 'teleop_max_forward:=1.2'])
def test_invalid_safety_parameters_are_rejected(param):
    rclpy.init(args=['--ros-args', '-p', param])
    try:
        with pytest.raises(ValueError):
            ThrusterDriver()
    finally:
        rclpy.shutdown()
