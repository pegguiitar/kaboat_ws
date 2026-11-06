"""젯슨 thruster_driver: 노트북 [속도 단계, 선회] → PWM, 링크 끊김 중립, ESP 모드 문자열."""

import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int8MultiArray, String

from kaboat_hardware.thruster_driver import ThrusterDriver, level_to_pwm


def twist(linear=0.0, angular=0.0):
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    return msg


def levels(level, turn):
    """노트북이 보내는 [속도 단계, 선회 +1(A)/0/-1(D)]."""
    return Int8MultiArray(data=[level, turn])


def test_level_to_pwm_table_and_inversion():
    offsets = [50, 100, 150, 200]
    assert [level_to_pwm(k, offsets, 1500) for k in range(-4, 5)] == [
        1300, 1350, 1400, 1450, 1500, 1550, 1600, 1650, 1700]
    assert level_to_pwm(3, offsets, 1500, invert=True) == 1350


def test_laptop_levels_override_auto_and_timeout_stays_neutral():
    rclpy.init()
    node = ThrusterDriver()  # dummy backend; no physical port
    try:
        node._on_cmd_vel(twist(0.08))
        node._control_loop()
        assert node.active_mode == 'auto'

        node._on_teleop_enable(Bool(data=True))
        node._control_loop()
        assert node.active_mode == 'teleop_timeout'
        assert node.backend.last_left == node.config.neutral_pwm
        assert node.backend.last_right == node.config.neutral_pwm

        node._on_teleop_levels(levels(3, 0))        # 전진 3단계 직진
        node._control_loop()
        assert node.active_mode == 'teleop'
        assert (node.backend.last_left, node.backend.last_right) == (1650, 1650)

        node._on_teleop_levels(levels(3, 1))        # 전진 3단계 + A (선회 폭 100 µs)
        node._control_loop()
        assert (node.backend.last_left, node.backend.last_right) == (1550, 1750)

        node._on_teleop_levels(levels(3, -1))       # 전진 3단계 + D
        node._control_loop()
        assert (node.backend.last_left, node.backend.last_right) == (1750, 1550)

        node._on_teleop_levels(levels(0, 1))        # 중립 + A = 제자리 좌회전
        node._control_loop()
        assert (node.backend.last_left, node.backend.last_right) == (1400, 1600)

        node.last_teleop_cmd_time = time.monotonic() - 2.0
        node._control_loop()
        assert node.active_mode == 'teleop_timeout'
        assert node.backend.last_left == node.config.neutral_pwm
        assert node.backend.last_right == node.config.neutral_pwm

        node._on_teleop_enable(Bool(data=False))
        node._control_loop()
        assert node.active_mode == 'timeout'  # stale auto command is discarded
        node._on_cmd_vel(twist(0.08))
        node._control_loop()
        assert node.active_mode == 'auto'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_invalid_levels_and_disabled_teleop_are_ignored():
    rclpy.init()
    node = ThrusterDriver()
    try:
        node._on_teleop_levels(levels(1, 1))        # 텔레옵 모드 전에는 무시
        assert node.current_teleop_levels == (0, 0)

        node._on_teleop_enable(Bool(data=True))
        node._on_teleop_levels(levels(3, 0))
        node._on_teleop_levels(levels(2, 4))        # 선회는 -1~1 (예전 [좌, 우] 형식) → 중립
        node._control_loop()
        assert node.active_mode == 'teleop_timeout'
        assert node.backend.last_left == node.config.neutral_pwm
        assert node.backend.last_right == node.config.neutral_pwm

        node._on_teleop_levels(Int8MultiArray(data=[1]))   # 길이 오류
        node._control_loop()
        assert node.active_mode == 'teleop_timeout'
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_max_output_ratio_still_caps_keyboard_levels():
    rclpy.init()
    node = ThrusterDriver()
    try:
        node.teleop_pwm_high, node.teleop_pwm_low = 1600, 1400   # max_output_ratio ≈ 0.2
        node._on_teleop_enable(Bool(data=True))
        node._on_teleop_levels(levels(4, 0))
        node._control_loop()
        assert (node.backend.last_left, node.backend.last_right) == (1600, 1600)
        node._on_teleop_levels(levels(-4, 1))
        node._control_loop()
        assert (node.backend.last_left, node.backend.last_right) == (1400, 1400)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_esp_mode_strings_are_sent_and_others_rejected():
    rclpy.init()
    node = ThrusterDriver()
    events = []
    try:
        node.teleop_event_pub.publish = events.append
        node._on_esp_mode(String(data='computer_control_mode'))
        node._on_esp_mode(String(data='rc_control_mode'))
        node._on_esp_mode(String(data='15001500'))           # 화이트리스트 밖
        assert node.backend.lines == ['computer_control_mode', 'rc_control_mode']
        assert node.esp_mode == 'rc_control_mode'
        assert '전송 완료' in events[0].data and 'REJECTED' in events[-1].data
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_turn_width_and_inversion():
    rclpy.init()
    node = ThrusterDriver()
    try:
        node._on_teleop_enable(Bool(data=True))
        node.teleop_turn_offset = 150
        node._on_teleop_levels(levels(1, 1))        # 1550 ± 150
        node._control_loop()
        assert (node.backend.last_left, node.backend.last_right) == (1400, 1700)
        node.config.invert_left = True              # 좌 배선 반전 → 중립 기준 대칭
        node._control_loop()
        assert (node.backend.last_left, node.backend.last_right) == (1600, 1700)
    finally:
        node.destroy_node()
        rclpy.shutdown()
