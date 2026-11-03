"""Verify Jetson arbitration keeps a lost laptop link neutral."""

import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from kaboat_hardware.thruster_driver import ThrusterDriver


def twist(linear=0.0, angular=0.0):
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    return msg


def test_laptop_mode_overrides_auto_and_timeout_stays_neutral():
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

        node._on_teleop_cmd(twist(-0.04, 0.02))
        node._control_loop()
        assert node.active_mode == 'teleop'
        assert node.current_teleop_cmd.linear.x < 0.0
        assert node.current_teleop_cmd.angular.z > 0.0

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


def test_jetson_caps_remote_commands_and_rejects_nan():
    rclpy.init()
    node = ThrusterDriver()
    try:
        node._on_teleop_enable(Bool(data=True))
        node._on_teleop_cmd(twist(-1.0, 1.0))
        assert node.current_teleop_cmd.linear.x == -node.teleop_max_reverse
        assert node.current_teleop_cmd.angular.z == node.teleop_max_angular
        node._on_teleop_cmd(twist(float('nan'), 0.0))
        node._control_loop()
        assert node.active_mode == 'teleop_timeout'
        assert node.backend.last_left == node.config.neutral_pwm
        assert node.backend.last_right == node.config.neutral_pwm
    finally:
        node.destroy_node()
        rclpy.shutdown()
