"""공통 속도벡터 조종기 단위 테스트."""

import math

import pytest

from kaboat_behaviors.motion_controller import (
    DesiredVelocity, MotionControllerParams, control_velocity,
    speed_to_command)


PARAMS = MotionControllerParams(
    yaw_kp=0.2,
    yaw_kd=0.1,
    cmd_full=0.1,
    v_full=1.5,
    cmd_exp=1.0,
)


def test_aligned_world_velocity_commands_forward_thrust():
    out = control_velocity(DesiredVelocity(1.5, 0.0), 0.0, 0.0, PARAMS)
    assert out.linear_x == pytest.approx(0.1)
    assert out.angular_z == pytest.approx(0.0)


def test_leftward_velocity_generates_positive_yaw_command():
    out = control_velocity(DesiredVelocity(0.0, 1.0), 0.0, 0.0, PARAMS)
    assert out.heading_error == pytest.approx(math.pi / 2)
    assert out.angular_z > 0.0


def test_yaw_rate_is_damped():
    out = control_velocity(DesiredVelocity(1.0, 0.0), 0.0, 0.5, PARAMS)
    assert out.angular_z == pytest.approx(-0.05)


def test_large_heading_error_caps_speed():
    out = control_velocity(
        DesiredVelocity(-1.5, 0.0), 0.0, 0.0, PARAMS,
        turn_speed_limit=0.3)
    assert out.linear_x == pytest.approx(speed_to_command(0.3, PARAMS))


def test_reverse_vector_uses_stern_as_heading_reference():
    out = control_velocity(
        DesiredVelocity(-0.5, 0.0, reverse=True), 0.0, 0.0, PARAMS)
    assert out.linear_x < 0.0
    assert out.angular_z == pytest.approx(0.0)


def test_zero_velocity_stops_without_undefined_heading():
    out = control_velocity(DesiredVelocity(0.0, 0.0), 1.2, 0.4, PARAMS)
    assert out.linear_x == 0.0
    assert out.angular_z == 0.0
    assert out.heading_error == 0.0


def test_invalid_velocity_calibration_is_rejected():
    invalid = MotionControllerParams(0.2, 0.1, 0.1, 0.0, 1.0)
    with pytest.raises(ValueError, match='v_full must be positive'):
        speed_to_command(1.0, invalid)
