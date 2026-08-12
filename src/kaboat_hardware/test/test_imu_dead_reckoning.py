import math

import pytest

from kaboat_hardware.imu_dead_reckoning_core import (
    DeadReckoningParams,
    ImuDeadReckoner,
    rotate_vector,
)


IDENTITY = (0.0, 0.0, 0.0, 1.0)


def _calibrate(estimator, accel=(0.0, 0.0, 9.80665), duration=0.3):
    state = None
    for index in range(int(duration / 0.01) + 2):
        state = estimator.update(
            index * 0.01, IDENTITY, (0.0, 0.0, 0.001), accel)
    assert estimator.ready
    return state


def test_rotate_vector_quarter_turn_about_z():
    half = math.pi / 4.0
    q = (0.0, 0.0, math.sin(half), math.cos(half))
    x, y, z = rotate_vector(q, (1.0, 0.0, 0.0))
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(1.0, abs=1e-9)
    assert z == pytest.approx(0.0, abs=1e-9)


def test_stationary_calibration_estimates_sensor_bias():
    params = DeadReckoningParams(calibration_duration_sec=0.2)
    estimator = ImuDeadReckoner(params)
    _calibrate(estimator, accel=(0.04, -0.03, params.gravity + 0.02))
    assert estimator.accel_bias_body == pytest.approx((0.04, -0.03, 0.02))
    assert estimator.gyro_bias_z == pytest.approx(0.001)


def test_one_second_constant_acceleration_integrates_position():
    params = DeadReckoningParams(
        calibration_duration_sec=0.2,
        accel_deadband=0.0,
        accel_filter_tau=0.0,
        velocity_damping_tau=0.0,
        max_dt=0.02,
    )
    estimator = ImuDeadReckoner(params)
    _calibrate(estimator)
    start = estimator._last_t
    state = None
    for index in range(1, 101):
        state = estimator.update(
            start + index * 0.01, IDENTITY, (0.0, 0.0, 0.0),
            (1.0, 0.0, params.gravity))
    assert state.vx_world == pytest.approx(1.0, abs=0.02)
    assert state.x == pytest.approx(0.5, abs=0.02)
    assert state.y == pytest.approx(0.0, abs=1e-9)


def test_large_timestamp_gap_does_not_jump_position():
    params = DeadReckoningParams(
        calibration_duration_sec=0.2,
        accel_deadband=0.0,
        accel_filter_tau=0.0,
        velocity_damping_tau=0.0,
    )
    estimator = ImuDeadReckoner(params)
    _calibrate(estimator)
    before = estimator.x
    estimator.update(
        estimator._last_t + 1.0, IDENTITY, (0.0, 0.0, 0.0),
        (2.0, 0.0, params.gravity))
    assert estimator.x == before


def test_reset_pose_preserves_calibration_and_zeros_motion():
    params = DeadReckoningParams(
        calibration_duration_sec=0.2,
        accel_deadband=0.0,
        accel_filter_tau=0.0,
        velocity_damping_tau=0.0,
    )
    estimator = ImuDeadReckoner(params)
    _calibrate(estimator)
    for index in range(1, 20):
        estimator.update(
            estimator._last_t + 0.01, IDENTITY, (0.0, 0.0, 0.0),
            (1.0, 0.0, params.gravity))
    estimator.reset_pose()
    assert estimator.ready
    assert estimator.x == estimator.y == 0.0
    assert estimator.vx == estimator.vy == 0.0
