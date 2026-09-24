import math

from kaboat_hardware.pose_velocity import normalize_angle
from kaboat_hardware.yaw_bias_ekf import YawBiasEkf


def test_predict_integrates_bias_corrected_gyro():
    ekf = YawBiasEkf(initial_bias=0.1)
    ekf.initialize(0.0, 0.01)
    assert not ekf.predict(0.6, 10.0)
    assert ekf.predict(0.6, 10.1)
    assert math.isclose(ekf.yaw, 0.05, abs_tol=1e-9)


def test_lidar_updates_converge_constant_gyro_bias():
    ekf = YawBiasEkf(
        gyro_noise_stddev=0.005, bias_walk_stddev=0.0005,
        initial_bias_stddev=0.10)
    ekf.initialize(0.0, math.radians(1.0) ** 2)
    ekf.predict(0.01, 0.0)
    for step in range(1, 601):
        timestamp = step * 0.01
        ekf.predict(0.01, timestamp)
        if step % 10 == 0:
            assert ekf.update_yaw(
                0.0, math.radians(1.0) ** 2, math.radians(45.0))

    assert abs(ekf.yaw) < math.radians(0.2)
    assert math.isclose(ekf.bias, 0.01, abs_tol=0.002)


def test_update_uses_shortest_angle_across_wraparound():
    ekf = YawBiasEkf()
    ekf.initialize(math.radians(179.0), math.radians(5.0) ** 2)
    assert ekf.update_yaw(
        math.radians(-179.0), math.radians(1.0) ** 2,
        math.radians(10.0))
    error = normalize_angle(ekf.yaw - math.radians(-179.0))
    assert abs(error) < math.radians(0.2)


def test_large_lidar_innovation_is_rejected():
    ekf = YawBiasEkf()
    ekf.initialize(0.0, math.radians(1.0) ** 2)
    assert not ekf.update_yaw(
        math.radians(100.0), math.radians(1.0) ** 2,
        math.radians(45.0))
    assert ekf.yaw == 0.0


def test_reset_yaw_preserves_learned_bias():
    ekf = YawBiasEkf()
    ekf.initialize(0.0, 0.01)
    ekf.set_bias(0.012, variance=1e-6)
    ekf.reset_yaw(1.2, 0.02)
    assert math.isclose(ekf.yaw, 1.2)
    assert math.isclose(ekf.bias, 0.012)

