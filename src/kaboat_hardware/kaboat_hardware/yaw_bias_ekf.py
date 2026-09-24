"""LiDAR 절대 yaw와 IMU gyro-z를 융합하는 2상태 EKF.

상태는 [yaw, gyro_bias]이다. IMU 각속도로 고속 예측하고 LiDAR 표식 yaw로
보정하므로, 절대 heading과 자이로 바이어스를 함께 추정할 수 있다.
"""

import math
from typing import Optional

import numpy as np

from kaboat_hardware.pose_velocity import normalize_angle


class YawBiasEkf:
    def __init__(
        self,
        gyro_noise_stddev: float = 0.01,
        bias_walk_stddev: float = 0.001,
        initial_bias: float = 0.0,
        initial_bias_stddev: float = 0.05,
        max_bias_abs: float = 0.2,
    ):
        self.gyro_noise_var = float(gyro_noise_stddev) ** 2
        self.bias_walk_var = float(bias_walk_stddev) ** 2
        self.initial_bias = float(initial_bias)
        self.initial_bias_var = float(initial_bias_stddev) ** 2
        self.max_bias_abs = abs(float(max_bias_abs))

        self.state = np.array([0.0, self.initial_bias], dtype=float)
        self.covariance = np.diag([math.pi ** 2, self.initial_bias_var])
        self.initialized = False
        self.last_time: Optional[float] = None

    @property
    def yaw(self) -> float:
        return float(self.state[0])

    @property
    def bias(self) -> float:
        return float(self.state[1])

    @property
    def yaw_variance(self) -> float:
        return max(0.0, float(self.covariance[0, 0]))

    def initialize(self, yaw: float, yaw_variance: float) -> None:
        self.state[0] = normalize_angle(float(yaw))
        self.state[1] = self.initial_bias
        self.covariance = np.diag([
            max(float(yaw_variance), 1e-9), self.initial_bias_var])
        self.initialized = True
        self.last_time = None

    def reset_yaw(self, yaw: float, yaw_variance: float) -> None:
        """재획득한 LiDAR yaw로 자세만 재설정하고 학습된 bias는 보존한다."""
        self.state[0] = normalize_angle(float(yaw))
        self.covariance[0, 0] = max(float(yaw_variance), 1e-9)
        self.covariance[0, 1] = 0.0
        self.covariance[1, 0] = 0.0
        self.initialized = True

    def set_bias(self, bias: float, variance: Optional[float] = None) -> None:
        self.state[1] = float(np.clip(bias, -self.max_bias_abs, self.max_bias_abs))
        if variance is not None:
            self.covariance[1, 1] = max(float(variance), 1e-12)
        self.covariance[0, 1] = 0.0
        self.covariance[1, 0] = 0.0

    def predict(self, gyro_z: float, timestamp: float, max_dt: float = 0.2) -> bool:
        if not self.initialized:
            return False
        timestamp = float(timestamp)
        if self.last_time is None:
            self.last_time = timestamp
            return False

        dt = timestamp - self.last_time
        self.last_time = timestamp
        if not math.isfinite(dt) or dt <= 0.0 or dt > max_dt:
            return False

        self.state[0] = normalize_angle(
            self.state[0] + (float(gyro_z) - self.state[1]) * dt)

        transition = np.array([[1.0, -dt], [0.0, 1.0]])
        process_noise = np.array([
            [self.gyro_noise_var * dt * dt, 0.0],
            [0.0, self.bias_walk_var * dt],
        ])
        self.covariance = (
            transition @ self.covariance @ transition.T + process_noise)
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        return True

    def update_yaw(
        self,
        measured_yaw: float,
        measurement_variance: float,
        innovation_gate: float = math.pi,
    ) -> bool:
        measured_yaw = normalize_angle(float(measured_yaw))
        measurement_variance = max(float(measurement_variance), 1e-9)
        if not self.initialized:
            self.initialize(measured_yaw, measurement_variance)
            return True

        innovation = normalize_angle(measured_yaw - self.state[0])
        if abs(innovation) > abs(float(innovation_gate)):
            return False

        measurement = np.array([[1.0, 0.0]])
        innovation_cov = float(
            (measurement @ self.covariance @ measurement.T)[0, 0]
            + measurement_variance)
        gain = (self.covariance @ measurement.T) / innovation_cov
        self.state += gain[:, 0] * innovation
        self.state[0] = normalize_angle(self.state[0])
        self.state[1] = float(np.clip(
            self.state[1], -self.max_bias_abs, self.max_bias_abs))

        identity = np.eye(2)
        residual = identity - gain @ measurement
        # Joseph form은 반복 갱신에서 공분산의 양의 준정부호성을 더 잘 보존한다.
        self.covariance = (
            residual @ self.covariance @ residual.T
            + (gain * measurement_variance) @ gain.T)
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        return True
