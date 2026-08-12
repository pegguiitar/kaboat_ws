"""짧은 실내 시험용 IMU dead-reckoning의 ROS 비의존 계산 코어."""

from dataclasses import dataclass
import math


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def normalize_quaternion(q):
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1e-9:
        raise ValueError('zero-norm quaternion')
    return tuple(value / norm for value in q)


def conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def rotate_vector(q, vector):
    """ROS quaternion으로 body 벡터를 world 좌표로 회전한다."""
    x, y, z, w = normalize_quaternion(q)
    vx, vy, vz = vector
    # q * v * q^-1을 행렬 형태로 전개했다.
    return (
        (1.0 - 2.0 * (y*y + z*z)) * vx
        + 2.0 * (x*y - z*w) * vy
        + 2.0 * (x*z + y*w) * vz,
        2.0 * (x*y + z*w) * vx
        + (1.0 - 2.0 * (x*x + z*z)) * vy
        + 2.0 * (y*z - x*w) * vz,
        2.0 * (x*z - y*w) * vx
        + 2.0 * (y*z + x*w) * vy
        + (1.0 - 2.0 * (x*x + y*y)) * vz,
    )


def yaw_from_quaternion(q) -> float:
    x, y, z, w = normalize_quaternion(q)
    return math.atan2(2.0 * (w*z + x*y),
                      1.0 - 2.0 * (y*y + z*z))


@dataclass(frozen=True)
class DeadReckoningParams:
    gravity: float = 9.80665
    calibration_duration_sec: float = 2.0
    calibration_gyro_limit: float = 0.08
    calibration_accel_tolerance: float = 0.8
    accel_deadband: float = 0.06
    accel_filter_tau: float = 0.08
    velocity_damping_tau: float = 8.0
    max_accel: float = 3.0
    max_speed: float = 2.0
    max_dt: float = 0.05
    zupt_enabled: bool = False
    stationary_accel_threshold: float = 0.10
    stationary_gyro_threshold: float = 0.025
    stationary_hold_sec: float = 0.5


@dataclass(frozen=True)
class DeadReckoningState:
    x: float
    y: float
    yaw: float
    vx_world: float
    vy_world: float
    ax_world: float
    ay_world: float
    yaw_rate: float
    stationary: bool


class ImuDeadReckoner:
    """자세로 중력을 제거하고 수평 가속도를 두 번 적분한다.

    절대 위치 센서가 없는 동안 Occupancy Grid의 파이프라인만 확인하기 위한
    추정기다. 장기 항법 정확도를 제공하지 않는다.
    """

    def __init__(self, params=None):
        self.params = params or DeadReckoningParams()
        self._calibration_start = None
        self._calibration_count = 0
        self._bias_sum = [0.0, 0.0, 0.0]
        self._gyro_z_sum = 0.0
        self.ready = False
        self.accel_bias_body = (0.0, 0.0, 0.0)
        self.gyro_bias_z = 0.0
        self._last_t = None
        self._last_abs_yaw = 0.0
        self._yaw_origin = 0.0
        self._filtered_accel = (0.0, 0.0)
        self._previous_accel = (0.0, 0.0)
        self._stationary_since = None
        self.x = self.y = 0.0
        self.vx = self.vy = 0.0

    @property
    def calibration_progress(self) -> float:
        if self.ready:
            return 1.0
        if self._calibration_start is None or self._last_t is None:
            return 0.0
        duration = max(self.params.calibration_duration_sec, 1e-6)
        return min(1.0, max(0.0, (self._last_t - self._calibration_start) / duration))

    def _reset_calibration(self, timestamp):
        self._calibration_start = timestamp
        self._calibration_count = 0
        self._bias_sum = [0.0, 0.0, 0.0]
        self._gyro_z_sum = 0.0
        self._last_t = timestamp

    def reset_pose(self):
        """보정값은 유지하고 위치·속도·상대 yaw만 원점으로 되돌린다."""
        self.x = self.y = 0.0
        self.vx = self.vy = 0.0
        self._filtered_accel = (0.0, 0.0)
        self._previous_accel = (0.0, 0.0)
        self._stationary_since = None
        self._yaw_origin = self._last_abs_yaw

    def update(self, timestamp, quaternion, angular_velocity, linear_acceleration):
        q = normalize_quaternion(quaternion)
        gyro = tuple(float(value) for value in angular_velocity)
        accel = tuple(float(value) for value in linear_acceleration)
        abs_yaw = yaw_from_quaternion(q)
        self._last_abs_yaw = abs_yaw

        if not self.ready:
            return self._calibrate(timestamp, q, gyro, accel, abs_yaw)

        dt = timestamp - self._last_t
        self._last_t = timestamp
        yaw = normalize_angle(abs_yaw - self._yaw_origin)
        yaw_rate = gyro[2] - self.gyro_bias_z
        if dt <= 0.0 or dt > self.params.max_dt:
            self._previous_accel = (0.0, 0.0)
            return self._state(yaw, yaw_rate, False)

        corrected_body = tuple(
            accel[i] - self.accel_bias_body[i] for i in range(3))
        aw = rotate_vector(q, corrected_body)
        ax, ay = aw[0], aw[1]
        magnitude = math.hypot(ax, ay)
        if magnitude <= self.params.accel_deadband:
            ax = ay = 0.0
        elif magnitude > 0.0:
            scale = (magnitude - self.params.accel_deadband) / magnitude
            ax *= scale
            ay *= scale

        magnitude = math.hypot(ax, ay)
        if magnitude > self.params.max_accel:
            scale = self.params.max_accel / magnitude
            ax *= scale
            ay *= scale

        tau = self.params.accel_filter_tau
        alpha = 1.0 if tau <= 0.0 else dt / (tau + dt)
        ax = self._filtered_accel[0] + alpha * (ax - self._filtered_accel[0])
        ay = self._filtered_accel[1] + alpha * (ay - self._filtered_accel[1])
        self._filtered_accel = (ax, ay)

        stationary = self._stationary(timestamp, gyro, ax, ay)
        if stationary:
            self.vx = self.vy = 0.0
            self._previous_accel = (0.0, 0.0)
            return self._state(yaw, yaw_rate, True)

        old_vx, old_vy = self.vx, self.vy
        self.vx += 0.5 * (self._previous_accel[0] + ax) * dt
        self.vy += 0.5 * (self._previous_accel[1] + ay) * dt
        damping_tau = self.params.velocity_damping_tau
        if damping_tau > 0.0:
            damping = math.exp(-dt / damping_tau)
            self.vx *= damping
            self.vy *= damping

        speed = math.hypot(self.vx, self.vy)
        if speed > self.params.max_speed:
            scale = self.params.max_speed / speed
            self.vx *= scale
            self.vy *= scale

        self.x += 0.5 * (old_vx + self.vx) * dt
        self.y += 0.5 * (old_vy + self.vy) * dt
        self._previous_accel = (ax, ay)
        return self._state(yaw, yaw_rate, False)

    def _calibrate(self, timestamp, q, gyro, accel, abs_yaw):
        accel_norm = math.sqrt(sum(value * value for value in accel))
        gyro_norm = math.sqrt(sum(value * value for value in gyro))
        moving = (
            abs(accel_norm - self.params.gravity)
            > self.params.calibration_accel_tolerance
            or gyro_norm > self.params.calibration_gyro_limit
        )
        if self._calibration_start is None or moving:
            self._reset_calibration(timestamp)
            return None

        gravity_body = rotate_vector(
            conjugate(q), (0.0, 0.0, self.params.gravity))
        for i in range(3):
            self._bias_sum[i] += accel[i] - gravity_body[i]
        self._gyro_z_sum += gyro[2]
        self._calibration_count += 1
        self._last_t = timestamp

        elapsed = timestamp - self._calibration_start
        if elapsed < self.params.calibration_duration_sec or self._calibration_count < 10:
            return None

        self.accel_bias_body = tuple(
            total / self._calibration_count for total in self._bias_sum)
        self.gyro_bias_z = self._gyro_z_sum / self._calibration_count
        self.ready = True
        self._yaw_origin = abs_yaw
        self._last_abs_yaw = abs_yaw
        return self._state(0.0, gyro[2] - self.gyro_bias_z, True)

    def _stationary(self, timestamp, gyro, ax, ay):
        if not self.params.zupt_enabled:
            self._stationary_since = None
            return False
        corrected_gyro = math.sqrt(
            gyro[0]**2 + gyro[1]**2 + (gyro[2] - self.gyro_bias_z)**2)
        candidate = (
            math.hypot(ax, ay) < self.params.stationary_accel_threshold
            and corrected_gyro < self.params.stationary_gyro_threshold
        )
        if not candidate:
            self._stationary_since = None
            return False
        if self._stationary_since is None:
            self._stationary_since = timestamp
        return timestamp - self._stationary_since >= self.params.stationary_hold_sec

    def _state(self, yaw, yaw_rate, stationary):
        return DeadReckoningState(
            x=self.x,
            y=self.y,
            yaw=yaw,
            vx_world=self.vx,
            vy_world=self.vy,
            ax_world=self._filtered_accel[0],
            ay_world=self._filtered_accel[1],
            yaw_rate=yaw_rate,
            stationary=stationary,
        )
