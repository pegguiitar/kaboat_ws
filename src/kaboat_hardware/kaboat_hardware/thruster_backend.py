"""스러스터 하드웨어 통신 백엔드 및 PWM 변환 유틸리티 (ROS 무의존 순수 모듈).

모터 드라이버(ESC)에 인가할 PWM 신호 계산, 불감대 보정, 슬루레이트(가속도 제한)
및 시리얼/I2C/더미 통신 인터페이스를 제공합니다.
"""

from dataclasses import dataclass
import math
import time
from typing import Optional, Tuple


@dataclass
class ThrusterConfig:
    """스러스터 및 PWM 매핑 설정."""

    neutral_pwm: int = 1500        # [µs] 중립 PWM (정지)
    max_forward_pwm: int = 1900    # [µs] 최대 전진 PWM
    max_reverse_pwm: int = 1100    # [µs] 최대 후진 PWM
    deadband_us: int = 25          # [µs] ESC 불감대 폭 (1500 ± deadband_us)

    left_trim: float = 1.0         # 좌측 모터 출력 배율
    right_trim: float = 1.0        # 우측 모터 출력 배율
    invert_left: bool = False      # 좌측 회전 방향 반전
    invert_right: bool = False     # 우측 회전 방향 반전

    reverse_scale: float = 0.8     # 후진 추력 효율 감쇠 보정 계수
    max_slew_rate: float = 2.0     # [초당 최대 변화율] 1.0/s 이면 0->1까지 1초 소요


def clamp(val: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, val))


def ratio_to_pwm(ratio: float, cfg: ThrusterConfig) -> int:
    """정규화된 추력 비율 [-1.0, 1.0]을 ESC PWM 펄스폭[µs]으로 변환한다.

    - ratio == 0.0 이면 neutral_pwm (1500µs)
    - 0.0 < ratio <= 1.0 이면 deadband를 건너뛰고 (neutral_pwm + deadband_us) ~ max_forward_pwm 매핑
    - -1.0 <= ratio < 0.0 이면 deadband를 건너뛰고 (neutral_pwm - deadband_us) ~ max_reverse_pwm 매핑
    """
    ratio = clamp(ratio, -1.0, 1.0)
    if abs(ratio) < 1e-4:
        return cfg.neutral_pwm

    if ratio > 0.0:
        span = cfg.max_forward_pwm - (cfg.neutral_pwm + cfg.deadband_us)
        return int(round(cfg.neutral_pwm + cfg.deadband_us + ratio * span))
    else:
        # 후진: 음수 ratio
        span = (cfg.neutral_pwm - cfg.deadband_us) - cfg.max_reverse_pwm
        return int(round(cfg.neutral_pwm - cfg.deadband_us + ratio * span))


class SlewRateLimiter:
    """급격한 출력 변화를 완화하는 가속도 제한기."""

    def __init__(self, max_rate: float = 2.0, initial_val: float = 0.0):
        self.max_rate = max_rate
        self.current_val = initial_val
        self.last_time: Optional[float] = None

    def reset(self, initial_val: float = 0.0):
        self.current_val = initial_val
        self.last_time = None

    def update(self, target: float, now: Optional[float] = None) -> float:
        if now is None:
            now = time.monotonic()

        if self.last_time is None:
            self.last_time = now
            self.current_val = target
            return self.current_val

        dt = now - self.last_time
        self.last_time = now

        if dt <= 0.0:
            return self.current_val

        max_delta = self.max_rate * dt
        delta = target - self.current_val
        delta_clamped = clamp(delta, -max_delta, max_delta)

        self.current_val += delta_clamped
        return self.current_val


class ThrusterMixer:
    """차동 구동 명령(/cmd_vel) -> 좌우 추력 비율 및 PWM 변환기."""

    def __init__(self, config: ThrusterConfig):
        self.config = config
        self.left_limiter = SlewRateLimiter(max_rate=config.max_slew_rate)
        self.right_limiter = SlewRateLimiter(max_rate=config.max_slew_rate)

    def reset(self):
        self.left_limiter.reset(0.0)
        self.right_limiter.reset(0.0)

    def mix(self, linear_x: float, angular_z: float, now: Optional[float] = None) -> Tuple[int, int, float, float]:
        """선속도/각속도 명령을 좌/우 PWM 및 슬루 제한된 비율로 계산.

        Returns:
            (left_pwm, right_pwm, left_ratio_limited, right_ratio_limited)
        """
        # 차동 믹싱: 좌 = 전진 - 회전, 우 = 전진 + 회전
        raw_left = linear_x - angular_z
        raw_right = linear_x + angular_z

        # 반전 및 트림 적용
        if self.config.invert_left:
            raw_left = -raw_left
        if self.config.invert_right:
            raw_right = -raw_right

        raw_left *= self.config.left_trim
        raw_right *= self.config.right_trim

        # 포화 클램프
        clamped_left = clamp(raw_left, -1.0, 1.0)
        clamped_right = clamp(raw_right, -1.0, 1.0)

        # 슬루 레이트 제한
        limited_left = self.left_limiter.update(clamped_left, now)
        limited_right = self.right_limiter.update(clamped_right, now)

        # PWM 변환
        left_pwm = ratio_to_pwm(limited_left, self.config)
        right_pwm = ratio_to_pwm(limited_right, self.config)

        return left_pwm, right_pwm, limited_left, limited_right


class BaseThrusterBackend:
    """하드웨어 통신 백엔드 기본 인터페이스."""

    def open(self) -> bool:
        raise NotImplementedError

    def send_pwm(self, left_pwm: int, right_pwm: int) -> bool:
        raise NotImplementedError

    def close(self):
        pass


class DummyBackend(BaseThrusterBackend):
    """데스크탑/시뮬레이션/벤치 테스트용 더미 백엔드."""

    def __init__(self):
        self.last_left = 1500
        self.last_right = 1500
        self.is_open = False

    def open(self) -> bool:
        self.is_open = True
        return True

    def send_pwm(self, left_pwm: int, right_pwm: int) -> bool:
        self.last_left = left_pwm
        self.last_right = right_pwm
        return True

    def close(self):
        self.is_open = False


class SerialBackend(BaseThrusterBackend):
    """아두이노/MCU와 시리얼 통신을 수행하는 백엔드.

    프로토콜 형식:
      텍스트 형식: `<PWM_L,PWM_R>\n` (예: `<1650,1520>\n`)
    """

    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200, timeout: float = 0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None

    def open(self) -> bool:
        try:
            import serial
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            return True
        except Exception:
            self.serial = None
            return False

    def send_pwm(self, left_pwm: int, right_pwm: int) -> bool:
        if self.serial is None or not self.serial.is_open:
            return False
        try:
            msg = f"<{left_pwm},{right_pwm}>\n".encode('ascii')
            self.serial.write(msg)
            return True
        except Exception:
            return False

    def close(self):
        if self.serial and self.serial.is_open:
            try:
                # 종료 시 중립 PWM 전송
                self.serial.write(b"<1500,1500>\n")
                self.serial.close()
            except Exception:
                pass
            self.serial = None


class PCA9685Backend(BaseThrusterBackend):
    """Jetson I2C 버스를 통해 PCA9685 16채널 PWM 칩을 제어하는 백엔드."""

    def __init__(self, i2c_bus: int = 1, address: int = 0x40,
                 left_channel: int = 0, right_channel: int = 1,
                 pwm_freq: int = 50):
        self.i2c_bus = i2c_bus
        self.address = address
        self.left_channel = left_channel
        self.right_channel = right_channel
        self.pwm_freq = pwm_freq
        self.bus = None

    def open(self) -> bool:
        try:
            import smbus2
            self.bus = smbus2.SMBus(self.i2c_bus)
            # Normal mode
            self.bus.write_byte_data(self.address, 0x00, 0x00)
            time.sleep(0.005)
            # 50Hz prescale: 25MHz / (4096 * 50) - 1 ≈ 121 (0x79)
            prescale = int(round(25000000.0 / (4096.0 * self.pwm_freq)) - 1)
            old_mode = self.bus.read_byte_data(self.address, 0x00)
            self.bus.write_byte_data(self.address, 0x00, (old_mode & 0x7F) | 0x10) # Sleep
            self.bus.write_byte_data(self.address, 0xFE, prescale)
            self.bus.write_byte_data(self.address, 0x00, old_mode)
            time.sleep(0.005)
            self.bus.write_byte_data(self.address, 0x00, old_mode | 0xA1) # Auto-increment + restart
            return True
        except Exception:
            self.bus = None
            return False

    def _set_channel_pwm_us(self, channel: int, pulse_us: int):
        if self.bus is None:
            return
        # 50Hz = 20000us 주기 -> 4096 틱
        tick = int(round((pulse_us / 20000.0) * 4096))
        tick = max(0, min(4095, tick))
        reg_base = 0x06 + 4 * channel
        data = [0, 0, tick & 0xFF, (tick >> 8) & 0xFF]
        self.bus.write_i2c_block_data(self.address, reg_base, data)

    def send_pwm(self, left_pwm: int, right_pwm: int) -> bool:
        try:
            self._set_channel_pwm_us(self.left_channel, left_pwm)
            self._set_channel_pwm_us(self.right_channel, right_pwm)
            return True
        except Exception:
            return False

    def close(self):
        try:
            self._set_channel_pwm_us(self.left_channel, 1500)
            self._set_channel_pwm_us(self.right_channel, 1500)
            if self.bus:
                self.bus.close()
        except Exception:
            pass

