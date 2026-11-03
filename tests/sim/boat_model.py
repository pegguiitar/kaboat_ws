"""L2/L3 시험용 3자유도(surge, sway, yaw) 선체 모델 — ROS 무의존.

계수는 Gazebo 선체(kaboat_hull.xacro, target_length 1.1m)와 같은 값을 쓴다.
  - 질량 ∝ s³, 관성 ∝ s⁵, 병진 감쇠 ∝ s², 회전 감쇠 ∝ s⁴ (s = 1.1 / 4.9)
  - 추력 변환은 twist2thrust.py 와 같다:
      좌 = (linear − angular) × max_thrust, 우 = (linear + angular) × max_thrust
    좌우 추진기 간격은 ±1.03·s 이다.
added mass 는 Gazebo 와 같이 0 으로 둔다. 파도·바람은 넣지 않고,
외란은 일정 조류(current)와 odom 잡음으로만 표현한다.
"""
from dataclasses import dataclass, field
import math

import numpy as np

SCALE = 1.1 / 4.9
MAX_THRUST = 118.6  # [N] twist2thrust max_thrust, behavior_base.MAX_THRUST 와 동일


@dataclass
class HullParams:
    mass: float = (180.0 + 2 * 23.5 + 2 * 15.0 + 2 * 0.5) * SCALE ** 3   # [kg] 선체+엔진+프로펠러
    inertia_z: float = (446.0 + 2 * 1.0183 + 2 * 0.078 + 2 * 0.0144) * SCALE ** 5  # [kg·m²]
    half_beam: float = 1.03 * SCALE      # [m] 추진기 좌우 간격의 절반
    max_thrust: float = MAX_THRUST       # [N] 추진기 1개 최대 추력
    x_u: float = 100.0 * SCALE ** 2      # surge 선형 감쇠
    x_uu: float = 150.0 * SCALE ** 2     # surge 2차 감쇠
    y_v: float = 100.0 * SCALE ** 2
    y_vv: float = 100.0 * SCALE ** 2
    n_r: float = 800.0 * SCALE ** 4
    n_rr: float = 800.0 * SCALE ** 4


@dataclass
class BoatState:
    x: float = 0.0      # [m] odom 프레임
    y: float = 0.0
    yaw: float = 0.0    # [rad]
    u: float = 0.0      # [m/s] 선체 전방 속도
    v: float = 0.0      # [m/s] 선체 좌측 속도
    r: float = 0.0      # [rad/s] 요레이트 (좌회전 +)


def thrusts_from_command(linear: float, angular: float, max_thrust: float = MAX_THRUST):
    """정규화 명령 → (좌, 우) 추력 [N]. twist2thrust.py 와 같은 변환·포화."""
    left = (linear - angular) * max_thrust
    right = (linear + angular) * max_thrust
    return (max(-max_thrust, min(max_thrust, left)),
            max(-max_thrust, min(max_thrust, right)))


@dataclass
class BoatModel:
    state: BoatState = field(default_factory=BoatState)
    params: HullParams = field(default_factory=HullParams)
    current: tuple = (0.0, 0.0)   # [m/s] odom 프레임 일정 조류 (수면이 배를 미는 속도)
    physics_dt: float = 0.02      # [s] 적분 주기
    time: float = 0.0             # [s] 시뮬레이션 경과 시간

    def step(self, linear: float, angular: float, dt: float = 0.1) -> BoatState:
        """명령 (linear, angular) 를 dt 동안 유지한다고 보고 상태를 적분한다."""
        p = self.params
        left, right = thrusts_from_command(linear, angular, p.max_thrust)
        force_x = left + right
        moment_z = (right - left) * p.half_beam
        n = max(1, int(round(dt / self.physics_dt)))
        h = dt / n
        s = self.state
        for _ in range(n):
            # 감쇠는 물에 대한 상대속도로 계산한다 (조류를 선체 좌표로 회전).
            c, si = math.cos(s.yaw), math.sin(s.yaw)
            cu = c * self.current[0] + si * self.current[1]
            cv = -si * self.current[0] + c * self.current[1]
            ur, vr = s.u - cu, s.v - cv
            du = (force_x - p.x_u * ur - p.x_uu * abs(ur) * ur) / p.mass + s.v * s.r
            dv = (-p.y_v * vr - p.y_vv * abs(vr) * vr) / p.mass - s.u * s.r
            dr = (moment_z - p.n_r * s.r - p.n_rr * abs(s.r) * s.r) / p.inertia_z
            s.u += du * h
            s.v += dv * h
            s.r += dr * h
            s.x += (c * s.u - si * s.v) * h
            s.y += (si * s.u + c * s.v) * h
            s.yaw = math.atan2(math.sin(s.yaw + s.r * h), math.cos(s.yaw + s.r * h))
        self.time += dt
        return s

    @property
    def world_velocity(self):
        s = self.state
        c, si = math.cos(s.yaw), math.sin(s.yaw)
        return c * s.u - si * s.v, si * s.u + c * s.v


@dataclass
class OdomNoise:
    """odom 측정 잡음. seed 를 고정해 S2 시나리오를 재현 가능하게 한다."""
    position_std: float = 0.0   # [m]
    yaw_std: float = 0.0        # [rad]
    seed: int = 0

    def __post_init__(self):
        self._rng = np.random.default_rng(self.seed)

    def measure(self, state: BoatState) -> BoatState:
        out = BoatState(**vars(state))
        if self.position_std > 0.0:
            out.x += float(self._rng.normal(0.0, self.position_std))
            out.y += float(self._rng.normal(0.0, self.position_std))
        if self.yaw_std > 0.0:
            yaw = out.yaw + float(self._rng.normal(0.0, self.yaw_std))
            out.yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        return out
