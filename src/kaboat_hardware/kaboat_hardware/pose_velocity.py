"""절대 pose 열에서 속도를 뽑는 계산 (ROS 무의존).

AprilTag 는 프레임마다 자세(pose)만 준다 — 속도는 없다. 그런데 스택은
`/odom` 의 twist 를 실제로 쓴다:
  - behavior_base.seek_goal() 의 yaw PD D항 (없으면 P 제어만 남아 요 발진 →
    전복. SKELETON §6 의 실측 사고 2건 중 하나)
  - obstacle_planner 가 avoid FSM 에 넘기는 속도벡터

그래서 위치를 미분해 채운다. 설계 규칙 셋:

1) **적분은 절대 안 한다.** 가속도계를 적분하면 바이어스가 시간에 비례해
   쌓이고, 자세 오차 1° 만 있어도 중력이 수평축으로 새어 10초에 1.7 m/s 의
   가짜 속도가 생긴다(전속 1.48 m/s 보다 크다). 절대 측정(AprilTag)을
   미분하면 오차가 안 쌓인다.

2) **각속도는 미분하지 않고 자이로에서 직접 받는다.** yaw 를 미분하면
   각도 노이즈 1° 가 30fps 에서 0.52 rad/s 로 증폭되는데, 이건 이 배의
   실측 ω_max 와 같은 크기다. D항에 넣으면 노이즈만으로 조향 명령이
   포화한다. 자이로는 각속도를 직접 재므로 증폭이 없다. IMU 가 없을 때만
   폴백으로 미분한다(각도 랩어라운드 처리 포함).

3) **필터는 월드 좌표에서 걸고, 마지막에 body 로 돌린다.** body 에서 필터를
   걸면 배가 회전할 때 회전 성분이 속도에 섞인다.

출력 규약: `/odom` 의 twist 는 **body frame** 이다
(obstacle_planner 가 body→world 로 되돌리는 회전을 한다).
"""
from collections import deque
from dataclasses import dataclass
import math


def normalize_angle(a: float) -> float:
    """[-π, π) 로 감는다 — yaw 차분에 필수(+179°→-179° 가 -358° 로 튀는 것 방지)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def world_to_body(vx: float, vy: float, yaw: float):
    """월드 속도 → 선체 기준 (전방 x, 좌측 y). obstacle_planner 의 역변환."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


@dataclass
class VelocityParams:
    # 미분 baseline [s]. 인접 두 프레임(1/30s)으로 차분하면 노이즈가 크다 —
    # 간격을 늘린 만큼 노이즈가 줄고, 대가는 그만큼의 지연이다.
    window_sec: float = 0.15
    # 미분 후 EMA 시정수 [s]. 0 이면 필터 없음. 키우면 조용해지지만 지연이
    # 늘어 D항의 감쇠 효과가 떨어진다.
    filter_tau: float = 0.15
    # 이 속도[m/s]를 넘는 미분값은 검출 튐으로 보고 버린다 (실측 전속 1.48).
    max_speed: float = 3.0
    # 이 각속도[rad/s]를 넘는 yaw 미분은 버린다 (실측 ω_max 0.52).
    max_yaw_rate: float = 3.0


class VelocityEstimator:
    """pose 표본을 받아 body frame 속도와 (폴백용) yaw rate 를 낸다.

    표본 시각은 **검출 시각**을 쓴다(수신 시각이 아니라) — 네트워크 지터가
    그대로 속도에 들어가는 것을 막는다.
    """

    def __init__(self, params: VelocityParams | None = None):
        self.p = params or VelocityParams()
        self._samples = deque()      # (t, x, y, yaw)
        self._vx_w = 0.0             # 필터 통과 월드 속도
        self._vy_w = 0.0
        self._yaw_rate = 0.0
        self._last_t = None

    def reset(self) -> None:
        """태그 유실 후 재획득 등 — 시간 공백을 넘어 미분하지 않도록 비운다."""
        self._samples.clear()
        self._vx_w = self._vy_w = self._yaw_rate = 0.0
        self._last_t = None

    def update(self, t: float, x: float, y: float, yaw: float):
        """표본 하나를 넣고 (vx_body, vy_body, yaw_rate) 를 돌려준다."""
        self._samples.append((t, x, y, yaw))
        # window 를 덮는 최소한만 남긴다 — 가장 오래된 표본이 baseline 시작점
        while len(self._samples) > 2 and self._samples[1][0] <= t - self.p.window_sec:
            self._samples.popleft()

        t0, x0, y0, yaw0 = self._samples[0]
        dt = t - t0
        if dt > 0.0:
            vx_raw = (x - x0) / dt
            vy_raw = (y - y0) / dt
            yaw_rate_raw = normalize_angle(yaw - yaw0) / dt

            # 이상치 게이트 — 검출이 튀면 직전 추정을 유지한다
            if math.hypot(vx_raw, vy_raw) <= self.p.max_speed:
                a = self._alpha(t)
                self._vx_w += a * (vx_raw - self._vx_w)
                self._vy_w += a * (vy_raw - self._vy_w)
            if abs(yaw_rate_raw) <= self.p.max_yaw_rate:
                a = self._alpha(t)
                self._yaw_rate += a * (yaw_rate_raw - self._yaw_rate)

        self._last_t = t
        vx_b, vy_b = world_to_body(self._vx_w, self._vy_w, yaw)
        return vx_b, vy_b, self._yaw_rate

    def _alpha(self, t: float) -> float:
        """실제 경과시간 기반 EMA 계수 — 표본 주기가 흔들려도 시정수가 유지된다."""
        if self.p.filter_tau <= 0.0 or self._last_t is None:
            return 1.0
        step = max(0.0, t - self._last_t)
        return step / (self.p.filter_tau + step) if step > 0.0 else 0.0
