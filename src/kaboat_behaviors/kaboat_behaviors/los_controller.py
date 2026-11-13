"""Reusable LOS follower for sampled B-splines and circular paths.

The path is prepared once in the odom frame. Call ``step`` each control tick;
it tracks forward progress, follows a lookahead point with the same controller
as bspline_track_test, and slows for the maximum curvature ahead.
"""

from dataclasses import dataclass, replace
import math

import numpy as np
from scipy.interpolate import BSpline

from .avoid_fsm import FsmParams, follow_cmd
from .bspline_planner import SampledPlan, curvature_ahead


def sample_bspline(control_points, spacing: float = 0.05, degree: int = 3):
    """Return arc-spaced X/Y, tangent, analytic curvature and length.

    This is the geometry previously local to bspline_track_test; its public
    return contract is kept so that existing tank-test callers still work.
    """
    pts = np.asarray(control_points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2:
        raise ValueError('B-spline needs at least two XY control points')
    if not np.all(np.isfinite(pts)):
        raise ValueError('B-spline control points must be finite')
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError('spacing must be positive and finite')
    if not isinstance(degree, int) or degree < 1:
        raise ValueError('degree must be a positive integer')
    k = min(degree, len(pts) - 1)
    internal = np.linspace(0.0, 1.0, len(pts) - k + 1)[1:-1]
    knots = np.concatenate((np.zeros(k + 1), internal, np.ones(k + 1)))
    spline = BSpline(knots, pts, k, axis=0)

    dense_u = np.linspace(0.0, 1.0, 1000)
    dense = spline(dense_u)
    segment_lengths = np.hypot(*np.diff(dense, axis=0).T)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    length = float(cumulative[-1])
    if length <= 1e-9:
        raise ValueError('B-spline has no length')
    distances = (np.array([0.0, length]) if length <= spacing
                 else np.arange(0.0, length, spacing))
    if distances[-1] < length:
        distances = np.append(distances, length)
    u = np.interp(distances, cumulative, dense_u)
    points = spline(u)
    d1 = spline(u, nu=1)
    d2 = spline(u, nu=2) if k >= 2 else np.zeros_like(d1)
    headings = np.arctan2(d1[:, 1], d1[:, 0])
    denominator = np.maximum((d1[:, 0] ** 2 + d1[:, 1] ** 2) ** 1.5, 1e-6)
    curvature = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / denominator
    return points[:, 0], points[:, 1], headings, curvature, length


def _validate_path(path: SampledPlan) -> None:
    if not isinstance(path, SampledPlan):
        raise TypeError('path must be built with sampled_path, bspline_path or circle_path')
    samples = np.asarray(path.samples)
    curvature = np.asarray(path.curvatures)
    if (samples.ndim != 2 or samples.shape[1] != 2 or len(samples) < 2
            or curvature.shape != (len(samples),)
            or not np.all(np.isfinite(samples))
            or not np.all(np.isfinite(curvature))):
        raise ValueError('path needs finite XY samples and one curvature per point')
    if not math.isfinite(path.spacing) or path.spacing <= 0:
        raise ValueError('path spacing must be positive and finite')
    if path.closed and len(samples) < 3:
        raise ValueError('closed path needs at least three samples')
    if (path.target_laps is not None and
            (not path.closed or not math.isfinite(path.target_laps)
             or path.target_laps <= 0)):
        raise ValueError('target_laps requires a positive closed path')


def sampled_path(samples, *, spacing: float, curvatures=None,
                 closed: bool = False, target_laps: float | None = None,
                 frame_id: str = 'odom') -> SampledPlan:
    """Build a path from nearly arc-spaced XY points in the odom frame.

    Supplying analytic curvatures is preferred for B-splines; otherwise a
    centred finite difference estimate is used for ordinary sampled paths.
    """
    points = np.asarray(samples, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
        raise ValueError('samples must contain at least two XY points')
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError('spacing must be positive and finite')
    if not np.all(np.isfinite(points)):
        raise ValueError('path samples must be finite')
    if (closed and len(points) >= 3
            and np.linalg.norm(points[0] - points[-1]) < spacing * 0.1):
        raise ValueError('closed path must omit the duplicate final point')
    if curvatures is None:
        if len(points) == 2:
            kappas = np.zeros(2, dtype=np.float64)
        else:
            if closed:
                d1 = (np.roll(points, -1, axis=0) - np.roll(points, 1, axis=0)) / (2 * spacing)
                d2 = (np.roll(points, -1, axis=0) - 2 * points
                      + np.roll(points, 1, axis=0)) / (spacing ** 2)
            else:
                d1 = np.gradient(points, spacing, axis=0, edge_order=2)
                d2 = np.gradient(d1, spacing, axis=0, edge_order=2)
            denominator = np.maximum(np.sum(d1 * d1, axis=1) ** 1.5, 1e-9)
            kappas = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / denominator
    else:
        kappas = np.asarray(curvatures, dtype=np.float64)
    path = SampledPlan(points, kappas, float(spacing), closed=closed,
                       target_laps=target_laps, frame_id=frame_id)
    _validate_path(path)
    return path


def bspline_path(control_points, *, spacing: float = 0.05,
                 degree: int = 3) -> SampledPlan:
    """Prepare an open B-spline with analytically evaluated curvature."""
    x, y, _, curvature, _ = sample_bspline(control_points, spacing, degree)
    return sampled_path(np.column_stack((x, y)), spacing=spacing,
                        curvatures=curvature)


def circle_path(center_x: float, center_y: float, radius: float, *,
                direction: str = 'ccw', spacing: float = 0.05,
                target_laps: float | None = 1.0) -> SampledPlan:
    """Prepare a closed circle; use target_laps=None for continuous orbiting."""
    values = (center_x, center_y, radius, spacing)
    if (not all(math.isfinite(v) for v in values)
            or radius <= 0 or spacing <= 0):
        raise ValueError('circle centre, radius and spacing must be finite')
    if direction not in ('cw', 'ccw'):
        raise ValueError("direction must be 'cw' or 'ccw'")
    n = max(32, int(math.ceil(2 * math.pi * radius / spacing)))
    sign = -1.0 if direction == 'cw' else 1.0
    theta = sign * np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    points = np.column_stack((center_x + radius * np.cos(theta),
                              center_y + radius * np.sin(theta)))
    curvature = np.full(n, sign / radius)
    return sampled_path(points, spacing=2 * math.pi * radius / n,
                        curvatures=curvature, closed=True,
                        target_laps=target_laps)


@dataclass(frozen=True)
class LosStep:
    linear: float = 0.0
    angular: float = 0.0
    reached: bool = False
    phase: str = 'WAIT_PATH'
    progress_idx: int = 0
    curvature: float = 0.0


# bspline_track_test 의 제어 상수. 수조에서 경로를 잘 따라간다고 확인된 값이다
# (cruise_speed 0.30 = thrust_ratio 0.3, max_angular 0.30, kp_yaw 1.5, kd_yaw 0.15).
# LOS·게이트·회피가 함께 쓴다. 2026-11-06 실물 튜닝: kp 1.5 → 1.4, kd 0.15 → 0.16
# (kp 1.3, kd 0.18·0.17·0.14·0.12 도 시험). bspline_track_test 의 kp_yaw/kd_yaw(1.5/0.15)는 별개.
BSPLINE_TRACK_YAW_KP = 1.4
BSPLINE_TRACK_YAW_KD = 0.16

# 공용 LOS 출력 기본값 = 수조 장애물 회피 SOTA (2026-11-06, AVOID_TEST_LOG.md 설정 4).
# 직진(전진 기준이자 상한)과 회전 상한을 따로 둔다. los()(게이트·탐색)·test_los 가 쓰고,
# 회피(FsmParams·노드 상한)와 같은 값이다. 2026-11-09 적용, 이전엔 둘 다 0.3·lookahead 1.0.
LOS_FORWARD_RATIO = 0.25
LOS_TURN_RATIO = 0.31
LOS_LOOKAHEAD = 0.8   # 경로 위 목표점 전방 거리 [m]

# 실물 추력 곡선 (2026-11-14): 정지 수면 속도 v ≈ MEASURED_SPEED_SCALE × 명령^MEASURED_SPEED_EXP [m/s]
# (명령 = 좌우 평균 추력 비율). dock_test1·dock2(11-13)·run(11-12) 야외 bag 의 명령 → thruster_driver 믹서
# PWM → /odom 속도로 맞춘 값 (scripts/e2e_sim/nomoto.json a·pw). 진행 방향별 대지속도로 물살을 분리해 확인:
# 0.40 → 1.06 m/s(107 s, 6 방위, 물살 0.03), 0.467 → 1.21 m/s(72 s, 물살 0.01). 0.10 → 약 0.34 m/s(두 방향만).
# 0.07~0.33 은 여러 방향 직진 자료가 없어 곡선 보간이다.
# 공용 변환 FsmParams(cmd = 상한 × (v / 1.48)^1.64)는 Gazebo 시뮬 맞춤값이라 상한 0.25 에서 1.48 m/s 를
# 가정하지만 실물은 0.75 m/s — 저속 구간 명령이 실물보다 3~4 배 작게 나온다. 지금은 탐색만 이 곡선을 쓴다
# (LosFollower measured_speed, SearchParams.measured_speed).
MEASURED_SPEED_SCALE = 2.15   # [m/s] 명령 1.0 일 때 (외삽)
MEASURED_SPEED_EXP = 0.76


class LosFollower:
    """Stateful, ROS-free progress tracker around the shared LOS control law.

    ``ratio_limits_only`` 가 False(기본, 게이트·탐색 behavior 가 쓰는 기존 동작)면
    thrust_ratio 를 **최종 명령 전체**에 곱한다. 그러면 요 PD 의 루프 게인도 같이
    ratio 배로 줄어든다. True 면 bspline_track_test 와 같이 게인은 그대로 두고
    ratio 를 **출력 한계**(전진 cmd_full·상한, 회전 상한)에만 적용한다. 이때
    max_linear/max_angular 는 ratio=1 일 때의 전체 스케일이다.

    ``measured_speed`` (ratio_limits_only 일 때만): 원하는 속도 → 전진 명령을 공용 변환
    (상한 × (v/1.48)^1.64 — 상한에서 1.48 m/s 가정) 대신 실물 곡선
    (v / MEASURED_SPEED_SCALE)^(1/MEASURED_SPEED_EXP) 로 바꾸고 thrust_ratio 는 상한으로만 쓴다.
    직선·선회처럼 빠른 구간은 둘 다 상한에 걸려 같고, 선수각 오차·급한 굽이·목표 근처처럼
    속도를 줄이는 구간에서 명령이 실물에 맞게 커진다 (탐색 상한 0.25: 0.42 m/s 요청에 0.032 → 0.117).
    """

    def __init__(self, *, params: FsmParams | None = None,
                 yaw_kp: float = 18.0 / 118.6,
                 yaw_kd: float = 9.0 / 118.6,
                 max_linear: float = 12.0 / 118.6,
                 max_angular: float = 3.0 / 118.6,
                 goal_tolerance: float = 0.4,
                 ratio_limits_only: bool = False,
                 measured_speed: bool = False):
        if measured_speed and not ratio_limits_only:
            raise ValueError('measured_speed needs ratio_limits_only (thrust_ratio is then a cap)')
        self.params = params or replace(FsmParams(), lookahead=LOS_LOOKAHEAD,
                                        slow_radius=1.5,
                                        cmd_full=max_linear)
        self.yaw_kp, self.yaw_kd = yaw_kp, yaw_kd
        self.max_linear, self.max_angular = max_linear, max_angular
        self.goal_tolerance = goal_tolerance
        self.ratio_limits_only = ratio_limits_only
        self.measured_speed = measured_speed
        self.reset()

    @classmethod
    def bspline_track(cls, *, goal_tolerance: float = 0.4,
                      measured_speed: bool = False) -> 'LosFollower':
        """bspline_track_test 와 같은 제어 상수·출력 규칙의 follower.

        step(thrust_ratio=r, turn_ratio=t) 이면 전진 상한과 cmd_full 이 r, 회전 상한이 t
        (생략하면 r) 이고 게인은 kp 1.4 / kd 0.16, lookahead 0.8 m 다
        (bspline_track_test 는 kp 1.5 / kd 0.15, lookahead 1.0).
        measured_speed 면 cmd_full 대신 실물 곡선으로 속도 → 명령 (클래스 설명).
        """
        return cls(yaw_kp=BSPLINE_TRACK_YAW_KP, yaw_kd=BSPLINE_TRACK_YAW_KD,
                   max_linear=1.0, max_angular=1.0,
                   goal_tolerance=goal_tolerance, ratio_limits_only=True,
                   measured_speed=measured_speed)

    def reset(self):
        self.path = None
        self.progress_idx = None
        self.start_idx = None
        self.completed = False

    def set_path(self, path: SampledPlan):
        if path is not self.path:
            _validate_path(path)
            self.path = path
            self.progress_idx = None
            self.start_idx = None
            self.completed = False

    def _advance(self, boat: np.ndarray):
        path = self.path
        n = len(path.samples)
        if self.progress_idx is None:
            # Open courses begin at their first segment, like bspline_track_test.
            # A global search could jump to a later branch at a self-crossing.
            first_window = (n if path.closed else
                            min(n, max(2, int(math.ceil(3.0 / path.spacing)) + 1)))
            nearest = int(np.argmin(np.linalg.norm(
                path.samples[:first_window] - boat, axis=1)))
            self.progress_idx = nearest
            self.start_idx = nearest
            return
        back = max(1, int(math.ceil(0.25 / path.spacing)))
        ahead = max(1, int(math.ceil(3.0 / path.spacing)))
        if path.closed:
            # Never look a full lap ahead: the same XY would skip lap counts.
            back = min(back, max(1, n // 6))
            ahead = min(ahead, max(1, n // 3))
            candidates = np.arange(self.progress_idx - back,
                                   self.progress_idx + ahead + 1)
            points = path.samples[candidates % n]
        else:
            candidates = np.arange(max(0, self.progress_idx - back),
                                   min(n - 1, self.progress_idx + ahead) + 1)
            points = path.samples[candidates]
        nearest = int(candidates[np.argmin(np.linalg.norm(points - boat, axis=1))])
        self.progress_idx = max(self.progress_idx, nearest)

    def step(self, path: SampledPlan, boat_xy, yaw: float, yaw_rate: float,
             thrust_ratio: float = 1.0, turn_ratio: float | None = None) -> LosStep:
        """turn_ratio 를 주면 회전 상한만 따로 정한다(생략하면 thrust_ratio 와 같음)."""
        ratio = float(thrust_ratio)
        turn = ratio if turn_ratio is None else float(turn_ratio)
        if not all(math.isfinite(v) and 0.0 <= v <= 1.0 for v in (ratio, turn)):
            raise ValueError('thrust_ratio and turn_ratio must be between 0 and 1')
        self.set_path(path)
        boat = np.asarray(boat_xy, dtype=np.float64)
        if (boat.shape != (2,) or not np.all(np.isfinite(boat))
                or not math.isfinite(yaw) or not math.isfinite(yaw_rate)):
            return LosStep(phase='INVALID_ODOM')
        if self.completed:
            return LosStep(reached=True, phase='REACHED',
                           progress_idx=self.progress_idx)
        self._advance(boat)
        n = len(path.samples)
        if path.closed:
            if path.target_laps is not None:
                finish_progress = self.start_idx + int(math.ceil(path.target_laps * n))
                finish_distance = float(np.linalg.norm(
                    boat - path.samples[finish_progress % n]))
                if (self.progress_idx >= finish_progress
                        and finish_distance <= self.goal_tolerance):
                    self.completed = True
                    return LosStep(reached=True, phase='REACHED',
                                   progress_idx=self.progress_idx)
        else:
            goal_distance = float(np.linalg.norm(boat - path.samples[-1]))
            remaining = (n - 1 - self.progress_idx) * path.spacing
            if (goal_distance <= self.goal_tolerance
                    and remaining <= self.params.lookahead):
                self.completed = True
                return LosStep(reached=True, phase='REACHED',
                               progress_idx=self.progress_idx)

        plan = replace(path, progress_idx=self.progress_idx % n)
        waypoint_xy = None if path.closed else path.samples[-1]
        curvature = curvature_ahead(plan, self.params.kappa_window)
        if self.ratio_limits_only:
            # bspline_track_test 와 같은 규칙: ratio 는 cmd_full 과 출력 상한에만,
            # 요 PD 게인은 그대로.
            linear_cap = self.max_linear * ratio
            angular_cap = self.max_angular * turn
            if self.measured_speed:
                # 실물 곡선 cmd = (v / scale)^(1/exp) — 아래에서 linear_cap 으로 자른다
                speed_curve = replace(self.params, cmd_full=1.0, v_full=MEASURED_SPEED_SCALE,
                                      cmd_exp=1.0 / MEASURED_SPEED_EXP)
            else:
                speed_curve = replace(self.params, cmd_full=linear_cap)
            linear, angular = follow_cmd(
                plan, boat, yaw, yaw_rate, waypoint_xy, speed_curve,
                self.yaw_kp, self.yaw_kd)
            return LosStep(
                linear=min(max(linear, 0.0), linear_cap),
                angular=max(-angular_cap, min(angular_cap, angular)),
                phase='FOLLOW', progress_idx=self.progress_idx,
                curvature=curvature)
        linear, angular = follow_cmd(
            plan, boat, yaw, yaw_rate, waypoint_xy,
            self.params, self.yaw_kp, self.yaw_kd)
        return LosStep(
            linear=min(max(linear, 0.0), self.max_linear) * ratio,
            angular=max(-self.max_angular,
                        min(self.max_angular, angular)) * turn,
            phase='FOLLOW', progress_idx=self.progress_idx,
            curvature=curvature)
