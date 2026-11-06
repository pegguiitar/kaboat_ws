"""Open B-spline and closed-circle LOS share one follower and speed law."""

import math

import numpy as np
import pytest

from kaboat_behaviors.bspline_planner import curvature_ahead
from kaboat_behaviors.los_controller import (
    LosFollower, bspline_path, circle_path, sampled_path)


def test_bspline_path_has_analytic_curvature_and_reaches_end():
    path = bspline_path([(0.0, 0.0), (2.0, 0.0),
                         (4.0, 1.0), (6.0, 1.0)], spacing=0.1)
    assert not path.closed
    assert len(path.samples) == len(path.curvatures)
    assert np.all(np.isfinite(path.curvatures))
    follower = LosFollower()
    first = follower.step(path, path.samples[0], 0.0, 0.0)
    assert first.phase == 'FOLLOW' and first.linear > 0
    for point in path.samples[::10]:
        follower.step(path, point, 0.0, 0.0)
    end = follower.step(path, path.samples[-1], 0.0, 0.0)
    assert end.reached and end.linear == 0 and end.angular == 0
    assert follower.step(path, path.samples[0], 0.0, 0.0).reached


def test_curvature_inverse_speed_reduces_thrust():
    samples = np.column_stack((np.arange(0.0, 10.1, 0.1),
                               np.zeros(101)))
    straight = sampled_path(samples, spacing=0.1,
                            curvatures=np.zeros(101))
    curved = sampled_path(samples, spacing=0.1,
                          curvatures=np.ones(101))
    full = LosFollower().step(straight, (0.0, 0.0), 0.0, 0.0)
    slow = LosFollower().step(curved, (0.0, 0.0), 0.0, 0.0)
    assert slow.curvature == pytest.approx(1.0)
    assert 0 < slow.linear < full.linear


def test_circle_lookahead_and_curvature_wrap_seam():
    path = circle_path(5.0, 2.5, 1.2, spacing=0.1)
    assert path.closed and path.target_laps == 1.0
    assert np.max(np.abs(path.curvatures - 1 / 1.2)) < 1e-12
    path.curvatures[0] = 2.0
    path.progress_idx = len(path.samples) - 2
    assert curvature_ahead(path, 0.5) == pytest.approx(2.0)


def test_circle_counts_one_lap_and_stops():
    path = circle_path(0.0, 0.0, 1.2, spacing=0.2)
    follower = LosFollower()
    n = len(path.samples)
    for i in range(n):
        theta = 2 * math.pi * i / n
        step = follower.step(path, path.samples[i], theta + math.pi / 2, 0.0)
        assert not step.reached
    end = follower.step(path, path.samples[0], math.pi / 2, 0.0)
    assert end.reached and end.progress_idx >= n
    assert (end.linear, end.angular) == (0.0, 0.0)
    assert follower.step(path, path.samples[1], 0.0, 0.0).reached


def test_circle_can_run_continuously_and_reverse_direction():
    path = circle_path(0.0, 0.0, 1.2, direction='cw',
                       spacing=0.2, target_laps=None)
    assert np.all(path.curvatures < 0)
    follower = LosFollower()
    n = len(path.samples)
    for i in range(n + 2):
        point = path.samples[i % n]
        step = follower.step(path, point, -math.pi / 2, 0.0)
        assert not step.reached
    assert step.progress_idx >= n


def test_circle_supports_fractional_laps():
    path = circle_path(0.0, 0.0, 1.2, spacing=0.2, target_laps=0.5)
    follower = LosFollower()
    n = len(path.samples)
    for i in range(n // 2):
        assert not follower.step(path, path.samples[i], 0.0, 0.0).reached
    end = follower.step(path, path.samples[n // 2], 0.0, 0.0)
    assert end.reached


def test_thrust_ratio_scales_los_and_rejects_invalid_values():
    path = circle_path(0.0, 0.0, 1.2, spacing=0.2)
    full = LosFollower().step(path, path.samples[0], 0.0, 0.0, 1.0)
    half = LosFollower().step(path, path.samples[0], 0.0, 0.0, 0.5)
    assert half.linear == pytest.approx(full.linear * 0.5)
    assert half.angular == pytest.approx(full.angular * 0.5)
    zero = LosFollower().step(path, path.samples[0], 0.0, 0.0, 0.0)
    assert (zero.linear, zero.angular) == (0.0, 0.0)
    for ratio in (-0.1, 1.1, float('nan')):
        with pytest.raises(ValueError):
            LosFollower().step(path, path.samples[0], 0.0, 0.0, ratio)


def test_invalid_path_rejected():
    with pytest.raises(ValueError):
        sampled_path([(0.0, 0.0), (1.0, 0.0)], spacing=0.0)
    with pytest.raises(ValueError):
        circle_path(0.0, 0.0, 0.0)


# ───────── bspline_track_test 와 같은 제어 모드 ─────────
import math

from dataclasses import replace

import numpy as np

from kaboat_behaviors.avoid_fsm import FsmParams, follow_cmd
from kaboat_behaviors.los_controller import (
    BSPLINE_TRACK_YAW_KD, BSPLINE_TRACK_YAW_KP, LosFollower, bspline_path)

TRACK_CPS = [(8.5, 2.0), (7.0, 3.5), (5.0, 1.5), (3.0, 3.5), (1.5, 2.5)]


def bspline_test_reference(path, progress_idx, boat, yaw, yaw_rate,
                           cruise=0.30, max_angular=0.30, kd=BSPLINE_TRACK_YAW_KD):
    """bspline_track_test._control_loop 의 명령 계산을 그대로 옮긴 기준값.

    D 게인은 LOS 공용 상수를 따른다(지금은 bspline_track_test 와 같은 0.15).
    """
    params = replace(FsmParams(), lookahead=1.0, slow_radius=1.5, cmd_full=cruise)
    plan = replace(path, progress_idx=progress_idx)
    linear, angular = follow_cmd(plan, boat, yaw, yaw_rate, path.samples[-1],
                                 params, 1.5, kd)
    return (max(0.0, min(cruise, linear)),
            max(-max_angular, min(max_angular, angular)))


def sample_states(path):
    states = []
    for idx, offset, heading_deg, rate in ((20, 0.0, 0, 0.0), (20, 0.0, 15, 0.1),
                                           (60, 0.3, 0, -0.2), (60, 0.3, 20, 0.0),
                                           (70, 0.2, 30, 0.05), (70, 0.2, 60, 0.0),
                                           (110, -0.3, -25, 0.0)):
        p, q = path.samples[idx], path.samples[idx + 1]
        tangent = math.atan2(q[1] - p[1], q[0] - p[0])
        normal = np.array([-math.sin(tangent), math.cos(tangent)])
        states.append((tuple(p + offset * normal),
                       tangent + math.radians(heading_deg), rate))
    return states


def test_bspline_track_mode_reproduces_bspline_track_test_commands():
    path = bspline_path(TRACK_CPS, spacing=0.05)
    for boat, yaw, rate in sample_states(path):
        follower = LosFollower.bspline_track()
        step = follower.step(path, boat, yaw, rate, 0.3)
        want_lin, want_ang = bspline_test_reference(
            path, follower.progress_idx, boat, yaw, rate)
        assert step.linear == pytest.approx(want_lin, abs=1e-12)
        assert step.angular == pytest.approx(want_ang, abs=1e-12)


def test_bspline_track_constants_are_the_tested_ones():
    f = LosFollower.bspline_track()
    assert (f.yaw_kp, f.yaw_kd) == (BSPLINE_TRACK_YAW_KP, BSPLINE_TRACK_YAW_KD) == (1.5, 0.15)
    assert f.ratio_limits_only and f.max_linear == f.max_angular == 1.0


def test_default_los_behaviour_is_unchanged_for_missions():
    """게이트·탐색 behavior 가 쓰는 기존 규칙(ratio 를 최종 명령에 곱함)은 그대로다."""
    path = bspline_path(TRACK_CPS, spacing=0.05)
    boat, yaw, rate = sample_states(path)[3]
    legacy = LosFollower().step(path, boat, yaw, rate, 0.3)
    assert abs(legacy.angular) <= 3.0 / 118.6 * 0.3 + 1e-12
    assert legacy.linear <= 12.0 / 118.6 * 0.3 + 1e-12


def test_old_los_test_settings_were_far_weaker_than_bspline_test():
    """회귀 방지: 예전 LOS 테스트 설정은 같은 상황에서 회전·전진 한계가 훨씬 작았다."""
    path = bspline_path(TRACK_CPS, spacing=0.05)
    boat, yaw, rate = sample_states(path)[3]            # 옆 0.3 m 이탈, 헤딩 20°
    old = LosFollower().step(path, boat, yaw, rate, 0.3)
    new = LosFollower.bspline_track().step(path, boat, yaw, rate, 0.3)
    assert new.angular == pytest.approx(-0.30)           # 상한까지 조향
    assert abs(new.angular) > 20 * abs(old.angular)
    assert new.linear > 5 * old.linear


def test_thrust_ratio_limits_outputs_but_not_the_yaw_gain():
    path = bspline_path(TRACK_CPS, spacing=0.05)
    # 상한에 걸리는 큰 오차(옆 0.3 m 이탈, 헤딩 20°): ratio 가 상한을 정한다
    boat, yaw, rate = sample_states(path)[3]
    lo = LosFollower.bspline_track().step(path, boat, yaw, rate, 0.3)
    hi = LosFollower.bspline_track().step(path, boat, yaw, rate, 0.6)
    assert lo.angular == pytest.approx(-0.3) and abs(hi.angular) > 0.3
    assert hi.linear > lo.linear                         # cmd_full 도 ratio 를 따른다
    # 상한에 안 걸리는 작은 오차: 요 PD 게인은 ratio 와 무관하다 (예전 방식은 ratio 배)
    boat, yaw, rate = sample_states(path)[1]             # 경로 위, 헤딩 오차 15°
    a = LosFollower.bspline_track().step(path, boat, yaw, rate, 0.3)
    b = LosFollower.bspline_track().step(path, boat, yaw, rate, 0.6)
    assert abs(a.angular) < 0.3 and a.angular == pytest.approx(b.angular)
    legacy_a = LosFollower().step(path, boat, yaw, rate, 0.3)
    legacy_b = LosFollower().step(path, boat, yaw, rate, 0.6)
    assert legacy_b.angular == pytest.approx(2 * legacy_a.angular)   # 기존 모드는 ratio 에 비례


def test_ratio_zero_and_invalid_ratio_still_safe_in_bspline_mode():
    path = bspline_path(TRACK_CPS, spacing=0.05)
    boat, yaw, rate = sample_states(path)[0]
    zero = LosFollower.bspline_track().step(path, boat, yaw, rate, 0.0)
    assert zero.linear == 0.0 and zero.angular == 0.0
    for ratio in (-0.1, 1.1, float('nan')):
        with pytest.raises(ValueError):
            LosFollower.bspline_track().step(path, boat, yaw, rate, ratio)
