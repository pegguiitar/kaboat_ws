import math
import random

from kaboat_hardware.pose_velocity import (
    VelocityEstimator, VelocityParams, normalize_angle, world_to_body)


def _feed(est, samples):
    out = None
    for t, x, y, yaw in samples:
        out = est.update(t, x, y, yaw)
    return out


def _straight(vx, vy, yaw=0.0, n=40, dt=1 / 30):
    return [(i * dt, vx * i * dt, vy * i * dt, yaw) for i in range(n)]


def test_normalize_angle_wraps_across_pi():
    # +179° → -179° 를 그냥 빼면 -358° 가 나온다 — yaw 미분이 폭발하는 지점
    delta = normalize_angle(math.radians(-179) - math.radians(179))
    assert math.isclose(delta, math.radians(2), abs_tol=1e-6)


def test_world_to_body_is_inverse_of_planner_rotation():
    # obstacle_planner 는 body→world 로 되돌린다. 왕복이 항등이어야 한다.
    vx_w, vy_w, yaw = 1.3, -0.4, 0.7
    bx, by = world_to_body(vx_w, vy_w, yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    assert math.isclose(c * bx - s * by, vx_w, abs_tol=1e-9)
    assert math.isclose(s * bx + c * by, vy_w, abs_tol=1e-9)


def test_constant_velocity_converges():
    est = VelocityEstimator(VelocityParams(filter_tau=0.05))
    vx, vy, _ = _feed(est, _straight(1.0, 0.0))
    assert math.isclose(vx, 1.0, abs_tol=0.05)
    assert math.isclose(vy, 0.0, abs_tol=0.05)


def test_velocity_is_body_frame_not_world():
    """배가 +90° 를 보고 월드 +y 로 가면 body 로는 '전진'이어야 한다."""
    est = VelocityEstimator(VelocityParams(filter_tau=0.05))
    vx, vy, _ = _feed(est, _straight(0.0, 1.0, yaw=math.pi / 2))
    assert math.isclose(vx, 1.0, abs_tol=0.05)   # 전방
    assert math.isclose(vy, 0.0, abs_tol=0.05)   # 횡방향 아님


def test_stationary_stays_zero():
    est = VelocityEstimator()
    vx, vy, yaw_rate = _feed(est, [(i / 30, 0.0, 0.0, 0.0) for i in range(30)])
    assert abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(yaw_rate) < 1e-6


def test_detection_glitch_is_rejected():
    """한 프레임 튄 검출이 속도에 반영되면 안 된다 (max_speed 게이트)."""
    est = VelocityEstimator(VelocityParams(filter_tau=0.05, max_speed=3.0))
    _feed(est, _straight(1.0, 0.0, n=20))
    before = est.update(20 / 30, 20 / 30, 0.0, 0.0)[0]
    after = est.update(21 / 30, 50.0, 0.0, 0.0)[0]   # 50m 순간이동
    assert math.isclose(after, before, abs_tol=0.05)


def test_yaw_rate_fallback_tracks_rotation():
    """IMU 폴백 경로 — 등속 회전이면 그 각속도가 나와야 한다."""
    est = VelocityEstimator(VelocityParams(filter_tau=0.05))
    rate, dt = 0.4, 1 / 30
    samples = [(i * dt, 0.0, 0.0, normalize_angle(rate * i * dt))
               for i in range(60)]
    _, _, yaw_rate = _feed(est, samples)
    assert math.isclose(yaw_rate, rate, abs_tol=0.05)


def test_yaw_rate_fallback_survives_pi_crossing():
    """랩어라운드 지점을 지나도 각속도가 튀면 안 된다."""
    est = VelocityEstimator(VelocityParams(filter_tau=0.05))
    rate, dt = 0.4, 1 / 30
    samples = [(i * dt, 0.0, 0.0, normalize_angle(math.pi - 0.2 + rate * i * dt))
               for i in range(60)]
    _, _, yaw_rate = _feed(est, samples)
    assert math.isclose(yaw_rate, rate, abs_tol=0.05)


def test_reset_prevents_differencing_across_gap():
    """태그 재획득 시 공백을 건너뛰어 미분하면 가짜 속도가 나온다."""
    est = VelocityEstimator(VelocityParams(filter_tau=0.0))
    _feed(est, _straight(1.0, 0.0, n=10))
    est.reset()
    vx, _, _ = est.update(100.0, 500.0, 0.0, 0.0)   # 긴 공백 후 먼 곳에서 재획득
    assert vx == 0.0


def _velocity_rms(est, samples):
    """정지 상태 표본을 흘리고 속도 추정의 RMS(=순수 노이즈)를 잰다."""
    total, n = 0.0, 0
    for i, (t, x, y, yaw) in enumerate(samples):
        vx, _, _ = est.update(t, x, y, yaw)
        if i >= 10:                      # 초기 과도 구간 제외
            total += vx * vx
            n += 1
    return math.sqrt(total / n)


def test_longer_window_reduces_noise():
    """미분 baseline 을 늘리면 노이즈가 줄어야 한다 (설계 근거의 회귀 테스트).

    5mm 검출 노이즈를 인접 프레임(1/30s)으로 미분하면 0.15 m/s 수준의 가짜
    속도가 나온다 — 수조 주행 속도와 맞먹는다. baseline 을 늘려 줄인다.
    """
    rng = random.Random(1209)
    dt, sigma = 1 / 30, 0.005      # 5mm 검출 노이즈
    stationary = [(i * dt, rng.gauss(0.0, sigma), rng.gauss(0.0, sigma), 0.0)
                  for i in range(600)]

    short = _velocity_rms(
        VelocityEstimator(VelocityParams(window_sec=dt, filter_tau=0.0)),
        stationary)
    long_ = _velocity_rms(
        VelocityEstimator(VelocityParams(window_sec=0.15, filter_tau=0.0)),
        stationary)

    assert short > 0.1                   # 짧은 baseline 은 실제로 위험한 수준
    assert long_ < short / 2.0           # baseline 을 늘리면 절반 이하로


def test_filter_further_reduces_noise():
    """EMA 를 얹으면 baseline 만 늘린 것보다 더 조용해야 한다."""
    rng = random.Random(1209)
    dt, sigma = 1 / 30, 0.005
    stationary = [(i * dt, rng.gauss(0.0, sigma), rng.gauss(0.0, sigma), 0.0)
                  for i in range(600)]

    unfiltered = _velocity_rms(
        VelocityEstimator(VelocityParams(window_sec=0.15, filter_tau=0.0)),
        stationary)
    filtered = _velocity_rms(
        VelocityEstimator(VelocityParams(window_sec=0.15, filter_tau=0.15)),
        stationary)
    assert filtered < unfiltered
