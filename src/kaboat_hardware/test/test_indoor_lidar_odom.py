import math
import pytest
from geometry_msgs.msg import Quaternion
from kaboat_hardware.indoor_lidar_odom import (
    _yaw_from_quat, _quat_from_yaw, normalize_angle
)


def test_quaternion_yaw_conversion():
    angles = [0.0, 0.5, math.pi / 2, math.pi - 0.01, -math.pi / 2, -2.5]
    for angle in angles:
        q = _quat_from_yaw(angle)
        recovered = _yaw_from_quat(q)
        err = normalize_angle(angle - recovered)
        assert math.isclose(err, 0.0, abs_tol=1e-6)


def test_normalize_angle():
    assert math.isclose(abs(normalize_angle(3 * math.pi)), math.pi, abs_tol=1e-6)
    assert math.isclose(abs(normalize_angle(-3 * math.pi)), math.pi, abs_tol=1e-6)
    assert math.isclose(normalize_angle(math.radians(370)), math.radians(10), abs_tol=1e-6)


def test_resume_behavior():
    """위치 신호 유실 후 재수신 시 초기화 및 복구 검증."""
    from kaboat_hardware.pose_velocity import VelocityEstimator, VelocityParams
    est = VelocityEstimator(VelocityParams(filter_tau=0.15))

    # 1. 초기 10개 정상 수신 (dt=0.1s, vx=1.0m/s)
    for i in range(10):
        t = i * 0.1
        vx, vy, _ = est.update(t, t * 1.0, 0.0, 0.0)

    assert math.isclose(vx, 1.0, abs_tol=0.1)

    # 2. 3초간 신호 단절 (gap)
    # 복구 시 estimator.reset()을 호출하지 않으면 3초 동안의 위치 차분으로 비정상적인 거대 속도가 계산됨
    est.reset()

    # 3. 신호 복구 후 첫 샘플 (t=4.0s)
    vx_rec, vy_rec, _ = est.update(4.0, 4.0, 0.0, 0.0)
    assert vx_rec == 0.0 and vy_rec == 0.0  # reset 덕분에 거대 스파이크 없이 안전하게 0부터 시작

    # 4. 몇 샘플 후 정상 속도로 EMA 수렴 (스파이크 없이 부드러운 복구)
    for i in range(1, 6):
        vx_rec, vy_rec, _ = est.update(4.0 + i * 0.1, 4.0 + i * 0.1, 0.0, 0.0)
    assert math.isclose(vx_rec, 1.0, abs_tol=0.1)


