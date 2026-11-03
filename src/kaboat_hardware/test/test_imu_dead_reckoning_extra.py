"""IMU 추측 항법 — 경계(B)·실패 시 안전(C) 보강: 속도 상한, 움직이는 중 보정 거부."""
import pytest

from kaboat_hardware.imu_dead_reckoning_core import DeadReckoningParams, ImuDeadReckoner
from test_imu_dead_reckoning import IDENTITY, _calibrate


@pytest.mark.check('B')
def test_speed_is_capped_at_max_speed():
    params = DeadReckoningParams(calibration_duration_sec=0.2, accel_deadband=0.0,
                                 accel_filter_tau=0.0, velocity_damping_tau=0.0,
                                 max_dt=0.02, max_speed=0.5)
    estimator = ImuDeadReckoner(params)
    _calibrate(estimator)
    start = estimator._last_t
    state = None
    for k in range(1, 301):                     # 2 m/s² 로 3 s → 상한이 없으면 6 m/s
        state = estimator.update(start + k * 0.01, IDENTITY, (0.0, 0.0, 0.0),
                                 (2.0, 0.0, params.gravity))
    assert state.vx_world == pytest.approx(0.5, abs=1e-6)


@pytest.mark.check('C')
def test_calibration_restarts_while_boat_is_rotating():
    params = DeadReckoningParams(calibration_duration_sec=0.2)
    estimator = ImuDeadReckoner(params)
    for k in range(50):                         # 0.5 s 동안 0.2 rad/s 회전 (한계 0.08)
        out = estimator.update(k * 0.01, IDENTITY, (0.0, 0.0, 0.2), (0.0, 0.0, params.gravity))
        assert out is None
    assert not estimator.ready
