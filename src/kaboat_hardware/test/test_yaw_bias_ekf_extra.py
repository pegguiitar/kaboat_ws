"""yaw bias EKF — 실패 시 안전(C) 보강: 초기화 전·시간 간격 이상은 예측하지 않는다."""
import pytest

from kaboat_hardware.yaw_bias_ekf import YawBiasEkf


@pytest.mark.check('C')
def test_predict_skips_before_init_and_on_bad_time_gaps():
    ekf = YawBiasEkf()
    assert not ekf.predict(0.5, 1.0)             # 초기화 전
    ekf.initialize(0.3, 0.01)
    assert not ekf.predict(0.5, 1.0)             # 첫 시각은 기준만 잡는다
    assert not ekf.predict(0.5, 2.0)             # 1 s 간격 > max_dt 0.2 s
    assert not ekf.predict(0.5, 1.9)             # 시간이 거꾸로 감
    assert ekf.yaw == pytest.approx(0.3)         # 어느 경우도 yaw 를 적분하지 않았다
    assert ekf.predict(0.5, 2.0)                 # 정상 간격 0.1 s 는 예측한다
