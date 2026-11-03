"""추진기 출력 변환 — 실패 시 안전(C) 보강: NaN 명령은 중립이어야 한다."""
import math

import pytest

from kaboat_hardware.thruster_backend import ThrusterConfig, ThrusterMixer, ratio_to_pwm


@pytest.mark.check('C')
def test_nan_command_maps_to_neutral_pwm():
    cfg = ThrusterConfig()
    assert ratio_to_pwm(math.nan, cfg) == cfg.neutral_pwm
    left, right, _, _ = ThrusterMixer(cfg).mix(math.nan, 0.0, 1.0)
    assert left == right == cfg.neutral_pwm
