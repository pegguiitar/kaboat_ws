"""LOS 경로 추종 — 실패 시 안전(C) 보강: 잘못된 odom 은 명령 0."""
import math

import pytest

from kaboat_behaviors.los_controller import LosFollower, bspline_path


@pytest.mark.check('C')
@pytest.mark.parametrize('boat, yaw', [((math.nan, 0.0), 0.0), ((0.0, 0.0), math.inf)])
def test_invalid_odom_gives_zero_command(boat, yaw):
    path = bspline_path([(0.0, 0.0), (2.0, 0.0), (4.0, 1.0), (6.0, 1.0)], spacing=0.1)
    step = LosFollower().step(path, boat, yaw, 0.0)
    assert step.phase == 'INVALID_ODOM'
    assert step.linear == 0.0 and step.angular == 0.0 and not step.reached
