"""판 자세 추정 — 정상 동작(A)·실패 시 안전(C) 보강."""
import math

import numpy as np
import pytest

from kaboat_hardware.lidar_panel_pose import fit_panel_line, select_panel_line

KW = dict(min_points=5, min_span=0.3, max_span=1.2, max_line_rms=0.03)


@pytest.mark.check('A')
def test_axis_aligned_panel_gives_center_and_heading():
    # 판이 y 축과 나란하면(좌현 → 우현) 선체 선수는 ±x 방향이다.
    pts = np.column_stack((np.full(20, 2.0), np.linspace(1.6, 2.4, 20)))
    panel = fit_panel_line(pts, **KW)
    assert panel is not None
    assert (panel.x, panel.y) == pytest.approx((2.0, 2.0), abs=1e-9)
    assert panel.span == pytest.approx(0.8)
    assert abs(math.sin(panel.yaw)) == pytest.approx(0.0, abs=1e-9)   # 선수 ±x (π 모호)


@pytest.mark.check('C')
def test_non_finite_points_or_far_jump_are_rejected():
    pts = np.column_stack((np.full(20, 2.0), np.linspace(1.6, 2.4, 20)))
    bad = pts.copy()
    bad[3, 0] = np.nan
    assert fit_panel_line(bad, **KW) is None
    far = select_panel_line([pts], expected_span=0.8, previous_pose=(6.0, 2.0, 0.0),
                            max_position_jump=1.0, **KW)
    assert far is None                           # 이전 pose 에서 4 m 순간 이동은 거부
