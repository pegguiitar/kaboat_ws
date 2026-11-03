"""게이트 통로 선택 — 경계·파라미터(B) 보강: 최소 게이트 폭."""
import pytest

from kaboat_behaviors.gate_corridor import GateParams, select_gate_pair
from test_gate_corridor import gate_grid


@pytest.mark.check('B')
def test_min_gate_width_boundary():
    # 기둥 2×2칸(0.4 m)이 칠해지므로 실제 간격은 중심 간격 + 0.2 m 안팎이다.
    narrow = gate_grid([(3.0, -0.6), (3.0, 0.6)])     # 약 1.2 m — min_gate_width 1.5 m 미만
    wide = gate_grid([(3.0, -1.0), (3.0, 1.0)])       # 약 2.0 m
    params = GateParams()
    assert select_gate_pair(narrow, (0.0, 0.0), (9.0, 0.0), [], params) is None
    pair = select_gate_pair(wide, (0.0, 0.0), (9.0, 0.0), [], params)
    assert pair is not None and pair.width >= params.min_gate_width
